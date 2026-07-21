#include "thor/region_grid/medoid.h"
#include "thor/region_grid/progress_log.h"

#include "midgard/logging.h"

#include <h3api.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace valhalla {
namespace thor {
namespace region_grid {

namespace {

struct DijkstraState {
  uint32_t node;
  uint32_t dist;
  bool operator>(const DijkstraState& o) const {
    return dist > o.dist;
  }
};

// Dijkstra from source, only relaxing nodes in allow_set. Returns distances
// (UINT32_MAX if unreachable) for all keys in allow_set that were touched.
std::unordered_map<uint32_t, uint32_t>
dijkstra_restricted(const cch::CchGraph& graph,
                    uint32_t source,
                    const std::unordered_set<uint32_t>& allow_set) {
  std::unordered_map<uint32_t, uint32_t> dist;
  if (allow_set.find(source) == allow_set.end())
    return dist;
  dist[source] = 0;
  std::priority_queue<DijkstraState, std::vector<DijkstraState>, std::greater<DijkstraState>> pq;
  pq.push({source, 0});
  while (!pq.empty()) {
    const auto cur = pq.top();
    pq.pop();
    auto it = dist.find(cur.node);
    if (it == dist.end() || cur.dist > it->second)
      continue;
    if (cur.node >= graph.out_offsets.size() - 1)
      continue;
    for (uint32_t ei = graph.out_offsets[cur.node]; ei < graph.out_offsets[cur.node + 1]; ++ei) {
      const uint32_t edge_idx = graph.out_edges[ei];
      const auto& e = graph.edges[edge_idx];
      if (allow_set.find(e.v) == allow_set.end())
        continue;
      const uint32_t nd = cur.dist + e.time_s;
      auto dit = dist.find(e.v);
      if (dit == dist.end() || nd < dit->second) {
        dist[e.v] = nd;
        pq.push({e.v, nd});
      }
    }
  }
  return dist;
}

std::vector<uint32_t> sample_nodes(const std::vector<uint32_t>& nodes, uint32_t cap) {
  if (nodes.size() <= cap)
    return nodes;
  std::vector<uint32_t> sampled;
  sampled.reserve(cap);
  // Even stride sample for determinism.
  const double step = static_cast<double>(nodes.size()) / static_cast<double>(cap);
  for (uint32_t i = 0; i < cap; ++i) {
    const size_t idx = std::min(nodes.size() - 1, static_cast<size_t>(i * step));
    sampled.push_back(nodes[idx]);
  }
  // Always include first (often near cell center-ish after sort).
  std::sort(sampled.begin(), sampled.end());
  sampled.erase(std::unique(sampled.begin(), sampled.end()), sampled.end());
  return sampled;
}

uint8_t best_incident_roadclass(const cch::CchGraph& graph, uint32_t node) {
  uint8_t best = 255;
  if (node >= graph.out_offsets.size() - 1)
    return best;
  for (uint32_t ei = graph.out_offsets[node]; ei < graph.out_offsets[node + 1]; ++ei) {
    const auto& e = graph.edges[graph.out_edges[ei]];
    if (e.roadclass == 255)
      continue;
    best = std::min(best, e.roadclass);
  }
  return best;
}

std::unordered_set<uint32_t> build_halo_allow(const cch::CchGraph& graph,
                                               const CellSeed& cell,
                                               const std::unordered_map<uint64_t, std::vector<uint32_t>>&
                                                   h3_to_nodes) {
  std::unordered_set<uint32_t> allow;
  allow.insert(cell.node_indices.begin(), cell.node_indices.end());
  // 1-ring H3 halo: include nodes whose H3 (same res as cell) is a neighbor.
  const int res = H3Resolution(cell.h3_index);
  auto disk = H3GridDisk(cell.h3_index, 1);
  for (uint64_t h : disk) {
    auto it = h3_to_nodes.find(h);
    if (it == h3_to_nodes.end())
      continue;
    allow.insert(it->second.begin(), it->second.end());
  }
  // Also add any graph neighbors of cell nodes that share country (connectivity glue).
  for (uint32_t ni : cell.node_indices) {
    if (ni >= graph.out_offsets.size() - 1)
      continue;
    for (uint32_t ei = graph.out_offsets[ni]; ei < graph.out_offsets[ni + 1]; ++ei) {
      const auto& e = graph.edges[graph.out_edges[ei]];
      if (e.v < graph.nodes.size() && graph.nodes[e.v].country == cell.country)
        allow.insert(e.v);
    }
  }
  (void)res;
  return allow;
}

} // namespace

std::vector<uint32_t> ChooseMedoids(const cch::CchGraph& graph,
                                    const std::vector<CellSeed>& cells,
                                    uint32_t sample_cap,
                                    uint32_t unreachable_penalty_s) {
  using clock = std::chrono::steady_clock;
  // Index all nodes by H3 at each cell's resolution; build per-res maps lazily.
  std::unordered_map<int, std::unordered_map<uint64_t, std::vector<uint32_t>>> by_res;

  LOG_INFO("region_grid: medoids start  cells=" + std::to_string(cells.size()) +
           " sample_cap=" + std::to_string(sample_cap));
  const auto t0 = clock::now();
  auto last_log = t0;

  std::vector<uint32_t> medoids(cells.size(), 0);
  for (size_t ci = 0; ci < cells.size(); ++ci) {
    const auto& cell = cells[ci];
    if (cell.node_indices.empty()) {
      medoids[ci] = 0;
    } else if (cell.node_indices.size() == 1) {
      medoids[ci] = cell.node_indices[0];
    } else {
      const int res = H3Resolution(cell.h3_index);
      auto rit = by_res.find(res);
      if (rit == by_res.end()) {
        auto& m = by_res[res];
        for (uint32_t i = 0; i < graph.nodes.size(); ++i) {
          const auto& n = graph.nodes[i];
          const uint64_t h = LatLngToH3(n.lat, n.lon, res);
          if (h != 0)
            m[h].push_back(i);
        }
        rit = by_res.find(res);
      }
      auto allow = build_halo_allow(graph, cell, rit->second);
      auto samples = sample_nodes(cell.node_indices, sample_cap);

      uint64_t best_score = std::numeric_limits<uint64_t>::max();
      uint32_t best_node = cell.node_indices[0];
      uint8_t best_rc = 255;

      for (uint32_t cand : samples) {
        auto dist = dijkstra_restricted(graph, cand, allow);
        uint64_t score = 0;
        for (uint32_t t : samples) {
          auto it = dist.find(t);
          if (it == dist.end())
            score += unreachable_penalty_s;
          else
            score += it->second;
        }
        const uint8_t rc = best_incident_roadclass(graph, cand);
        if (score < best_score || (score == best_score && rc < best_rc) ||
            (score == best_score && rc == best_rc && cand < best_node)) {
          best_score = score;
          best_node = cand;
          best_rc = rc;
        }
      }
      medoids[ci] = best_node;
    }

    MaybeLogProgress("medoids", ci + 1, cells.size(), t0, &last_log, 30.0,
                     "country=" + (cell.country.empty() ? "XX" : cell.country));
  }

  LOG_INFO("region_grid: medoids done  cells=" + std::to_string(cells.size()) +
           "  elapsed=" + FormatDuration(ElapsedSeconds(t0)));
  return medoids;
}

} // namespace region_grid
} // namespace thor
} // namespace valhalla
