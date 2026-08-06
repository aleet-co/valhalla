#include "thor/region_grid/medoid.h"
#include "thor/region_grid/progress_log.h"

#include "midgard/logging.h"

#include <h3api.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <queue>
#include <string>
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
  return allow;
}

std::string country_iso(const cch::CchNode& n) {
  return n.country.empty() ? "XX" : n.country;
}

} // namespace

std::vector<uint8_t> MarkGiantComponentNodes(const cch::CchGraph& graph) {
  const size_t n = graph.nodes.size();
  std::vector<uint8_t> in_giant(n, 0);
  if (n == 0)
    return in_giant;

  auto same_country = [&](uint32_t a, uint32_t b) {
    return country_iso(graph.nodes[a]) == country_iso(graph.nodes[b]);
  };

  // Kosaraju pass 1: finish order via iterative DFS on out-edges.
  // color: 0=white, 1=gray, 2=black
  std::vector<uint8_t> color(n, 0);
  std::vector<uint32_t> order;
  order.reserve(n);
  std::vector<std::pair<uint32_t, uint32_t>> stack; // (node, next_out_edge_cursor)
  stack.reserve(64);

  for (uint32_t start = 0; start < n; ++start) {
    if (color[start] != 0)
      continue;
    stack.clear();
    stack.emplace_back(start, graph.out_offsets.size() > start + 1 ? graph.out_offsets[start] : 0);
    color[start] = 1;
    while (!stack.empty()) {
      const uint32_t u = stack.back().first;
      uint32_t ei = stack.back().second;
      const uint32_t ei_end = (u + 1 < graph.out_offsets.size()) ? graph.out_offsets[u + 1] : ei;
      bool pushed = false;
      while (ei < ei_end) {
        const auto& e = graph.edges[graph.out_edges[ei++]];
        if (e.v >= n || color[e.v] != 0 || !same_country(u, e.v))
          continue;
        stack.back().second = ei;
        color[e.v] = 1;
        stack.emplace_back(e.v, e.v + 1 < graph.out_offsets.size() ? graph.out_offsets[e.v] : 0);
        pushed = true;
        break;
      }
      if (pushed)
        continue;
      color[u] = 2;
      order.push_back(u);
      stack.pop_back();
    }
  }

  // Kosaraju pass 2: assign components on transposed graph (in-edges).
  std::vector<uint32_t> comp(n, std::numeric_limits<uint32_t>::max());
  uint32_t ncomp = 0;
  std::vector<uint32_t> q;
  q.reserve(64);
  for (auto it = order.rbegin(); it != order.rend(); ++it) {
    const uint32_t start = *it;
    if (comp[start] != std::numeric_limits<uint32_t>::max())
      continue;
    const uint32_t cid = ncomp++;
    q.clear();
    q.push_back(start);
    comp[start] = cid;
    for (size_t qi = 0; qi < q.size(); ++qi) {
      const uint32_t u = q[qi];
      if (u + 1 >= graph.in_offsets.size())
        continue;
      for (uint32_t ei = graph.in_offsets[u]; ei < graph.in_offsets[u + 1]; ++ei) {
        const auto& e = graph.edges[graph.in_edges[ei]];
        if (e.u >= n || comp[e.u] != std::numeric_limits<uint32_t>::max())
          continue;
        if (!same_country(u, e.u))
          continue;
        comp[e.u] = cid;
        q.push_back(e.u);
      }
    }
  }

  // Per country: largest SCC (tie → lower component id).
  std::unordered_map<std::string, std::unordered_map<uint32_t, uint32_t>> sizes;
  std::unordered_map<std::string, uint32_t> country_nodes;
  for (uint32_t i = 0; i < n; ++i) {
    const std::string iso = country_iso(graph.nodes[i]);
    ++country_nodes[iso];
    ++sizes[iso][comp[i]];
  }

  std::unordered_map<std::string, std::pair<uint32_t, uint32_t>> best; // cid, size
  for (const auto& ck : sizes) {
    for (const auto& cs : ck.second) {
      auto it = best.find(ck.first);
      if (it == best.end() || cs.second > it->second.second ||
          (cs.second == it->second.second && cs.first < it->second.first)) {
        best[ck.first] = {cs.first, cs.second};
      }
    }
  }

  uint32_t marked = 0;
  for (uint32_t i = 0; i < n; ++i) {
    const std::string iso = country_iso(graph.nodes[i]);
    if (comp[i] == best[iso].first) {
      in_giant[i] = 1;
      ++marked;
    }
  }

  for (const auto& kv : best) {
    const uint32_t total = country_nodes[kv.first];
    LOG_INFO("region_grid: giant SCC country=" + kv.first + " size=" +
             std::to_string(kv.second.second) + "/" + std::to_string(total) + " (" +
             std::to_string(total == 0 ? 0 : (100 * kv.second.second / total)) + "%)");
  }
  LOG_INFO("region_grid: giant SCC marked " + std::to_string(marked) + "/" + std::to_string(n) +
           " nodes");
  return in_giant;
}

std::vector<uint32_t> ChooseMedoids(const cch::CchGraph& graph,
                                    const std::vector<CellSeed>& cells,
                                    uint32_t sample_cap,
                                    uint32_t unreachable_penalty_s) {
  using clock = std::chrono::steady_clock;
  // Index all nodes by H3 at each cell's resolution; build per-res maps lazily.
  std::unordered_map<int, std::unordered_map<uint64_t, std::vector<uint32_t>>> by_res;

  const auto in_giant = MarkGiantComponentNodes(graph);

  LOG_INFO("region_grid: medoids start  cells=" + std::to_string(cells.size()) +
           " sample_cap=" + std::to_string(sample_cap));
  const auto t0 = clock::now();
  auto last_log = t0;

  uint32_t dropped_no_giant = 0;
  std::vector<uint32_t> medoids(cells.size(), 0);
  for (size_t ci = 0; ci < cells.size(); ++ci) {
    const auto& cell = cells[ci];
    std::vector<uint32_t> giant_nodes;
    giant_nodes.reserve(cell.node_indices.size());
    for (uint32_t ni : cell.node_indices) {
      if (ni < in_giant.size() && in_giant[ni])
        giant_nodes.push_back(ni);
    }

    if (giant_nodes.empty()) {
      medoids[ci] = std::numeric_limits<uint32_t>::max();
      ++dropped_no_giant;
    } else if (giant_nodes.size() == 1) {
      medoids[ci] = giant_nodes[0];
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
      auto samples = sample_nodes(giant_nodes, sample_cap);

      uint64_t best_score = std::numeric_limits<uint64_t>::max();
      uint32_t best_node = samples[0];
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
           " dropped_no_giant=" + std::to_string(dropped_no_giant) +
           "  elapsed=" + FormatDuration(ElapsedSeconds(t0)));
  return medoids;
}

} // namespace region_grid
} // namespace thor
} // namespace valhalla
