#include "thor/region_grid/network_voronoi.h"
#include "thor/region_grid/progress_log.h"

#include "midgard/logging.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

namespace valhalla {
namespace thor {
namespace region_grid {

namespace {

struct State {
  uint32_t node;
  uint32_t dist;
  uint32_t region_id;
  bool operator>(const State& o) const {
    if (dist != o.dist)
      return dist > o.dist;
    return region_id > o.region_id;
  }
};

double haversine_m(double lat1, double lon1, double lat2, double lon2) {
  constexpr double kR = 6371000.0;
  constexpr double kDeg2Rad = 0.017453292519943295;
  const double p1 = lat1 * kDeg2Rad;
  const double p2 = lat2 * kDeg2Rad;
  const double dphi = (lat2 - lat1) * kDeg2Rad;
  const double dlmb = (lon2 - lon1) * kDeg2Rad;
  const double a =
      std::sin(dphi / 2) * std::sin(dphi / 2) +
      std::cos(p1) * std::cos(p2) * std::sin(dlmb / 2) * std::sin(dlmb / 2);
  return 2 * kR * std::asin(std::min(1.0, std::sqrt(a)));
}

} // namespace

VoronoiResult ComputeNetworkVoronoi(const cch::CchGraph& graph,
                                    const std::vector<uint32_t>& medoid_node_indices,
                                    const std::vector<uint32_t>& medoid_region_ids) {
  using clock = std::chrono::steady_clock;
  VoronoiResult result;
  const size_t n = graph.nodes.size();
  result.node_regions.assign(n, NodeAssignment{});
  std::vector<uint32_t> best_dist(n, std::numeric_limits<uint32_t>::max());
  std::vector<uint32_t> best_region(n, std::numeric_limits<uint32_t>::max());

  LOG_INFO("region_grid: voronoi start  nodes=" + std::to_string(n) +
           " seeds=" + std::to_string(medoid_node_indices.size()));
  const auto t0 = clock::now();
  auto last_log = t0;

  std::priority_queue<State, std::vector<State>, std::greater<State>> pq;
  size_t settled = 0;
  for (size_t i = 0; i < medoid_node_indices.size(); ++i) {
    const uint32_t node = medoid_node_indices[i];
    const uint32_t rid = medoid_region_ids[i];
    if (node >= n)
      continue;
    if (best_dist[node] == std::numeric_limits<uint32_t>::max())
      ++settled;
    best_dist[node] = 0;
    best_region[node] = rid;
    pq.push({node, 0, rid});
  }

  size_t pops = 0;
  while (!pq.empty()) {
    const State cur = pq.top();
    pq.pop();
    ++pops;
    if (cur.dist > best_dist[cur.node] || cur.region_id != best_region[cur.node])
      continue;
    if (cur.node >= graph.out_offsets.size() - 1)
      continue;
    const std::string& cur_country = graph.nodes[cur.node].country;
    for (uint32_t ei = graph.out_offsets[cur.node]; ei < graph.out_offsets[cur.node + 1]; ++ei) {
      const auto& e = graph.edges[graph.out_edges[ei]];
      if (e.v >= n)
        continue;
      if (graph.nodes[e.v].country != cur_country)
        continue;
      const uint32_t nd = cur.dist + e.time_s;
      if (nd < best_dist[e.v] ||
          (nd == best_dist[e.v] && cur.region_id < best_region[e.v])) {
        if (best_dist[e.v] == std::numeric_limits<uint32_t>::max())
          ++settled;
        best_dist[e.v] = nd;
        best_region[e.v] = cur.region_id;
        pq.push({e.v, nd, cur.region_id});
      }
    }

    if ((pops & 1023u) == 0) {
      // Progress against nodes with a finite network distance so far.
      // Disconnected / other-country leftovers are handled in the fallback pass.
      MaybeLogProgress("voronoi expand", settled, n, t0, &last_log, 30.0,
                       "queue=" + std::to_string(pq.size()) + " pops=" + std::to_string(pops));
    }
  }

  LOG_INFO("region_grid: voronoi expand done  settled=" + std::to_string(settled) + "/" +
           std::to_string(n) + "  elapsed=" + FormatDuration(ElapsedSeconds(t0)));

  // Index medoids by country for Euclidean fallback.
  std::unordered_map<std::string, std::vector<size_t>> medoids_by_country;
  for (size_t i = 0; i < medoid_node_indices.size(); ++i) {
    const uint32_t node = medoid_node_indices[i];
    if (node >= n)
      continue;
    std::string iso = graph.nodes[node].country.empty() ? "XX" : graph.nodes[node].country;
    medoids_by_country[iso].push_back(i);
  }

  uint32_t fallback = 0;
  const auto t_fb = clock::now();
  last_log = t_fb;
  for (uint32_t i = 0; i < n; ++i) {
    if (best_region[i] != std::numeric_limits<uint32_t>::max()) {
      result.node_regions[i].region_id = best_region[i];
      result.node_regions[i].time_to_rep_s = best_dist[i];
    } else {
      ++fallback;
      const auto& node = graph.nodes[i];
      std::string iso = node.country.empty() ? "XX" : node.country;
      auto it = medoids_by_country.find(iso);
      uint32_t best_rid = 0;
      double best_d = std::numeric_limits<double>::infinity();
      if (it != medoids_by_country.end()) {
        for (size_t mi : it->second) {
          const uint32_t mn = medoid_node_indices[mi];
          const auto& mnode = graph.nodes[mn];
          const double d = haversine_m(node.lat, node.lon, mnode.lat, mnode.lon);
          if (d < best_d) {
            best_d = d;
            best_rid = medoid_region_ids[mi];
          }
        }
      } else if (!medoid_region_ids.empty()) {
        best_rid = medoid_region_ids[0];
      }
      result.node_regions[i].region_id = best_rid;
      result.node_regions[i].time_to_rep_s =
          best_d == std::numeric_limits<double>::infinity()
              ? 0
              : static_cast<uint32_t>(std::llround(best_d / 20.0)); // ~20 m/s proxy
    }

    if (((i + 1) & 4095u) == 0 || i + 1 == n) {
      MaybeLogProgress("voronoi assign", static_cast<size_t>(i) + 1, n, t_fb, &last_log, 30.0,
                       "fallback_so_far=" + std::to_string(fallback));
    }
  }

  result.euclidean_fallback_count = fallback;
  LOG_INFO("region_grid: voronoi done  fallback_nodes=" + std::to_string(fallback) +
           "  elapsed=" + FormatDuration(ElapsedSeconds(t0)));
  return result;
}

} // namespace region_grid
} // namespace thor
} // namespace valhalla
