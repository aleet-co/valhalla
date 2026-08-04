#include "thor/cch/contracted_search.h"

#include "thor/pathalgorithm.h"

#include <queue>
#include <utility>

namespace valhalla {
namespace thor {
namespace cch {

bool EdgeFeasibleAt(const Profile& p, int64_t entry_sow) {
  return !is_forbidden(p.forbidden, entry_sow);
}

void BanFreeDownwardReach(const CchOrder& order,
                          const CustomizedMetric& metric,
                          uint32_t target,
                          std::unordered_set<uint32_t>& nodes,
                          std::unordered_map<uint32_t, float>* down_dist_out,
                          const std::function<void()>* interrupt) {
  using QItem = std::pair<float, uint32_t>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<>> pq;
  std::unordered_map<uint32_t, float> dist;
  dist[target] = 0.f;
  pq.push({0.f, target});
  size_t n = 0;
  while (!pq.empty()) {
    if (interrupt && (n++ % kInterruptIterationsInterval) == 0)
      (*interrupt)();
    auto [d, u] = pq.top();
    pq.pop();
    auto it = dist.find(u);
    if (it == dist.end() || d > it->second)
      continue;
    nodes.insert(u);
    if (u >= order.bwd_adj.size())
      continue;
    for (auto [v, eid] : order.bwd_adj[u]) {
      float nd = d + metric.profiles[eid].time_s;
      auto vit = dist.find(v);
      if (vit == dist.end() || nd < vit->second) {
        dist[v] = nd;
        pq.push({nd, v});
      }
    }
  }
  if (down_dist_out)
    *down_dist_out = std::move(dist);
}

void ContractedTdEarliest(const CchOrder& order,
                          const CustomizedMetric& metric,
                          uint32_t source,
                          const std::vector<uint32_t>& targets,
                          int64_t depart_sow,
                          const std::unordered_set<uint32_t>* down_allowed,
                          std::unordered_map<uint32_t, float>& arrival,
                          const std::function<void()>* interrupt) {
  const uint32_t nnodes = static_cast<uint32_t>(order.rank.size());

  // Invert bwd_adj: for each (pred,eid) in bwd_adj[v], pred→v is a down edge.
  std::vector<std::vector<std::pair<uint32_t, uint32_t>>> down_adj(nnodes);
  for (uint32_t v = 0; v < order.bwd_adj.size() && v < nnodes; ++v) {
    for (auto [pred, eid] : order.bwd_adj[v]) {
      if (pred < nnodes)
        down_adj[pred].push_back({v, eid});
    }
  }

  using QItem = std::pair<float, uint32_t>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<>> pq;
  std::unordered_map<uint32_t, float> dist;
  std::unordered_set<uint32_t> want(targets.begin(), targets.end());
  dist[source] = 0.f;
  pq.push({0.f, source});
  size_t found = 0;
  size_t n = 0;

  auto relax = [&](uint32_t v, uint32_t eid, float d, int64_t entry) {
    if (eid >= metric.profiles.size())
      return;
    const auto& p = metric.profiles[eid];
    if (!EdgeFeasibleAt(p, entry))
      return;
    float nd = d + static_cast<float>(p.time_s);
    auto vit = dist.find(v);
    if (vit == dist.end() || nd < vit->second) {
      dist[v] = nd;
      pq.push({nd, v});
    }
  };

  while (!pq.empty() && found < want.size()) {
    if (interrupt && (n++ % kInterruptIterationsInterval) == 0)
      (*interrupt)();
    auto [d, u] = pq.top();
    pq.pop();
    auto it = dist.find(u);
    if (it == dist.end() || d > it->second)
      continue;
    if (want.count(u) && !arrival.count(u)) {
      arrival[u] = d;
      ++found;
    }
    const int64_t entry = depart_sow + static_cast<int64_t>(d);

    // Upward edges: always allowed.
    if (u < order.fwd_adj.size()) {
      for (auto [v, eid] : order.fwd_adj[u])
        relax(v, eid, d, entry);
    }

    // Downward edges: unrestricted when down_allowed is null; else head must be marked.
    if (u < down_adj.size()) {
      for (auto [v, eid] : down_adj[u]) {
        if (down_allowed && !down_allowed->count(v))
          continue;
        relax(v, eid, d, entry);
      }
    }
  }
}

} // namespace cch
} // namespace thor
} // namespace valhalla
