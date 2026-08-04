#include "thor/cch/contracted_search.h"

#include "midgard/logging.h"
#include "thor/pathalgorithm.h"

#include <queue>
#include <string>
#include <tuple>
#include <utility>

namespace valhalla {
namespace thor {
namespace cch {
namespace {

constexpr uint32_t kMaxParetoLabelsPerNode = 32;

// Feasibility class = bitmask of EdgeFeasibleAt over outgoing edges in Stage-1
// adjacency order (fwd_adj then inverted-bwd down successors). Bits beyond 64
// fold with (i % 64). down_allowed does not affect the class — only bans do.
uint64_t FeasibilityClass(uint32_t u,
                          int64_t entry,
                          const CchOrder& order,
                          const CustomizedMetric& metric,
                          const std::vector<std::vector<std::pair<uint32_t, uint32_t>>>& down_adj) {
  uint64_t mask = 0;
  size_t i = 0;
  auto consider = [&](uint32_t eid) {
    if (eid < metric.profiles.size() && EdgeFeasibleAt(metric.profiles[eid], entry))
      mask |= (uint64_t{1} << (i % 64));
    ++i;
  };
  if (u < order.fwd_adj.size()) {
    for (auto [v, eid] : order.fwd_adj[u]) {
      (void)v;
      consider(eid);
    }
  }
  if (u < down_adj.size()) {
    for (auto [v, eid] : down_adj[u]) {
      (void)v;
      consider(eid);
    }
  }
  return mask;
}

// parent[u] = {next_toward_target, eid}; empty forbidden along u→…→target.
bool DownPathBanFree(uint32_t u,
                     uint32_t target,
                     const std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>>& parent,
                     const CustomizedMetric& metric) {
  if (u == target)
    return true;
  uint32_t cur = u;
  while (cur != target) {
    auto it = parent.find(cur);
    if (it == parent.end())
      return false;
    const uint32_t eid = it->second.second;
    if (eid >= metric.profiles.size() || !empty(metric.profiles[eid].forbidden))
      return false;
    cur = it->second.first;
  }
  return true;
}

// Invert bwd_adj: for each (pred,eid) in bwd_adj[v], pred→v is a down edge.
std::vector<std::vector<std::pair<uint32_t, uint32_t>>> BuildDownAdj(const CchOrder& order) {
  const uint32_t nnodes = static_cast<uint32_t>(order.rank.size());
  std::vector<std::vector<std::pair<uint32_t, uint32_t>>> down_adj(nnodes);
  for (uint32_t v = 0; v < order.bwd_adj.size() && v < nnodes; ++v) {
    for (auto [pred, eid] : order.bwd_adj[v]) {
      if (pred < nnodes)
        down_adj[pred].push_back({v, eid});
    }
  }
  return down_adj;
}

} // namespace

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

void BuildRphastPhaseA(const CchOrder& order,
                       const CustomizedMetric& metric,
                       const std::vector<uint32_t>& targets,
                       std::unordered_set<uint32_t>& down_allowed,
                       RphastBuckets* buckets,
                       const std::function<void()>* interrupt) {
  using QItem = std::pair<float, uint32_t>;
  for (uint32_t target : targets) {
    std::priority_queue<QItem, std::vector<QItem>, std::greater<>> pq;
    std::unordered_map<uint32_t, float> dist;
    // parent[ancestor] = {next_toward_target, eid}
    std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>> parent;
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
      down_allowed.insert(u);
      if (u >= order.bwd_adj.size())
        continue;
      for (auto [v, eid] : order.bwd_adj[u]) {
        if (eid >= metric.profiles.size())
          continue;
        float nd = d + metric.profiles[eid].time_s;
        auto vit = dist.find(v);
        if (vit == dist.end() || nd < vit->second) {
          dist[v] = nd;
          parent[v] = {u, eid};
          pq.push({nd, v});
        }
      }
    }
    if (!buckets)
      continue;
    for (const auto& [u, dd] : dist) {
      if (!DownPathBanFree(u, target, parent, metric))
        continue;
      (*buckets)[u].push_back(RphastBucketEntry{target, dd});
    }
  }
}

void ContractedTdEarliest(const CchOrder& order,
                          const CustomizedMetric& metric,
                          uint32_t source,
                          const std::vector<uint32_t>& targets,
                          int64_t depart_sow,
                          const std::unordered_set<uint32_t>* down_allowed,
                          const RphastBuckets* buckets,
                          std::unordered_map<uint32_t, float>& arrival,
                          const std::function<void()>* interrupt) {
  const uint32_t nnodes = static_cast<uint32_t>(order.rank.size());
  auto down_adj = BuildDownAdj(order);

  using QItem = std::pair<float, uint32_t>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<>> pq;
  std::unordered_map<uint32_t, float> dist;
  std::unordered_set<uint32_t> want(targets.begin(), targets.end());
  dist[source] = 0.f;
  pq.push({0.f, source});
  size_t found = 0;
  size_t n = 0;
  std::unordered_set<uint32_t> settled_targets;

  // Bucket fills may write early arrival[t] but must not drive `found` /
  // early-exit: a later meeting node can improve the same target.
  auto apply_buckets = [&](uint32_t u, float d) {
    if (!buckets)
      return;
    auto bit = buckets->find(u);
    if (bit == buckets->end())
      return;
    for (const auto& e : bit->second) {
      if (!want.count(e.target))
        continue;
      const float cand = d + e.down_dist;
      auto ait = arrival.find(e.target);
      if (ait == arrival.end())
        arrival[e.target] = cand;
      else if (cand < ait->second)
        ait->second = cand;
    }
  };

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
    if (want.count(u) && settled_targets.insert(u).second) {
      ++found;
      auto ait = arrival.find(u);
      if (ait == arrival.end())
        arrival[u] = d;
      else if (d < ait->second)
        ait->second = d;
    }
    apply_buckets(u, d);
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

void ContractedTdPareto(const CchOrder& order,
                        const CustomizedMetric& metric,
                        uint32_t source,
                        const std::vector<uint32_t>& targets,
                        int64_t depart_sow,
                        const std::unordered_set<uint32_t>* down_allowed,
                        const RphastBuckets* buckets,
                        std::unordered_map<uint32_t, float>& arrival,
                        std::vector<uint32_t>* label_counts_out,
                        const std::function<void()>* interrupt) {
  const uint32_t nnodes = static_cast<uint32_t>(order.rank.size());
  auto down_adj = BuildDownAdj(order);

  // Per-node: feas_class → best arrival for that class.
  std::vector<std::unordered_map<uint64_t, float>> labels(nnodes);
  using QItem = std::tuple<float, uint32_t, uint64_t>; // arrival, node, class
  std::priority_queue<QItem, std::vector<QItem>, std::greater<>> pq;
  std::unordered_set<uint32_t> want(targets.begin(), targets.end());

  const int64_t src_entry = depart_sow;
  const uint64_t src_cls = FeasibilityClass(source, src_entry, order, metric, down_adj);
  labels[source][src_cls] = 0.f;
  pq.push({0.f, source, src_cls});

  size_t found = 0;
  size_t n = 0;
  std::unordered_set<uint32_t> settled_targets;

  // Bucket fills may write early arrival[t] but must not drive `found` /
  // early-exit: a later meeting node can improve the same target.
  auto apply_buckets = [&](uint32_t u, float d) {
    if (!buckets)
      return;
    auto bit = buckets->find(u);
    if (bit == buckets->end())
      return;
    for (const auto& e : bit->second) {
      if (!want.count(e.target))
        continue;
      const float cand = d + e.down_dist;
      auto ait = arrival.find(e.target);
      if (ait == arrival.end())
        arrival[e.target] = cand;
      else if (cand < ait->second)
        ait->second = cand;
    }
  };

  auto try_insert = [&](uint32_t v, float nd) {
    if (v >= nnodes)
      return;
    const int64_t entry = depart_sow + static_cast<int64_t>(nd);
    const uint64_t cls = FeasibilityClass(v, entry, order, metric, down_adj);
    auto& m = labels[v];
    auto it = m.find(cls);
    if (it != m.end()) {
      if (nd >= it->second)
        return;
      it->second = nd;
      pq.push({nd, v, cls});
      return;
    }
    if (m.size() >= kMaxParetoLabelsPerNode) {
      LOG_WARN("cch: Pareto label cap (" + std::to_string(kMaxParetoLabelsPerNode) +
               ") hit at node " + std::to_string(v) + "; dropping new feasibility class");
      return;
    }
    m[cls] = nd;
    pq.push({nd, v, cls});
  };

  auto relax = [&](uint32_t v, uint32_t eid, float d, int64_t entry) {
    if (eid >= metric.profiles.size())
      return;
    const auto& p = metric.profiles[eid];
    if (!EdgeFeasibleAt(p, entry))
      return;
    try_insert(v, d + static_cast<float>(p.time_s));
  };

  while (!pq.empty() && found < want.size()) {
    if (interrupt && (n++ % kInterruptIterationsInterval) == 0)
      (*interrupt)();
    auto [d, u, cls] = pq.top();
    pq.pop();
    auto lit = labels[u].find(cls);
    if (lit == labels[u].end() || d > lit->second)
      continue;
    if (want.count(u) && settled_targets.insert(u).second) {
      ++found;
      auto ait = arrival.find(u);
      if (ait == arrival.end())
        arrival[u] = d;
      else if (d < ait->second)
        ait->second = d;
    }
    apply_buckets(u, d);
    const int64_t entry = depart_sow + static_cast<int64_t>(d);

    if (u < order.fwd_adj.size()) {
      for (auto [v, eid] : order.fwd_adj[u])
        relax(v, eid, d, entry);
    }
    if (u < down_adj.size()) {
      for (auto [v, eid] : down_adj[u]) {
        if (down_allowed && !down_allowed->count(v))
          continue;
        relax(v, eid, d, entry);
      }
    }
  }

  if (label_counts_out) {
    label_counts_out->assign(nnodes, 0);
    for (uint32_t u = 0; u < nnodes; ++u)
      (*label_counts_out)[u] = static_cast<uint32_t>(labels[u].size());
  }
}

} // namespace cch
} // namespace thor
} // namespace valhalla
