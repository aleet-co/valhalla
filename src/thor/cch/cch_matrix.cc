#include "thor/cch/cch_matrix.h"

#include <cstdint>
#include <fstream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "baldr/datetime.h"
#include "baldr/graphid.h"
#include "baldr/time_info.h"
#include "midgard/logging.h"

namespace valhalla {
namespace thor {

CCHMatrix::CCHMatrix(const boost::property_tree::ptree& config)
    : MatrixAlgorithm(config),
      artifact_path_(config.get<std::string>("cch.artifact", "/custom_files/cch_truck.bin")),
      hops_(config.get<uint32_t>("cch.corridor_hops", 16)) {
}

void CCHMatrix::Clear() {
}

bool CCHMatrix::ensure_customized(baldr::GraphReader& reader) {
  if (ready_)
    return true;

  // Negative-cache latch: a missing/mismatched artifact must not cause the
  // (expensive) truck-graph build to rerun on every request. Try once.
  if (customize_attempted_)
    return false;
  customize_attempted_ = true;

  std::ifstream probe(artifact_path_, std::ios::binary);
  if (!probe.good()) {
    LOG_WARN("cch: artifact not found at " + artifact_path_);
    return false;
  }
  probe.close();

  try {
    order_ = cch::CchOrder::load(artifact_path_);
    graph_ = cch::BuildTruckGraph(reader);
    if (graph_.nodes.size() != order_.rank.size() ||
        graph_.tile_build_hash != order_.tile_build_hash) {
      LOG_WARN("cch: artifact does not match current tiles; disabling cch");
      return false;
    }
    order_.build_adjacency(graph_);
    metric_ = cch::Customize(graph_, order_);
    ready_ = true;
    LOG_INFO("cch: customized metric ready (" + std::to_string(graph_.nodes.size()) + " nodes)");
  } catch (const std::exception& e) {
    LOG_WARN(std::string("cch: customization failed: ") + e.what());
    ready_ = false;
  }
  return ready_;
}

namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();

// Static (ban-free) upward Dijkstra from `src` over fwd_adj. Fills up_dist and
// up_parent (edge id used to reach each settled node). Reused across targets.
void UpwardTree(const cch::CchOrder& order,
                const cch::CustomizedMetric& metric,
                uint32_t src,
                std::unordered_map<uint32_t, float>& up_dist,
                std::unordered_map<uint32_t, uint32_t>& up_parent) {
  using QItem = std::pair<float, uint32_t>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<>> pq;
  up_dist[src] = 0.f;
  pq.push({0.f, src});
  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();
    auto it = up_dist.find(u);
    if (it == up_dist.end() || d > it->second)
      continue;
    for (auto [v, eid] : order.fwd_adj[u]) {
      float nd = d + metric.profiles[eid].time_s;
      auto vit = up_dist.find(v);
      if (vit == up_dist.end() || nd < vit->second) {
        up_dist[v] = nd;
        up_parent[v] = eid;
        pq.push({nd, v});
      }
    }
  }
}

// Downward Dijkstra from `tgt` over bwd_adj (edges pointing down into tgt).
void DownwardTree(const cch::CchOrder& order,
                  const cch::CustomizedMetric& metric,
                  uint32_t tgt,
                  std::unordered_map<uint32_t, float>& dn_dist,
                  std::unordered_map<uint32_t, uint32_t>& dn_parent) {
  using QItem = std::pair<float, uint32_t>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<>> pq;
  dn_dist[tgt] = 0.f;
  pq.push({0.f, tgt});
  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();
    auto it = dn_dist.find(u);
    if (it == dn_dist.end() || d > it->second)
      continue;
    for (auto [v, eid] : order.bwd_adj[u]) {
      float nd = d + metric.profiles[eid].time_s;
      auto vit = dn_dist.find(v);
      if (vit == dn_dist.end() || nd < vit->second) {
        dn_dist[v] = nd;
        dn_parent[v] = eid;
        pq.push({nd, v});
      }
    }
  }
}

// Recursively add the base node indices spanned by an edge id into `out`.
void UnpackBaseNodes(const cch::CchOrder& order, const cch::CchGraph& g, uint32_t eid,
                     std::unordered_set<uint32_t>& out) {
  if (eid < order.num_base_edges) {
    out.insert(g.edges[eid].u);
    out.insert(g.edges[eid].v);
    return;
  }
  const auto& sc = order.shortcuts[eid - order.num_base_edges];
  UnpackBaseNodes(order, g, sc.left, out);
  UnpackBaseNodes(order, g, sc.right, out);
}

// Grow a node set by `hops` undirected base-graph hops (BFS over CSR).
void ExpandCorridorHops(const cch::CchGraph& g, std::unordered_set<uint32_t>& corridor,
                        uint32_t hops) {
  if (hops == 0 || corridor.empty())
    return;
  std::vector<uint32_t> frontier(corridor.begin(), corridor.end());
  for (uint32_t h = 0; h < hops; ++h) {
    std::vector<uint32_t> next;
    for (uint32_t u : frontier) {
      for (uint32_t k = g.out_offsets[u]; k < g.out_offsets[u + 1]; ++k) {
        uint32_t v = g.edges[g.out_edges[k]].v;
        if (corridor.insert(v).second)
          next.push_back(v);
      }
      for (uint32_t k = g.in_offsets[u]; k < g.in_offsets[u + 1]; ++k) {
        uint32_t v = g.edges[g.in_edges[k]].u;
        if (corridor.insert(v).second)
          next.push_back(v);
      }
    }
    if (next.empty())
      break;
    frontier.swap(next);
  }
}

// No-waiting TD Dijkstra restricted to `corridor`; fills arrival seconds per
// target (or leaves it absent). depart_sow is UTC second-of-week at the source;
// the ban is evaluated at the departure time of entry onto each edge.
void TdRepair(const cch::CchGraph& g, const cch::CustomizedMetric& metric,
              uint32_t source, const std::vector<uint32_t>& targets, int64_t depart_sow,
              const std::unordered_set<uint32_t>& corridor,
              std::unordered_map<uint32_t, float>& arrival) {
  using QItem = std::pair<float, uint32_t>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<>> pq;
  std::unordered_map<uint32_t, float> dist;
  std::unordered_set<uint32_t> want(targets.begin(), targets.end());
  dist[source] = 0.f;
  pq.push({0.f, source});
  size_t found = 0;
  while (!pq.empty() && found < want.size()) {
    auto [d, u] = pq.top();
    pq.pop();
    auto it = dist.find(u);
    if (it == dist.end() || d > it->second)
      continue;
    if (want.count(u) && !arrival.count(u)) {
      arrival[u] = d;
      ++found;
    }
    int64_t entry = depart_sow + static_cast<int64_t>(d);
    for (uint32_t k = g.out_offsets[u]; k < g.out_offsets[u + 1]; ++k) {
      uint32_t eid = g.out_edges[k];
      uint32_t v = g.edges[eid].v;
      if (!corridor.count(v))
        continue;
      if (cch::is_forbidden(metric.profiles[eid].forbidden, entry))
        continue;
      float nd = d + metric.profiles[eid].time_s;
      auto vit = dist.find(v);
      if (vit == dist.end() || nd < vit->second) {
        dist[v] = nd;
        pq.push({nd, v});
      }
    }
  }
}

} // namespace

bool CCHMatrix::SourceToTarget(Api& request,
                               baldr::GraphReader& graphreader,
                               const sif::mode_costing_t& /*mode_costing*/,
                               const sif::travel_mode_t /*mode*/,
                               const float /*max_matrix_distance*/) {
  // Report as TimeDistanceMatrix in the PBF payload (MVP: no new enum value).
  request.mutable_matrix()->set_algorithm(Matrix::TimeDistanceMatrix);

  if (!ensure_customized(graphreader)) {
    return false; // worker will have already chosen fallback; defensive.
  }

  const auto& options = request.options();
  const auto& srcs = options.sources();
  const auto& tgts = options.targets();
  const size_t num_elements = static_cast<size_t>(srcs.size()) * tgts.size();

  auto& matrix = *request.mutable_matrix();
  // Reserve/zero the PBF arrays (mirror TimeDistanceMatrix sizing so the
  // serializer sees fully-sized date_time/tz arrays too).
  reserve_pbf_arrays(matrix, num_elements, options.verbose());

  // Snap each source/target to the nearest truck-subgraph base node (MVP).
  auto snap = [&](const valhalla::Location& loc) -> int {
    if (loc.correlation().edges_size() == 0)
      return -1;
    baldr::GraphId edge_id(loc.correlation().edges(0).graph_id());
    auto tile = graphreader.GetGraphTile(edge_id);
    if (!tile)
      return -1;
    const auto* de = tile->directededge(edge_id);
    return graph_.index_of(de->endnode().value);
  };

  std::vector<int> src_idx(srcs.size()), tgt_idx(tgts.size());
  for (int s = 0; s < srcs.size(); ++s)
    src_idx[s] = snap(srcs[s]);
  for (int t = 0; t < tgts.size(); ++t)
    tgt_idx[t] = snap(tgts[t]);

  // Depart second-of-week (UTC) per source. Mirror TimeDistanceMatrix::SetTime:
  // TimeInfo::make derives the source-side timezone from the graph and yields a
  // UTC epoch (local_time), which we fold to a Monday-aligned second-of-week.
  baldr::DateTime::tz_sys_info_cache_t tz_cache;
  auto& mutable_srcs = *request.mutable_options()->mutable_sources();
  const std::string& default_dt = options.date_time();
  std::vector<int64_t> depart_sow(srcs.size(), 0);
  for (int s = 0; s < mutable_srcs.size(); ++s) {
    auto* src = mutable_srcs.Mutable(s);
    if (src->date_time().empty() && !default_dt.empty())
      src->set_date_time(default_dt);
    auto ti = baldr::TimeInfo::make(*src, graphreader, &tz_cache);
    int64_t epoch = ti.valid ? static_cast<int64_t>(ti.local_time) : 0;
    depart_sow[s] = cch::utc_second_of_week(epoch);
  }

  for (int s = 0; s < srcs.size(); ++s) {
    if (src_idx[s] < 0) {
      // unreachable/unsnapped source row -> leave defaults (0) and continue (MVP).
      continue;
    }
    // Shared upward tree per source.
    std::unordered_map<uint32_t, float> up_dist;
    std::unordered_map<uint32_t, uint32_t> up_parent;
    UpwardTree(order_, metric_, static_cast<uint32_t>(src_idx[s]), up_dist, up_parent);

    // Build one corridor spanning all reachable targets, then TD-repair once.
    std::unordered_set<uint32_t> corridor;
    corridor.insert(static_cast<uint32_t>(src_idx[s]));
    std::vector<uint32_t> reachable_targets;
    for (int t = 0; t < tgts.size(); ++t) {
      if (tgt_idx[t] < 0)
        continue;
      std::unordered_map<uint32_t, float> dn_dist;
      std::unordered_map<uint32_t, uint32_t> dn_parent;
      DownwardTree(order_, metric_, static_cast<uint32_t>(tgt_idx[t]), dn_dist, dn_parent);
      // Meeting node = argmin over up_dist ∩ dn_dist.
      float best = kInf;
      for (const auto& [node, du] : up_dist) {
        auto dit = dn_dist.find(node);
        if (dit != dn_dist.end() && du + dit->second < best)
          best = du + dit->second;
      }
      if (best == kInf)
        continue; // target not connected in the up/down DAG
      corridor.insert(static_cast<uint32_t>(tgt_idx[t]));
      reachable_targets.push_back(static_cast<uint32_t>(tgt_idx[t]));
      // Unpack shortcuts spanned by the up/down trees into the corridor. This
      // over-includes (safe) rather than tracing only the meeting path; the
      // hop-ball below is what actually admits ban detours.
      for (const auto& kv : up_dist) {
        auto pit = up_parent.find(kv.first);
        if (pit != up_parent.end())
          UnpackBaseNodes(order_, graph_, pit->second, corridor);
      }
      for (const auto& kv : dn_dist) {
        auto pit = dn_parent.find(kv.first);
        if (pit != dn_parent.end())
          UnpackBaseNodes(order_, graph_, pit->second, corridor);
      }
    }

    ExpandCorridorHops(graph_, corridor, hops_);

    std::unordered_map<uint32_t, float> arrival;
    TdRepair(graph_, metric_, static_cast<uint32_t>(src_idx[s]), reachable_targets, depart_sow[s],
             corridor, arrival);

    for (int t = 0; t < tgts.size(); ++t) {
      const size_t pbf = static_cast<size_t>(s) * tgts.size() + t;
      matrix.mutable_from_indices()->Set(pbf, s);
      matrix.mutable_to_indices()->Set(pbf, t);
      float secs = 0.f;
      if (tgt_idx[t] >= 0) {
        auto it = arrival.find(static_cast<uint32_t>(tgt_idx[t]));
        secs = (it == arrival.end()) ? 0.f : it->second;
      }
      matrix.mutable_times()->Set(pbf, secs);
      matrix.mutable_distances()->Set(pbf, 0); // MVP: distance not tracked (time-only)
    }
  }
  return true;
}

} // namespace thor
} // namespace valhalla
