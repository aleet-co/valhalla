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
#include "thor/cch/contracted_search.h"
#include "thor/matrixalgorithm.h"
#include "thor/pathalgorithm.h"

namespace valhalla {
namespace thor {

CCHMatrix::CCHMatrix(const boost::property_tree::ptree& config)
    : MatrixAlgorithm(config),
      artifact_path_(config.get<std::string>("cch.artifact", "/custom_files/cch_truck.bin")),
      hops_(config.get<uint32_t>("cch.corridor_hops", 16)),
      enabled_(config.get<bool>("cch.enabled", false)) {
  const std::string mode = config.get<std::string>("cch.query_mode", "corridor");
  if (mode == "contracted") {
    query_mode_ = cch::QueryMode::Contracted;
  } else {
    query_mode_ = cch::QueryMode::Corridor;
    if (mode != "corridor") {
      LOG_WARN("cch: unknown query_mode '" + mode + "'; defaulting to corridor");
    }
  }
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

  // WIP kill-switch: keep CCH code paths but always fall back to TDM until the
  // offline artifact build is production-ready.
  if (!enabled_) {
    LOG_INFO("cch: disabled (thor.cch.enabled=false); matrix requests fall back to TDM");
    return false;
  }

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
                std::unordered_map<uint32_t, uint32_t>& up_parent,
                const std::function<void()>* interrupt) {
  using QItem = std::pair<float, uint32_t>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<>> pq;
  up_dist[src] = 0.f;
  pq.push({0.f, src});
  size_t n = 0;
  while (!pq.empty()) {
    // Allow this process to be aborted (mirror TimeDistanceMatrix::ComputeMatrix).
    if (interrupt && (n++ % kInterruptIterationsInterval) == 0)
      (*interrupt)();
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
                  std::unordered_map<uint32_t, uint32_t>& dn_parent,
                  const std::function<void()>* interrupt) {
  using QItem = std::pair<float, uint32_t>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<>> pq;
  dn_dist[tgt] = 0.f;
  pq.push({0.f, tgt});
  size_t n = 0;
  while (!pq.empty()) {
    // Allow this process to be aborted (mirror TimeDistanceMatrix::ComputeMatrix).
    if (interrupt && (n++ % kInterruptIterationsInterval) == 0)
      (*interrupt)();
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
              std::unordered_map<uint32_t, float>& arrival,
              const std::function<void()>* interrupt) {
  using QItem = std::pair<float, uint32_t>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<>> pq;
  std::unordered_map<uint32_t, float> dist;
  std::unordered_set<uint32_t> want(targets.begin(), targets.end());
  dist[source] = 0.f;
  pq.push({0.f, source});
  size_t found = 0;
  size_t n = 0;
  while (!pq.empty() && found < want.size()) {
    // Allow this process to be aborted (mirror TimeDistanceMatrix::ComputeMatrix).
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

  // Depart second-of-week per source. The tz is derived from the snapped source
  // node exactly as TimeDistanceMatrix::SetTime does (TimeInfo::make, which also
  // normalizes a "current" date_time). The epoch MUST be a PLAIN Unix timestamp
  // to match the customizer's mask clock: DeriveMask indexes bans on
  // utc_second_of_week(kDefaultReferenceWeek + slot*900), a plain-Unix grid.
  // TimeInfo::local_time (and DateTime::seconds_since_epoch) are leap-inclusive
  // (date::to_utc_time), ~27s off, which can flip a ban near a slot boundary --
  // so we take sys_time (get_ldt(...).get_sys_time()) instead of local_time.
  baldr::DateTime::tz_sys_info_cache_t tz_cache;
  auto& mutable_srcs = *request.mutable_options()->mutable_sources();
  const std::string& default_dt = options.date_time();
  std::vector<int64_t> depart_sow(srcs.size(), 0);
  for (int s = 0; s < mutable_srcs.size(); ++s) {
    auto* src = mutable_srcs.Mutable(s);
    if (src->date_time().empty() && !default_dt.empty())
      src->set_date_time(default_dt);
    auto ti = baldr::TimeInfo::make(*src, graphreader, &tz_cache);
    int64_t epoch = 0;
    if (ti.valid) {
      const auto* tz = baldr::DateTime::get_tz_db().from_index(ti.timezone_index);
      if (tz) {
        epoch = baldr::DateTime::get_ldt(baldr::DateTime::get_formatted_date(src->date_time()), tz)
                    .get_sys_time()
                    .time_since_epoch()
                    .count();
      }
    }
    depart_sow[s] = cch::utc_second_of_week(epoch);
  }

  // Stage 1 contracted query: ban-aware search on the CCH overlay (consumes
  // shortcut B). Skip corridor unpack / hop-ball / TdRepair.
  if (query_mode_ == cch::QueryMode::Contracted) {
    // Phase A: union ban-free downward reach over all snapped targets.
    std::unordered_set<uint32_t> g_down_union;
    std::vector<uint32_t> snapped_targets;
    snapped_targets.reserve(static_cast<size_t>(tgts.size()));
    for (int t = 0; t < tgts.size(); ++t) {
      if (tgt_idx[t] < 0)
        continue;
      const uint32_t tn = static_cast<uint32_t>(tgt_idx[t]);
      snapped_targets.push_back(tn);
      cch::BanFreeDownwardReach(order_, metric_, tn, g_down_union, nullptr, interrupt_);
    }

    // Phase B: per-source contracted TD earliest arrival into the union G↓.
    for (int s = 0; s < srcs.size(); ++s) {
      if (src_idx[s] < 0) {
        for (int t = 0; t < tgts.size(); ++t) {
          const size_t pbf = static_cast<size_t>(s) * tgts.size() + t;
          matrix.mutable_from_indices()->Set(pbf, s);
          matrix.mutable_to_indices()->Set(pbf, t);
          matrix.mutable_times()->Set(pbf, kMaxCost);
          matrix.mutable_distances()->Set(pbf, 0);
        }
        continue;
      }
      std::unordered_map<uint32_t, float> arrival;
      cch::ContractedTdEarliest(order_, metric_, static_cast<uint32_t>(src_idx[s]), snapped_targets,
                                depart_sow[s], &g_down_union, arrival, interrupt_);
      for (int t = 0; t < tgts.size(); ++t) {
        const size_t pbf = static_cast<size_t>(s) * tgts.size() + t;
        matrix.mutable_from_indices()->Set(pbf, s);
        matrix.mutable_to_indices()->Set(pbf, t);
        float secs = kMaxCost;
        if (tgt_idx[t] >= 0) {
          auto it = arrival.find(static_cast<uint32_t>(tgt_idx[t]));
          if (it != arrival.end())
            secs = it->second;
        }
        matrix.mutable_times()->Set(pbf, secs);
        matrix.mutable_distances()->Set(pbf, 0);
      }
    }
    return true;
  }

  // Downward trees are SOURCE-INDEPENDENT: a target's down-tree (and the base
  // nodes its shortcuts unpack to) depend only on the target and the static
  // metric, never on the source. Compute each ONCE here instead of rebuilding
  // it for every source inside the loop below -- the previous code ran a full
  // DownwardTree (plus its unpack) for every (source, target) pair, i.e. S*T
  // builds for an S*T matrix. Hoisting to T builds is a pure amortization: the
  // per-pair meeting-min in the source loop is unchanged, so the output matrix
  // is byte-for-byte identical. dn_nodes_all[t] holds the target's unpacked
  // down-side base nodes (also source-invariant), merged into each source's
  // corridor only when that target is actually reachable.
  std::vector<std::unordered_map<uint32_t, float>> dn_dist_all(tgts.size());
  std::vector<std::unordered_set<uint32_t>> dn_nodes_all(tgts.size());
  for (int t = 0; t < tgts.size(); ++t) {
    if (tgt_idx[t] < 0)
      continue;
    std::unordered_map<uint32_t, uint32_t> dn_parent;
    DownwardTree(order_, metric_, static_cast<uint32_t>(tgt_idx[t]), dn_dist_all[t], dn_parent,
                 interrupt_);
    auto& nodes = dn_nodes_all[t];
    for (const auto& kv : dn_dist_all[t]) {
      auto pit = dn_parent.find(kv.first);
      if (pit != dn_parent.end())
        UnpackBaseNodes(order_, graph_, pit->second, nodes);
    }
  }

  for (int s = 0; s < srcs.size(); ++s) {
    if (src_idx[s] < 0) {
      // Unsnapped source: no target is reachable from this row. Still write the
      // index metadata for EVERY cell (from_indices=s, to_indices=t) -- only the
      // time value reflects reachability -- and mark each time with the kMaxCost
      // unreachable sentinel (mirroring TimeDistanceMatrix's default best_cost)
      // so the shared serializer emits null rather than a spurious 0.
      for (int t = 0; t < tgts.size(); ++t) {
        const size_t pbf = static_cast<size_t>(s) * tgts.size() + t;
        matrix.mutable_from_indices()->Set(pbf, s);
        matrix.mutable_to_indices()->Set(pbf, t);
        matrix.mutable_times()->Set(pbf, kMaxCost);
        matrix.mutable_distances()->Set(pbf, 0); // MVP: distance not tracked (time-only)
      }
      continue;
    }
    // Shared upward tree per source.
    //
    // The CH phase (this upward tree and each target's downward tree) is
    // intentionally BAN-FREE: it searches purely on static `time_s` to select a
    // corridor of candidate base nodes. No shortcut `B` (forbidden) mask is ever
    // consulted here (see design C). The base-graph `TdRepair` pass below is the
    // sole ban authority -- it re-evaluates time-dependent bans on BASE edges at
    // their actual entry time. Do not add shortcut-ban checks to the CH search.
    std::unordered_map<uint32_t, float> up_dist;
    std::unordered_map<uint32_t, uint32_t> up_parent;
    UpwardTree(order_, metric_, static_cast<uint32_t>(src_idx[s]), up_dist, up_parent, interrupt_);

    // Build one corridor spanning all reachable targets, then TD-repair once.
    std::unordered_set<uint32_t> corridor;
    corridor.insert(static_cast<uint32_t>(src_idx[s]));
    // Up-side unpack is source-invariant: unpack it once per source (hoisted
    // out of the target loop; set-inserts are idempotent so this is a pure
    // speedup with no behavior change).
    for (const auto& kv : up_dist) {
      auto pit = up_parent.find(kv.first);
      if (pit != up_parent.end())
        UnpackBaseNodes(order_, graph_, pit->second, corridor);
    }
    std::vector<uint32_t> reachable_targets;
    for (int t = 0; t < tgts.size(); ++t) {
      if (tgt_idx[t] < 0)
        continue;
      const auto& dn_dist = dn_dist_all[t];
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
      // Merge this target's precomputed down-side base nodes into the corridor.
      // This over-includes (safe) rather than tracing only the meeting path; the
      // hop-ball below is what actually admits ban detours.
      corridor.insert(dn_nodes_all[t].begin(), dn_nodes_all[t].end());
    }

    ExpandCorridorHops(graph_, corridor, hops_);

    std::unordered_map<uint32_t, float> arrival;
    TdRepair(graph_, metric_, static_cast<uint32_t>(src_idx[s]), reachable_targets, depart_sow[s],
             corridor, arrival, interrupt_);

    for (int t = 0; t < tgts.size(); ++t) {
      const size_t pbf = static_cast<size_t>(s) * tgts.size() + t;
      matrix.mutable_from_indices()->Set(pbf, s);
      matrix.mutable_to_indices()->Set(pbf, t);
      // Default to the kMaxCost unreachable sentinel (mirrors TimeDistanceMatrix's
      // default Destination::best_cost, which the shared serializer maps to null).
      // Only a target that TdRepair genuinely SETTLED keeps its real arrival --
      // which may legitimately be 0 for a self/zero-distance pair. Snapped-but-
      // unsettled (true path left the corridor) and unsnapped (tgt_idx < 0)
      // targets stay at the sentinel so they are not reported as time=0.
      float secs = kMaxCost;
      if (tgt_idx[t] >= 0) {
        auto it = arrival.find(static_cast<uint32_t>(tgt_idx[t]));
        if (it != arrival.end())
          secs = it->second;
      }
      matrix.mutable_times()->Set(pbf, secs);
      matrix.mutable_distances()->Set(pbf, 0); // MVP: distance not tracked (time-only)
    }
  }
  return true;
}

} // namespace thor
} // namespace valhalla
