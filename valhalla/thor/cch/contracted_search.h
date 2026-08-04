#ifndef VALHALLA_THOR_CCH_CONTRACTED_SEARCH_H_
#define VALHALLA_THOR_CCH_CONTRACTED_SEARCH_H_

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "thor/cch/customizer.h"
#include "thor/cch/order.h"
#include "thor/cch/profile.h"

namespace valhalla {
namespace thor {
namespace cch {

// Per-node RPHAST bucket: targets reachable via a ban-free-safe static down path
// from this node, with the corresponding BanFreeDownwardReach distance.
struct RphastBucketEntry {
  uint32_t target = 0;
  float down_dist = 0.f;
};
using RphastBuckets = std::unordered_map<uint32_t, std::vector<RphastBucketEntry>>;

// Stage 1 contracted-graph TD earliest arrival (single label per node).
//
// Adjacency walk from settled node `u` with travel-time label `d`:
//   entry_sow = depart_sow + (int64_t)d
//   1. Always relax upward edges in order.fwd_adj[u] (rank increases) when
//      EdgeFeasibleAt(profiles[eid], entry_sow).
//   2. Relax downward edges u→v obtained by inverting order.bwd_adj:
//      for each (pred,eid) in bwd_adj[v], pred→v is a down edge. From u, relax
//      each such down successor v when:
//        - down_allowed == nullptr (unrestricted), or
//        - down_allowed->count(v) (head is in the ban-free G↓ mark set).
//      Feasibility uses the same EdgeFeasibleAt check on entry_sow.
// On success, nd = d + profiles[eid].time_s. Missing arrival keys = unsettled.
// If buckets != nullptr, on each settled label at u also apply
//   arrival[t] = min(arrival[t], d + down_dist) for each ban-free-safe bucket
//   entry (t, down_dist) at u (Stage 3 RPHAST settle). nullptr buckets = Stage 1/2.
void ContractedTdEarliest(const CchOrder& order,
                          const CustomizedMetric& metric,
                          uint32_t source,
                          const std::vector<uint32_t>& targets,
                          int64_t depart_sow,
                          const std::unordered_set<uint32_t>* down_allowed,
                          const RphastBuckets* buckets,
                          std::unordered_map<uint32_t, float>& arrival,
                          const std::function<void()>* interrupt);

// Stage 2 contracted-graph TD search with Pareto-on-arrival labels.
//
// Same adjacency / EdgeFeasibleAt / down_allowed / buckets rules as
// ContractedTdEarliest.
// Label rule (sufficient for static travel times + weekly slot bans, no waiting):
//   Feasibility class at node u for arrival d is the bitmask of which outgoing
//   edges (fwd_adj[u] then inverted bwd down-successors, index order) are
//   EdgeFeasibleAt(entry) with entry = depart_sow + d. Out-degree > 64 folds
//   bits with (i % 64). Within a class keep only the minimum arrival; keep
//   distinct classes. Cap: at most 32 labels per node (LOG_WARN if a new class
//   would exceed the cap and is dropped).
// `arrival` out-param = earliest among retained Pareto labels at each settled
// target. If label_counts_out != nullptr, resized to |rank| with the number of
// retained labels per node (0 if never labeled).
void ContractedTdPareto(const CchOrder& order,
                        const CustomizedMetric& metric,
                        uint32_t source,
                        const std::vector<uint32_t>& targets,
                        int64_t depart_sow,
                        const std::unordered_set<uint32_t>* down_allowed,
                        const RphastBuckets* buckets,
                        std::unordered_map<uint32_t, float>& arrival,
                        std::vector<uint32_t>* label_counts_out,
                        const std::function<void()>* interrupt);

// Ban-free downward settle set from target over bwd_adj (Phase A primitive).
// Ignores forbidden masks; distances use static time_s only.
void BanFreeDownwardReach(const CchOrder& order,
                          const CustomizedMetric& metric,
                          uint32_t target,
                          std::unordered_set<uint32_t>& nodes,
                          std::unordered_map<uint32_t, float>* down_dist_out,
                          const std::function<void()>* interrupt);

// Stage 3 Phase A: union BanFreeDownwardReach over targets into down_allowed.
// When buckets != nullptr, also fill per-node ban-free-safe bucket entries
// (empty forbidden along the stored shortest down parent-tree path to the
// target). Edges with any ban mask are omitted from buckets so static
// forward+down addition cannot under-estimate vs TD search.
void BuildRphastPhaseA(const CchOrder& order,
                       const CustomizedMetric& metric,
                       const std::vector<uint32_t>& targets,
                       std::unordered_set<uint32_t>& down_allowed,
                       RphastBuckets* buckets,
                       const std::function<void()>* interrupt);

// !is_forbidden(p.forbidden, entry_sow)
bool EdgeFeasibleAt(const Profile& p, int64_t entry_sow);

} // namespace cch
} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_CCH_CONTRACTED_SEARCH_H_
