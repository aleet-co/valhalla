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
void ContractedTdEarliest(const CchOrder& order,
                          const CustomizedMetric& metric,
                          uint32_t source,
                          const std::vector<uint32_t>& targets,
                          int64_t depart_sow,
                          const std::unordered_set<uint32_t>* down_allowed,
                          std::unordered_map<uint32_t, float>& arrival,
                          const std::function<void()>* interrupt);

// Ban-free downward settle set from target over bwd_adj (Phase A primitive).
// Ignores forbidden masks; distances use static time_s only.
void BanFreeDownwardReach(const CchOrder& order,
                          const CustomizedMetric& metric,
                          uint32_t target,
                          std::unordered_set<uint32_t>& nodes,
                          std::unordered_map<uint32_t, float>* down_dist_out,
                          const std::function<void()>* interrupt);

// !is_forbidden(p.forbidden, entry_sow)
bool EdgeFeasibleAt(const Profile& p, int64_t entry_sow);

} // namespace cch
} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_CCH_CONTRACTED_SEARCH_H_
