#ifndef VALHALLA_THOR_CCH_NESTED_DISSECTION_H_
#define VALHALLA_THOR_CCH_NESTED_DISSECTION_H_

#include <cstdint>
#include <vector>

#include "thor/cch/cch_graph.h"

namespace valhalla {
namespace thor {
namespace cch {

// Inertial-flow nested dissection order for the truck subgraph.
// Returns rank[v] with low rank = contract first (separator nodes last).
// concurrency == 0 → hardware_concurrency().
std::vector<uint32_t> ComputeNestedDissectionOrder(const CchGraph& g,
                                                   uint32_t concurrency = 0);

} // namespace cch
} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_CCH_NESTED_DISSECTION_H_
