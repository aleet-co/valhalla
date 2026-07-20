#ifndef VALHALLA_THOR_CCH_NESTED_DISSECTION_H_
#define VALHALLA_THOR_CCH_NESTED_DISSECTION_H_

#include <cstdint>
#include <vector>

#include "thor/cch/cch_graph.h"

namespace valhalla {
namespace thor {
namespace cch {

// Nested-dissection elimination order via METIS_NodeND (fill-reducing ND).
// Returns rank[v] with low rank = contract first (separators last).
// concurrency is accepted for API compatibility; METIS uses its own parallelism.
std::vector<uint32_t> ComputeNestedDissectionOrder(const CchGraph& g,
                                                   uint32_t concurrency = 0);

} // namespace cch
} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_CCH_NESTED_DISSECTION_H_
