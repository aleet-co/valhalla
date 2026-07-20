#ifndef VALHALLA_THOR_REGION_GRID_MEDOID_H_
#define VALHALLA_THOR_REGION_GRID_MEDOID_H_

#include <cstdint>
#include <vector>

#include <valhalla/thor/cch/cch_graph.h>
#include <valhalla/thor/region_grid/h3_partition.h>

namespace valhalla {
namespace thor {
namespace region_grid {

// Pick a network medoid for each cell (minimize sum of truck times to other
// sampled nodes in the cell + 1-ring halo). Returns rep node index per cell
// (same order as cells). Unreachable samples are charged unreachable_penalty_s.
std::vector<uint32_t> ChooseMedoids(const cch::CchGraph& graph,
                                    const std::vector<CellSeed>& cells,
                                    uint32_t sample_cap,
                                    uint32_t unreachable_penalty_s);

} // namespace region_grid
} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_REGION_GRID_MEDOID_H_
