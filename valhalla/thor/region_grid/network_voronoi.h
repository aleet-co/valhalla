#ifndef VALHALLA_THOR_REGION_GRID_NETWORK_VORONOI_H_
#define VALHALLA_THOR_REGION_GRID_NETWORK_VORONOI_H_

#include <cstdint>
#include <vector>

#include <valhalla/thor/cch/cch_graph.h>
#include <valhalla/thor/region_grid/region_grid.h>

namespace valhalla {
namespace thor {
namespace region_grid {

struct VoronoiResult {
  std::vector<NodeAssignment> node_regions;
  uint32_t euclidean_fallback_count = 0;
};

// Multi-source Dijkstra from medoids. Relaxation stays within the same country.
// Nodes never reached get nearest in-country medoid by Euclidean distance.
VoronoiResult ComputeNetworkVoronoi(const cch::CchGraph& graph,
                                    const std::vector<uint32_t>& medoid_node_indices,
                                    const std::vector<uint32_t>& medoid_region_ids);

} // namespace region_grid
} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_REGION_GRID_NETWORK_VORONOI_H_
