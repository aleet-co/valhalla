#ifndef VALHALLA_THOR_REGION_GRID_H3_PARTITION_H_
#define VALHALLA_THOR_REGION_GRID_H3_PARTITION_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <valhalla/thor/cch/cch_graph.h>
#include <valhalla/thor/region_grid/region_grid.h>

namespace valhalla {
namespace thor {
namespace region_grid {

struct CellSeed {
  uint64_t h3_index = 0;
  std::string country;
  std::vector<uint32_t> node_indices;
  uint64_t weight = 0; // sum of incident truck edge times (proxy)
};

// Partition truck-graph nodes into ~target_regions H3 cells, allocated by
// per-country truck-edge weight. Empty cells are dropped; undersized cells are
// merged into same-country neighbors; overweight cells may split one res finer.
std::vector<CellSeed> PartitionH3Cells(const cch::CchGraph& graph,
                                       const RegionGridOptions& options);

// H3 helpers exposed for tests / GeoJSON.
uint64_t LatLngToH3(double lat, double lon, int res);
int H3Resolution(uint64_t h3_index);
uint64_t H3Parent(uint64_t h3_index, int parent_res);
std::vector<uint64_t> H3GridDisk(uint64_t h3_index, int k);
std::vector<std::pair<double, double>> H3BoundaryLatLng(uint64_t h3_index);

} // namespace region_grid
} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_REGION_GRID_H3_PARTITION_H_
