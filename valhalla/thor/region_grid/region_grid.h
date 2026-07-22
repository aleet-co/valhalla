#ifndef VALHALLA_THOR_REGION_GRID_REGION_GRID_H_
#define VALHALLA_THOR_REGION_GRID_REGION_GRID_H_

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/thor/cch/cch_graph.h>

namespace valhalla {
namespace thor {
namespace region_grid {

struct Region {
  uint32_t region_id = 0;
  std::string country;
  uint64_t h3_index = 0;
  uint32_t rep_node_index = 0;
  double rep_lat = 0;
  double rep_lon = 0;
  uint32_t node_count = 0;
};

struct NodeAssignment {
  uint32_t region_id = 0;
  uint32_t time_to_rep_s = 0;
};

struct RegionGridResult {
  std::vector<Region> regions;
  std::vector<NodeAssignment> node_regions; // parallel to graph.nodes
  uint32_t euclidean_fallback_count = 0;
  uint32_t unassigned_nodes = 0; // excluded-country / no same-country medoid
};

struct RegionGridOptions {
  uint32_t target_regions = 6000;
  int base_h3_res = 5;
  int max_h3_res = 6;
  // Dense (high weight/km²) cells stop refining at this resolution; sparser
  // cells may still split up to max_h3_res. Cuts urban medoid clusters.
  int dense_max_h3_res = 5;
  // Cell is "dense" when weight/km² ≥ factor × country mean cell density.
  double dense_density_factor = 1.5;
  uint32_t medoid_sample_cap = 64;
  uint32_t unreachable_penalty_s = 1000000;
  // Drop these ISO2 codes from partitioning (no regions / no budget share).
  std::vector<std::string> exclude_countries{"RU", "BY"};
};

// Build country-clipped H3 regions with network medoids and in-country Voronoi
// ownership on the truck subgraph.
RegionGridResult BuildRegionGrid(const cch::CchGraph& graph,
                                 const RegionGridOptions& options = {});

} // namespace region_grid
} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_REGION_GRID_REGION_GRID_H_
