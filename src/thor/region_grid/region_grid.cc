#include "thor/region_grid/region_grid.h"

#include "thor/region_grid/h3_partition.h"
#include "thor/region_grid/medoid.h"
#include "thor/region_grid/network_voronoi.h"

#include "midgard/logging.h"

#include <limits>
#include <unordered_map>

namespace valhalla {
namespace thor {
namespace region_grid {

RegionGridResult BuildRegionGrid(const cch::CchGraph& graph, const RegionGridOptions& options) {
  RegionGridResult result;
  if (graph.nodes.empty()) {
    LOG_WARN("region_grid: empty truck graph");
    return result;
  }

  auto cells = PartitionH3Cells(graph, options);
  if (cells.empty()) {
    LOG_WARN("region_grid: no H3 cells produced");
    return result;
  }

  LOG_INFO("region_grid: choosing medoids for " + std::to_string(cells.size()) + " cells...");
  auto medoid_nodes =
      ChooseMedoids(graph, cells, options.medoid_sample_cap, options.unreachable_penalty_s);

  result.regions.reserve(cells.size());
  std::vector<uint32_t> medoid_region_ids;
  medoid_region_ids.reserve(cells.size());

  for (uint32_t i = 0; i < cells.size(); ++i) {
    Region r;
    r.region_id = i;
    r.country = cells[i].country;
    r.h3_index = cells[i].h3_index;
    r.rep_node_index = medoid_nodes[i];
    if (r.rep_node_index < graph.nodes.size()) {
      r.rep_lat = graph.nodes[r.rep_node_index].lat;
      r.rep_lon = graph.nodes[r.rep_node_index].lon;
    }
    r.node_count = 0; // filled after Voronoi
    result.regions.push_back(r);
    medoid_region_ids.push_back(i);
  }

  LOG_INFO("region_grid: computing network Voronoi...");
  auto voronoi = ComputeNetworkVoronoi(graph, medoid_nodes, medoid_region_ids);
  result.node_regions = std::move(voronoi.node_regions);
  result.euclidean_fallback_count = voronoi.euclidean_fallback_count;

  for (const auto& a : result.node_regions) {
    if (a.region_id < result.regions.size())
      ++result.regions[a.region_id].node_count;
    else if (a.region_id == std::numeric_limits<uint32_t>::max())
      ++result.unassigned_nodes;
  }

  LOG_INFO("region_grid: build complete  regions=" + std::to_string(result.regions.size()) +
           " nodes=" + std::to_string(graph.nodes.size()) +
           " fallback=" + std::to_string(result.euclidean_fallback_count) +
           " unassigned=" + std::to_string(result.unassigned_nodes));
  return result;
}

} // namespace region_grid
} // namespace thor
} // namespace valhalla
