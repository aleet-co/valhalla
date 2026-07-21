#ifndef VALHALLA_SIF_TRUCK_BAN_RULES_H_
#define VALHALLA_SIF_TRUCK_BAN_RULES_H_

#include <cstdint>
#include <string>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtileptr.h>

namespace valhalla {
namespace baldr {
class GraphReader;
} // namespace baldr

namespace sif {
namespace truck_ban {

// Default EU HGV threshold (AT/DE). Per-country thresholds live in
// truck_ban_schedules.h (e.g. CH/LI use 3.5 t).
constexpr float kMinGrossWeightMetricTons = 7.5f;

/**
 * Returns true when a truck edge may be traversed at the given local time in the
 * given country. Evaluated per-edge during routing so bans apply when the
 * vehicle actually enters a country, not only at departure.
 *
 * @param country_iso       ISO-3166-1 alpha-2 country code from graph admin data
 * @param current_time      Seconds since epoch at edge entry; 0 disables ban checks
 * @param tz_index          Timezone index for the edge end node
 * @param weight_metric_tons Vehicle gross weight in metric tons
 * @param road_class        Road classification of the directed edge
 */
bool IsEdgeAllowed(const std::string& country_iso,
                   uint64_t current_time,
                   uint32_t tz_index,
                   float weight_metric_tons,
                   baldr::RoadClass road_class);

/**
 * Returns false when either endpoint of a directed edge lies in a banned country.
 * Uses GraphReader for cross-tile node lookups at country borders.
 */
bool IsTraverseAllowed(const baldr::GraphId& from_node,
                       const baldr::GraphId& to_node,
                       const baldr::graph_tile_ptr& tile,
                       baldr::GraphReader* reader,
                       uint64_t depart_time,
                       uint64_t arrive_time,
                       float weight_metric_tons,
                       baldr::RoadClass road_class);

} // namespace truck_ban
} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_TRUCK_BAN_RULES_H_
