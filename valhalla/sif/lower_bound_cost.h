#ifndef VALHALLA_SIF_LOWER_BOUND_COST_H_
#define VALHALLA_SIF_LOWER_BOUND_COST_H_

namespace valhalla {
namespace baldr {
class DirectedEdge;
} // namespace baldr

namespace sif {

/**
 * Static lower-bound edge travel time for TDALT graph G_lambda.
 * lambda = length / max_speed with no traffic, bans, or restrictions.
 */
struct LowerBoundCost {
  static float seconds(const baldr::DirectedEdge* edge, float length_override = -1.f);
};

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_LOWER_BOUND_COST_H_
