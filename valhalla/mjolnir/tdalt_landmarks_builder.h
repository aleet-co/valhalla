#ifndef VALHALLA_MJOLNIR_TDALT_LANDMARKS_BUILDER_H_
#define VALHALLA_MJOLNIR_TDALT_LANDMARKS_BUILDER_H_

#include <boost/property_tree/ptree_fwd.hpp>

namespace valhalla {
namespace mjolnir {

/**
 * Offline preprocessing for TDALT ALT landmarks (graph G_λ).
 *
 * Pipeline:
 *   1. Subsample level-2 (local) nodes as landmark candidates
 *   2. maxCover — greedily pick L landmarks that maximize min λ-distance to prior picks
 *   3. Run λ-Dijkstra from each landmark (forward + reverse on G_λ)
 *   4. Write tdalt_landmarks.bin for mmap load at service startup
 */
void build_tdalt_landmarks(const boost::property_tree::ptree& config);

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_TDALT_LANDMARKS_BUILDER_H_
