#ifndef VALHALLA_MJOLNIR_TDALT_LANDMARKS_BUILDER_H_
#define VALHALLA_MJOLNIR_TDALT_LANDMARKS_BUILDER_H_

#include <boost/property_tree/ptree_fwd.hpp>

namespace valhalla {
namespace mjolnir {

/**
 * Build the TDALT landmark distance sidecar from graph tiles.
 * Task 5 writes a valid header stub; Task 6 adds maxCover selection and Dijkstra tables.
 */
void build_tdalt_landmarks(const boost::property_tree::ptree& config);

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_TDALT_LANDMARKS_BUILDER_H_
