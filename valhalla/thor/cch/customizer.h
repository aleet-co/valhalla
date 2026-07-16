#ifndef VALHALLA_THOR_CCH_CUSTOMIZER_H_
#define VALHALLA_THOR_CCH_CUSTOMIZER_H_

#include <cstdint>
#include <vector>

#include "thor/cch/cch_graph.h"
#include "thor/cch/order.h"
#include "thor/cch/profile.h"

namespace valhalla {
namespace thor {
namespace cch {

// Monday 2025-02-24 00:00 UTC: no AT/DE public holiday, no DST transition.
constexpr int64_t kDefaultReferenceWeek = 1740355200;

class CustomizedMetric {
public:
  std::vector<Profile> profiles; // by edge id: base [0,num_base) then shortcuts
  const CchGraph* graph = nullptr;
  const CchOrder* order = nullptr;
};

CustomizedMetric Customize(const CchGraph& g, const CchOrder& order,
                           int64_t reference_week_monday_utc = kDefaultReferenceWeek);

} // namespace cch
} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_CCH_CUSTOMIZER_H_
