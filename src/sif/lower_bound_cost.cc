#include "sif/lower_bound_cost.h"

#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "midgard/constants.h"

#include <algorithm>

namespace valhalla {
namespace sif {

namespace {
constexpr float kMinAssumedSpeedKph = baldr::kMinSpeedKph;
constexpr float kKphToMps = static_cast<float>(midgard::kKPHtoMetersPerSec);
} // namespace

float LowerBoundCost::seconds(const baldr::DirectedEdge* edge, float length_override) {
  // λ(u,v): optimistic seconds = distance / peak speed (ignores τ-dependent c(u,v,τ)).
  const float length =
      length_override >= 0.f ? length_override : static_cast<float>(edge->length());
  const float speed =
      std::max(static_cast<float>(edge->speed()), kMinAssumedSpeedKph) * kKphToMps;
  return length / speed;
}

} // namespace sif
} // namespace valhalla
