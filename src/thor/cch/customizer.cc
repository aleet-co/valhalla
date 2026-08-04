#include "thor/cch/customizer.h"

#include <valhalla/baldr/graphconstants.h>

#include "sif/truck_ban_rules.h"

namespace valhalla {
namespace thor {
namespace cch {

namespace {
constexpr float kTruckWeightTons = 40.0f; // ensure > 7.5t so bans always evaluate

// Sample one representative week at slot resolution and set forbidden slots.
WeeklyMask DeriveMask(const std::string& country_iso, uint32_t tz_index, uint8_t roadclass,
                      int64_t week_monday_utc) {
  WeeklyMask mask;
  // Transition connectors and unknown classes are never banned.
  if (roadclass == 255)
    return mask;
  const auto rc = static_cast<baldr::RoadClass>(roadclass);
  for (int slot = 0; slot < kNumSlots; ++slot) {
    const int64_t epoch = week_monday_utc + static_cast<int64_t>(slot) * kSlotSeconds;
    const bool allowed = sif::truck_ban::IsEdgeAllowed(
        country_iso, static_cast<uint64_t>(epoch), tz_index, kTruckWeightTons, rc);
    if (!allowed)
      set_slot(mask, utc_second_of_week(epoch) / kSlotSeconds);
  }
  return mask;
}
} // namespace

CustomizedMetric Customize(const CchGraph& g, const CchOrder& order,
                           int64_t reference_week_monday_utc) {
  CustomizedMetric metric;
  metric.graph = &g;
  metric.order = &order;
  const size_t total = order.num_base_edges + order.shortcuts.size();
  metric.profiles.resize(total);

  // Base edges: T from the graph, B from replaying the ban rules at the destination.
  for (uint32_t ei = 0; ei < order.num_base_edges; ++ei) {
    const auto& e = g.edges[ei];
    Profile p;
    p.time_s = e.time_s;
    const auto& dest = g.nodes[e.v];
    p.forbidden = DeriveMask(dest.country, dest.tz_index, e.roadclass, reference_week_monday_utc);
    metric.profiles[ei] = p;
  }

  // Shortcuts: compose in creation order (children always have lower edge ids,
  // since a shortcut is built only after its children exist).
  //
  // NOTE: the composed shortcut `B` (forbidden) mask is an algebra invariant
  // (guarded by ShortcutProfileMatchesComposition) and is consumed by
  // query_mode=contracted (ContractedTdEarliest / EdgeFeasibleAt). Corridor mode
  // (design C) still ignores shortcut `B`: its CH up/down search runs on static
  // `time_s` only, and the base-graph TdRepair pass is the sole ban authority.
  for (uint32_t si = 0; si < order.shortcuts.size(); ++si) {
    const auto& sc = order.shortcuts[si];
    metric.profiles[order.num_base_edges + si] =
        compose(metric.profiles[sc.left], metric.profiles[sc.right]);
  }

  return metric;
}

} // namespace cch
} // namespace thor
} // namespace valhalla
