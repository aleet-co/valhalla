#include "thor/cch/customizer.h"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <valhalla/baldr/graphconstants.h>

#include "midgard/logging.h"
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

// Ban masks depend only on (country, tz, roadclass) for a fixed reference week —
// not on the edge itself. Without this cache, Customize does ~num_base_edges × 672
// IsEdgeAllowed calls (≈1e11 for central-EU) and the first CCH request dies or hangs.
struct MaskKey {
  std::string country;
  uint32_t tz_index = 0;
  uint8_t roadclass = 0;
  bool operator==(const MaskKey& o) const {
    return tz_index == o.tz_index && roadclass == o.roadclass && country == o.country;
  }
};
struct MaskKeyHash {
  size_t operator()(const MaskKey& k) const noexcept {
    size_t h = std::hash<std::string>{}(k.country);
    h ^= static_cast<size_t>(k.tz_index) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(k.roadclass) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};
} // namespace

CustomizedMetric Customize(const CchGraph& g, const CchOrder& order,
                           int64_t reference_week_monday_utc) {
  CustomizedMetric metric;
  metric.graph = &g;
  metric.order = &order;
  const size_t total = order.num_base_edges + order.shortcuts.size();
  const auto t0 = std::chrono::steady_clock::now();
  // Profile ≈ 96B; ~1e9 profiles ≈ 100GiB. Refuse before the allocator OOM-kills
  // the process (which surfaces as a proxy 502 with no customize progress).
  constexpr size_t kMaxProfiles = 200'000'000ull; // ~19GiB ceiling for profiles alone
  LOG_INFO("cch: customize begin base_edges=" + std::to_string(order.num_base_edges) +
           " shortcuts=" + std::to_string(order.shortcuts.size()) +
           " profiles=" + std::to_string(total));
  if (total > kMaxProfiles) {
    throw std::runtime_error(
        "cch: customize aborted — profiles=" + std::to_string(total) +
        " exceeds limit " + std::to_string(kMaxProfiles) +
        " (~" + std::to_string(total * sizeof(Profile) / (1024 * 1024)) +
        " MiB). Shortcut count is pathological for in-memory Customize; rebuild "
        "cch_truck.bin with a better order / tighter truck subgraph (hgv_only), "
        "or raise the limit only on a machine that can hold it.");
  }
  metric.profiles.resize(total);

  std::unordered_map<MaskKey, WeeklyMask, MaskKeyHash> mask_cache;
  mask_cache.reserve(256);

  auto cached_mask = [&](const std::string& country, uint32_t tz_index,
                         uint8_t roadclass) -> WeeklyMask {
    MaskKey key{country, tz_index, roadclass};
    auto it = mask_cache.find(key);
    if (it != mask_cache.end())
      return it->second;
    WeeklyMask m = DeriveMask(country, tz_index, roadclass, reference_week_monday_utc);
    mask_cache.emplace(std::move(key), m);
    return m;
  };

  // Base edges: T from the graph, B from replaying the ban rules at the destination.
  uint32_t banned_base = 0;
  uint32_t banned_slots = 0;
  const uint32_t n_base = order.num_base_edges;
  const uint32_t progress_every = std::max<uint32_t>(1, n_base / 20); // ~5%
  for (uint32_t ei = 0; ei < n_base; ++ei) {
    const auto& e = g.edges[ei];
    Profile p;
    p.time_s = e.time_s;
    const auto& dest = g.nodes[e.v];
    p.forbidden = cached_mask(dest.country, dest.tz_index, e.roadclass);
    const uint32_t nslots = popcount(p.forbidden);
    if (nslots > 0) {
      ++banned_base;
      banned_slots += nslots;
    }
    metric.profiles[ei] = p;
    if (ei > 0 && (ei % progress_every == 0 || ei + 1 == n_base)) {
      const auto dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      LOG_INFO("cch: customize base edges " + std::to_string(ei + 1) + "/" +
               std::to_string(n_base) + " (" +
               std::to_string(static_cast<int>(100.0 * (ei + 1) / std::max<uint32_t>(1, n_base))) +
               "%) mask_cache=" + std::to_string(mask_cache.size()) +
               " elapsed_s=" + std::to_string(dt));
    }
  }

  // Shortcuts: compose in creation order (children always have lower edge ids,
  // since a shortcut is built only after its children exist).
  //
  // NOTE: the composed shortcut `B` (forbidden) mask is an algebra invariant
  // (guarded by ShortcutProfileMatchesComposition) and is always consumed by
  // contracted query modes (contracted_pareto default; contracted single-label)
  // via EdgeFeasibleAt. Design-C corridor query has been retired.
  const auto t_sc0 = std::chrono::steady_clock::now();
  const uint32_t n_sc = static_cast<uint32_t>(order.shortcuts.size());
  const uint32_t sc_every = std::max<uint32_t>(1, n_sc / 10);
  for (uint32_t si = 0; si < n_sc; ++si) {
    const auto& sc = order.shortcuts[si];
    metric.profiles[order.num_base_edges + si] =
        compose(metric.profiles[sc.left], metric.profiles[sc.right]);
    if (n_sc > 0 && (si + 1 == n_sc || (si > 0 && si % sc_every == 0))) {
      const auto dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_sc0).count();
      LOG_INFO("cch: customize shortcuts " + std::to_string(si + 1) + "/" + std::to_string(n_sc) +
               " elapsed_s=" + std::to_string(dt));
    }
  }

  const auto dt_total =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  LOG_INFO("cch: customized bans reference_week_utc=" + std::to_string(reference_week_monday_utc) +
           " base_edges=" + std::to_string(order.num_base_edges) +
           " with_forbidden_slots=" + std::to_string(banned_base) +
           " total_forbidden_slot_marks=" + std::to_string(banned_slots) +
           " shortcuts=" + std::to_string(order.shortcuts.size()) +
           " mask_cache_entries=" + std::to_string(mask_cache.size()) +
           " elapsed_s=" + std::to_string(dt_total));
  if (banned_base == 0) {
    LOG_WARN("cch: zero base edges have ban slots — check tile admin ISO/timezones; "
             "CCH will run but is NOT ban-aware");
  }

  return metric;
}

} // namespace cch
} // namespace thor
} // namespace valhalla
