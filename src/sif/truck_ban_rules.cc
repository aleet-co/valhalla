#include "sif/truck_ban_rules.h"

#include "baldr/datetime.h"
#include "baldr/graphreader.h"
#include "sif/truck_ban_cache.h"
#include "sif/truck_ban_holidays.h"

#include <shared_mutex>
#include <string>
#include <unordered_map>

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {
namespace truck_ban {
namespace {

constexpr uint8_t kSunday = 0;
constexpr uint8_t kSaturday = 6;

uint32_t ResolveTzIndex(uint32_t tz_index, const char* country_iso) {
  if (tz_index != 0 && DateTime::get_tz_db().from_index(tz_index) != nullptr) {
    return tz_index;
  }
  if (country_iso[0] == 'A' && country_iso[1] == 'T') {
    return static_cast<uint32_t>(DateTime::get_tz_db().to_index("Europe/Vienna"));
  }
  if (country_iso[0] == 'D' && country_iso[1] == 'E') {
    return static_cast<uint32_t>(DateTime::get_tz_db().to_index("Europe/Berlin"));
  }
  return tz_index;
}

LocalBanTime ToLocalTime(uint64_t current_time, uint32_t tz_index) {
  LocalBanTime local{};
  if (current_time == 0) {
    return local;
  }

  const date::time_zone* tz = DateTime::get_tz_db().from_index(tz_index);
  if (tz == nullptr) {
    return local;
  }

  const std::string iso = DateTime::seconds_to_date(current_time, tz, false);
  const std::tm tm = DateTime::iso_to_tm(iso);
  if (tm.tm_year == 0) {
    return local;
  }

  local.year = tm.tm_year + 1900;
  local.month = tm.tm_mon + 1;
  local.day = tm.tm_mday;
  // iso_to_tm only parses YYYY-mm-ddTHH:MM; tm_wday stays zero-initialized (Sunday).
  local.weekday = static_cast<int>(DateTime::day_of_week(iso));
  local.hour = tm.tm_hour;
  local.minute = tm.tm_min;
  local.ymd = local.year * 10000 + local.month * 100 + local.day;
  return local;
}

uint64_t LocalHourKey(uint32_t resolved_tz, uint64_t current_time) {
  if (current_time == 0) {
    return 0;
  }

  const uint64_t bucket = current_time / 3600ULL;
  const uint64_t cache_key = (static_cast<uint64_t>(resolved_tz) << 32) | bucket;

  static std::shared_mutex mutex;
  static std::unordered_map<uint64_t, uint64_t> cache;

  {
    std::shared_lock lock(mutex);
    const auto found = cache.find(cache_key);
    if (found != cache.end()) {
      RecordLocalHourHit();
      return found->second;
    }
  }

  RecordLocalHourMiss();
  const LocalBanTime local = ToLocalTime(current_time, resolved_tz);
  if (local.year == 0) {
    return 0;
  }

  const uint64_t hour_key =
      (static_cast<uint64_t>(static_cast<uint32_t>(local.ymd)) << 5) |
      static_cast<uint64_t>(static_cast<uint8_t>(local.hour));

  {
    std::unique_lock lock(mutex);
    cache.emplace(cache_key, hour_key);
  }

  return hour_key;
}

void ClearLocalHourKeyCache() {
  static std::shared_mutex mutex;
  static std::unordered_map<uint64_t, uint64_t> cache;
  std::unique_lock lock(mutex);
  cache.clear();
}

bool InMinutes(int hour, int minute, int start_hour, int start_minute, int end_hour, int end_minute) {
  const int current = hour * 60 + minute;
  const int start = start_hour * 60 + start_minute;
  const int end = end_hour * 60 + end_minute;
  if (start <= end) {
    return current >= start && current < end;
  }
  // Overnight window, e.g. 22:00-05:00
  return current >= start || current < end;
}

bool IsHoliday(const char* country_iso, int ymd) {
  if (country_iso[0] == 'A' && country_iso[1] == 'T') {
    return AtHolidays().find(static_cast<uint32_t>(ymd)) != AtHolidays().end();
  }
  if (country_iso[0] == 'D' && country_iso[1] == 'E') {
    return DeHolidays().find(static_cast<uint32_t>(ymd)) != DeHolidays().end();
  }
  return false;
}

bool IsMotorwayOrTrunk(RoadClass road_class) {
  return road_class == RoadClass::kMotorway || road_class == RoadClass::kTrunk;
}

bool AustriaAllRoadBan(const LocalBanTime& local) {
  if (local.year == 0) {
    return false;
  }

  // Saturday 15:00-24:00
  if (local.weekday == kSaturday && InMinutes(local.hour, local.minute, 15, 0, 24, 0)) {
    return true;
  }

  // Sunday 00:00-22:00
  if (local.weekday == kSunday && InMinutes(local.hour, local.minute, 0, 0, 22, 0)) {
    return true;
  }

  // Public holidays 00:00-22:00
  if (IsHoliday("AT", local.ymd) && InMinutes(local.hour, local.minute, 0, 0, 22, 0)) {
    return true;
  }

  // Night ban every day 22:00-05:00
  if (InMinutes(local.hour, local.minute, 22, 0, 5, 0)) {
    return true;
  }

  return false;
}

bool GermanyAllRoadBan(const LocalBanTime& local) {
  if (local.year == 0) {
    return false;
  }

  // Sunday 00:00-22:00
  if (local.weekday == kSunday && InMinutes(local.hour, local.minute, 0, 0, 22, 0)) {
    return true;
  }

  // Public holidays 00:00-22:00
  if (IsHoliday("DE", local.ymd) && InMinutes(local.hour, local.minute, 0, 0, 22, 0)) {
    return true;
  }

  return false;
}

bool GermanyMotorwayBan(const LocalBanTime& local, RoadClass road_class) {
  if (local.year == 0 || !IsMotorwayOrTrunk(road_class)) {
    return false;
  }

  // July-August Saturday 07:00-20:00 on motorways and trunk roads
  if ((local.month == 7 || local.month == 8) && local.weekday == kSaturday &&
      InMinutes(local.hour, local.minute, 7, 0, 20, 0)) {
    return true;
  }

  return false;
}

bool IsBanCountry(const char* country_iso) {
  return (country_iso[0] == 'A' && country_iso[1] == 'T') ||
         (country_iso[0] == 'D' && country_iso[1] == 'E');
}

bool EvaluateBanRules(const char* country_iso, const LocalBanTime& local, RoadClass road_class) {
  if (local.year == 0) {
    return true;
  }

  if (country_iso[0] == 'A' && country_iso[1] == 'T' && AustriaAllRoadBan(local)) {
    return false;
  }

  if (country_iso[0] == 'D' && country_iso[1] == 'E') {
    if (GermanyAllRoadBan(local)) {
      return false;
    }
    if (GermanyMotorwayBan(local, road_class)) {
      return false;
    }
  }

  return true;
}

bool CachedBanAllowed(const char* country_iso,
                      uint32_t resolved_tz,
                      uint64_t current_time,
                      RoadClass road_class) {
  const uint64_t hour_key = LocalHourKey(resolved_tz, current_time);
  if (hour_key == 0) {
    return true;
  }

  auto& cache = BanDecisionCache::instance();
  if (const auto cached = cache.lookup_hour_key(country_iso, resolved_tz, hour_key, road_class)) {
    return *cached;
  }

  const LocalBanTime local = ToLocalTime(current_time, resolved_tz);
  if (local.year == 0) {
    return true;
  }

  const bool allowed = EvaluateBanRules(country_iso, local, road_class);
  cache.store_hour_key(country_iso, resolved_tz, hour_key, road_class, allowed);
  return allowed;
}

bool EvaluateTraverseAllowed(const baldr::GraphId& from_node,
                             const baldr::GraphId& to_node,
                             const baldr::graph_tile_ptr& tile,
                             baldr::GraphReader* reader,
                             uint64_t depart_time,
                             uint64_t arrive_time,
                             float weight_metric_tons,
                             RoadClass road_class,
                             bool* touched_ban_country) {
  auto check_node = [&](const baldr::GraphId& nodeid, uint64_t time_at_node) -> bool {
    if (time_at_node == 0 || !nodeid.is_valid()) {
      return true;
    }

    baldr::graph_tile_ptr node_tile = tile;
    if (!node_tile || nodeid.tile_base() != node_tile->id()) {
      if (reader == nullptr) {
        return true;
      }
      node_tile = reader->GetGraphTile(nodeid);
      if (!node_tile) {
        return true;
      }
    }

    const baldr::NodeInfo* node = node_tile->node(nodeid);
    if (node == nullptr) {
      return true;
    }

    const baldr::Admin* admin = node_tile->admin(node->admin_index());
    if (admin == nullptr) {
      return true;
    }

    const std::string country = admin->country_iso();
    if (country.size() < 2 || !IsBanCountry(country.data())) {
      return true;
    }

    if (touched_ban_country != nullptr) {
      *touched_ban_country = true;
    }

    const uint32_t resolved_tz = ResolveTzIndex(node->timezone(), country.data());
    return CachedBanAllowed(country.data(), resolved_tz, time_at_node, road_class);
  };

  // Same admin and same local hour: one evaluation is enough for intra-country edges.
  if (tile && from_node.is_valid() && to_node.is_valid() && depart_time != 0 && arrive_time != 0) {
    baldr::graph_tile_ptr node_tile = tile;
    if (from_node.tile_base() == to_node.tile_base() && from_node.tile_base() == node_tile->id()) {
      const baldr::NodeInfo* from = node_tile->node(from_node);
      const baldr::NodeInfo* to = node_tile->node(to_node);
      if (from != nullptr && to != nullptr && from->admin_index() == to->admin_index()) {
        const baldr::Admin* admin = node_tile->admin(to->admin_index());
        if (admin != nullptr) {
          const std::string country = admin->country_iso();
          if (country.size() >= 2 && IsBanCountry(country.data())) {
            const uint32_t resolved_tz = ResolveTzIndex(to->timezone(), country.data());
            const uint64_t depart_hour_key = LocalHourKey(resolved_tz, depart_time);
            const uint64_t arrive_hour_key = LocalHourKey(resolved_tz, arrive_time);
            if (depart_hour_key != 0 && arrive_hour_key != 0 &&
                (depart_hour_key >> 5) == (arrive_hour_key >> 5) &&
                (depart_hour_key & 0x1F) == (arrive_hour_key & 0x1F)) {
              return check_node(to_node, arrive_time);
            }
          }
        }
      }
    }
  }

  if (!check_node(from_node, depart_time)) {
    return false;
  }
  if (!check_node(to_node, arrive_time)) {
    return false;
  }
  return true;
}

} // namespace

bool IsEdgeAllowed(const std::string& country_iso,
                   uint64_t current_time,
                   uint32_t tz_index,
                   float weight_metric_tons,
                   RoadClass road_class) {
  if (current_time == 0 || weight_metric_tons < kMinGrossWeightMetricTons) {
    return true;
  }

  if (country_iso.size() < 2 || !IsBanCountry(country_iso.data())) {
    return true;
  }

  const uint32_t resolved_tz = ResolveTzIndex(tz_index, country_iso.data());
  return CachedBanAllowed(country_iso.data(), resolved_tz, current_time, road_class);
}

bool IsTraverseAllowed(const baldr::GraphId& from_node,
                       const baldr::GraphId& to_node,
                       const baldr::graph_tile_ptr& tile,
                       baldr::GraphReader* reader,
                       uint64_t depart_time,
                       uint64_t arrive_time,
                       float weight_metric_tons,
                       RoadClass road_class) {
  if ((depart_time == 0 && arrive_time == 0) || weight_metric_tons < kMinGrossWeightMetricTons) {
    return true;
  }

  RecordTraverseCheck();

  auto& traverse_cache = TraverseBanCache::instance();
  if (const auto cached =
          traverse_cache.lookup(from_node, to_node, depart_time, arrive_time, road_class)) {
    return *cached;
  }

  bool touched_ban_country = false;
  const bool allowed = EvaluateTraverseAllowed(from_node,
                                               to_node,
                                               tile,
                                               reader,
                                               depart_time,
                                               arrive_time,
                                               weight_metric_tons,
                                               road_class,
                                               &touched_ban_country);
  if (touched_ban_country) {
    traverse_cache.store(from_node, to_node, depart_time, arrive_time, road_class, allowed);
  }
  return allowed;
}

} // namespace truck_ban
} // namespace sif
} // namespace valhalla
