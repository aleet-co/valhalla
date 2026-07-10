#ifndef VALHALLA_SIF_TRUCK_BAN_CACHE_H_
#define VALHALLA_SIF_TRUCK_BAN_CACHE_H_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <thread>
#include <memory>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace sif {
namespace truck_ban {

struct LocalBanTime {
  int year{0};
  int month{0};
  int day{0};
  int weekday{0}; // 0 = Sunday
  int hour{0};
  int minute{0};
  int ymd{0}; // YYYYMMDD
};

enum class CacheMode {
  kOff,
  kDecisionOnly,
  kFull,
};

struct CacheStatsSnapshot {
  uint64_t traverse_checks{0};
  uint64_t traverse_hits{0};
  uint64_t traverse_misses{0};
  uint64_t traverse_stores{0};
  uint64_t decision_hits{0};
  uint64_t decision_misses{0};
  uint64_t decision_stores{0};
  uint64_t local_hour_hits{0};
  uint64_t local_hour_misses{0};
};

CacheMode GetCacheMode();
bool DecisionCacheEnabled();
bool TraverseCacheEnabled();

void ResetCacheStats();
CacheStatsSnapshot GetCacheStats();
void LogCacheStats(const char* label = nullptr);
void ClearAllCaches();

/** RAII: tags cache stats with route endpoints; logs once on scope end when stats logging is on. */
class RouteCacheLogScope {
public:
  RouteCacheLogScope(double origin_lat,
                     double origin_lon,
                     double dest_lat,
                     double dest_lon);
  ~RouteCacheLogScope();

  RouteCacheLogScope(const RouteCacheLogScope&) = delete;
  RouteCacheLogScope& operator=(const RouteCacheLogScope&) = delete;

private:
  bool active_{false};
};

void RecordTraverseCheck();
void RecordLocalHourHit();
void RecordLocalHourMiss();

/**
 * Country-level ban decision cache (mmap-backed, shared across Valhalla worker processes).
 * Keys use local calendar hour; entries expire after one week. Optional .bin snapshot export.
 */
class BanDecisionCache {
public:
  static BanDecisionCache& instance();

  std::optional<bool> lookup_hour_key(const char* country_iso,
                                      uint32_t resolved_tz,
                                      uint64_t local_hour_key,
                                      baldr::RoadClass road_class);

  void store_hour_key(const char* country_iso,
                      uint32_t resolved_tz,
                      uint64_t local_hour_key,
                      baldr::RoadClass road_class,
                      bool allowed);

  std::optional<bool> lookup(const char* country_iso,
                             uint32_t resolved_tz,
                             const LocalBanTime& local,
                             baldr::RoadClass road_class);

  void store(const char* country_iso,
             uint32_t resolved_tz,
             const LocalBanTime& local,
             baldr::RoadClass road_class,
             bool allowed);

  void clear_memory();

private:
  BanDecisionCache();
  ~BanDecisionCache();

  BanDecisionCache(const BanDecisionCache&) = delete;
  BanDecisionCache& operator=(const BanDecisionCache&) = delete;

  struct Impl;
  std::unique_ptr<Impl> impl_;

  static uint64_t make_key(const char* country_iso,
                           uint32_t resolved_tz,
                           uint64_t local_hour_key,
                           baldr::RoadClass road_class);

  static uint64_t pack_local_hour_key(const LocalBanTime& local);
};

/**
 * Edge traverse result cache (mmap-backed, shared across Valhalla worker processes).
 * On hit, IsTraverseAllowed returns without graph, timezone, or rule evaluation.
 */
class TraverseBanCache {
public:
  static TraverseBanCache& instance();

  std::optional<bool> lookup(const baldr::GraphId& from_node,
                             const baldr::GraphId& to_node,
                             uint64_t depart_time,
                             uint64_t arrive_time,
                             baldr::RoadClass road_class);

  void store(const baldr::GraphId& from_node,
             const baldr::GraphId& to_node,
             uint64_t depart_time,
             uint64_t arrive_time,
             baldr::RoadClass road_class,
             bool allowed);

  void clear_memory();

private:
  TraverseBanCache();
  ~TraverseBanCache();

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace truck_ban
} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_TRUCK_BAN_CACHE_H_
