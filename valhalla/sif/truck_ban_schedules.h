#ifndef VALHALLA_SIF_TRUCK_BAN_SCHEDULES_H_
#define VALHALLA_SIF_TRUCK_BAN_SCHEDULES_H_

#include <cstddef>
#include <cstdint>

namespace valhalla {
namespace sif {
namespace truck_ban {

// Data-driven ban schedule primitives. Keep AT/DE entries in sync with
// config/truck_ban_schedules.yaml (status: implemented). Weekday bits use
// Sunday=0 .. Saturday=6 to match LocalBanTime / DateTime::day_of_week.

enum class BanScope : uint8_t {
  kAllRoads = 0,
  kMotorwayTrunk = 1,
};

struct TimeWindow {
  uint8_t start_hour;
  uint8_t start_minute;
  uint8_t end_hour;
  uint8_t end_minute;
};

struct WeeklyRule {
  BanScope scope;
  uint8_t weekdays_mask; // bit N set ⇒ weekday N (Sun=0)
  TimeWindow window;
};

struct HolidayRule {
  BanScope scope;
  TimeWindow window;
};

struct SeasonalRule {
  BanScope scope;
  uint8_t start_month; // inclusive 1-12
  uint8_t end_month;   // inclusive 1-12
  uint8_t weekdays_mask;
  TimeWindow window;
};

// Eve-of-holiday and month-filtered weekly rules are reserved for Tier A
// countries beyond AT/DE (FR/HU/IT). Not evaluated yet.
struct EveOfHolidayRule {
  BanScope scope;
  TimeWindow window;
};

struct CountryRules {
  const char iso[3]; // "AT\0"
  const char* tz_name;
  float min_weight_tons;
  const WeeklyRule* weekly;
  size_t weekly_count;
  const HolidayRule* holiday;
  size_t holiday_count;
  const SeasonalRule* seasonal;
  size_t seasonal_count;
};

inline constexpr uint8_t WeekdayBit(uint8_t sunday_based_weekday) {
  return static_cast<uint8_t>(1u << sunday_based_weekday);
}

inline constexpr uint8_t kSun = WeekdayBit(0);
inline constexpr uint8_t kMon = WeekdayBit(1);
inline constexpr uint8_t kTue = WeekdayBit(2);
inline constexpr uint8_t kWed = WeekdayBit(3);
inline constexpr uint8_t kThu = WeekdayBit(4);
inline constexpr uint8_t kFri = WeekdayBit(5);
inline constexpr uint8_t kSat = WeekdayBit(6);
inline constexpr uint8_t kEveryDay = static_cast<uint8_t>(kSun | kMon | kTue | kWed | kThu | kFri | kSat);

// ── Austria ──────────────────────────────────────────────────────────────
inline constexpr WeeklyRule kAtWeekly[] = {
    {BanScope::kAllRoads, kSat, {15, 0, 24, 0}},      // Sat 15:00-24:00
    {BanScope::kAllRoads, kSun, {0, 0, 22, 0}},       // Sun 00:00-22:00
    {BanScope::kAllRoads, kEveryDay, {22, 0, 5, 0}},  // night 22:00-05:00
};
inline constexpr HolidayRule kAtHoliday[] = {
    {BanScope::kAllRoads, {0, 0, 22, 0}},
};

// ── Germany ──────────────────────────────────────────────────────────────
inline constexpr WeeklyRule kDeWeekly[] = {
    {BanScope::kAllRoads, kSun, {0, 0, 22, 0}}, // Sun 00:00-22:00
};
inline constexpr HolidayRule kDeHoliday[] = {
    {BanScope::kAllRoads, {0, 0, 22, 0}},
};
inline constexpr SeasonalRule kDeSeasonal[] = {
    // Jul–Aug Sat 07:00-20:00 on motorway/trunk
    {BanScope::kMotorwayTrunk, 7, 8, kSat, {7, 0, 20, 0}},
};

// Implemented countries only (Phase 0). Order does not matter; lookup is linear.
inline constexpr CountryRules kCountrySchedules[] = {
    {"AT", "Europe/Vienna", 7.5f, kAtWeekly, 3, kAtHoliday, 1, nullptr, 0},
    {"DE", "Europe/Berlin", 7.5f, kDeWeekly, 1, kDeHoliday, 1, kDeSeasonal, 1},
};

inline constexpr size_t kCountryScheduleCount =
    sizeof(kCountrySchedules) / sizeof(kCountrySchedules[0]);

inline const CountryRules* FindCountryRules(const char* country_iso) {
  if (country_iso == nullptr || country_iso[0] == '\0' || country_iso[1] == '\0') {
    return nullptr;
  }
  for (size_t i = 0; i < kCountryScheduleCount; ++i) {
    const auto& rules = kCountrySchedules[i];
    if (rules.iso[0] == country_iso[0] && rules.iso[1] == country_iso[1]) {
      return &rules;
    }
  }
  return nullptr;
}

} // namespace truck_ban
} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_TRUCK_BAN_SCHEDULES_H_
