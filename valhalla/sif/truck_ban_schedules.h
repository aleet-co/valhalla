#ifndef VALHALLA_SIF_TRUCK_BAN_SCHEDULES_H_
#define VALHALLA_SIF_TRUCK_BAN_SCHEDULES_H_

#include <cstddef>
#include <cstdint>

namespace valhalla {
namespace sif {
namespace truck_ban {

// Data-driven ban schedule primitives. Keep in sync with
// config/truck_ban_schedules.yaml (status: implemented, ban_mode: weekly_only)
// and aleet private/modules/valhalla/valhalla/truck_bans.py.
// Weekday bits use Sunday=0 .. Saturday=6 to match LocalBanTime / DateTime::day_of_week.
// months_mask: bit (month-1) set ⇒ applies in that month; 0 ⇒ all months.
//
// Holiday / seasonal / eve-of-holiday counts are intentionally 0 so the joint
// ban state is period-7-days (modulo DST). Offline rep_matrix tiles one week
// of fingerprints over a year; if you re-add calendar-date rules here, TD fill
// and live Allowed() will diverge unless the Python calendar grows too.
// HolidayRule / SeasonalRule / EveOfHolidayRule structs remain so the evaluator
// in truck_ban_rules.cc can stay unchanged.
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
  uint16_t months_mask; // 0 = all months
};

struct HolidayRule {
  BanScope scope;
  TimeWindow window;
  uint16_t months_mask; // 0 = all months
};

struct SeasonalRule {
  BanScope scope;
  uint8_t start_month; // inclusive 1-12
  uint8_t end_month;   // inclusive 1-12
  uint8_t weekdays_mask;
  TimeWindow window;
};

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
  const EveOfHolidayRule* eve_of_holiday;
  size_t eve_of_holiday_count;
};

inline constexpr uint8_t WeekdayBit(uint8_t sunday_based_weekday) {
  return static_cast<uint8_t>(1u << sunday_based_weekday);
}

inline constexpr uint16_t MonthBit(uint8_t month /* 1-12 */) {
  return static_cast<uint16_t>(1u << (month - 1));
}

inline constexpr uint8_t kSun = WeekdayBit(0);
inline constexpr uint8_t kMon = WeekdayBit(1);
inline constexpr uint8_t kTue = WeekdayBit(2);
inline constexpr uint8_t kWed = WeekdayBit(3);
inline constexpr uint8_t kThu = WeekdayBit(4);
inline constexpr uint8_t kFri = WeekdayBit(5);
inline constexpr uint8_t kSat = WeekdayBit(6);
inline constexpr uint8_t kEveryDay =
    static_cast<uint8_t>(kSun | kMon | kTue | kWed | kThu | kFri | kSat);

// ── Austria ──────────────────────────────────────────────────────────────
inline constexpr WeeklyRule kAtWeekly[] = {
    {BanScope::kAllRoads, kSat, {15, 0, 24, 0}, 0},     // Sat 15:00-24:00
    {BanScope::kAllRoads, kSun, {0, 0, 22, 0}, 0},      // Sun 00:00-22:00
    {BanScope::kAllRoads, kEveryDay, {22, 0, 5, 0}, 0}, // night 22:00-05:00
};

// ── Germany ──────────────────────────────────────────────────────────────
// Sunday all-roads only. Former Jul–Aug Sat motorway ban omitted (weekly-only).
inline constexpr WeeklyRule kDeWeekly[] = {
    {BanScope::kAllRoads, kSun, {0, 0, 22, 0}, 0},
};

// ── Switzerland ──────────────────────────────────────────────────────────
inline constexpr WeeklyRule kChWeekly[] = {
    {BanScope::kAllRoads, kSun, {0, 0, 24, 0}, 0},
    {BanScope::kAllRoads, kEveryDay, {22, 0, 5, 0}, 0},
};

// ── Liechtenstein (mirrors CH) ───────────────────────────────────────────
inline constexpr WeeklyRule kLiWeekly[] = {
    {BanScope::kAllRoads, kSun, {0, 0, 24, 0}, 0},
    {BanScope::kAllRoads, kEveryDay, {22, 0, 5, 0}, 0},
};

// ── France ───────────────────────────────────────────────────────────────
inline constexpr WeeklyRule kFrWeekly[] = {
    {BanScope::kAllRoads, kSat, {22, 0, 24, 0}, 0},
    {BanScope::kAllRoads, kSun, {0, 0, 22, 0}, 0},
};

// ── Hungary ──────────────────────────────────────────────────────────────
inline constexpr WeeklyRule kHuWeekly[] = {
    {BanScope::kAllRoads, kSat, {22, 0, 24, 0}, 0},
    {BanScope::kAllRoads, kSun, {0, 0, 22, 0}, 0},
};

// ── Italy (fixed summer Sunday hours year-round for week periodicity) ────
// Real rule is winter 09-22 / summer 07-22; pin 07-22 (longer ban) so the
// offline week template does not need month masks.
inline constexpr WeeklyRule kItWeekly[] = {
    {BanScope::kAllRoads, kSun, {7, 0, 22, 0}, 0},
};

// ── Czechia ──────────────────────────────────────────────────────────────
inline constexpr WeeklyRule kCzWeekly[] = {
    {BanScope::kMotorwayTrunk, kSun, {13, 0, 22, 0}, 0},
};

// ── Slovakia ─────────────────────────────────────────────────────────────
inline constexpr WeeklyRule kSkWeekly[] = {
    {BanScope::kMotorwayTrunk, kSun, {0, 0, 22, 0}, 0},
};

// Tier A implemented countries. Order does not matter; lookup is linear.
inline constexpr CountryRules kCountrySchedules[] = {
    {"AT", "Europe/Vienna", 7.5f, kAtWeekly, 3, nullptr, 0, nullptr, 0, nullptr, 0},
    {"DE", "Europe/Berlin", 7.5f, kDeWeekly, 1, nullptr, 0, nullptr, 0, nullptr, 0},
    {"CH", "Europe/Zurich", 3.5f, kChWeekly, 2, nullptr, 0, nullptr, 0, nullptr, 0},
    {"LI", "Europe/Zurich", 3.5f, kLiWeekly, 2, nullptr, 0, nullptr, 0, nullptr, 0},
    {"FR", "Europe/Paris", 7.5f, kFrWeekly, 2, nullptr, 0, nullptr, 0, nullptr, 0},
    {"HU", "Europe/Budapest", 7.5f, kHuWeekly, 2, nullptr, 0, nullptr, 0, nullptr, 0},
    {"IT", "Europe/Rome", 7.5f, kItWeekly, 1, nullptr, 0, nullptr, 0, nullptr, 0},
    {"CZ", "Europe/Prague", 7.5f, kCzWeekly, 1, nullptr, 0, nullptr, 0, nullptr, 0},
    // Europe/Bratislava is a deprecated IANA link; Europe/Prague is the canonical zone.
    {"SK", "Europe/Prague", 7.5f, kSkWeekly, 1, nullptr, 0, nullptr, 0, nullptr, 0},
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
