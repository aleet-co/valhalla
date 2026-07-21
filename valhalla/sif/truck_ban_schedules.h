#ifndef VALHALLA_SIF_TRUCK_BAN_SCHEDULES_H_
#define VALHALLA_SIF_TRUCK_BAN_SCHEDULES_H_

#include <cstddef>
#include <cstdint>

namespace valhalla {
namespace sif {
namespace truck_ban {

// Data-driven ban schedule primitives. Keep in sync with
// config/truck_ban_schedules.yaml (status: implemented). Weekday bits use
// Sunday=0 .. Saturday=6 to match LocalBanTime / DateTime::day_of_week.
// months_mask: bit (month-1) set ⇒ applies in that month; 0 ⇒ all months.

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

// IT winter (Jan–May, Oct–Dec) / summer (Jun–Sep) month masks.
inline constexpr uint16_t kItWinterMonths = static_cast<uint16_t>(
    MonthBit(1) | MonthBit(2) | MonthBit(3) | MonthBit(4) | MonthBit(5) | MonthBit(10) |
    MonthBit(11) | MonthBit(12));
inline constexpr uint16_t kItSummerMonths =
    static_cast<uint16_t>(MonthBit(6) | MonthBit(7) | MonthBit(8) | MonthBit(9));

// ── Austria ──────────────────────────────────────────────────────────────
inline constexpr WeeklyRule kAtWeekly[] = {
    {BanScope::kAllRoads, kSat, {15, 0, 24, 0}, 0},     // Sat 15:00-24:00
    {BanScope::kAllRoads, kSun, {0, 0, 22, 0}, 0},      // Sun 00:00-22:00
    {BanScope::kAllRoads, kEveryDay, {22, 0, 5, 0}, 0}, // night 22:00-05:00
};
inline constexpr HolidayRule kAtHoliday[] = {
    {BanScope::kAllRoads, {0, 0, 22, 0}, 0},
};

// ── Germany ──────────────────────────────────────────────────────────────
inline constexpr WeeklyRule kDeWeekly[] = {
    {BanScope::kAllRoads, kSun, {0, 0, 22, 0}, 0},
};
inline constexpr HolidayRule kDeHoliday[] = {
    {BanScope::kAllRoads, {0, 0, 22, 0}, 0},
};
inline constexpr SeasonalRule kDeSeasonal[] = {
    {BanScope::kMotorwayTrunk, 7, 8, kSat, {7, 0, 20, 0}},
};

// ── Switzerland ──────────────────────────────────────────────────────────
inline constexpr WeeklyRule kChWeekly[] = {
    {BanScope::kAllRoads, kSun, {0, 0, 24, 0}, 0},
    {BanScope::kAllRoads, kEveryDay, {22, 0, 5, 0}, 0},
};
inline constexpr HolidayRule kChHoliday[] = {
    {BanScope::kAllRoads, {0, 0, 24, 0}, 0},
};

// ── Liechtenstein (mirrors CH) ───────────────────────────────────────────
inline constexpr WeeklyRule kLiWeekly[] = {
    {BanScope::kAllRoads, kSun, {0, 0, 24, 0}, 0},
    {BanScope::kAllRoads, kEveryDay, {22, 0, 5, 0}, 0},
};
inline constexpr HolidayRule kLiHoliday[] = {
    {BanScope::kAllRoads, {0, 0, 24, 0}, 0},
};

// ── France ───────────────────────────────────────────────────────────────
inline constexpr WeeklyRule kFrWeekly[] = {
    {BanScope::kAllRoads, kSat, {22, 0, 24, 0}, 0},
    {BanScope::kAllRoads, kSun, {0, 0, 22, 0}, 0},
};
inline constexpr HolidayRule kFrHoliday[] = {
    {BanScope::kAllRoads, {0, 0, 22, 0}, 0},
};
inline constexpr SeasonalRule kFrSeasonal[] = {
    // Approx. Bison Futé summer Saturdays as all Jul–Aug Saturdays 07-19
    {BanScope::kAllRoads, 7, 8, kSat, {7, 0, 19, 0}},
};
inline constexpr EveOfHolidayRule kFrEve[] = {
    {BanScope::kAllRoads, {22, 0, 24, 0}},
};

// ── Hungary ──────────────────────────────────────────────────────────────
inline constexpr WeeklyRule kHuWeekly[] = {
    {BanScope::kAllRoads, kSat, {22, 0, 24, 0}, 0},
    {BanScope::kAllRoads, kSun, {0, 0, 22, 0}, 0},
};
inline constexpr HolidayRule kHuHoliday[] = {
    {BanScope::kAllRoads, {0, 0, 22, 0}, 0},
};
inline constexpr SeasonalRule kHuSeasonal[] = {
    // Jul–Aug Saturday from 15:00 (extends the year-round 22:00 start)
    {BanScope::kAllRoads, 7, 8, kSat, {15, 0, 24, 0}},
};
inline constexpr EveOfHolidayRule kHuEve[] = {
    {BanScope::kAllRoads, {22, 0, 24, 0}},
};

// ── Italy ────────────────────────────────────────────────────────────────
inline constexpr WeeklyRule kItWeekly[] = {
    {BanScope::kAllRoads, kSun, {9, 0, 22, 0}, kItWinterMonths},
    {BanScope::kAllRoads, kSun, {7, 0, 22, 0}, kItSummerMonths},
};
inline constexpr HolidayRule kItHoliday[] = {
    {BanScope::kAllRoads, {9, 0, 22, 0}, kItWinterMonths},
    {BanScope::kAllRoads, {7, 0, 22, 0}, kItSummerMonths},
};

// ── Czechia ──────────────────────────────────────────────────────────────
inline constexpr WeeklyRule kCzWeekly[] = {
    {BanScope::kMotorwayTrunk, kSun, {13, 0, 22, 0}, 0},
};
inline constexpr HolidayRule kCzHoliday[] = {
    {BanScope::kMotorwayTrunk, {13, 0, 22, 0}, 0},
};
inline constexpr SeasonalRule kCzSeasonal[] = {
    {BanScope::kMotorwayTrunk, 7, 8, kFri, {17, 0, 21, 0}},
    {BanScope::kMotorwayTrunk, 7, 8, kSat, {7, 0, 13, 0}},
};

// ── Slovakia ─────────────────────────────────────────────────────────────
inline constexpr WeeklyRule kSkWeekly[] = {
    {BanScope::kMotorwayTrunk, kSun, {0, 0, 22, 0}, 0},
};
inline constexpr HolidayRule kSkHoliday[] = {
    {BanScope::kMotorwayTrunk, {0, 0, 22, 0}, 0},
};
inline constexpr SeasonalRule kSkSeasonal[] = {
    {BanScope::kMotorwayTrunk, 7, 8, kSat, {7, 0, 19, 0}},
};

// Tier A implemented countries. Order does not matter; lookup is linear.
inline constexpr CountryRules kCountrySchedules[] = {
    {"AT", "Europe/Vienna", 7.5f, kAtWeekly, 3, kAtHoliday, 1, nullptr, 0, nullptr, 0},
    {"DE", "Europe/Berlin", 7.5f, kDeWeekly, 1, kDeHoliday, 1, kDeSeasonal, 1, nullptr, 0},
    {"CH", "Europe/Zurich", 3.5f, kChWeekly, 2, kChHoliday, 1, nullptr, 0, nullptr, 0},
    {"LI", "Europe/Zurich", 3.5f, kLiWeekly, 2, kLiHoliday, 1, nullptr, 0, nullptr, 0},
    {"FR", "Europe/Paris", 7.5f, kFrWeekly, 2, kFrHoliday, 1, kFrSeasonal, 1, kFrEve, 1},
    {"HU", "Europe/Budapest", 7.5f, kHuWeekly, 2, kHuHoliday, 1, kHuSeasonal, 1, kHuEve, 1},
    {"IT", "Europe/Rome", 7.5f, kItWeekly, 2, kItHoliday, 2, nullptr, 0, nullptr, 0},
    {"CZ", "Europe/Prague", 7.5f, kCzWeekly, 1, kCzHoliday, 1, kCzSeasonal, 2, nullptr, 0},
    // Europe/Bratislava is a deprecated IANA link; Europe/Prague is the canonical zone.
    {"SK", "Europe/Prague", 7.5f, kSkWeekly, 1, kSkHoliday, 1, kSkSeasonal, 1, nullptr, 0},
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
