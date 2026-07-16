#ifndef VALHALLA_THOR_CCH_PROFILE_H_
#define VALHALLA_THOR_CCH_PROFILE_H_

#include <array>
#include <cstddef>
#include <cstdint>

namespace valhalla {
namespace thor {
namespace cch {

constexpr int kSlotSeconds = 900;
constexpr int kWeekSeconds = 7 * 24 * 3600; // 604800
constexpr int kNumSlots = kWeekSeconds / kSlotSeconds; // 672
constexpr int kNumWords = (kNumSlots + 63) / 64; // 11
constexpr int64_t kEpochToMondayOffset = 259200; // 3 days: Thu epoch -> Mon-aligned week

// Weekly-periodic UTC forbidden-departure bitmask (one bit per 15-min slot).
struct WeeklyMask {
  std::array<uint64_t, kNumWords> words{};
};

// Round a duration to the nearest number of weekly slots (mod kNumSlots).
size_t seconds_to_slots(int64_t seconds);

// Slot index of an absolute UTC second-of-week value.
size_t slot_of(int64_t sow_seconds);

// Monday-00:00-UTC-aligned second-of-week for a Unix epoch timestamp.
int64_t utc_second_of_week(int64_t epoch_utc);

void set_slot(WeeklyMask& m, size_t slot);
bool is_forbidden(const WeeklyMask& m, int64_t sow_seconds);
bool empty(const WeeklyMask& m);

WeeklyMask union_mask(const WeeklyMask& a, const WeeklyMask& b);

// Rotate so bit p moves to p - slots (mod kNumSlots): "was it banned `slots`
// after departure?". Used to compose the downstream segment's mask.
WeeklyMask shift_earlier(const WeeklyMask& m, size_t slots);

uint32_t popcount(const WeeklyMask& m);
uint32_t count_runs(const WeeklyMask& m); // number of contiguous forbidden runs (|B|)

// A (T, B) profile: static travel time + forbidden-departure mask.
struct Profile {
  uint32_t time_s = 0;
  WeeklyMask forbidden;
};

// T_uw = T_uv + T_vw ; B_uw = B_uv ∪ shift_earlier(B_vw, T_uv).
Profile compose(const Profile& uv, const Profile& vw);

} // namespace cch
} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_CCH_PROFILE_H_
