#include "thor/cch/profile.h"

namespace valhalla {
namespace thor {
namespace cch {

namespace {
inline void mask_off_tail(WeeklyMask& m) {
  // Zero any bits beyond kNumSlots in the last word so popcount/shift are exact.
  constexpr int used_in_last = kNumSlots - (kNumWords - 1) * 64; // bits used in last word
  if (used_in_last < 64) {
    m.words[kNumWords - 1] &= (used_in_last == 0) ? 0ULL : ((1ULL << used_in_last) - 1ULL);
  }
}
} // namespace

size_t seconds_to_slots(int64_t seconds) {
  int64_t slots = (seconds + kSlotSeconds / 2) / kSlotSeconds; // round to nearest
  slots %= kNumSlots;
  if (slots < 0)
    slots += kNumSlots;
  return static_cast<size_t>(slots);
}

size_t slot_of(int64_t sow_seconds) {
  int64_t s = sow_seconds % kWeekSeconds;
  if (s < 0)
    s += kWeekSeconds;
  return static_cast<size_t>(s / kSlotSeconds);
}

int64_t utc_second_of_week(int64_t epoch_utc) {
  int64_t s = (epoch_utc + kEpochToMondayOffset) % kWeekSeconds;
  if (s < 0)
    s += kWeekSeconds;
  return s;
}

void set_slot(WeeklyMask& m, size_t slot) {
  slot %= kNumSlots;
  m.words[slot / 64] |= (1ULL << (slot % 64));
}

bool is_forbidden(const WeeklyMask& m, int64_t sow_seconds) {
  size_t slot = slot_of(sow_seconds);
  return (m.words[slot / 64] >> (slot % 64)) & 1ULL;
}

bool empty(const WeeklyMask& m) {
  for (auto w : m.words)
    if (w)
      return false;
  return true;
}

WeeklyMask union_mask(const WeeklyMask& a, const WeeklyMask& b) {
  WeeklyMask r;
  for (int i = 0; i < kNumWords; ++i)
    r.words[i] = a.words[i] | b.words[i];
  return r;
}

WeeklyMask shift_earlier(const WeeklyMask& m, size_t slots) {
  slots %= kNumSlots;
  WeeklyMask r;
  if (slots == 0)
    return m;
  // Result bit q is set iff original bit (q + slots) mod kNumSlots is set.
  for (int q = 0; q < kNumSlots; ++q) {
    size_t src = (static_cast<size_t>(q) + slots) % kNumSlots;
    if ((m.words[src / 64] >> (src % 64)) & 1ULL)
      r.words[q / 64] |= (1ULL << (q % 64));
  }
  mask_off_tail(r);
  return r;
}

uint32_t popcount(const WeeklyMask& m) {
  uint32_t c = 0;
  for (auto w : m.words)
    c += static_cast<uint32_t>(__builtin_popcountll(w));
  return c;
}

uint32_t count_runs(const WeeklyMask& m) {
  if (empty(m))
    return 0;
  uint32_t runs = 0;
  bool prev = (m.words[(kNumSlots - 1) / 64] >> ((kNumSlots - 1) % 64)) & 1ULL;
  for (int i = 0; i < kNumSlots; ++i) {
    bool cur = (m.words[i / 64] >> (i % 64)) & 1ULL;
    if (cur && !prev)
      ++runs;
    prev = cur;
  }
  return runs;
}

Profile compose(const Profile& uv, const Profile& vw) {
  Profile out;
  out.time_s = uv.time_s + vw.time_s;
  out.forbidden = union_mask(uv.forbidden, shift_earlier(vw.forbidden, seconds_to_slots(uv.time_s)));
  return out;
}

} // namespace cch
} // namespace thor
} // namespace valhalla
