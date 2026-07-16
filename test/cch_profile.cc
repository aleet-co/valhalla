#include "thor/cch/profile.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla::thor::cch;

namespace {

TEST(CchProfile, SecondsToSlotsRounds) {
  EXPECT_EQ(seconds_to_slots(0), 0u);
  EXPECT_EQ(seconds_to_slots(900), 1u);
  EXPECT_EQ(seconds_to_slots(1349), 1u);  // rounds down
  EXPECT_EQ(seconds_to_slots(1350), 2u);  // rounds up (nearest)
  EXPECT_EQ(seconds_to_slots(kWeekSeconds), 0u); // wraps mod 672
}

TEST(CchProfile, SetAndForbidden) {
  WeeklyMask m;
  EXPECT_TRUE(empty(m));
  set_slot(m, 100);
  EXPECT_FALSE(empty(m));
  // sow at slot 100 => 100 * 900 seconds
  EXPECT_TRUE(is_forbidden(m, 100 * kSlotSeconds));
  EXPECT_FALSE(is_forbidden(m, 101 * kSlotSeconds));
  // weekly periodicity
  EXPECT_TRUE(is_forbidden(m, 100 * kSlotSeconds + kWeekSeconds));
}

TEST(CchProfile, UnionAndPopcount) {
  WeeklyMask a, b;
  set_slot(a, 3);
  set_slot(b, 3);
  set_slot(b, 200);
  auto u = union_mask(a, b);
  EXPECT_EQ(popcount(u), 2u);
  EXPECT_TRUE(is_forbidden(u, 3 * kSlotSeconds));
  EXPECT_TRUE(is_forbidden(u, 200 * kSlotSeconds));
}

TEST(CchProfile, ShiftEarlierRotates) {
  WeeklyMask m;
  set_slot(m, 10);
  // shift_earlier by 4 slots => bit moves from 10 to 6
  auto s = shift_earlier(m, 4);
  EXPECT_TRUE(is_forbidden(s, 6 * kSlotSeconds));
  EXPECT_FALSE(is_forbidden(s, 10 * kSlotSeconds));
  // shift by a full week is identity
  EXPECT_TRUE(is_forbidden(shift_earlier(m, kNumSlots), 10 * kSlotSeconds));
}

TEST(CchProfile, ShiftEarlierWrapsRing) {
  WeeklyMask m;
  set_slot(m, 2);
  auto s = shift_earlier(m, 5); // 2 - 5 = -3 => 669 on the ring
  EXPECT_TRUE(is_forbidden(s, (kNumSlots - 3) * kSlotSeconds));
}

TEST(CchProfile, CountRuns) {
  WeeklyMask m;
  EXPECT_EQ(count_runs(m), 0u);
  set_slot(m, 5);
  set_slot(m, 6);
  set_slot(m, 7);   // one run
  set_slot(m, 100); // second run
  EXPECT_EQ(count_runs(m), 2u);
}

TEST(CchProfile, CountRunsWrapMerges) {
  WeeklyMask m;
  set_slot(m, 0);
  set_slot(m, kNumSlots - 1); // adjacent across the ring => single run
  EXPECT_EQ(count_runs(m), 1u);
}

TEST(CchProfile, UtcSecondOfWeekMondayAligned) {
  // 1970-01-01 00:00 UTC is Thursday => sow = 3 days.
  EXPECT_EQ(utc_second_of_week(0), 3 * 86400);
  EXPECT_EQ(utc_second_of_week(kWeekSeconds), 3 * 86400);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
