#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "sif/truck_ban_rules.h"

#include <cstdlib>
#include <gtest/gtest.h>

using namespace valhalla::baldr;
using namespace valhalla::sif::truck_ban;

namespace {

uint32_t TzIndex(const std::string& tz_name) {
  return static_cast<uint32_t>(DateTime::get_tz_db().to_index(tz_name));
}

uint64_t LocalEpoch(const std::string& local_date_time, const std::string& tz_name) {
  const auto* tz = DateTime::get_tz_db().from_index(TzIndex(tz_name));
  return DateTime::seconds_since_epoch(local_date_time, tz);
}

} // namespace

TEST(TruckBanRules, AustriaSundayBanBlocksHeavyTrucks) {
  const uint32_t tz = TzIndex("Europe/Vienna");
  const uint64_t current_time = LocalEpoch("2026-03-15T10:00", "Europe/Vienna");

  EXPECT_FALSE(IsEdgeAllowed("AT", current_time, tz, 8.0f, RoadClass::kPrimary));
  EXPECT_TRUE(IsEdgeAllowed("AT", current_time, tz, 3.5f, RoadClass::kPrimary));
}

TEST(TruckBanRules, AustriaSaturdayAfternoonBan) {
  const uint32_t tz = TzIndex("Europe/Vienna");
  const uint64_t before_ban = LocalEpoch("2026-03-14T14:00", "Europe/Vienna");
  const uint64_t during_ban = LocalEpoch("2026-03-14T16:00", "Europe/Vienna");

  EXPECT_TRUE(IsEdgeAllowed("AT", before_ban, tz, 8.0f, RoadClass::kPrimary));
  EXPECT_FALSE(IsEdgeAllowed("AT", during_ban, tz, 8.0f, RoadClass::kPrimary));
}

TEST(TruckBanRules, AustriaNightBan) {
  const uint32_t tz = TzIndex("Europe/Vienna");
  const uint64_t late_evening = LocalEpoch("2026-03-16T23:00", "Europe/Vienna");
  const uint64_t early_morning = LocalEpoch("2026-03-17T04:00", "Europe/Vienna");
  const uint64_t midday = LocalEpoch("2026-03-17T12:00", "Europe/Vienna");

  EXPECT_FALSE(IsEdgeAllowed("AT", late_evening, tz, 8.0f, RoadClass::kPrimary));
  EXPECT_FALSE(IsEdgeAllowed("AT", early_morning, tz, 8.0f, RoadClass::kPrimary));
  EXPECT_TRUE(IsEdgeAllowed("AT", midday, tz, 8.0f, RoadClass::kPrimary));
}

TEST(TruckBanRules, GermanySundayBan) {
  const uint32_t tz = TzIndex("Europe/Berlin");
  const uint64_t current_time = LocalEpoch("2026-03-15T10:00", "Europe/Berlin");

  EXPECT_FALSE(IsEdgeAllowed("DE", current_time, tz, 8.0f, RoadClass::kPrimary));
}

TEST(TruckBanRules, GermanyMondayAllowed) {
  const uint32_t tz = TzIndex("Europe/Berlin");
  const uint64_t current_time = LocalEpoch("2026-03-16T10:00", "Europe/Berlin");

  EXPECT_TRUE(IsEdgeAllowed("DE", current_time, tz, 8.0f, RoadClass::kPrimary));
}

TEST(TruckBanRules, AustriaMondayAllowed) {
  const uint32_t tz = TzIndex("Europe/Vienna");
  const uint64_t current_time = LocalEpoch("2026-03-16T10:00", "Europe/Vienna");

  EXPECT_TRUE(IsEdgeAllowed("AT", current_time, tz, 8.0f, RoadClass::kPrimary));
}

TEST(TruckBanRules, GermanySummerSaturdayMotorwayOnlyBan) {
  const uint32_t tz = TzIndex("Europe/Berlin");
  const uint64_t current_time = LocalEpoch("2026-07-11T10:00", "Europe/Berlin");

  EXPECT_FALSE(IsEdgeAllowed("DE", current_time, tz, 8.0f, RoadClass::kMotorway));
  EXPECT_FALSE(IsEdgeAllowed("DE", current_time, tz, 8.0f, RoadClass::kTrunk));
  EXPECT_TRUE(IsEdgeAllowed("DE", current_time, tz, 8.0f, RoadClass::kPrimary));
}

TEST(TruckBanRules, NonBanCountriesAreAlwaysAllowed) {
  const uint32_t tz = TzIndex("Europe/Warsaw");
  const uint64_t current_time = LocalEpoch("2026-03-15T10:00", "Europe/Warsaw");

  EXPECT_TRUE(IsEdgeAllowed("PL", current_time, tz, 40.0f, RoadClass::kMotorway));
}

TEST(TruckBanRules, ZeroTimeDisablesChecks) {
  const uint32_t tz = TzIndex("Europe/Vienna");
  EXPECT_TRUE(IsEdgeAllowed("AT", 0, tz, 40.0f, RoadClass::kMotorway));
}

TEST(TruckBanRules, MissingGraphTzIndexUsesCountryFallback) {
  const uint64_t monday = LocalEpoch("2026-03-16T10:00", "Europe/Berlin");
  const uint64_t sunday = LocalEpoch("2026-03-15T10:00", "Europe/Berlin");

  // Graph nodes often have timezone index 0; country fallback must still evaluate bans.
  EXPECT_TRUE(IsEdgeAllowed("DE", monday, 0, 8.0f, RoadClass::kPrimary));
  EXPECT_FALSE(IsEdgeAllowed("DE", sunday, 0, 8.0f, RoadClass::kPrimary));
  EXPECT_TRUE(IsEdgeAllowed("AT", monday, 0, 8.0f, RoadClass::kPrimary));
  EXPECT_FALSE(IsEdgeAllowed("AT", sunday, 0, 8.0f, RoadClass::kPrimary));
}

TEST(TruckBanRules, TransitTimeScenarioDepartureBeforeBanEntryDuringBan) {
  // Saturday 14:00 is still allowed in Austria, but 16:00 is not.
  const uint32_t tz = TzIndex("Europe/Vienna");
  const uint64_t departure = LocalEpoch("2026-03-14T14:00", "Europe/Vienna");
  const uint64_t entry_during_ban = LocalEpoch("2026-03-14T16:00", "Europe/Vienna");

  EXPECT_TRUE(IsEdgeAllowed("AT", departure, tz, 8.0f, RoadClass::kPrimary));
  EXPECT_FALSE(IsEdgeAllowed("AT", entry_during_ban, tz, 8.0f, RoadClass::kPrimary));
}

int main(int argc, char** argv) {
  setenv("TRUCK_BAN_CACHE_ENABLED", "0", 1);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
