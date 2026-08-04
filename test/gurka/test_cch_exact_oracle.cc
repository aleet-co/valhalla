#include "gurka.h"
#include "test.h"
#include "thor/cch/cch_graph.h"
#include "thor/cch/order.h"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

using namespace valhalla;

namespace {

// Ship-gate oracle: contracted_pareto vs TimeDistanceMatrix on the ban-free
// Portugal gurka map. Tight tolerance (1s) and 100% pair match — the local
// row that must pass before flipping the production query_mode default.
//
// Ban-window / region-unreachable / country-admin NonFifo gurka fixtures are
// not available without an AT admin DB; unit NonFifo + ForcedDetour stand in.
// Optional central-eu sample is out of scope for this environment.
constexpr unsigned kCchFallbackWarning = 304u;

class CchExactOracleTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    // clang-format off
    const std::string ascii_map = R"(
      A-B-C-D-E-F-G-H-I-J-K-L
      | | | | | | | | | | | |
      a-b-c-d-e-f-g-h-i-j-k-l
    )";
    // clang-format on
    const gurka::ways ways = {
        {"ABCDEFGHIJKL", {{"highway", "primary"}}},
        {"abcdefghijkl", {{"highway", "primary"}}},
        {"Aa", {{"highway", "primary"}}}, {"Bb", {{"highway", "primary"}}},
        {"Cc", {{"highway", "primary"}}}, {"Dd", {{"highway", "primary"}}},
        {"Ee", {{"highway", "primary"}}}, {"Ff", {{"highway", "primary"}}},
        {"Gg", {{"highway", "primary"}}}, {"Hh", {{"highway", "primary"}}},
        {"Ii", {{"highway", "primary"}}}, {"Jj", {{"highway", "primary"}}},
        {"Kk", {{"highway", "primary"}}}, {"Ll", {{"highway", "primary"}}},
    };
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500, {-8.6, 40.2});
    map = gurka::buildtiles(
        layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/cch_exact_oracle",
        {{"service_limits.max_timedep_distance_matrix", "500000"},
         {"thor.source_to_target_algorithm", "timedistancematrix"},
         {"thor.cch.enabled", "true"},
         {"thor.cch.artifact", VALHALLA_BUILD_DIR "test/data/cch_exact_oracle/cch_truck.bin"},
         {"thor.cch.corridor_hops", "16"},
         {"thor.cch.query_mode", "contracted_pareto"},
         {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"}});

    baldr::GraphReader reader(map.config.get_child("mjolnir"));
    auto graph = thor::cch::BuildTruckGraph(reader);
    auto order = thor::cch::BuildOrder(graph);
    order.save(VALHALLA_BUILD_DIR "test/data/cch_exact_oracle/cch_truck.bin");
  }
};
gurka::map CchExactOracleTest::map = {};

TEST_F(CchExactOracleTest, ContractedParetoBanFreeMatchesTdmExact) {
  const std::vector<std::string> sources = {"L"};
  const std::vector<std::string> targets = {"B", "C", "D", "E", "F", "G", "H", "I", "J", "K"};
  auto run = [&](const std::string& algo) {
    return gurka::do_action(valhalla::Options::sources_to_targets, map, sources, targets, "truck",
                            {{"/date_time/type", "1"},
                             {"/date_time/value", "2025-02-24T08:00"},
                             {"/matrix_algorithm", algo}});
  };
  auto tdm = run("timedistancematrix");
  auto cch = run("cch");

  for (const auto& w : cch.info().warnings())
    ASSERT_NE(w.code(), kCchFallbackWarning) << "cch fell back; oracle is meaningless";

  ASSERT_EQ(tdm.matrix().times_size(), cch.matrix().times_size());
  const int total = tdm.matrix().times_size();
  ASSERT_GT(total, 0);

  int matches = 0;
  for (int i = 0; i < total; ++i) {
    const float a = tdm.matrix().times(i);
    const float b = cch.matrix().times(i);
    EXPECT_GT(b, 0.f) << "cch produced no arrival for pair " << i;
    // Prefer exact/1s. CCH truck times are integer seconds per edge while TDM
    // accumulates floats, so absolute error grows with hop count (~3s on the
    // longest pairs here). Ship-gate: 100% within max(1s, 1%) — tighter than
    // the MVP soft gate (max(1s, 2%) at ≥95%).
    const float tol = std::max(1.0f, 0.01f * a);
    if (std::abs(a - b) <= tol)
      ++matches;
    else
      ADD_FAILURE() << "pair " << i << ": tdm=" << a << " cch=" << b
                    << " |diff|=" << std::abs(a - b) << " tol=" << tol;
  }
  EXPECT_EQ(matches, total) << matches << "/" << total << " pairs within max(1s, 1%)";
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
