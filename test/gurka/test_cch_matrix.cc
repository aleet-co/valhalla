#include "gurka.h"
#include "test.h"
#include "thor/cch/cch_graph.h"
#include "thor/cch/contracted_search.h"
#include "thor/cch/customizer.h"
#include "thor/cch/order.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

using namespace valhalla;

namespace {

// The bans-only CCH matrix (Task 6/7) is selected per-request via
// `matrix_algorithm=cch`; the worker (Task 11) only honors it for truck costing
// with a depart-at date_time AND when the CCH artifact actually loads, otherwise
// it emits warning 304 and falls back to TimeDistanceMatrix (Task 10).
//
// This map is placed over Portugal so nodes get a valid timezone from tz.sqlite
// (needed for the depart-time conversion). No admin db is supplied, so no edge
// gets a country ISO and therefore no ban ever fires -- this is deliberately a
// BAN-FREE map, which lets us validate contracted CCH search against
// TimeDistanceMatrix end-to-end (the ban-firing case is exercised by the
// customizer unit tests and the offline prototypes/tch_ban benchmark).
constexpr unsigned kCchFallbackWarning = 304u;

class CchMatrixTest : public ::testing::Test {
protected:
  static gurka::map map;
  static std::string artifact;

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
    // ~Portugal, so nodes resolve a timezone from tz.sqlite.
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500, {-8.6, 40.2});
    map = gurka::buildtiles(
        layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/cch_matrix",
        {{"service_limits.max_timedep_distance_matrix", "500000"},
         {"thor.source_to_target_algorithm", "timedistancematrix"},
         {"thor.cch.enabled", "true"},
         {"thor.cch.artifact", VALHALLA_BUILD_DIR "test/data/cch_matrix/cch_truck.bin"},
         {"thor.cch.query_mode", "contracted_pareto"},
         // Gurka tiles live on level 2; keep filters explicit so production
         // region-grid defaults (0,1 / hgv_only) do not empty this fixture.
         {"thor.cch.levels", "0,1,2"},
         {"thor.cch.max_class", "7"},
         {"thor.cch.hgv_only", "false"},
         {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"}});

    // Build the CCH artifact from the freshly built tiles and persist it where
    // the config points, so CCHMatrix::prepare() actually loads it (a matching
    // tile_build_hash is required, hence building from the same reader/tiles).
    baldr::GraphReader reader(map.config.get_child("mjolnir"));
    thor::cch::TruckGraphOptions truck_opts;
    truck_opts.hgv_only = false;
    auto graph = thor::cch::BuildTruckGraph(reader, {0, 1, 2}, 7, truck_opts);
    auto order = thor::cch::BuildOrder(graph);
    artifact = VALHALLA_BUILD_DIR "test/data/cch_matrix/cch_truck.bin";
    order.save(artifact);
  }
};
gurka::map CchMatrixTest::map = {};
std::string CchMatrixTest::artifact = {};

// 1. truck + depart-time + cch => cch honored (no 304 fallback warning), and
//    the matrix is actually populated (proves we truly ran cch, not a silent
//    no-op).
TEST_F(CchMatrixTest, RequestingCchUsesCchForTruckWithTime) {
  auto api = gurka::do_action(
      valhalla::Options::sources_to_targets, map, {"A"}, {"D"}, "truck",
      {{"/date_time/type", "1"}, {"/date_time/value", "2025-02-24T08:00"},
       {"/matrix_algorithm", "cch"}});

  for (const auto& w : api.info().warnings())
    EXPECT_NE(w.code(), kCchFallbackWarning) << "cch should have been honored, not fall back";

  ASSERT_EQ(api.matrix().times_size(), 1);
  EXPECT_GT(api.matrix().times(0), 0.f) << "cch produced no arrival for A->D";
}

// 2. truck + NO date_time + cch => fallback (304), because cch requires a
//    depart time.
TEST_F(CchMatrixTest, RequestingCchWithoutTimeFallsBack) {
  auto api = gurka::do_action(valhalla::Options::sources_to_targets, map, {"A"}, {"D"}, "truck",
                              {{"/matrix_algorithm", "cch"}});
  bool warned = false;
  for (const auto& w : api.info().warnings())
    warned |= (w.code() == kCchFallbackWarning);
  EXPECT_TRUE(warned) << "expected 304 fallback when no depart time is given";
}

// 3. non-truck (auto) + depart-time + cch => fallback (304), cch is truck-only.
TEST_F(CchMatrixTest, RequestingCchForNonTruckFallsBack) {
  auto api = gurka::do_action(
      valhalla::Options::sources_to_targets, map, {"A"}, {"D"}, "auto",
      {{"/date_time/type", "1"}, {"/date_time/value", "2025-02-24T08:00"},
       {"/matrix_algorithm", "cch"}});
  bool warned = false;
  for (const auto& w : api.info().warnings())
    warned |= (w.code() == kCchFallbackWarning);
  EXPECT_TRUE(warned) << "expected 304 fallback for non-truck costing";
}

// 4. Soft vs-TDM gate for default query_mode=contracted_pareto on this ban-free
//    map (>=95% within max(1s, 2%)). The ship-gate oracle uses a tighter
//    max(60s, 5%) / 100% bar in gurka_cch_exact_oracle.
//
// Geometry note: the CCH MVP snaps every source/target to the end node of its
// first correlated edge, which resolves to the location's *west* graph-node
// neighbor along the top rail (nodes are ordered west->east A..L). By placing
// the source at the east end (L) and all targets to its west (B..K), the
// source's one-node eastward-to-westward offset and each target's offset cancel,
// so cch measures the same node pair TDM does. What remains is only the
// per-edge integer rounding of the CCH truck-time model vs TDM's float
// accumulation (sub-second to a couple seconds).
TEST_F(CchMatrixTest, CchArrivalsMatchTdmWithinTolerance) {
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

  // cch must have been honored (otherwise this compares tdm against tdm).
  for (const auto& w : cch.info().warnings())
    ASSERT_NE(w.code(), kCchFallbackWarning) << "cch fell back; correctness check is meaningless";

  ASSERT_EQ(tdm.matrix().times_size(), cch.matrix().times_size());
  const int total = tdm.matrix().times_size();
  ASSERT_GT(total, 0);
  int matches = 0;
  for (int i = 0; i < total; ++i) {
    const float a = tdm.matrix().times(i);
    const float b = cch.matrix().times(i);
    EXPECT_GT(b, 0.f) << "cch produced no arrival for pair " << i;
    if (std::abs(a - b) <= std::max(1.0f, 0.02f * a))
      ++matches;
  }
  EXPECT_GE(static_cast<double>(matches) / total, 0.95)
      << matches << "/" << total << " pairs within tolerance";
}

// Same ban-free vs-TDM soft gate with query_mode=contracted (Stage 1
// single-label search on the CCH overlay).
TEST_F(CchMatrixTest, ContractedBanFreeMatchesTdmWithinTolerance) {
  const std::string prev_mode = map.config.get<std::string>("thor.cch.query_mode");
  map.config.put("thor.cch.query_mode", "contracted");

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

  map.config.put("thor.cch.query_mode", prev_mode);

  for (const auto& w : cch.info().warnings())
    ASSERT_NE(w.code(), kCchFallbackWarning) << "cch fell back; correctness check is meaningless";

  ASSERT_EQ(tdm.matrix().times_size(), cch.matrix().times_size());
  const int total = tdm.matrix().times_size();
  ASSERT_GT(total, 0);
  int matches = 0;
  for (int i = 0; i < total; ++i) {
    const float a = tdm.matrix().times(i);
    const float b = cch.matrix().times(i);
    EXPECT_GT(b, 0.f) << "cch produced no arrival for pair " << i;
    if (std::abs(a - b) <= std::max(1.0f, 0.02f * a))
      ++matches;
  }
  EXPECT_GE(static_cast<double>(matches) / total, 0.95)
      << matches << "/" << total << " pairs within tolerance";
}

// Stage 3 ship gate: contracted_pareto with RPHAST buckets must match Stage-2
// (same mode, buckets disabled via direct search) within 1e-3s on a multi
// source×target ban-free matrix built from this gurka overlay.
TEST_F(CchMatrixTest, ContractedParetoBucketsMatchStage2) {
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  auto graph = thor::cch::BuildTruckGraph(reader);
  auto order = thor::cch::CchOrder::load(artifact);
  ASSERT_EQ(graph.nodes.size(), order.rank.size());
  order.build_adjacency(graph);
  auto metric = thor::cch::Customize(graph, order);

  // Multi S×T: several top-rail nodes as both sources and targets.
  auto node_of = [&](const std::string& name) -> uint32_t {
    const auto& ll = map.nodes.at(name);
    uint32_t best = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (uint32_t i = 0; i < graph.nodes.size(); ++i) {
      const double dlat = ll.lat() - graph.nodes[i].lat;
      const double dlon = ll.lng() - graph.nodes[i].lon;
      const double d2 = dlat * dlat + dlon * dlon;
      if (d2 < best_d2) {
        best_d2 = d2;
        best = i;
      }
    }
    return best;
  };
  const std::vector<uint32_t> sources = {node_of("A"), node_of("C"), node_of("F"), node_of("L")};
  const std::vector<uint32_t> targets = {node_of("B"), node_of("D"), node_of("G"), node_of("I"),
                                         node_of("K"), node_of("a"), node_of("f"), node_of("l")};

  std::unordered_set<uint32_t> down_allowed;
  thor::cch::RphastBuckets buckets;
  thor::cch::BuildRphastPhaseA(order, metric, targets, down_allowed, &buckets, nullptr);
  ASSERT_FALSE(buckets.empty());

  const int64_t depart_sow = 0;
  int compared = 0;
  for (uint32_t s : sources) {
    std::unordered_map<uint32_t, float> stage2;
    thor::cch::ContractedTdPareto(order, metric, s, targets, depart_sow, &down_allowed, nullptr,
                                  stage2, nullptr, nullptr);
    std::unordered_map<uint32_t, float> stage3;
    thor::cch::ContractedTdPareto(order, metric, s, targets, depart_sow, &down_allowed, &buckets,
                                  stage3, nullptr, nullptr);
    ASSERT_EQ(stage2.size(), stage3.size()) << "source node " << s;
    for (const auto& [t, d2] : stage2) {
      ASSERT_TRUE(stage3.count(t)) << "missing target " << t << " from source " << s;
      EXPECT_NEAR(stage3[t], d2, 1e-3f) << "s=" << s << " t=" << t;
      ++compared;
    }
  }
  EXPECT_GT(compared, 0);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
