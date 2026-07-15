#include "gurka.h"
#include "loki/worker.h"
#include "mjolnir/tdalt_landmarks_builder.h"
#include "sif/costfactory.h"
#include "test.h"
#include "thor/tdalt.h"
#include "thor/unidirectional_astar.h"
#include "thor/worker.h"
#include "worker.h"

#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>
#include <vector>

// Integration: TDALT with truck_ban costing should match TimeDepForward on same depart_at request.

#if !defined(VALHALLA_BUILD_DIR)
#define VALHALLA_BUILD_DIR
#endif

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::loki;
using namespace valhalla::mjolnir;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

// Europe (Utrecht) fixture with truck_ban costing. Verifies TDALT and TimeDepForward
// agree on path shape and duration for depart_at routing.
//
// EC2 matrix benchmark (not run here): enable thor.tdalt.enabled on staging, then:
//   export VALHALLA_API_URL=http://...
//   export TRUCK_BAN_BENCHMARK_SUITE=matrix
//   poetry run python scripts/benchmark_truck_ban_cache.py

class TdaltTruckBanTest : public ::testing::Test {
protected:
  static boost::property_tree::ptree config_;
  static std::shared_ptr<GraphReader> reader_;

  static void SetUpTestSuite() {
    setenv("TRUCK_BAN_CACHE_ENABLED", "0", 1);

    config_ = test::make_config(VALHALLA_BUILD_DIR "test/data/utrecht_tiles");
    config_.put("mjolnir.tdalt.landmarks_file",
                VALHALLA_BUILD_DIR "test/data/utrecht_tiles/tdalt_landmarks.bin");
    config_.put("mjolnir.tdalt.landmark_count", 16u);
    build_tdalt_landmarks(config_);
    reader_ = test::make_clean_graphreader(config_.get_child("mjolnir"));
  }

  static std::pair<Location, Location> correlate() {
    loki_worker_t loki_worker(config_);
    // Correlate with auto (loki); route with truck_ban costing below.
    const auto request_json = R"({
      "locations":[
        {"lat":52.079079,"lon":5.115197,"date_time":"2018-06-28T07:00"},
        {"lat":52.078937,"lon":5.115321}
      ],
      "costing":"auto"
    })";

    Api request;
    ParseApi(request_json, Options::route, request);
    loki_worker.route(request);
    thor_worker_t::adjust_locations(request);
    return {request.options().locations(0), request.options().locations(1)};
  }

  static mode_costing_t truck_ban_costing(travel_mode_t& mode) {
    Api request;
    ParseApi(R"({
      "locations":[{"lat":0,"lon":0}],
      "costing":"truck_ban",
      "costing_options":{"truck_ban":{"weight":18}}
    })",
             Options::route, request);
    auto mode_costing = CostFactory().CreateModeCosting(request.options(), mode);
    mode_costing[static_cast<uint32_t>(mode)]->SetGraphReader(reader_.get());
    return mode_costing;
  }

  static float path_duration(const std::vector<PathInfo>& path) {
    if (path.empty()) {
      return 0.f;
    }
    return path.back().elapsed_cost.secs;
  }

  static std::vector<GraphId> path_edge_ids(const std::vector<PathInfo>& path) {
    std::vector<GraphId> edge_ids;
    edge_ids.reserve(path.size());
    for (const auto& info : path) {
      edge_ids.push_back(info.edgeid);
    }
    return edge_ids;
  }
};

boost::property_tree::ptree TdaltTruckBanTest::config_{};
std::shared_ptr<GraphReader> TdaltTruckBanTest::reader_{};

} // namespace

TEST_F(TdaltTruckBanTest, matches_timedep_forward_truck_ban_depart_at) {
  auto [origin, dest] = correlate();

  travel_mode_t mode{};
  const auto mode_costing = truck_ban_costing(mode);
  Options options;
  options.set_date_time_type(Options::depart_at);
  options.set_costing_type(Costing::truck_ban);

  TimeDependentBidirALT tdalt(config_);
  ASSERT_TRUE(tdalt.landmarks_available());

  auto tdalt_origin = origin;
  auto tdalt_dest = dest;
  const auto tdalt_paths = tdalt.GetBestPath(tdalt_origin, tdalt_dest, *reader_, mode_costing, mode,
                                             options);
  ASSERT_EQ(tdalt_paths.size(), 1u);
  ASSERT_FALSE(tdalt_paths.front().empty());

  TimeDepForward timedep_forward;
  auto timedep_origin = origin;
  auto timedep_dest = dest;
  const auto timedep_paths =
      timedep_forward.GetBestPath(timedep_origin, timedep_dest, *reader_, mode_costing, mode,
                                  options);
  ASSERT_EQ(timedep_paths.size(), 1u);
  ASSERT_FALSE(timedep_paths.front().empty());

  EXPECT_EQ(path_edge_ids(tdalt_paths.front()), path_edge_ids(timedep_paths.front()));
  EXPECT_NEAR(path_duration(tdalt_paths.front()), path_duration(timedep_paths.front()), 0.1f);
}
