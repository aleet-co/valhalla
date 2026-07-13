#include "baldr/datetime.h"
#include "baldr/time_info.h"
#include "loki/worker.h"
#include "mjolnir/tdalt_landmarks_builder.h"
#include "sif/costfactory.h"
#include "test.h"
#include "thor/tdalt.h"
#include "thor/unidirectional_astar.h"
#include "thor/worker.h"
#include "worker.h"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

class TdaltGurkaTest : public TimeDependentBidirALT {
public:
  using TimeDependentBidirALT::TimeDependentBidirALT;
  using TimeDependentBidirALT::InitBackwardSearch;
  using TimeDependentBidirALT::SetDestination;
  using TimeDependentBidirALT::SetDestinationBackward;
  using TimeDependentBidirALT::SetOrigin;
  using TimeDependentBidirALT::ExpandOneForward;
  using TimeDependentBidirALT::BaseBackwardPotential;
  using TimeDependentBidirALT::TightenedBackwardPotential;
};

class TdaltTest : public ::testing::Test {
protected:
  static boost::property_tree::ptree config_;
  static std::shared_ptr<GraphReader> reader_;

  static void SetUpTestSuite() {
    config_ = test::make_config(VALHALLA_BUILD_DIR "test/data/utrecht_tiles");
    config_.put("mjolnir.tdalt.landmarks_file",
                VALHALLA_BUILD_DIR "test/data/utrecht_tiles/tdalt_landmarks.bin");
    config_.put("mjolnir.tdalt.landmark_count", 16u);
    build_tdalt_landmarks(config_);
    reader_ = test::make_clean_graphreader(config_.get_child("mjolnir"));
  }

  static std::pair<Location, Location> correlate() {
    loki_worker_t loki_worker(config_);
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

  static mode_costing_t auto_costing(travel_mode_t& mode) {
    Api request;
    ParseApi(R"({"locations":[{"lat":0,"lon":0}],"costing":"auto"})", Options::route, request);
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
};

boost::property_tree::ptree TdaltTest::config_{};
std::shared_ptr<GraphReader> TdaltTest::reader_{};

} // namespace

TEST_F(TdaltTest, matches_timedep_forward_depart_at) {
  auto [origin, dest] = correlate();

  travel_mode_t mode{};
  const auto mode_costing = auto_costing(mode);
  Options options;
  options.set_date_time_type(Options::depart_at);
  options.set_costing_type(Costing::auto_);

  TimeDependentBidirALT tdalt(config_);
  ASSERT_TRUE(tdalt.landmarks_available());

  auto origin_copy = origin;
  auto dest_copy = dest;
  const auto tdalt_paths = tdalt.GetBestPath(origin_copy, dest_copy, *reader_, mode_costing, mode,
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

  EXPECT_EQ(tdalt_paths.front().size(), timedep_paths.front().size());
  EXPECT_NEAR(path_duration(tdalt_paths.front()), path_duration(timedep_paths.front()), 0.1f);
}

TEST_F(TdaltTest, tightened_backward_potential_at_least_base) {
  auto [origin, dest] = correlate();
  auto origin_mutable = origin;

  travel_mode_t mode{};
  const auto mode_costing = auto_costing(mode);

  GraphId origin_node;
  GraphId destination_node;
  for (const auto& edge : origin.correlation().edges()) {
    const GraphId edgeid(edge.graph_id());
    const graph_tile_ptr tile = reader_->GetGraphTile(edgeid);
    if (tile == nullptr) {
      continue;
    }
    const DirectedEdge* directededge = tile->directededge(edgeid);
    if (directededge == nullptr) {
      continue;
    }
    origin_node = directededge->endnode();
    break;
  }
  for (const auto& edge : dest.correlation().edges()) {
    const GraphId edgeid(edge.graph_id());
    const graph_tile_ptr tile = reader_->GetGraphTile(edgeid);
    if (tile == nullptr) {
      continue;
    }
    const DirectedEdge* directededge = tile->directededge(edgeid);
    if (directededge == nullptr) {
      continue;
    }
    graph_tile_ptr opp_tile = tile;
    const DirectedEdge* opp_dir_edge = nullptr;
    const GraphId opp_edge_id = reader_->GetOpposingEdgeId(edgeid, opp_dir_edge, opp_tile);
    if (opp_dir_edge == nullptr) {
      continue;
    }
    destination_node = opp_dir_edge->endnode();
    break;
  }
  ASSERT_TRUE(origin_node.is_valid());
  ASSERT_TRUE(destination_node.is_valid());

  const PointLL origin_ll(origin.correlation().edges(0).ll().lng(),
                          origin.correlation().edges(0).ll().lat());
  const PointLL dest_ll(dest.correlation().edges(0).ll().lng(),
                        dest.correlation().edges(0).ll().lat());

  TdaltGurkaTest tdalt(config_);
  ASSERT_TRUE(tdalt.landmarks_available());

  tdalt.Clear();
  tdalt.InitBackwardSearch(*reader_, origin_ll, dest_ll, origin_node, destination_node, mode_costing,
                           mode);
  tdalt.SetDestination(*reader_, dest);
  tdalt.SetDestinationBackward(*reader_, dest);

  DateTime::tz_sys_info_cache_t tz_cache;
  const auto time_info = TimeInfo::make(origin_mutable, *reader_, &tz_cache);
  ASSERT_TRUE(time_info.valid);
  tdalt.SetOrigin(*reader_, origin_mutable, time_info);

  for (int i = 0; i < 50 && tdalt.ExpandOneForward(*reader_); ++i) {
  }

  const std::vector<GraphId> sample_nodes = {origin_node, destination_node};
  for (const GraphId& node : sample_nodes) {
    const float base = tdalt.BaseBackwardPotential(node);
    const float tightened = tdalt.TightenedBackwardPotential(*reader_, node);
    EXPECT_GE(tightened, base);
  }
}
