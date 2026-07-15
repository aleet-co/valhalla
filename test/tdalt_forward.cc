#include "baldr/datetime.h"
#include "baldr/time_info.h"
#include "gurka.h"
#include "loki/worker.h"
#include "sif/costfactory.h"
#include "sif/lower_bound_cost.h"
#include "test.h"
#include "thor/tdalt.h"
#include "thor/worker.h"
#include "worker.h"

#include <gtest/gtest.h>

#include <memory>

// Unit tests for forward G tree: time-dependent costs, meet detection (μ), tightened π*_b.

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

class TdaltForwardTest : public TimeDependentBidirALT {
public:
  using TimeDependentBidirALT::TimeDependentBidirALT;
  using TimeDependentBidirALT::InitBackwardSearch;
  using TimeDependentBidirALT::SetDestination;
  using TimeDependentBidirALT::SetOrigin;
  using TimeDependentBidirALT::ExpandOneForward;
  using TimeDependentBidirALT::forward_settled_nodes;
};

struct TdaltForwardFixture {
  gurka::map map;
  std::shared_ptr<GraphReader> reader;
  gurka::nodelayout layout;
  std::unique_ptr<TdaltForwardTest> tdalt;

  TdaltForwardFixture(const std::string& ascii_map,
                      const gurka::ways& ways,
                      const std::string& tile_dir,
                      bool with_traffic = false) {
    layout = gurka::detail::map_to_coordinates(ascii_map, 1000);
    map = gurka::buildtiles(layout, ways, {}, {}, tile_dir);
    if (with_traffic) {
      map.config.put("mjolnir.traffic_extract", tile_dir + "/traffic.tar");
      test::build_live_traffic_data(map.config);
      test::customize_live_traffic_data(map.config, [](GraphReader&, TrafficTile&, uint32_t,
                                                        TrafficSpeed* traffic_speed) {
        traffic_speed->overall_encoded_speed = 20 >> 1;
        traffic_speed->encoded_speed1 = 20 >> 1;
        traffic_speed->breakpoint1 = 255;
      });
    }
    reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
    tdalt = std::make_unique<TdaltForwardTest>(map.config);
  }

  mode_costing_t auto_costing(travel_mode_t& mode) {
    Api request;
    ParseApi(R"({"locations":[{"lat":0,"lon":0}],"costing":"auto"})", Options::route, request);
    auto mode_costing = CostFactory().CreateModeCosting(request.options(), mode);
    mode_costing[static_cast<uint32_t>(mode)]->SetGraphReader(reader.get());
    return mode_costing;
  }

  std::pair<Location, Location> correlate(const std::vector<std::string>& waypoints,
                                          const std::string& depart_at = "") {
    loki_worker_t loki_worker(map.config);
    std::string req = R"({"locations":[)";
    for (size_t i = 0; i < waypoints.size(); ++i) {
      if (i > 0) {
        req += ',';
      }
      const auto& ll = layout.at(waypoints[i]);
      req += R"({"lat":)" + std::to_string(ll.lat()) + R"(,"lon":)" + std::to_string(ll.lng());
      if (i == 0 && !depart_at.empty()) {
        req += R"(,"date_time":")" + depart_at + '"';
      }
      req += '}';
    }
    req += R"(],"costing":"auto"})";

    Api request;
    ParseApi(req, Options::route, request);
    loki_worker.route(request);
    thor_worker_t::adjust_locations(request);
    return {request.options().locations(0), request.options().locations(1)};
  }

  size_t run_forward_until_exhausted(const std::vector<std::string>& waypoints,
                                     const std::string& depart_at) {
    const GraphId origin_node = gurka::findNode(*reader, layout, waypoints.front());
    const GraphId dest_node = gurka::findNode(*reader, layout, waypoints.back());
    auto [origin, dest] = correlate(waypoints, depart_at);
    travel_mode_t mode{};
    const auto mode_costing = auto_costing(mode);

    tdalt->Clear();
    tdalt->InitBackwardSearch(*reader, layout.at(waypoints.front()), layout.at(waypoints.back()),
                              origin_node, dest_node, mode_costing, mode);
    tdalt->SetDestination(*reader, dest);
    DateTime::tz_sys_info_cache_t tz_cache;
    const auto time_info = TimeInfo::make(origin, *reader, &tz_cache);
    if (!time_info.valid) {
      return 0;
    }
    tdalt->SetOrigin(*reader, origin, time_info);

    while (tdalt->ExpandOneForward(*reader)) {
    }
    return tdalt->forward_settled_nodes().size();
  }
};

} // namespace

TEST(TdaltForward, settle_count_on_line_graph_depart_at) {
  const std::string ascii_map = R"(
    A----B----C
  )";
  const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"maxspeed", "60"}}},
                            {"BC", {{"highway", "primary"}, {"maxspeed", "60"}}}};

  TdaltForwardFixture fixture(ascii_map, ways, "test/data/tdalt_forward_line");

  const size_t settled_count =
      fixture.run_forward_until_exhausted({"A", "C"}, "2026-07-14T08:00");
  EXPECT_GE(settled_count, 2u);

  const GraphId node_b = gurka::findNode(*fixture.reader, fixture.layout, "B");
  const GraphId node_a = gurka::findNode(*fixture.reader, fixture.layout, "A");
  const GraphId node_c = gurka::findNode(*fixture.reader, fixture.layout, "C");
  EXPECT_TRUE(fixture.tdalt->forward_settled_nodes().count(node_b.value) > 0);
  EXPECT_TRUE(fixture.tdalt->forward_settled_nodes().count(node_a.value) > 0);
  EXPECT_TRUE(fixture.tdalt->forward_settled_nodes().count(node_c.value) > 0);
}

TEST(TdaltForward, forward_edge_cost_exceeds_lower_bound_with_traffic) {
  const std::string ascii_map = R"(
    A----B
  )";
  const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"maxspeed", "60"}}}};

  TdaltForwardFixture fixture(ascii_map, ways, "test/data/tdalt_forward_traffic", true);

  const GraphId origin_node = gurka::findNode(*fixture.reader, fixture.layout, "A");
  const GraphId dest_node = gurka::findNode(*fixture.reader, fixture.layout, "B");
  auto [origin, dest] = fixture.correlate({"A", "B"}, "2026-07-14T08:00");
  travel_mode_t mode{};
  const auto mode_costing = fixture.auto_costing(mode);

  fixture.tdalt->Clear();
  fixture.tdalt->InitBackwardSearch(*fixture.reader, fixture.layout.at("A"), fixture.layout.at("B"),
                                    origin_node, dest_node, mode_costing, mode);
  fixture.tdalt->SetDestination(*fixture.reader, dest);
  DateTime::tz_sys_info_cache_t tz_cache;
  const auto time_info = TimeInfo::make(origin, *fixture.reader, &tz_cache);
  ASSERT_TRUE(time_info.valid);
  ASSERT_TRUE(time_info.valid);
  fixture.tdalt->SetOrigin(*fixture.reader, origin, time_info);

  ASSERT_TRUE(fixture.tdalt->ExpandOneForward(*fixture.reader));

  GraphId edge_ab;
  const DirectedEdge* directededge = nullptr;
  std::tie(edge_ab, directededge, std::ignore, std::ignore) =
      gurka::findEdge(*fixture.reader, fixture.layout, "AB", "B");
  ASSERT_NE(directededge, nullptr);
  const auto tile = fixture.reader->GetGraphTile(edge_ab);
  ASSERT_NE(tile, nullptr);

  uint8_t flow_sources = 0;
  const Cost td_cost =
      mode_costing[static_cast<uint32_t>(mode)]->EdgeCost(directededge, edge_ab, tile, time_info,
                                                            flow_sources);
  const float lower_bound = LowerBoundCost::seconds(directededge);

  EXPECT_GT(td_cost.secs, lower_bound);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
