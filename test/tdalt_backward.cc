#include "gurka.h"
#include "loki/worker.h"
#include "sif/costfactory.h"
#include "test.h"
#include "thor/tdalt.h"
#include "thor/worker.h"
#include "worker.h"

#include <gtest/gtest.h>

#include <memory>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

class TdaltBackwardTest : public TimeDependentBidirALT {
public:
  using TimeDependentBidirALT::TimeDependentBidirALT;
  using TimeDependentBidirALT::InitBackwardSearch;
  using TimeDependentBidirALT::SetDestinationBackward;
  using TimeDependentBidirALT::ExpandOneBackward;
  using TimeDependentBidirALT::MarkForwardSettled;
  using TimeDependentBidirALT::backward_settled_nodes;
};

struct TdaltBackwardFixture {
  gurka::map map;
  std::shared_ptr<GraphReader> reader;
  gurka::nodelayout layout;
  std::unique_ptr<TdaltBackwardTest> tdalt;

  TdaltBackwardFixture(const std::string& ascii_map,
                       const gurka::ways& ways,
                       const std::string& tile_dir) {
    layout = gurka::detail::map_to_coordinates(ascii_map, 1000);
    map = gurka::buildtiles(layout, ways, {}, {}, tile_dir);
    reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
    tdalt = std::make_unique<TdaltBackwardTest>(map.config);
  }

  mode_costing_t auto_costing(travel_mode_t& mode) {
    Api request;
    ParseApi(R"({"locations":[{"lat":0,"lon":0}],"costing":"auto"})", Options::route, request);
    auto mode_costing = CostFactory().CreateModeCosting(request.options(), mode);
    mode_costing[static_cast<uint32_t>(mode)]->SetGraphReader(reader.get());
    return mode_costing;
  }

  std::pair<Location, Location> correlate(const std::vector<std::string>& waypoints) {
    loki_worker_t loki_worker(map.config);
    std::string req = R"({"locations":[)";
    for (size_t i = 0; i < waypoints.size(); ++i) {
      if (i > 0) {
        req += ',';
      }
      const auto& ll = layout.at(waypoints[i]);
      req += R"({"lat":)" + std::to_string(ll.lat()) + R"(,"lon":)" + std::to_string(ll.lng()) + '}';
    }
    req += R"(],"costing":"auto"})";

    Api request;
    ParseApi(req, Options::route, request);
    loki_worker.route(request);
    thor_worker_t::adjust_locations(request);
    return {request.options().locations(0), request.options().locations(1)};
  }

  size_t run_backward_until_exhausted(const std::vector<std::string>& waypoints) {
    const GraphId origin_node = gurka::findNode(*reader, layout, waypoints.front());
    const GraphId dest_node = gurka::findNode(*reader, layout, waypoints.back());
    const auto [origin, dest] = correlate(waypoints);
    travel_mode_t mode{};
    const auto mode_costing = auto_costing(mode);

    tdalt->Clear();
    tdalt->InitBackwardSearch(*reader, layout.at(waypoints.front()), layout.at(waypoints.back()),
                              origin_node, dest_node, mode_costing, mode);
    tdalt->SetDestinationBackward(*reader, dest);
    while (tdalt->ExpandOneBackward(*reader)) {
    }
    return tdalt->backward_settled_nodes().size();
  }
};

} // namespace

TEST(TdaltBackward, settle_count_on_line_graph) {
  const std::string ascii_map = R"(
    A----B----C
  )";
  const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"maxspeed", "60"}}},
                            {"BC", {{"highway", "primary"}, {"maxspeed", "60"}}}};

  TdaltBackwardFixture fixture(ascii_map, ways, "test/data/tdalt_backward_line");

  const size_t settled_count = fixture.run_backward_until_exhausted({"A", "C"});
  EXPECT_GE(settled_count, 2u);

  const GraphId node_b = gurka::findNode(*fixture.reader, fixture.layout, "B");
  const GraphId node_a = gurka::findNode(*fixture.reader, fixture.layout, "A");
  const GraphId node_c = gurka::findNode(*fixture.reader, fixture.layout, "C");
  EXPECT_TRUE(fixture.tdalt->backward_settled_nodes().count(node_b.value) > 0);
  EXPECT_TRUE(fixture.tdalt->backward_settled_nodes().count(node_a.value) > 0);
  EXPECT_TRUE(fixture.tdalt->backward_settled_nodes().count(node_c.value) > 0);
}

TEST(TdaltBackward, prune_at_forward_settled_node) {
  const std::string ascii_map = R"(
    A----B----C----D
  )";
  const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"maxspeed", "60"}}},
                            {"BC", {{"highway", "primary"}, {"maxspeed", "60"}}},
                            {"CD", {{"highway", "primary"}, {"maxspeed", "60"}}}};

  TdaltBackwardFixture fixture(ascii_map, ways, "test/data/tdalt_backward_prune");

  const GraphId origin_node = gurka::findNode(*fixture.reader, fixture.layout, "A");
  const GraphId dest_node = gurka::findNode(*fixture.reader, fixture.layout, "D");
  const GraphId node_b = gurka::findNode(*fixture.reader, fixture.layout, "B");
  const GraphId node_a = gurka::findNode(*fixture.reader, fixture.layout, "A");
  const GraphId node_c = gurka::findNode(*fixture.reader, fixture.layout, "C");
  const auto [origin, dest] = fixture.correlate({"A", "D"});

  travel_mode_t mode{};
  const auto mode_costing = fixture.auto_costing(mode);

  fixture.tdalt->Clear();
  fixture.tdalt->InitBackwardSearch(*fixture.reader, fixture.layout.at("A"), fixture.layout.at("D"),
                                    origin_node, dest_node, mode_costing, mode);
  fixture.tdalt->SetDestinationBackward(*fixture.reader, dest);
  fixture.tdalt->MarkForwardSettled(node_b);

  while (fixture.tdalt->ExpandOneBackward(*fixture.reader)) {
  }

  EXPECT_TRUE(fixture.tdalt->backward_settled_nodes().count(node_c.value) > 0);
  EXPECT_TRUE(fixture.tdalt->backward_settled_nodes().count(node_b.value) > 0);
  EXPECT_TRUE(fixture.tdalt->backward_settled_nodes().count(node_a.value) == 0);
  EXPECT_EQ(fixture.tdalt->backward_settled_nodes().size(), 2u);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
