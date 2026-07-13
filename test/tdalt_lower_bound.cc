#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/time_info.h"
#include "gurka.h"
#include "sif/costfactory.h"
#include "sif/lower_bound_cost.h"
#include "test.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <random>
#include <vector>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {

constexpr uint32_t kSampleEdgeCount = 32;
constexpr uint32_t kTimestampCount = 8;

class TdaltLowerBoundFixture : public ::testing::Test {
protected:
  static gurka::map map_;
  static std::shared_ptr<GraphReader> reader_;
  static cost_ptr_t costing_;

  static void SetUpTestSuite() {
    setenv("TRUCK_BAN_CACHE_ENABLED", "0", 1);

    const std::string ascii_map = R"(
      A----B----C
      |    |    |
      D----E----F
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "motorway"}, {"maxspeed", "100"}}},
        {"BC", {{"highway", "trunk"}, {"maxspeed", "80"}}},
        {"AD", {{"highway", "primary"}, {"maxspeed", "60"}}},
        {"BE", {{"highway", "secondary"}, {"maxspeed", "50"}}},
        {"CF", {{"highway", "tertiary"}, {"maxspeed", "40"}}},
        {"DE", {{"highway", "residential"}, {"maxspeed", "30"}}},
        {"EF", {{"highway", "service"}, {"maxspeed", "20"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 5000);
    map_ = gurka::buildtiles(layout, ways, {}, {}, "test/data/tdalt_lower_bound");
    map_.config.put("mjolnir.traffic_extract", "test/data/tdalt_lower_bound/traffic.tar");

    test::build_live_traffic_data(map_.config);
    test::customize_live_traffic_data(map_.config, [](GraphReader&, TrafficTile&, uint32_t,
                                                      TrafficSpeed* traffic_speed) {
      traffic_speed->overall_encoded_speed = 30 >> 1;
      traffic_speed->encoded_speed1 = 30 >> 1;
      traffic_speed->breakpoint1 = 255;
    });

    test::customize_historical_traffic(map_.config, [](DirectedEdge& edge) {
      edge.set_constrained_flow_speed(35);
      edge.set_free_flow_speed(45);
      std::array<float, kBucketsPerWeek> historical;
      for (size_t i = 0; i < historical.size(); ++i) {
        historical[i] = 25.f + static_cast<float>(i % 10);
      }
      return historical;
    });

    reader_ = test::make_clean_graphreader(map_.config.get_child("mjolnir"));

    Costing options;
    options.set_type(Costing::truck_ban);
    costing_ = CostFactory().Create(options);
    costing_->SetGraphReader(reader_.get());
  }
};

gurka::map TdaltLowerBoundFixture::map_ = {};
std::shared_ptr<GraphReader> TdaltLowerBoundFixture::reader_;
cost_ptr_t TdaltLowerBoundFixture::costing_;

struct SampleEdge {
  const DirectedEdge* edge;
  graph_tile_ptr tile;
  GraphId edge_id;
};

std::vector<SampleEdge> CollectDriveableEdges(GraphReader& reader) {
  std::vector<SampleEdge> edges;
  for (const auto& tile_id : reader.GetTileSet()) {
    auto tile = reader.GetGraphTile(tile_id);
    uint32_t idx = 0;
    for (const auto& edge : tile->GetDirectedEdges()) {
      if (edge.use() == Use::kFerry || edge.use() == Use::kRailFerry) {
        ++idx;
        continue;
      }
      if ((edge.forwardaccess() & kTruckAccess) == 0) {
        ++idx;
        continue;
      }
      if (edge.speed() < kMinValidSpeedKph) {
        ++idx;
        continue;
      }
      edges.push_back({&edge, tile, GraphId(tile_id.tileid(), tile_id.level(), idx)});
      ++idx;
    }
  }
  return edges;
}

std::vector<TimeInfo> WeekTimestamps() {
  static const std::vector<std::string> iso_times = {
      "2026-03-16T08:00", // Monday
      "2026-03-17T12:00", // Tuesday
      "2026-03-18T18:00", // Wednesday
      "2026-03-19T06:00", // Thursday
      "2026-03-20T22:00", // Friday
      "2026-03-21T14:00", // Saturday
      "2026-03-22T10:00", // Sunday
      "2026-03-23T04:00", // Monday
  };

  const uint32_t tz_index = static_cast<uint32_t>(DateTime::get_tz_db().to_index("Europe/Berlin"));
  std::vector<TimeInfo> timestamps;
  timestamps.reserve(iso_times.size());
  for (const auto& iso_time : iso_times) {
    std::string date_time = iso_time;
    auto time_info = TimeInfo::make(date_time, tz_index);
    if (!time_info.valid) {
      ADD_FAILURE() << "Failed to build TimeInfo for " << iso_time;
      continue;
    }
    timestamps.push_back(time_info);
  }
  return timestamps;
}

} // namespace

TEST(TdaltLowerBound, seconds_uses_length_override) {
  DirectedEdge edge;
  edge.set_length(1000);
  edge.set_speed(50);

  EXPECT_FLOAT_EQ(LowerBoundCost::seconds(&edge), LowerBoundCost::seconds(&edge, 1000.f));
  EXPECT_FLOAT_EQ(LowerBoundCost::seconds(&edge, 500.f), 500.f / (50.f * 1000.f / 3600.f));
}

TEST(TdaltLowerBound, seconds_clamps_zero_speed_to_minimum) {
  DirectedEdge edge;
  edge.set_length(1000);
  edge.set_speed(0);

  const float expected = 1000.f / (static_cast<float>(kMinSpeedKph) * 1000.f / 3600.f);
  EXPECT_FLOAT_EQ(LowerBoundCost::seconds(&edge), expected);
}

TEST_F(TdaltLowerBoundFixture, lambda_leq_truck_ban_cost) {
  auto edges = CollectDriveableEdges(*reader_);
  ASSERT_FALSE(edges.empty());

  std::mt19937 rng(42);
  if (edges.size() > kSampleEdgeCount) {
    std::shuffle(edges.begin(), edges.end(), rng);
    edges.resize(kSampleEdgeCount);
  }

  const auto timestamps = WeekTimestamps();
  ASSERT_EQ(timestamps.size(), kTimestampCount);

  for (const auto& sample : edges) {
    const auto* edge = sample.edge;
    const float lambda = LowerBoundCost::seconds(edge);
    ASSERT_GT(lambda, 0.f);

    for (const auto& time_info : timestamps) {
      uint8_t flow_sources = 0;
      const Cost td =
          costing_->EdgeCost(edge, sample.edge_id, sample.tile, time_info, flow_sources);
      EXPECT_LE(lambda, td.secs) << "edge speed=" << edge->speed() << " truck_speed=" << edge->truck_speed()
                                 << " second_of_week=" << time_info.second_of_week;
    }
  }
}

int main(int argc, char** argv) {
  setenv("TRUCK_BAN_CACHE_ENABLED", "0", 1);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
