#include "gurka/gurka.h"
#include "test/test.h"

#include "baldr/graphreader.h"
#include "thor/cch/cch_graph.h"
#include "thor/region_grid/region_grid.h"

#include <string>

using namespace valhalla;

namespace {

class RegionGridGurkaTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;
    const std::string ascii_map = R"(
      A----B----C
      |         |
      D----E----F
    )";
    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}, {"oneway", "no"}}},
        {"BC", {{"highway", "primary"}, {"oneway", "no"}}},
        {"AD", {{"highway", "primary"}, {"oneway", "no"}}},
        {"CF", {{"highway", "primary"}, {"oneway", "no"}}},
        {"DE", {{"highway", "primary"}, {"oneway", "no"}}},
        {"EF", {{"highway", "primary"}, {"oneway", "no"}}},
    };
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_region_grid");
  }
};

gurka::map RegionGridGurkaTest::map = {};

} // namespace

TEST_F(RegionGridGurkaTest, BuildFromTiles) {
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  auto graph = thor::cch::BuildTruckGraph(reader, {0, 1, 2}, 7, /*hgv_only=*/false);
  ASSERT_FALSE(graph.nodes.empty());
  ASSERT_FALSE(graph.edges.empty());

  thor::region_grid::RegionGridOptions opts;
  opts.target_regions = 4;
  opts.base_h3_res = 7;
  opts.max_h3_res = 8;
  auto result = thor::region_grid::BuildRegionGrid(graph, opts);

  ASSERT_GE(result.regions.size(), 1u);
  ASSERT_EQ(result.node_regions.size(), graph.nodes.size());
  for (const auto& a : result.node_regions) {
    EXPECT_LT(a.region_id, result.regions.size());
  }
  for (const auto& r : result.regions) {
    EXPECT_LT(r.rep_node_index, graph.nodes.size());
  }
}
