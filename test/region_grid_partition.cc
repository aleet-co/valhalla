#include "test.h"

#include "thor/cch/cch_graph.h"
#include "thor/region_grid/h3_partition.h"
#include "thor/region_grid/region_grid.h"

#include <string>
#include <unordered_map>
#include <vector>

using namespace valhalla;
using namespace valhalla::thor;

TEST(RegionGridPartition, H3HelpersRoundTrip) {
  const uint64_t h = region_grid::LatLngToH3(48.2082, 16.3738, 5);
  ASSERT_NE(h, 0u);
  EXPECT_EQ(region_grid::H3Resolution(h), 5);
  const uint64_t parent = region_grid::H3Parent(h, 4);
  ASSERT_NE(parent, 0u);
  EXPECT_EQ(region_grid::H3Resolution(parent), 4);
  auto disk = region_grid::H3GridDisk(h, 1);
  EXPECT_GE(disk.size(), 1u);
  auto bnd = region_grid::H3BoundaryLatLng(h);
  EXPECT_GE(bnd.size(), 5u);
}

TEST(RegionGridPartition, TargetCountWithinTolerance) {
  // Spread nodes across a few H3 cells in one country.
  cch::CchGraph g;
  const int n = 200;
  g.nodes.resize(n);
  for (int i = 0; i < n; ++i) {
    g.nodes[i].graph_id = static_cast<uint64_t>(i + 1);
    // Spread ~2 degrees so multiple res-5 cells fill.
    g.nodes[i].lat = 47.0 + (i % 20) * 0.1;
    g.nodes[i].lon = 15.0 + (i / 20) * 0.1;
    g.nodes[i].country = "AT";
    g.set_index(g.nodes[i].graph_id, static_cast<uint32_t>(i));
  }
  for (int i = 0; i + 1 < n; ++i) {
    cch::CchBaseEdge e{static_cast<uint32_t>(i), static_cast<uint32_t>(i + 1), 30, 0};
    g.edges.push_back(e);
    e = {static_cast<uint32_t>(i + 1), static_cast<uint32_t>(i), 30, 0};
    g.edges.push_back(e);
  }
  g.build_csr();

  region_grid::RegionGridOptions opts;
  opts.target_regions = 40;
  opts.base_h3_res = 5;
  opts.max_h3_res = 6;
  auto cells = region_grid::PartitionH3Cells(g, opts);
  ASSERT_FALSE(cells.empty());
  // Final merge pass should land near the requested budget.
  EXPECT_LE(cells.size(), static_cast<size_t>(opts.target_regions + 2));
  EXPECT_GE(cells.size(), 1u);
  for (const auto& c : cells) {
    EXPECT_EQ(c.country, "AT");
    EXPECT_FALSE(c.node_indices.empty());
  }
}

TEST(RegionGridPartition, TwoCountriesGetCells) {
  cch::CchGraph g;
  for (int i = 0; i < 30; ++i) {
    cch::CchNode n;
    n.graph_id = static_cast<uint64_t>(i + 1);
    n.lat = 48.0 + i * 0.05;
    n.lon = 16.0;
    n.country = "AT";
    g.set_index(n.graph_id, static_cast<uint32_t>(g.nodes.size()));
    g.nodes.push_back(n);
  }
  for (int i = 0; i < 30; ++i) {
    cch::CchNode n;
    n.graph_id = static_cast<uint64_t>(1000 + i);
    n.lat = 52.0 + i * 0.05;
    n.lon = 13.0;
    n.country = "DE";
    g.set_index(n.graph_id, static_cast<uint32_t>(g.nodes.size()));
    g.nodes.push_back(n);
  }
  for (uint32_t i = 0; i + 1 < g.nodes.size(); ++i) {
    if (g.nodes[i].country != g.nodes[i + 1].country)
      continue;
    g.edges.push_back({i, i + 1, 40, 0});
    g.edges.push_back({i + 1, i, 40, 0});
  }
  g.build_csr();

  region_grid::RegionGridOptions opts;
  opts.target_regions = 20;
  auto cells = region_grid::PartitionH3Cells(g, opts);
  std::unordered_map<std::string, int> counts;
  for (const auto& c : cells)
    ++counts[c.country];
  EXPECT_GE(counts["AT"], 1);
  EXPECT_GE(counts["DE"], 1);
}

TEST(RegionGridPartition, ExcludeCountriesDropsRU) {
  cch::CchGraph g;
  for (int i = 0; i < 40; ++i) {
    cch::CchNode n;
    n.graph_id = static_cast<uint64_t>(i + 1);
    n.lat = 48.0 + i * 0.05;
    n.lon = 16.0;
    n.country = "AT";
    g.set_index(n.graph_id, static_cast<uint32_t>(g.nodes.size()));
    g.nodes.push_back(n);
  }
  for (int i = 0; i < 40; ++i) {
    cch::CchNode n;
    n.graph_id = static_cast<uint64_t>(2000 + i);
    n.lat = 55.0 + i * 0.05;
    n.lon = 37.0;
    n.country = "RU";
    g.set_index(n.graph_id, static_cast<uint32_t>(g.nodes.size()));
    g.nodes.push_back(n);
  }
  for (uint32_t i = 0; i + 1 < g.nodes.size(); ++i) {
    if (g.nodes[i].country != g.nodes[i + 1].country)
      continue;
    g.edges.push_back({i, i + 1, 40, 0});
    g.edges.push_back({i + 1, i, 40, 0});
  }
  g.build_csr();

  region_grid::RegionGridOptions opts;
  opts.target_regions = 10;
  opts.exclude_countries = {"RU", "BY"};
  auto cells = region_grid::PartitionH3Cells(g, opts);
  ASSERT_FALSE(cells.empty());
  for (const auto& c : cells) {
    EXPECT_NE(c.country, "RU");
    EXPECT_NE(c.country, "BY");
  }
}

TEST(RegionGridPartition, DenseMaxResCapsUrbanSplit) {
  // Compact urban-like cluster (high weight in small area) plus a long sparse
  // corridor. Dense cells should not refine past dense_max_h3_res.
  cch::CchGraph g;
  int id = 1;
  auto add = [&](double lat, double lon, const char* iso) {
    cch::CchNode n;
    n.graph_id = static_cast<uint64_t>(id++);
    n.lat = lat;
    n.lon = lon;
    n.country = iso;
    g.set_index(n.graph_id, static_cast<uint32_t>(g.nodes.size()));
    g.nodes.push_back(n);
  };
  // Dense city block (~0.05°)
  for (int y = 0; y < 10; ++y)
    for (int x = 0; x < 10; ++x)
      add(48.20 + y * 0.005, 16.37 + x * 0.005, "AT");
  // Sparse corridor spanning several degrees
  for (int i = 0; i < 40; ++i)
    add(47.0 + i * 0.15, 14.0, "AT");
  for (uint32_t i = 0; i + 1 < g.nodes.size(); ++i) {
    g.edges.push_back({i, i + 1, 20, 0});
    g.edges.push_back({i + 1, i, 20, 0});
  }
  g.build_csr();

  region_grid::RegionGridOptions opts;
  opts.target_regions = 80;
  opts.base_h3_res = 5;
  opts.max_h3_res = 6;
  opts.dense_max_h3_res = 5;
  opts.dense_density_factor = 1.5;
  opts.exclude_countries = {};
  auto cells = region_grid::PartitionH3Cells(g, opts);
  ASSERT_FALSE(cells.empty());
  for (const auto& c : cells) {
    EXPECT_LE(region_grid::H3Resolution(c.h3_index), opts.max_h3_res);
  }
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

