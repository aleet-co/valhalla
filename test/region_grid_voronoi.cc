#include "test.h"

#include "thor/cch/cch_graph.h"
#include "thor/region_grid/h3_partition.h"
#include "thor/region_grid/medoid.h"
#include "thor/region_grid/network_voronoi.h"
#include "thor/region_grid/region_grid.h"

#include <limits>
#include <vector>

using namespace valhalla;
using namespace valhalla::thor;

namespace {

cch::CchGraph make_branch_graph() {
  // Line 0-1-2-3 in AT, plus spur 1-4. Two medoids at 0 and 3.
  cch::CchGraph g;
  const int n = 5;
  g.nodes.resize(n);
  for (int i = 0; i < n; ++i) {
    g.nodes[i].graph_id = static_cast<uint64_t>(i + 1);
    g.nodes[i].lat = 48.0 + i * 0.01;
    g.nodes[i].lon = 16.0;
    g.nodes[i].country = "AT";
    g.set_index(g.nodes[i].graph_id, static_cast<uint32_t>(i));
  }
  g.nodes[4].lat = 48.02;
  g.nodes[4].lon = 16.05;

  auto add = [&](uint32_t u, uint32_t v, uint32_t t) {
    g.edges.push_back({u, v, t, 0});
    g.edges.push_back({v, u, t, 0});
  };
  add(0, 1, 10);
  add(1, 2, 10);
  add(2, 3, 10);
  add(1, 4, 5);
  g.build_csr();
  return g;
}

} // namespace

TEST(RegionGridVoronoi, OwnershipFromMedoids) {
  auto g = make_branch_graph();
  std::vector<uint32_t> medoids = {0, 3};
  std::vector<uint32_t> region_ids = {10, 20};
  auto vr = region_grid::ComputeNetworkVoronoi(g, medoids, region_ids);

  ASSERT_EQ(vr.node_regions.size(), g.nodes.size());
  EXPECT_EQ(vr.node_regions[0].region_id, 10u);
  EXPECT_EQ(vr.node_regions[0].time_to_rep_s, 0u);
  EXPECT_EQ(vr.node_regions[3].region_id, 20u);
  EXPECT_EQ(vr.node_regions[3].time_to_rep_s, 0u);

  // Node 1 is 10s from 0 and 20s from 3 → region 10
  EXPECT_EQ(vr.node_regions[1].region_id, 10u);
  EXPECT_EQ(vr.node_regions[1].time_to_rep_s, 10u);

  // Node 2 is 20s from 0 and 10s from 3 → region 20
  EXPECT_EQ(vr.node_regions[2].region_id, 20u);
  EXPECT_EQ(vr.node_regions[2].time_to_rep_s, 10u);

  // Spur node 4 attaches via 1 → region 10, time 15
  EXPECT_EQ(vr.node_regions[4].region_id, 10u);
  EXPECT_EQ(vr.node_regions[4].time_to_rep_s, 15u);
  EXPECT_EQ(vr.euclidean_fallback_count, 0u);
}

TEST(RegionGridVoronoi, CountryBarrier) {
  cch::CchGraph g;
  g.nodes.resize(4);
  for (uint32_t i = 0; i < 4; ++i) {
    g.nodes[i].graph_id = i + 1;
    g.nodes[i].lat = 48.0;
    g.nodes[i].lon = 16.0 + i * 0.1;
    g.set_index(g.nodes[i].graph_id, i);
  }
  g.nodes[0].country = "AT";
  g.nodes[1].country = "AT";
  g.nodes[2].country = "DE";
  g.nodes[3].country = "DE";
  // Cross-border edge exists but must not transfer ownership.
  g.edges.push_back({0, 1, 5, 0});
  g.edges.push_back({1, 0, 5, 0});
  g.edges.push_back({1, 2, 5, 0});
  g.edges.push_back({2, 1, 5, 0});
  g.edges.push_back({2, 3, 5, 0});
  g.edges.push_back({3, 2, 5, 0});
  g.build_csr();

  auto vr = region_grid::ComputeNetworkVoronoi(g, {0, 3}, {1, 2});
  EXPECT_EQ(vr.node_regions[0].region_id, 1u);
  EXPECT_EQ(vr.node_regions[1].region_id, 1u);
  EXPECT_EQ(vr.node_regions[2].region_id, 2u);
  EXPECT_EQ(vr.node_regions[3].region_id, 2u);
}

TEST(RegionGridMedoid, GiantComponentMarksMainlandOnly) {
  // Mainland bidirectional chain 0-1-2 plus disconnected island 3-4.
  cch::CchGraph g;
  g.nodes.resize(5);
  for (uint32_t i = 0; i < 5; ++i) {
    g.nodes[i].graph_id = i + 1;
    g.nodes[i].lat = 48.0 + i * 0.01;
    g.nodes[i].lon = 16.0;
    g.nodes[i].country = "PL";
    g.set_index(g.nodes[i].graph_id, i);
  }
  auto add = [&](uint32_t u, uint32_t v) {
    g.edges.push_back({u, v, 10, 0});
    g.edges.push_back({v, u, 10, 0});
  };
  add(0, 1);
  add(1, 2);
  add(3, 4);
  g.build_csr();

  auto in_giant = region_grid::MarkGiantComponentNodes(g);
  ASSERT_EQ(in_giant.size(), 5u);
  EXPECT_TRUE(in_giant[0]);
  EXPECT_TRUE(in_giant[1]);
  EXPECT_TRUE(in_giant[2]);
  EXPECT_FALSE(in_giant[3]);
  EXPECT_FALSE(in_giant[4]);
}

TEST(RegionGridMedoid, GiantSccExcludesOneWaySpur) {
  // Strongly connected triangle 0↔1↔2↔0; node 3 only feeds into it (3→1).
  cch::CchGraph g;
  g.nodes.resize(4);
  for (uint32_t i = 0; i < 4; ++i) {
    g.nodes[i].graph_id = i + 1;
    g.nodes[i].lat = 48.0;
    g.nodes[i].lon = 16.0 + i * 0.01;
    g.nodes[i].country = "PL";
    g.set_index(g.nodes[i].graph_id, i);
  }
  auto add_bidir = [&](uint32_t u, uint32_t v) {
    g.edges.push_back({u, v, 10, 0});
    g.edges.push_back({v, u, 10, 0});
  };
  add_bidir(0, 1);
  add_bidir(1, 2);
  add_bidir(2, 0);
  g.edges.push_back({3, 1, 10, 0}); // one-way into giant SCC
  g.build_csr();

  auto in_giant = region_grid::MarkGiantComponentNodes(g);
  EXPECT_TRUE(in_giant[0]);
  EXPECT_TRUE(in_giant[1]);
  EXPECT_TRUE(in_giant[2]);
  EXPECT_FALSE(in_giant[3]);
}

TEST(RegionGridMedoid, ChooseMedoidsSkipsIslandCell) {
  cch::CchGraph g;
  g.nodes.resize(5);
  for (uint32_t i = 0; i < 5; ++i) {
    g.nodes[i].graph_id = i + 1;
    g.nodes[i].lat = 48.0;
    g.nodes[i].lon = 16.0 + i * 0.01;
    g.nodes[i].country = "PL";
    g.set_index(g.nodes[i].graph_id, i);
  }
  auto add = [&](uint32_t u, uint32_t v) {
    g.edges.push_back({u, v, 10, 0});
    g.edges.push_back({v, u, 10, 0});
  };
  add(0, 1);
  add(1, 2);
  add(3, 4);
  g.build_csr();

  const uint64_t h = region_grid::LatLngToH3(48.0, 16.0, 5);
  ASSERT_NE(h, 0u);

  std::vector<region_grid::CellSeed> cells = {
      {h, "PL", {0, 1, 2}, 30},
      {h, "PL", {3, 4}, 20},
  };
  auto medoids = region_grid::ChooseMedoids(g, cells, /*sample_cap=*/64, /*penalty=*/1000000);
  ASSERT_EQ(medoids.size(), 2u);
  EXPECT_LT(medoids[0], 3u); // mainland
  EXPECT_EQ(medoids[1], std::numeric_limits<uint32_t>::max());
}

TEST(RegionGridMedoid, MixedCellPrefersGiantComponent) {
  // Same H3 cell contains mainland + island nodes; medoid must be mainland.
  cch::CchGraph g;
  g.nodes.resize(4);
  for (uint32_t i = 0; i < 4; ++i) {
    g.nodes[i].graph_id = i + 1;
    g.nodes[i].lat = 52.2;
    g.nodes[i].lon = 21.0 + i * 0.001;
    g.nodes[i].country = "PL";
    g.set_index(g.nodes[i].graph_id, i);
  }
  auto add = [&](uint32_t u, uint32_t v) {
    g.edges.push_back({u, v, 10, 0});
    g.edges.push_back({v, u, 10, 0});
  };
  add(0, 1);
  add(1, 2); // mainland triangle chain
  // node 3 isolated
  g.build_csr();

  const uint64_t h = region_grid::LatLngToH3(52.2, 21.0, 5);
  std::vector<region_grid::CellSeed> cells = {{h, "PL", {0, 1, 2, 3}, 40}};
  auto medoids = region_grid::ChooseMedoids(g, cells, 64, 1000000);
  ASSERT_EQ(medoids.size(), 1u);
  EXPECT_NE(medoids[0], 3u);
  EXPECT_LT(medoids[0], 3u);
}

TEST(RegionGridBuild, DropsIslandOnlyRegions) {
  // Dense mainland line + far disconnected island pair → island cell dropped.
  cch::CchGraph g;
  const int mainland = 40;
  g.nodes.resize(mainland + 2);
  for (int i = 0; i < mainland; ++i) {
    g.nodes[i].graph_id = static_cast<uint64_t>(i + 1);
    g.nodes[i].lat = 48.0 + (i % 8) * 0.08;
    g.nodes[i].lon = 16.0 + (i / 8) * 0.08;
    g.nodes[i].country = "AT";
    g.set_index(g.nodes[i].graph_id, static_cast<uint32_t>(i));
  }
  for (int i = 0; i + 1 < mainland; ++i) {
    g.edges.push_back({static_cast<uint32_t>(i), static_cast<uint32_t>(i + 1), 20, 0});
    g.edges.push_back({static_cast<uint32_t>(i + 1), static_cast<uint32_t>(i), 20, 0});
  }
  // Island far away so it gets its own H3 cell.
  g.nodes[mainland].graph_id = 9001;
  g.nodes[mainland].lat = 41.0;
  g.nodes[mainland].lon = 10.0;
  g.nodes[mainland].country = "AT";
  g.set_index(9001, mainland);
  g.nodes[mainland + 1].graph_id = 9002;
  g.nodes[mainland + 1].lat = 41.01;
  g.nodes[mainland + 1].lon = 10.01;
  g.nodes[mainland + 1].country = "AT";
  g.set_index(9002, mainland + 1);
  g.edges.push_back({static_cast<uint32_t>(mainland), static_cast<uint32_t>(mainland + 1), 5, 0});
  g.edges.push_back({static_cast<uint32_t>(mainland + 1), static_cast<uint32_t>(mainland), 5, 0});
  g.build_csr();

  region_grid::RegionGridOptions opts;
  opts.target_regions = 6;
  auto result = region_grid::BuildRegionGrid(g, opts);
  ASSERT_FALSE(result.regions.empty());
  for (const auto& r : result.regions) {
    EXPECT_LT(r.rep_node_index, static_cast<uint32_t>(mainland));
  }
  // Island nodes are not seeds; Voronoi attaches them via Euclidean fallback.
  EXPECT_EQ(result.euclidean_fallback_count, 2u);
}

TEST(RegionGridBuild, EndToEndSynthetic) {
  cch::CchGraph g;
  const int n = 80;
  g.nodes.resize(n);
  for (int i = 0; i < n; ++i) {
    g.nodes[i].graph_id = static_cast<uint64_t>(i + 1);
    g.nodes[i].lat = 48.0 + (i % 10) * 0.05;
    g.nodes[i].lon = 16.0 + (i / 10) * 0.05;
    g.nodes[i].country = "AT";
    g.set_index(g.nodes[i].graph_id, static_cast<uint32_t>(i));
  }
  for (int i = 0; i + 1 < n; ++i) {
    g.edges.push_back({static_cast<uint32_t>(i), static_cast<uint32_t>(i + 1), 20, 0});
    g.edges.push_back({static_cast<uint32_t>(i + 1), static_cast<uint32_t>(i), 20, 0});
  }
  g.build_csr();
  g.tile_build_hash = 42;

  region_grid::RegionGridOptions opts;
  opts.target_regions = 8;
  auto result = region_grid::BuildRegionGrid(g, opts);
  ASSERT_FALSE(result.regions.empty());
  ASSERT_EQ(result.node_regions.size(), g.nodes.size());
  for (const auto& a : result.node_regions) {
    EXPECT_LT(a.region_id, result.regions.size());
  }
  for (const auto& r : result.regions) {
    EXPECT_LT(r.rep_node_index, g.nodes.size());
    EXPECT_EQ(r.country, "AT");
  }
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
