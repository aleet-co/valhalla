#include "thor/cch/cch_graph.h"
#include "thor/cch/nested_dissection.h"
#include "thor/cch/order.h"
#include "test.h"
#include <gtest/gtest.h>
#include <set>

using namespace valhalla::thor::cch;

namespace {

// Build a tiny hand graph: a path 0-1-2-3 (bidirectional), all unit-ish.
CchGraph line_graph() {
  CchGraph g;
  for (int i = 0; i < 4; ++i) {
    CchNode n;
    n.graph_id = i;
    n.lat = 0;
    n.lon = static_cast<double>(i);
    n.country = "PL";
    g.set_index(i, i);
    g.nodes.push_back(n);
  }
  auto add = [&](uint32_t u, uint32_t v) {
    CchBaseEdge e;
    e.u = u;
    e.v = v;
    e.time_s = 60;
    e.roadclass = 2;
    g.edges.push_back(e);
  };
  add(0, 1);
  add(1, 0);
  add(1, 2);
  add(2, 1);
  add(2, 3);
  add(3, 2);
  g.build_csr();
  return g;
}

CchGraph grid_graph(uint32_t w, uint32_t h) {
  CchGraph g;
  auto id = [&](uint32_t x, uint32_t y) { return y * w + x; };
  for (uint32_t y = 0; y < h; ++y) {
    for (uint32_t x = 0; x < w; ++x) {
      CchNode n;
      n.graph_id = id(x, y);
      n.lat = static_cast<double>(y);
      n.lon = static_cast<double>(x);
      n.country = "PL";
      g.set_index(n.graph_id, id(x, y));
      g.nodes.push_back(n);
    }
  }
  auto add = [&](uint32_t u, uint32_t v) {
    CchBaseEdge e;
    e.u = u;
    e.v = v;
    e.time_s = 60;
    e.roadclass = 2;
    g.edges.push_back(e);
  };
  for (uint32_t y = 0; y < h; ++y) {
    for (uint32_t x = 0; x < w; ++x) {
      if (x + 1 < w) {
        add(id(x, y), id(x + 1, y));
        add(id(x + 1, y), id(x, y));
      }
      if (y + 1 < h) {
        add(id(x, y), id(x, y + 1));
        add(id(x, y + 1), id(x, y));
      }
    }
  }
  g.build_csr();
  return g;
}

void expect_valid_order(const CchOrder& order, size_t n) {
  EXPECT_EQ(order.rank.size(), n);
  std::set<uint32_t> ranks(order.rank.begin(), order.rank.end());
  EXPECT_EQ(ranks.size(), n);
  for (uint32_t u = 0; u < order.fwd_adj.size(); ++u) {
    for (auto [v, eid] : order.fwd_adj[u])
      EXPECT_LT(order.rank[u], order.rank[v]);
  }
  for (const auto& sc : order.shortcuts) {
    EXPECT_LT(sc.left, order.num_base_edges + order.shortcuts.size());
    EXPECT_LT(sc.right, order.num_base_edges + order.shortcuts.size());
  }
}

TEST(CchOrder, EveryNodeRanked) {
  auto g = line_graph();
  auto order = BuildOrder(g);
  EXPECT_EQ(order.rank.size(), 4u);
  std::set<uint32_t> ranks(order.rank.begin(), order.rank.end());
  EXPECT_EQ(ranks.size(), 4u); // all distinct 0..3
}

TEST(CchOrder, ShortcutsReferenceChildren) {
  auto g = line_graph();
  auto order = BuildOrder(g);
  for (const auto& sc : order.shortcuts) {
    EXPECT_LT(sc.left, order.num_base_edges + order.shortcuts.size());
    EXPECT_LT(sc.right, order.num_base_edges + order.shortcuts.size());
  }
}

TEST(CchOrder, AdjacencyIsUpwardDownward) {
  auto g = line_graph();
  auto order = BuildOrder(g);
  for (uint32_t u = 0; u < order.fwd_adj.size(); ++u) {
    for (auto [v, eid] : order.fwd_adj[u])
      EXPECT_LT(order.rank[u], order.rank[v]);
  }
}

TEST(CchOrder, SaveLoadRoundTrip) {
  auto g = line_graph();
  auto order = BuildOrder(g);
  const std::string path = "test_cch_order.bin";
  order.save(path);
  auto loaded = CchOrder::load(path);
  EXPECT_EQ(loaded.rank, order.rank);
  EXPECT_EQ(loaded.shortcuts.size(), order.shortcuts.size());
  EXPECT_EQ(loaded.num_base_edges, order.num_base_edges);
  EXPECT_EQ(loaded.tile_build_hash, order.tile_build_hash);
  std::remove(path.c_str());
}

TEST(CchOrder, MultiThreadProducesValidOrder) {
  auto g = line_graph();
  auto o1 = BuildOrder(g, 1);
  auto o4 = BuildOrder(g, 4);
  expect_valid_order(o1, 4);
  expect_valid_order(o4, 4);
  // Deterministic ND on a path → same ranks regardless of thread count.
  EXPECT_EQ(o1.rank, o4.rank);
  EXPECT_EQ(o1.shortcuts.size(), o4.shortcuts.size());
}

TEST(CchOrder, IndependentSetMultiThreadDeterministic) {
  auto g = line_graph();
  auto o1 = BuildOrder(g, 1, OrderMethod::IndependentSet);
  auto o4 = BuildOrder(g, 4, OrderMethod::IndependentSet);
  expect_valid_order(o1, 4);
  expect_valid_order(o4, 4);
  EXPECT_EQ(o1.rank, o4.rank);
  EXPECT_EQ(o1.shortcuts.size(), o4.shortcuts.size());
}

// Fully connected K3: every contraction triangle already has a base edge, so
// chordal completion must not emit redundant shortcuts.
TEST(CchOrder, NoRedundantShortcutsWhenChordExists) {
  CchGraph g;
  for (int i = 0; i < 3; ++i) {
    CchNode n;
    n.graph_id = i;
    n.lat = 0;
    n.lon = static_cast<double>(i);
    n.country = "PL";
    g.set_index(i, i);
    g.nodes.push_back(n);
  }
  auto add = [&](uint32_t u, uint32_t v) {
    CchBaseEdge e;
    e.u = u;
    e.v = v;
    e.time_s = 60;
    e.roadclass = 2;
    g.edges.push_back(e);
  };
  add(0, 1);
  add(1, 0);
  add(0, 2);
  add(2, 0);
  add(1, 2);
  add(2, 1);
  g.build_csr();
  auto order = BuildOrder(g);
  expect_valid_order(order, 3);
  EXPECT_EQ(order.shortcuts.size(), 0u);
}

TEST(CchOrder, NestedDissectionRanksAllNodes) {
  auto g = grid_graph(8, 8);
  auto rank = ComputeNestedDissectionOrder(g, 2);
  EXPECT_EQ(rank.size(), 64u);
  std::set<uint32_t> ranks(rank.begin(), rank.end());
  EXPECT_EQ(ranks.size(), 64u);
}

TEST(CchOrder, NestedDissectionBuildOrderValidOnGrid) {
  auto g = grid_graph(12, 12);
  auto nd = BuildOrder(g, 2, OrderMethod::NestedDissection);
  expect_valid_order(nd, 144);
  // Sanity: chordal completion on a 12x12 grid stays well below dense fill-in.
  EXPECT_LT(nd.shortcuts.size(), 50000u);
}

// Above leaf_size (8192): must bipartition with balanced cuts, not micro-peels.
TEST(CchOrder, NestedDissectionBipartitionsAboveLeaf) {
  auto g = grid_graph(96, 96); // 9216 nodes
  auto rank = ComputeNestedDissectionOrder(g, 2);
  EXPECT_EQ(rank.size(), 9216u);
  std::set<uint32_t> ranks(rank.begin(), rank.end());
  EXPECT_EQ(ranks.size(), 9216u);
  auto order = BuildOrder(g, 2, OrderMethod::NestedDissection);
  expect_valid_order(order, 9216);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
