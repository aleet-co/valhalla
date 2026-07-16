#include "thor/cch/cch_graph.h"
#include "thor/cch/order.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla::thor::cch;

namespace {

// Build a tiny hand graph: a path 0-1-2-3 (bidirectional), all unit-ish.
CchGraph line_graph() {
  CchGraph g;
  for (int i = 0; i < 4; ++i) {
    CchNode n; n.graph_id = i; n.country = "PL"; g.set_index(i, i); g.nodes.push_back(n);
  }
  auto add = [&](uint32_t u, uint32_t v) {
    CchBaseEdge e; e.u = u; e.v = v; e.time_s = 60; e.roadclass = 2; g.edges.push_back(e);
  };
  add(0,1); add(1,0); add(1,2); add(2,1); add(2,3); add(3,2);
  g.build_csr();
  return g;
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
  // fwd edges go low->high rank; bwd edges high->low, stored at the high end.
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

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
