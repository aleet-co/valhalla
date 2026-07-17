#include "thor/cch/cch_graph.h"
#include "thor/cch/customizer.h"
#include "thor/cch/order.h"
#include "thor/cch/profile.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla::thor::cch;

namespace {

// Bidirectional 4-cycle. Contracting any vertex creates a chord shortcut
// between its two neighbors (no base chord exists).
CchGraph two_country_cycle() {
  CchGraph g;
  auto node = [&](uint64_t id, const char* iso, double lon, double lat) {
    CchNode n;
    n.graph_id = id;
    n.country = iso;
    n.tz_index = 0;
    n.lat = lat;
    n.lon = lon;
    g.set_index(id, g.nodes.size());
    g.nodes.push_back(n);
  };
  node(0, "PL", 0.0, 0.0);
  node(1, "DE", 1.0, 0.0);
  node(2, "PL", 1.0, 1.0);
  node(3, "DE", 0.0, 1.0);
  auto add = [&](uint32_t u, uint32_t v, uint8_t rc) {
    CchBaseEdge e;
    e.u = u;
    e.v = v;
    e.time_s = 600;
    e.roadclass = rc;
    g.edges.push_back(e);
  };
  // 0->1 enters DE (Sunday ban); 1->2 enters PL (never banned).
  add(0, 1, 3);
  add(1, 0, 3);
  add(1, 2, 3);
  add(2, 1, 3);
  add(2, 3, 3);
  add(3, 2, 3);
  add(3, 0, 3);
  add(0, 3, 3);
  g.build_csr();
  return g;
}

TEST(CchCustomizer, DeEdgeHasBanPlEdgeDoesNot) {
  auto g = two_country_cycle();
  auto order = BuildOrder(g);
  auto metric = Customize(g, order, kDefaultReferenceWeek);
  ASSERT_EQ(metric.profiles.size(), order.num_base_edges + order.shortcuts.size());
  // edge 0 (->DE) should have some forbidden slots; edge 1 (->PL) none.
  EXPECT_FALSE(empty(metric.profiles[0].forbidden));
  EXPECT_TRUE(empty(metric.profiles[1].forbidden));
}

TEST(CchCustomizer, ShortcutProfileMatchesComposition) {
  auto g = two_country_cycle();
  auto order = BuildOrder(g);
  auto metric = Customize(g, order, kDefaultReferenceWeek);
  ASSERT_GT(order.shortcuts.size(), 0u)
      << "test graph must produce at least one shortcut to exercise composition";
  for (uint32_t si = 0; si < order.shortcuts.size(); ++si) {
    const auto& sc = order.shortcuts[si];
    Profile expect = compose(metric.profiles[sc.left], metric.profiles[sc.right]);
    const Profile& got = metric.profiles[order.num_base_edges + si];
    EXPECT_EQ(got.time_s, expect.time_s);
    EXPECT_EQ(got.forbidden.words, expect.forbidden.words);
  }
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
