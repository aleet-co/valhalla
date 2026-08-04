#include "thor/cch/contracted_search.h"
#include "thor/cch/customizer.h"
#include "thor/cch/order.h"
#include "thor/cch/profile.h"
#include "test.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace valhalla::thor::cch;

namespace {

constexpr uint32_t A = 0;
constexpr uint32_t B = 1;
constexpr uint32_t C = 2;

// Minimal 3-node chain A→B→C with increasing ranks and a composed A→C shortcut.
// For ban/feasible traversal tests the overlay exposes only the shortcut in
// fwd_adj (base profiles still exist for composition checks).
void BuildShortcutChain(CchOrder& order, CustomizedMetric& metric, bool ban_shortcut_slot0) {
  order.num_base_edges = 2;
  order.rank = {0, 1, 2};
  order.shortcuts = {CchShortcut{A, C, B, /*left=*/0, /*right=*/1}};
  order.fwd_adj.assign(3, {});
  order.bwd_adj.assign(3, {});
  // Overlay: only A→C shortcut (eid 2). Base A→B / B→C live in profiles only.
  order.fwd_adj[A].push_back({C, 2});

  metric.profiles.assign(3, {});
  metric.profiles[0].time_s = 50; // A→B
  metric.profiles[1].time_s = 50; // B→C
  metric.profiles[2].time_s = 100;
  if (ban_shortcut_slot0)
    set_slot(metric.profiles[2].forbidden, 0);
}

TEST(ContractedSearch, ShortcutBanBlocksTraversal) {
  CchOrder order;
  CustomizedMetric metric;
  BuildShortcutChain(order, metric, /*ban_shortcut_slot0=*/true);

  const int64_t sow_slot0 = 0; // slot 0
  std::unordered_map<uint32_t, float> arrival;
  ContractedTdEarliest(order, metric, A, {C}, sow_slot0, nullptr, arrival, nullptr);
  EXPECT_TRUE(arrival.find(C) == arrival.end());
}

TEST(ContractedSearch, FeasibleShortcutSettlesTarget) {
  CchOrder order;
  CustomizedMetric metric;
  BuildShortcutChain(order, metric, /*ban_shortcut_slot0=*/false);

  const int64_t sow_slot0 = 0;
  std::unordered_map<uint32_t, float> arrival;
  ContractedTdEarliest(order, metric, A, {C}, sow_slot0, nullptr, arrival, nullptr);
  ASSERT_TRUE(arrival.find(C) != arrival.end());
  EXPECT_FLOAT_EQ(arrival[C], 100.f);
}

TEST(ContractedSearch, ComposedBanMatchesUnpackedBase) {
  // Base legs with bans; shortcut B ≡ compose(AB, BC).
  Profile ab;
  ab.time_s = 900; // one slot
  set_slot(ab.forbidden, 5);

  Profile bc;
  bc.time_s = 1800; // two slots
  set_slot(bc.forbidden, 7); // absolute slot on BC; shifts to 6 on compose

  Profile ac = compose(ab, bc);

  // At sow of slot 5: AB forbidden → composed forbidden; unpack also fails on AB.
  const int64_t sow5 = 5 * kSlotSeconds;
  EXPECT_FALSE(EdgeFeasibleAt(ac, sow5));
  EXPECT_FALSE(EdgeFeasibleAt(ab, sow5));

  // At sow of slot 6: AB ok, but BC entry = sow6 + 900 → slot 7 forbidden.
  const int64_t sow6 = 6 * kSlotSeconds;
  EXPECT_FALSE(EdgeFeasibleAt(ac, sow6));
  EXPECT_TRUE(EdgeFeasibleAt(ab, sow6));
  EXPECT_FALSE(EdgeFeasibleAt(bc, sow6 + ab.time_s));

  // At sow of slot 8: both legs feasible → composed feasible.
  const int64_t sow8 = 8 * kSlotSeconds;
  EXPECT_TRUE(EdgeFeasibleAt(ac, sow8));
  EXPECT_TRUE(EdgeFeasibleAt(ab, sow8));
  EXPECT_TRUE(EdgeFeasibleAt(bc, sow8 + ab.time_s));
}

TEST(ContractedSearch, BanFreeDownwardReachCollectsAncestors) {
  // Downward DAG into target C: A→C and B→C (ranks A,B < C so edges are down into C
  // when searched backward via bwd_adj).
  CchOrder order;
  order.num_base_edges = 2;
  order.rank = {0, 1, 2};
  order.fwd_adj.assign(3, {});
  order.bwd_adj.assign(3, {});
  // Edge A→C (eid 0): rank[A]<rank[C] → stored as up from A; also as down into C in bwd.
  // For DownwardTree / BanFreeDownwardReach we walk bwd_adj[u] = predecessors of u.
  order.bwd_adj[C].push_back({A, 0});
  order.bwd_adj[C].push_back({B, 1});

  CustomizedMetric metric;
  metric.profiles.assign(2, {});
  metric.profiles[0].time_s = 10;
  metric.profiles[1].time_s = 20;

  std::unordered_set<uint32_t> nodes;
  std::unordered_map<uint32_t, float> down_dist;
  BanFreeDownwardReach(order, metric, C, nodes, &down_dist, nullptr);

  EXPECT_TRUE(nodes.count(C));
  EXPECT_TRUE(nodes.count(A));
  EXPECT_TRUE(nodes.count(B));
  EXPECT_FLOAT_EQ(down_dist[C], 0.f);
  EXPECT_FLOAT_EQ(down_dist[A], 10.f);
  EXPECT_FLOAT_EQ(down_dist[B], 20.f);
}

TEST(ContractedSearch, DownAllowedRestrictsDescent) {
  // Nodes: 0=A (rank0), 1=M (rank2), 2=T (rank1). A→M up, then M→T down.
  // Without T in down_allowed, search must not settle T via the down edge.
  constexpr uint32_t nA = 0;
  constexpr uint32_t nM = 1;
  constexpr uint32_t nT = 2;
  CchOrder order;
  order.num_base_edges = 2;
  order.rank = {0, 2, 1};
  order.fwd_adj.assign(3, {});
  order.bwd_adj.assign(3, {});
  order.fwd_adj[nA].push_back({nM, 0});
  order.bwd_adj[nT].push_back({nM, 1});

  CustomizedMetric metric;
  metric.profiles.assign(2, {});
  metric.profiles[0].time_s = 5;
  metric.profiles[1].time_s = 5;

  std::unordered_map<uint32_t, float> arrival_blocked;
  std::unordered_set<uint32_t> empty_down;
  ContractedTdEarliest(order, metric, nA, {nT}, 0, &empty_down, arrival_blocked, nullptr);
  EXPECT_TRUE(arrival_blocked.find(nT) == arrival_blocked.end());

  std::unordered_map<uint32_t, float> arrival_ok;
  std::unordered_set<uint32_t> allow_T{nT};
  ContractedTdEarliest(order, metric, nA, {nT}, 0, &allow_T, arrival_ok, nullptr);
  ASSERT_TRUE(arrival_ok.find(nT) != arrival_ok.end());
  EXPECT_FLOAT_EQ(arrival_ok[nT], 10.f);

  // nullptr = unrestricted down
  std::unordered_map<uint32_t, float> arrival_unrestricted;
  ContractedTdEarliest(order, metric, nA, {nT}, 0, nullptr, arrival_unrestricted, nullptr);
  ASSERT_TRUE(arrival_unrestricted.find(nT) != arrival_unrestricted.end());
  EXPECT_FLOAT_EQ(arrival_unrestricted[nT], 10.f);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
