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
  ContractedTdEarliest(order, metric, A, {C}, sow_slot0, nullptr, nullptr, arrival, nullptr);
  EXPECT_TRUE(arrival.find(C) == arrival.end());
}

TEST(ContractedSearch, FeasibleShortcutSettlesTarget) {
  CchOrder order;
  CustomizedMetric metric;
  BuildShortcutChain(order, metric, /*ban_shortcut_slot0=*/false);

  const int64_t sow_slot0 = 0;
  std::unordered_map<uint32_t, float> arrival;
  ContractedTdEarliest(order, metric, A, {C}, sow_slot0, nullptr, nullptr, arrival, nullptr);
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
  ContractedTdEarliest(order, metric, nA, {nT}, 0, &empty_down, nullptr, arrival_blocked, nullptr);
  EXPECT_TRUE(arrival_blocked.find(nT) == arrival_blocked.end());

  std::unordered_map<uint32_t, float> arrival_ok;
  std::unordered_set<uint32_t> allow_T{nT};
  ContractedTdEarliest(order, metric, nA, {nT}, 0, &allow_T, nullptr, arrival_ok, nullptr);
  ASSERT_TRUE(arrival_ok.find(nT) != arrival_ok.end());
  EXPECT_FLOAT_EQ(arrival_ok[nT], 10.f);

  // nullptr = unrestricted down
  std::unordered_map<uint32_t, float> arrival_unrestricted;
  ContractedTdEarliest(order, metric, nA, {nT}, 0, nullptr, nullptr, arrival_unrestricted, nullptr);
  ASSERT_TRUE(arrival_unrestricted.find(nT) != arrival_unrestricted.end());
  EXPECT_FLOAT_EQ(arrival_unrestricted[nT], 10.f);
}

// Stage 3: multi-target up→down DAG. S→M up; M→T0 and M→T1 down (ban-free).
void BuildMultiTargetDownDag(CchOrder& order, CustomizedMetric& metric) {
  constexpr uint32_t S = 0;
  constexpr uint32_t M = 1;
  constexpr uint32_t T0 = 2;
  constexpr uint32_t T1 = 3;
  order.num_base_edges = 3;
  order.rank = {/*S*/ 0, /*M*/ 3, /*T0*/ 1, /*T1*/ 2};
  order.fwd_adj.assign(4, {});
  order.bwd_adj.assign(4, {});
  order.fwd_adj[S].push_back({M, 0});
  order.bwd_adj[T0].push_back({M, 1});
  order.bwd_adj[T1].push_back({M, 2});
  metric.profiles.assign(3, {});
  metric.profiles[0].time_s = 10; // S→M
  metric.profiles[1].time_s = 5;  // M→T0
  metric.profiles[2].time_s = 7;  // M→T1
}

TEST(ContractedSearch, RphastPhaseAFillsBanFreeBuckets) {
  CchOrder order;
  CustomizedMetric metric;
  BuildMultiTargetDownDag(order, metric);
  constexpr uint32_t M = 1;
  constexpr uint32_t T0 = 2;
  constexpr uint32_t T1 = 3;

  std::unordered_set<uint32_t> down_allowed;
  RphastBuckets buckets;
  BuildRphastPhaseA(order, metric, {T0, T1}, down_allowed, &buckets, nullptr);

  EXPECT_TRUE(down_allowed.count(M));
  EXPECT_TRUE(down_allowed.count(T0));
  EXPECT_TRUE(down_allowed.count(T1));
  ASSERT_TRUE(buckets.count(M));
  // Meeting node M must bucket both targets with static down distances.
  bool saw_t0 = false, saw_t1 = false;
  for (const auto& e : buckets[M]) {
    if (e.target == T0) {
      saw_t0 = true;
      EXPECT_FLOAT_EQ(e.down_dist, 5.f);
    }
    if (e.target == T1) {
      saw_t1 = true;
      EXPECT_FLOAT_EQ(e.down_dist, 7.f);
    }
  }
  EXPECT_TRUE(saw_t0);
  EXPECT_TRUE(saw_t1);
}

TEST(ContractedSearch, RphastBucketSettlesWithoutDownWalk) {
  // down_allowed marks only M (not T): Stage-2 cannot descend to T, but Stage-3
  // bucket settle at M must still report T.
  CchOrder order;
  CustomizedMetric metric;
  BuildMultiTargetDownDag(order, metric);
  constexpr uint32_t S = 0;
  constexpr uint32_t M = 1;
  constexpr uint32_t T0 = 2;

  std::unordered_set<uint32_t> down_allowed;
  RphastBuckets buckets;
  BuildRphastPhaseA(order, metric, {T0}, down_allowed, &buckets, nullptr);
  // Strip T0 so descent alone cannot settle the target.
  down_allowed = {M};

  std::unordered_map<uint32_t, float> stage2;
  ContractedTdPareto(order, metric, S, {T0}, 0, &down_allowed, nullptr, stage2, nullptr, nullptr);
  EXPECT_TRUE(stage2.find(T0) == stage2.end());

  std::unordered_map<uint32_t, float> stage3;
  ContractedTdPareto(order, metric, S, {T0}, 0, &down_allowed, &buckets, stage3, nullptr, nullptr);
  ASSERT_TRUE(stage3.find(T0) != stage3.end());
  EXPECT_FLOAT_EQ(stage3[T0], 15.f); // 10 + 5
}

TEST(ContractedSearch, RphastBucketsMatchUnbucketedPareto) {
  CchOrder order;
  CustomizedMetric metric;
  BuildMultiTargetDownDag(order, metric);
  constexpr uint32_t S = 0;
  constexpr uint32_t T0 = 2;
  constexpr uint32_t T1 = 3;

  std::unordered_set<uint32_t> down_allowed;
  RphastBuckets buckets;
  BuildRphastPhaseA(order, metric, {T0, T1}, down_allowed, &buckets, nullptr);

  std::unordered_map<uint32_t, float> stage2;
  ContractedTdPareto(order, metric, S, {T0, T1}, 0, &down_allowed, nullptr, stage2, nullptr,
                     nullptr);
  std::unordered_map<uint32_t, float> stage3;
  ContractedTdPareto(order, metric, S, {T0, T1}, 0, &down_allowed, &buckets, stage3, nullptr,
                     nullptr);

  ASSERT_EQ(stage2.size(), stage3.size());
  for (const auto& [node, d2] : stage2) {
    ASSERT_TRUE(stage3.count(node));
    EXPECT_NEAR(stage3[node], d2, 1e-3f);
  }
}

TEST(ContractedSearch, RphastBucketsOmitBannedDownEdges) {
  // M→T banned: Phase A still marks M in down_allowed, but must NOT put a
  // ban-free bucket at M for T (static addition would under-estimate).
  constexpr uint32_t S = 0;
  constexpr uint32_t M = 1;
  constexpr uint32_t T = 2;
  CchOrder order;
  order.num_base_edges = 2;
  order.rank = {0, 2, 1};
  order.fwd_adj.assign(3, {});
  order.bwd_adj.assign(3, {});
  order.fwd_adj[S].push_back({M, 0});
  order.bwd_adj[T].push_back({M, 1});
  CustomizedMetric metric;
  metric.profiles.assign(2, {});
  metric.profiles[0].time_s = 10;
  metric.profiles[1].time_s = 10;
  set_slot(metric.profiles[1].forbidden, 0);

  std::unordered_set<uint32_t> down_allowed;
  RphastBuckets buckets;
  BuildRphastPhaseA(order, metric, {T}, down_allowed, &buckets, nullptr);
  EXPECT_TRUE(down_allowed.count(M));
  if (buckets.count(M)) {
    for (const auto& e : buckets[M])
      EXPECT_NE(e.target, T) << "banned down path must not enter buckets";
  }
}

TEST(ContractedSearch, RphastBucketsDoNotEarlyExitSuboptimal) {
  // Two meeting peaks: M1 is closer from S but has a long down to T; M2 is
  // farther from S but short down. Bucket settle at M1 must not ++found and
  // truncate before M2 (101 vs true 11).
  //   S→M1=1, M1→T=100 → tentative 101
  //   S→M2=10, M2→T=1  → true 11
  constexpr uint32_t S = 0;
  constexpr uint32_t M1 = 1;
  constexpr uint32_t M2 = 2;
  constexpr uint32_t T = 3;
  CchOrder order;
  order.num_base_edges = 4;
  order.rank = {/*S*/ 0, /*M1*/ 2, /*M2*/ 3, /*T*/ 1};
  order.fwd_adj.assign(4, {});
  order.bwd_adj.assign(4, {});
  order.fwd_adj[S].push_back({M1, 0});
  order.fwd_adj[S].push_back({M2, 1});
  order.bwd_adj[T].push_back({M1, 2});
  order.bwd_adj[T].push_back({M2, 3});
  CustomizedMetric metric;
  metric.profiles.assign(4, {});
  metric.profiles[0].time_s = 1;   // S→M1
  metric.profiles[1].time_s = 10;  // S→M2
  metric.profiles[2].time_s = 100; // M1→T
  metric.profiles[3].time_s = 1;   // M2→T

  std::unordered_set<uint32_t> down_allowed;
  RphastBuckets buckets;
  BuildRphastPhaseA(order, metric, {T}, down_allowed, &buckets, nullptr);
  ASSERT_TRUE(buckets.count(M1));
  ASSERT_TRUE(buckets.count(M2));
  // Bucket-only settle: strip T so descent cannot reach the target.
  down_allowed = {M1, M2};

  std::unordered_map<uint32_t, float> earliest;
  ContractedTdEarliest(order, metric, S, {T}, 0, &down_allowed, &buckets, earliest, nullptr);
  ASSERT_TRUE(earliest.count(T));
  EXPECT_FLOAT_EQ(earliest[T], 11.f);

  std::unordered_map<uint32_t, float> pareto;
  ContractedTdPareto(order, metric, S, {T}, 0, &down_allowed, &buckets, pareto, nullptr, nullptr);
  ASSERT_TRUE(pareto.count(T));
  EXPECT_FLOAT_EQ(pareto[T], 11.f);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
