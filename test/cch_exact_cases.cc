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

// ---------------------------------------------------------------------------
// Sketch §4 non-FIFO scenario (hand-built contracted graph):
//
//   Node X is reachable early (fast path) and ~20 min later (slow path).
//   The only useful exit X→T is banned for departures in slot 0
//   ([Mon 00:00, +15 min)). No-waiting: early arrival at X cannot proceed;
//   later arrival clears the window and reaches T.
//
//   Stage-1 ContractedTdEarliest keeps one label per node → settles X early,
//   discards the late label, and never reaches T. Stage 2 Pareto-on-arrival
//   must flip this test to EXPECT success / match the hand-computed later path.
// ---------------------------------------------------------------------------

TEST(CCHExactCases, NonFifoNeedsPareto) {
  // Nodes: S(r0) → X(r2) fast; S → A(r1) → X slow; X → T(r3).
  constexpr uint32_t S = 0;
  constexpr uint32_t A = 1;
  constexpr uint32_t X = 2;
  constexpr uint32_t T = 3;

  CchOrder order;
  order.num_base_edges = 4;
  order.rank = {0, 1, 2, 3};
  order.fwd_adj.assign(4, {});
  order.bwd_adj.assign(4, {});
  // All upward (rank-increasing) so down_allowed is irrelevant.
  order.fwd_adj[S].push_back({X, 0}); // fast S→X
  order.fwd_adj[S].push_back({A, 1}); // slow S→A
  order.fwd_adj[A].push_back({X, 2}); // slow A→X
  order.fwd_adj[X].push_back({T, 3}); // only useful exit

  CustomizedMetric metric;
  metric.profiles.assign(4, {});
  metric.profiles[0].time_s = 100;  // early arrive X at 100 (slot 0)
  metric.profiles[1].time_s = 700;  // slow leg 1
  metric.profiles[2].time_s = 700; // slow leg 2 → arrive X at 1400 (slot 1)
  metric.profiles[3].time_s = 100; // X→T
  set_slot(metric.profiles[3].forbidden, 0);

  // Hand-computed true earliest ban-feasible arrival at T (Pareto / waiting-free):
  // late path S→A→X (1400) then X→T at entry 1400 (slot 1, feasible) → 1500.
  constexpr float kTrueArrivalT = 1500.f;

  std::unordered_map<uint32_t, float> arrival;
  ContractedTdEarliest(order, metric, S, {T}, /*depart_sow=*/0, nullptr, arrival, nullptr);

  // Stage 1 RED bar: single-label search fails to settle T (or would mismatch
  // kTrueArrivalT if it somehow settled via another path — there is none).
  // Stage 2: replace with EXPECT_FLOAT_EQ(arrival[T], kTrueArrivalT).
  EXPECT_TRUE(arrival.find(T) == arrival.end())
      << "Stage-1 single-label must miss T; true later-path arrival would be "
      << kTrueArrivalT;

  // Document that the early label wins at X (late path discarded by single-label).
  std::unordered_map<uint32_t, float> to_x;
  ContractedTdEarliest(order, metric, S, {X}, 0, nullptr, to_x, nullptr);
  ASSERT_TRUE(to_x.find(X) != to_x.end());
  EXPECT_FLOAT_EQ(to_x[X], 100.f);
}

// ---------------------------------------------------------------------------
// Forced detour outside a ban-free thin corridor (Stage-1 covering signal):
//
//   Short up→down path S→U→T is banned at the down edge. Longer ban-free
//   detour S→D→T lies outside the ban-free static corridor (which would pick
//   the short path). Ban-free G↓(T) still marks both U and D; contracted TD
//   search with down_allowed covering must find the detour.
// ---------------------------------------------------------------------------

TEST(CCHExactCases, ForcedDetourOutsideBanFreeCorridor) {
  constexpr uint32_t S = 0;
  constexpr uint32_t U = 1; // short meeting node
  constexpr uint32_t D = 2; // detour meeting node
  constexpr uint32_t T = 3;

  CchOrder order;
  order.num_base_edges = 4;
  // Ranks: S low, T mid-low, U mid, D highest — both paths are up then down.
  order.rank = {/*S*/ 0, /*U*/ 2, /*D*/ 3, /*T*/ 1};
  order.fwd_adj.assign(4, {});
  order.bwd_adj.assign(4, {});
  order.fwd_adj[S].push_back({U, 0}); // short up
  order.fwd_adj[S].push_back({D, 1}); // detour up
  order.bwd_adj[T].push_back({U, 2}); // short down U→T
  order.bwd_adj[T].push_back({D, 3}); // detour down D→T

  CustomizedMetric metric;
  metric.profiles.assign(4, {});
  metric.profiles[0].time_s = 10;  // S→U
  metric.profiles[1].time_s = 100; // S→D
  metric.profiles[2].time_s = 10;  // U→T (banned at slot 0)
  metric.profiles[3].time_s = 100; // D→T
  set_slot(metric.profiles[2].forbidden, 0);

  // Ban-free static shortest would be S→U→T (20s) — the thin corridor path.
  // Legal TD detour: S→D→T = 200s.
  constexpr float kDetourArrival = 200.f;

  std::unordered_set<uint32_t> down_allowed;
  BanFreeDownwardReach(order, metric, T, down_allowed, nullptr, nullptr);
  EXPECT_TRUE(down_allowed.count(T));
  EXPECT_TRUE(down_allowed.count(U));
  EXPECT_TRUE(down_allowed.count(D));

  std::unordered_map<uint32_t, float> arrival;
  ContractedTdEarliest(order, metric, S, {T}, /*depart_sow=*/0, &down_allowed, arrival,
                       nullptr);
  ASSERT_TRUE(arrival.find(T) != arrival.end())
      << "Stage-1 contracted + covering must find the ban-free detour";
  EXPECT_FLOAT_EQ(arrival[T], kDetourArrival);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
