#ifndef VALHALLA_SIF_LOWER_BOUND_COST_H_
#define VALHALLA_SIF_LOWER_BOUND_COST_H_

namespace valhalla {
namespace baldr {
class DirectedEdge;
} // namespace baldr

namespace sif {

/**
 * Static lower-bound edge weight λ(u,v) for the TDALT backward graph G_λ.
 *
 * Symbols:
 *   c(u,v,τ)  True time-dependent traverse cost when leaving node u at time τ.
 *             Valhalla: costing_->EdgeCost(edge, tile, TimeInfo) — varies with traffic,
 *             truck_ban windows, conditional access, turn penalties at that τ.
 *   λ(u,v)    Optimistic static traverse time on the same directed edge (u→v).
 *             Valhalla: edge.length / max(edge.speed, kMinSpeed) — no TimeInfo, no bans.
 *
 * Invariant (admissibility of backward A*):
 *   λ(u,v) ≤ c(u,v,τ)   for every feasible departure time τ
 *
 * Intuition: λ is "best case driving" (clear road, max legal speed); c is "what you
 * actually pay" at τ (Sunday truck ban, rush-hour traffic, etc.). Because backward search
 * cannot know the arrival time at t, it explores G_λ with λ weights; forward search on G
 * then finds the true time-dependent shortest path inside the node set M discovered on G_λ.
 */
struct LowerBoundCost {
  static float seconds(const baldr::DirectedEdge* edge, float length_override = -1.f);
};

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_LOWER_BOUND_COST_H_
