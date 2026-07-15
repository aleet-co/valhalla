#ifndef VALHALLA_THOR_TDALT_H_
#define VALHALLA_THOR_TDALT_H_

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/tdalt_landmarks.h>
#include <valhalla/baldr/time_info.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/astarheuristic.h>
#include <valhalla/thor/bidirectional_astar.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathalgorithm.h>

#include <boost/property_tree/ptree.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace valhalla {
namespace thor {

/**
 * Time-Dependent Bidirectional ALT (TDALT) — Algorithm 1 from Nannicini et al.
 *
 * Notation (paper symbols → this implementation):
 *
 *   G = (V, A)     Directed road graph from Valhalla tiles (nodes V, edges A).
 *   s, t           Origin and destination graph nodes (origin_node_id_, destination_node_id_).
 *   τ, τ₀          Departure/arrival time at a node (seconds since epoch). Forward search
 *                  tracks τ along edges via TimeInfo; backward search does not use τ.
 *   c(u,v,τ)       Time-dependent cost to traverse directed edge (u→v) when leaving u at τ.
 *                  In code: costing_->EdgeCost() + TransitionCost() on the forward tree only.
 *                  Includes traffic, truck_ban, turn penalties, and access at that instant.
 *   λ(u,v)         Static lower-bound cost on the auxiliary graph G_λ (same topology as G).
 *                  In code: LowerBoundCost::seconds() — length / max(edge_speed), ignoring
 *                  traffic and bans. Must satisfy λ(u,v) ≤ c(u,v,τ) for all τ so backward
 *                  A* keys never underestimate the true forward cost.
 *   g(v), f(v)     Path cost from s to v (forward) and from v to t along the current tree.
 *                  Stored in BDEdgeLabel::cost().cost; A* sort key is g + π.
 *   π_f(v)         Forward potential: admissible lower bound on remaining c-cost from v to t.
 *                  landmarks_->potential_to_target(v, t) on G_λ (ALT landmarks).
 *   π_b(v)         Backward potential: admissible lower bound on remaining λ-cost from s to v.
 *                  landmarks_->potential_from_source(v, s).
 *   π*_b(v)        Tightened backward potential (Eq. 4): max(π_b(v), f(v*) + π_f(v*) − π_f(v))
 *                  using the latest forward checkpoint node v* — tightens β without breaking
 *                  admissibility.
 *   μ              Upper bound on optimal route cost γ_τ₀(p*) after the first meet node v:
 *                  μ = g_fwd(v) + g_bwd(v). Phase 2 stops expanding backward once β > μ/K.
 *   β              Minimum sort key in the backward priority queue (g_λ + π*_b).
 *   K              Approximation factor (approximation_factor_, default 1.0 = exact). Larger K
 *                  ends Phase 2 earlier for speed, at the cost of a K-bounded suboptimal path.
 *   M              Set of nodes settled by the backward G_λ search; Phase 3 forward search
 *                  may only expand through nodes in M (backward_settled_nodes_).
 *
 * Two trees:
 *   Forward on G:   c(u,v,τ), live TimeInfo, key = g + π_f
 *   Backward on G_λ: λ(u,v), no time, key = g_λ + π*_b
 *
 * Three phases:
 *   kMeet         — bidirectional expansion until some v ∈ V is settled by both trees;
 *                   set μ = g_fwd(v) + g_bwd(v)
 *   kBound        — continue while β ≤ μ/K; add each backward-settled node to M
 *   kForwardOnly  — forward on G within M until t is settled; FormPath recosts with c(·,τ)
 */
class TimeDependentBidirALT : public PathAlgorithm {
public:
  explicit TimeDependentBidirALT(const boost::property_tree::ptree& config);
  ~TimeDependentBidirALT() override;

  std::vector<std::vector<PathInfo>>
  GetBestPath(valhalla::Location& origin,
              valhalla::Location& dest,
              baldr::GraphReader& graphreader,
              const sif::mode_costing_t& mode_costing,
              const sif::TravelMode mode,
              const Options& options = Options::default_instance()) override;

  const char* name() const override {
    return "time_dependent_bidirectional_alt";
  }

  void Clear() override;

  bool landmarks_available() const {
    return landmarks_ != nullptr && landmarks_->available();
  }

protected:
  void InitBackwardSearch(baldr::GraphReader& graphreader,
                          const midgard::PointLL& origll,
                          const midgard::PointLL& destll,
                          const baldr::GraphId& origin_node,
                          const baldr::GraphId& destination_node,
                          const sif::mode_costing_t& mode_costing,
                          const sif::TravelMode mode);

  void SetOrigin(baldr::GraphReader& graphreader,
                 valhalla::Location& origin,
                 const baldr::TimeInfo& time_info);

  void SetDestination(baldr::GraphReader& graphreader, const valhalla::Location& dest);

  void SetDestinationBackward(baldr::GraphReader& graphreader, const valhalla::Location& dest);

  bool ExpandOneForward(baldr::GraphReader& graphreader);

  // Expand one forward edge on G with time-dependent costing_->EdgeCost(τ).
  void ExpandForward(baldr::GraphReader& graphreader,
                     const baldr::GraphId& node,
                     sif::BDEdgeLabel& pred,
                     const uint32_t pred_idx,
                     const baldr::TimeInfo& time_info);

  bool ExpandForwardInner(baldr::GraphReader& graphreader,
                          const sif::BDEdgeLabel& pred,
                          const baldr::DirectedEdge* opp_pred_edge,
                          const baldr::NodeInfo* nodeinfo,
                          const uint32_t pred_idx,
                          const EdgeMetadata& meta,
                          const baldr::graph_tile_ptr& tile,
                          const baldr::TimeInfo& time_info,
                          std::pair<int32_t, float>& best_path);

  float ForwardPotential(baldr::GraphReader& graphreader, const baldr::GraphId& node) const;

  // Pop minimum backward label and expand on G_λ (λ costs, π*_b keys).
  bool ExpandOneBackward(baldr::GraphReader& graphreader);

  void ExpandBackward(baldr::GraphReader& graphreader,
                      const baldr::GraphId& node,
                      const sif::BDEdgeLabel& pred,
                      const uint32_t pred_idx,
                      const baldr::DirectedEdge* opp_pred_edge);

  bool ExpandBackwardInner(baldr::GraphReader& graphreader,
                           const sif::BDEdgeLabel& pred,
                           const baldr::DirectedEdge* opp_pred_edge,
                           const baldr::NodeInfo* nodeinfo,
                           const uint32_t pred_idx,
                           const EdgeMetadata& meta,
                           const baldr::graph_tile_ptr& tile);

  float BackwardPotential(baldr::GraphReader& graphreader, const baldr::GraphId& node) const;

  float BaseBackwardPotential(const baldr::GraphId& node) const;

  // π*_b(v) = max(π_b(v), f(v*) + π_f(v*) − π_f(v)) — tightened backward heuristic (Eq. 4).
  float TightenedBackwardPotential(baldr::GraphReader& graphreader, const baldr::GraphId& node) const;

  // Re-sort open backward labels after a forward checkpoint advances v*.
  void RebuildBackwardQueueKeys(baldr::GraphReader& graphreader);

  // Advance forward checkpoint when g(v) crosses the next threshold; may rebuild β queue.
  void MaybeAdvanceCheckpoint(baldr::GraphReader& graphreader,
                              const baldr::GraphId& node,
                              uint32_t pred_idx);

  // Record node in settled sets; detect Phase 1 meet and set μ.
  void OnNodeSettled(const baldr::GraphId& node, bool forward, uint32_t pred_idx);

  std::vector<std::vector<PathInfo>>
  FormPath(baldr::GraphReader& graphreader,
           const Options& options,
           const valhalla::Location& origin,
           const valhalla::Location& dest,
           const baldr::TimeInfo& time_info);

  void MarkForwardSettled(const baldr::GraphId& node) {
    forward_settled_nodes_.insert(node.value);
  }

  const std::unordered_set<uint64_t>& backward_settled_nodes() const {
    return backward_settled_nodes_;
  }

  const std::unordered_set<uint64_t>& forward_settled_nodes() const {
    return forward_settled_nodes_;
  }

private:
  // Algorithm 1 phase; drives the main loop in GetBestPath().
  enum class Phase { kMeet, kBound, kForwardOnly };

  Phase phase_{Phase::kMeet};

  // μ — paper upper bound γ_τ₀(p) after first meet: g_fwd(v) + g_bwd(v) at shared node v.
  float mu_{std::numeric_limits<float>::max()};

  // K — approximation factor; Phase 2 ends when β > K·μ (1.0 means exact optimality).
  float approximation_factor_{1.f};

  // Checkpoints for tightened backward potential π*_b (§7.5); triggers queue rebuilds.
  uint32_t checkpoint_count_{10};

  // v* and f(v*) from Eq. 4: last forward checkpoint node and its g(v*) + π_f(v*) sort key.
  baldr::GraphId last_forward_settled_;
  float last_forward_key_{0.f};

  // π_f(s) — scales checkpoint thresholds across the forward cost range [0, π_f(origin)].
  float forward_potential_delta_{0.f};
  uint32_t next_checkpoint_{1};

  // M — nodes settled by the backward G_λ tree (Phase 3 forward-only domain).
  std::unordered_set<uint64_t> backward_settled_nodes_;

  // Forward-settled nodes; backward expansion stops when it hits these (§7.1).
  std::unordered_set<uint64_t> forward_settled_nodes_;
  std::unordered_map<uint64_t, uint32_t> forward_settled_labels_;
  std::unordered_map<uint64_t, uint32_t> backward_settled_labels_;

  // mmap-loaded ALT tables on G_λ (mjolnir.tdalt.landmarks_file); not POI landmarks.sqlite.
  std::unique_ptr<baldr::TDALTLandmarkIndex> landmarks_;

  // Graph-node seeds for ALT potentials π_f and π_b (correlated origin/dest edges).
  baldr::GraphId origin_node_id_;
  baldr::GraphId destination_node_id_;

  // Bidirectional search state (parallel forward/backward label queues and edge status).
  uint32_t access_mode_;
  sif::TravelMode mode_;
  uint8_t travel_type_;
  sif::cost_ptr_t costing_;
  std::vector<HierarchyLimits> hierarchy_limits_forward_;
  std::vector<HierarchyLimits> hierarchy_limits_reverse_;
  bool ignore_hierarchy_limits_;
  float cost_diff_;
  AStarHeuristic astarheuristic_forward_;
  AStarHeuristic astarheuristic_reverse_;
  std::vector<sif::BDEdgeLabel> edgelabels_forward_;
  std::vector<sif::BDEdgeLabel> edgelabels_reverse_;
  baldr::DoubleBucketQueue<sif::BDEdgeLabel> adjacencylist_forward_;
  baldr::DoubleBucketQueue<sif::BDEdgeLabel> adjacencylist_reverse_;
  EdgeStatus edgestatus_forward_;
  EdgeStatus edgestatus_reverse_;
  float cost_threshold_;
  uint32_t iterations_threshold_;
  uint32_t desired_paths_count_;
  std::vector<CandidateConnection> best_connections_;
  float threshold_delta_;
  float alternative_cost_extend_;
  uint32_t alternative_iterations_delta_;
  bool extended_search_;
  bool pruning_disabled_at_origin_;
  bool pruning_disabled_at_destination_;
  baldr::TimeInfo forward_time_info_{baldr::TimeInfo::invalid()};
  std::unordered_multimap<baldr::GraphId, std::reference_wrapper<const valhalla::PathEdge>>
      destinations_;
  uint32_t destination_label_idx_{baldr::kInvalidLabel};
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_TDALT_H_
