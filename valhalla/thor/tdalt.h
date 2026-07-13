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
 * Time-dependent bidirectional ALT path algorithm (TDALT).
 * Forward search on G with time-dependent costs; backward search on G_lambda
 * with lower-bound costs and ALT landmark potentials.
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

  float TightenedBackwardPotential(baldr::GraphReader& graphreader, const baldr::GraphId& node) const;

  void RebuildBackwardQueueKeys(baldr::GraphReader& graphreader);

  void MaybeAdvanceCheckpoint(baldr::GraphReader& graphreader,
                              const baldr::GraphId& node,
                              uint32_t pred_idx);

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
  enum class Phase { kMeet, kBound, kForwardOnly };
  Phase phase_{Phase::kMeet};
  float mu_{std::numeric_limits<float>::max()};
  float approximation_factor_{1.f};
  uint32_t checkpoint_count_{10};
  baldr::GraphId last_forward_settled_;
  float last_forward_key_{0.f};
  float forward_potential_delta_{0.f};
  uint32_t next_checkpoint_{1};
  std::unordered_set<uint64_t> backward_settled_nodes_;
  std::unordered_set<uint64_t> forward_settled_nodes_;
  std::unordered_map<uint64_t, uint32_t> forward_settled_labels_;
  std::unordered_map<uint64_t, uint32_t> backward_settled_labels_;
  std::unique_ptr<baldr::TDALTLandmarkIndex> landmarks_;
  baldr::GraphId origin_node_id_;
  baldr::GraphId destination_node_id_;

  // Mirror BidirectionalAStar search state (stubs for Tasks 10-11).
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
