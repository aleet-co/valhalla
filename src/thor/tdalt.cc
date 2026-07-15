#include "thor/tdalt.h"

#include "baldr/datetime.h"
#include "baldr/directededge.h"
#include "baldr/time_info.h"
#include "midgard/logging.h"
#include "sif/lower_bound_cost.h"
#include "sif/recost.h"

#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <limits>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::sif;

namespace {

inline float find_percent_along(const valhalla::Location& location, const GraphId& edge_id) {
  for (const auto& e : location.correlation().edges()) {
    if (e.graph_id() == edge_id) {
      return e.percent_along();
    }
  }
  throw std::logic_error("Could not find candidate edge for the location");
}

bool get_seed_nodes(GraphReader& graphreader,
                    const valhalla::Location& origin,
                    const valhalla::Location& dest,
                    GraphId& origin_node,
                    GraphId& destination_node) {
  for (const auto& edge : origin.correlation().edges()) {
    const GraphId edgeid(edge.graph_id());
    const graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
    if (tile == nullptr) {
      continue;
    }
    const DirectedEdge* directededge = tile->directededge(edgeid);
    if (directededge == nullptr) {
      continue;
    }
    origin_node = directededge->endnode();
    break;
  }

  for (const auto& edge : dest.correlation().edges()) {
    const GraphId edgeid(edge.graph_id());
    const graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
    if (tile == nullptr) {
      continue;
    }
    const DirectedEdge* directededge = tile->directededge(edgeid);
    if (directededge == nullptr) {
      continue;
    }
    graph_tile_ptr opp_tile = tile;
    const DirectedEdge* opp_dir_edge = nullptr;
    const GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgeid, opp_dir_edge, opp_tile);
    if (opp_dir_edge == nullptr) {
      continue;
    }
    destination_node = opp_dir_edge->endnode();
    break;
  }

  return origin_node.is_valid() && destination_node.is_valid();
}

} // namespace

namespace valhalla {
namespace thor {

TimeDependentBidirALT::TimeDependentBidirALT(const boost::property_tree::ptree& config)
    : PathAlgorithm(config.get<uint32_t>("thor.tdalt.max_reserved_labels_count",
                                         kInitialEdgeLabelCountBidirAstar),
                    config.get<bool>("thor.clear_reserved_memory", false)),
      approximation_factor_(config.get<float>("thor.tdalt.approximation_factor", 1.f)),
      checkpoint_count_(config.get<uint32_t>("thor.tdalt.checkpoint_count", 10)),
      access_mode_(kAutoAccess), mode_(travel_mode_t::kDrive), travel_type_(0), costing_(nullptr),
      ignore_hierarchy_limits_(false), cost_diff_(0.f), cost_threshold_(0.f), iterations_threshold_(0),
      desired_paths_count_(1), threshold_delta_(0.f), alternative_cost_extend_(0.f),
      alternative_iterations_delta_(0), extended_search_(false), pruning_disabled_at_origin_(false),
      pruning_disabled_at_destination_(false) {
  // Landmark file is built offline on G_λ (valhalla_build_tdalt_landmarks). Without it,
  // route_action falls back to TimeDepForward when thor.tdalt.fallback_to_unidirectional is set.
  const auto landmarks_file = config.get<std::string>("mjolnir.tdalt.landmarks_file", "");
  if (!landmarks_file.empty()) {
    landmarks_ = std::make_unique<baldr::TDALTLandmarkIndex>(landmarks_file);
    if (!landmarks_->available()) {
      landmarks_.reset();
    }
  }
}

TimeDependentBidirALT::~TimeDependentBidirALT() {
}

std::vector<std::vector<PathInfo>> TimeDependentBidirALT::GetBestPath(valhalla::Location& origin,
                                                                      valhalla::Location& dest,
                                                                      baldr::GraphReader& graphreader,
                                                                      const mode_costing_t& mode_costing,
                                                                      const sif::TravelMode mode,
                                                                      const Options& options) {
  // arrive_by is handled by TimeDepReverse in route_action; TDALT only serves depart_at/current.
  if (options.date_time_type() == Options::arrive_by) {
    return {};
  }

  Clear();

  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  travel_type_ = costing_->travel_type();
  access_mode_ = costing_->access_mode();
  costing_->SetGraphReader(&graphreader);

  // Forward tree tracks simulated departure time along G (traffic, truck_ban, restrictions).
  forward_time_info_ = TimeInfo::make(origin, graphreader, &tz_cache_);
  if (!forward_time_info_.valid) {
    return {};
  }

  GraphId origin_node;
  GraphId destination_node;
  if (!get_seed_nodes(graphreader, origin, dest, origin_node, destination_node)) {
    return {};
  }

  const PointLL origin_ll(origin.correlation().edges(0).ll().lng(),
                          origin.correlation().edges(0).ll().lat());
  const PointLL dest_ll(dest.correlation().edges(0).ll().lng(),
                        dest.correlation().edges(0).ll().lat());
  InitBackwardSearch(graphreader, origin_ll, dest_ll, origin_node, destination_node, mode_costing,
                     mode);
  SetDestination(graphreader, dest);
  // Backward seeds on opposing edges at destination; costs are λ, not time-dependent.
  SetDestinationBackward(graphreader, dest);
  SetOrigin(graphreader, origin, forward_time_info_);

  if (adjacencylist_forward_.empty() || adjacencylist_reverse_.empty()) {
    return {};
  }

  // --- Algorithm 1 main loop (Meet → Bound → Forward-only) ---
  int n = 0;
  while (true) {
    if (interrupt && (++n % kInterruptIterationsInterval) == 0) {
      (*interrupt)();
    }

    if (destination_label_idx_ != kInvalidLabel && phase_ == Phase::kForwardOnly) {
      return FormPath(graphreader, options, origin, dest, forward_time_info_);
    }

    // Phase 3: forward A* on G, expanding only into nodes discovered by backward G_λ search.
    if (phase_ == Phase::kForwardOnly) {
      if (!ExpandOneForward(graphreader)) {
        return destination_label_idx_ != kInvalidLabel
                   ? FormPath(graphreader, options, origin, dest, forward_time_info_)
                   : std::vector<std::vector<PathInfo>>{};
      }
      continue;
    }

    const bool forward_empty = adjacencylist_forward_.empty();
    const bool backward_empty = adjacencylist_reverse_.empty();
    if (forward_empty && backward_empty) {
      return {};
    }

    if (phase_ == Phase::kBound && backward_empty) {
      // Backward queue drained — M is complete; finish on forward tree only.
      phase_ = Phase::kForwardOnly;
      if (destination_label_idx_ != kInvalidLabel) {
        return FormPath(graphreader, options, origin, dest, forward_time_info_);
      }
    }

    const float backward_min =
        backward_empty ? std::numeric_limits<float>::max() : adjacencylist_reverse_.min_sortcost();

    // β = min open backward key (g_λ + π*_b). When β > K·μ, no path via M can beat μ.
    if (phase_ == Phase::kBound && !backward_empty &&
        backward_min > approximation_factor_ * mu_) {
      phase_ = Phase::kForwardOnly;
      if (destination_label_idx_ != kInvalidLabel) {
        return FormPath(graphreader, options, origin, dest, forward_time_info_);
      }
      continue;
    }

    if (!forward_empty) {
      ExpandOneForward(graphreader);
    }

    // Phases 1–2: alternate backward G_λ expansion (collects M, prunes at forward-settled nodes).
    if (!backward_empty) {
      ExpandOneBackward(graphreader);
    }
  }
}

void TimeDependentBidirALT::Clear() {
  auto reservation = clear_reserved_memory_ ? 0 : max_reserved_labels_count_;
  if (edgelabels_forward_.size() > reservation) {
    edgelabels_forward_.resize(reservation);
    edgelabels_forward_.shrink_to_fit();
  }
  if (edgelabels_reverse_.size() > reservation) {
    edgelabels_reverse_.resize(reservation);
    edgelabels_reverse_.shrink_to_fit();
  }
  edgelabels_forward_.clear();
  edgelabels_reverse_.clear();
  adjacencylist_forward_.clear();
  adjacencylist_reverse_.clear();
  edgestatus_forward_.clear();
  edgestatus_reverse_.clear();
  backward_settled_nodes_.clear();
  forward_settled_nodes_.clear();
  forward_settled_labels_.clear();
  backward_settled_labels_.clear();
  best_connections_.clear();
  destinations_.clear();
  forward_time_info_ = TimeInfo::invalid();
  phase_ = Phase::kMeet;
  mu_ = std::numeric_limits<float>::max();
  destination_label_idx_ = kInvalidLabel;
  origin_node_id_ = {};
  destination_node_id_ = {};
  last_forward_settled_ = {};
  last_forward_key_ = 0.f;
  forward_potential_delta_ = 0.f;
  next_checkpoint_ = 1;
  has_ferry_ = false;
  set_not_thru_pruning(true);
  pruning_disabled_at_origin_ = false;
  pruning_disabled_at_destination_ = false;
  ignore_hierarchy_limits_ = false;
}

float TimeDependentBidirALT::BaseBackwardPotential(const GraphId& node) const {
  // π_b(v) — ALT lower bound from origin s to v on G_λ (backward search heuristic).
  if (landmarks_ && landmarks_->available()) {
    return landmarks_->potential_from_source(node, origin_node_id_);
  }
  return 0.f;
}

float TimeDependentBidirALT::TightenedBackwardPotential(GraphReader& graphreader,
                                                        const GraphId& node) const {
  // π*_b(w) = max( π_b(w),  f(v*) + π_f(v*) − π_f(w) )  — Eq. 4 in the paper.
  // v* = last_forward_settled_, f(v*) = last_forward_key_ (g+π_f at checkpoint).
  // Tightens the backward heuristic using forward progress without using c(·,τ) backward.
  if (!landmarks_ || !landmarks_->available()) {
    const graph_tile_ptr tile = graphreader.GetGraphTile(node);
    if (tile == nullptr) {
      return 0.f;
    }
    return astarheuristic_reverse_.Get(tile->get_node_ll(node));
  }

  const float pi_b = landmarks_->potential_from_source(node, origin_node_id_);
  if (!last_forward_settled_.is_valid()) {
    return pi_b;
  }
  // π_f(v*), π_f(w) on G_λ; combined with f(v*) to bound how far backward search must go.
  const float pi_f_v = landmarks_->potential_to_target(last_forward_settled_, destination_node_id_);
  const float pi_f_w = landmarks_->potential_to_target(node, destination_node_id_);
  return std::max(pi_b, last_forward_key_ + pi_f_v - pi_f_w);
}

float TimeDependentBidirALT::BackwardPotential(GraphReader& graphreader, const GraphId& node) const {
  return TightenedBackwardPotential(graphreader, node);
}

void TimeDependentBidirALT::RebuildBackwardQueueKeys(GraphReader& graphreader) {
  // Checkpoint advanced: recompute sortcost = g_λ + π*_b for all open backward labels.
  adjacencylist_reverse_.clear();
  for (uint32_t idx = 0; idx < edgelabels_reverse_.size(); ++idx) {
    const auto& label = edgelabels_reverse_[idx];
    if (edgestatus_reverse_.Get(label.edgeid()).set() != EdgeSet::kTemporary) {
      continue;
    }

    const float sortcost = label.cost().cost + TightenedBackwardPotential(graphreader, label.endnode());
    auto& mutable_label = edgelabels_reverse_[idx];
    mutable_label.Update(mutable_label.predecessor(), mutable_label.cost(), sortcost,
                         mutable_label.transition_cost(), mutable_label.path_distance(),
                         mutable_label.restriction_idx());
    adjacencylist_reverse_.add(idx);
  }
}

void TimeDependentBidirALT::MaybeAdvanceCheckpoint(GraphReader& graphreader,
                                                   const GraphId& node,
                                                   const uint32_t pred_idx) {
  // Every checkpoint_count steps along forward cost, record (v*, f(v*)) and rebuild β queue.
  if (!landmarks_ || !landmarks_->available() || checkpoint_count_ == 0 ||
      forward_potential_delta_ <= 0.f) {
    return;
  }

  const float forward_cost = edgelabels_forward_[pred_idx].cost().cost;
  const float threshold_step = forward_potential_delta_ / static_cast<float>(checkpoint_count_);

  bool updated = false;
  while (next_checkpoint_ <= checkpoint_count_ &&
         forward_cost >= threshold_step * static_cast<float>(next_checkpoint_)) {
    last_forward_settled_ = node;
    last_forward_key_ = edgelabels_forward_[pred_idx].sortcost();
    ++next_checkpoint_;
    updated = true;
  }

  if (updated) {
    RebuildBackwardQueueKeys(graphreader);
  }
}

float TimeDependentBidirALT::ForwardPotential(GraphReader& graphreader, const GraphId& node) const {
  // π_f(v) — ALT lower bound from v to target t on G_λ (forward search heuristic).
  if (landmarks_ && landmarks_->available()) {
    return landmarks_->potential_to_target(node, destination_node_id_);
  }
  const graph_tile_ptr tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return 0.f;
  }
  return astarheuristic_forward_.Get(tile->get_node_ll(node));
}

void TimeDependentBidirALT::InitBackwardSearch(GraphReader& graphreader,
                                               const PointLL& origll,
                                               const PointLL& destll,
                                               const GraphId& origin_node,
                                               const GraphId& destination_node,
                                               const mode_costing_t& mode_costing,
                                               const sif::TravelMode mode) {
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  travel_type_ = costing_->travel_type();
  access_mode_ = costing_->access_mode();
  origin_node_id_ = origin_node;
  destination_node_id_ = destination_node;
  last_forward_settled_ = {};
  last_forward_key_ = 0.f;
  next_checkpoint_ = 1;

  const float factor = costing_->AStarCostFactor();
  astarheuristic_forward_.Init(destll, factor);
  astarheuristic_reverse_.Init(origll, factor);
  forward_potential_delta_ = ForwardPotential(graphreader, origin_node_id_);

  edgelabels_reverse_.clear();
  edgelabels_reverse_.reserve(max_reserved_labels_count_);
  adjacencylist_reverse_.clear();
  edgestatus_reverse_.clear();
  backward_settled_nodes_.clear();
  forward_settled_nodes_.clear();
  forward_settled_labels_.clear();
  backward_settled_labels_.clear();
  destinations_.clear();
  forward_time_info_ = TimeInfo::invalid();

  edgelabels_forward_.clear();
  edgelabels_forward_.reserve(max_reserved_labels_count_);
  adjacencylist_forward_.clear();
  edgestatus_forward_.clear();

  const uint32_t bucketsize = costing_->UnitSize();
  const float range = kBucketCount * bucketsize;
  adjacencylist_reverse_.reuse(0.f, range, bucketsize, &edgelabels_reverse_);
  adjacencylist_forward_.reuse(0.f, range, bucketsize, &edgelabels_forward_);
}

void TimeDependentBidirALT::SetDestination(GraphReader& /*graphreader*/,
                                           const valhalla::Location& dest) {
  const bool has_other_edges =
      std::any_of(dest.correlation().edges().begin(), dest.correlation().edges().end(),
                  [](const valhalla::PathEdge& e) { return !e.begin_node(); });

  for (const auto& edge : dest.correlation().edges()) {
    if (has_other_edges && edge.begin_node()) {
      continue;
    }

    const GraphId edgeid(edge.graph_id());
    if (costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
      continue;
    }

    destinations_.emplace(edgeid, edge);
  }
}

void TimeDependentBidirALT::SetOrigin(GraphReader& graphreader,
                                    valhalla::Location& origin,
                                    const TimeInfo& time_info) {
  forward_time_info_ = time_info;

  const bool has_other_edges =
      std::any_of(origin.correlation().edges().begin(), origin.correlation().edges().end(),
                  [](const valhalla::PathEdge& e) { return !e.end_node(); });

  const auto super_trivial = [&](const valhalla::PathEdge& edge) {
    const GraphId edgeid(edge.graph_id());
    const auto dests = destinations_.equal_range(edgeid);
    for (auto it = dests.first; it != dests.second; ++it) {
      const auto& dest_path_edge = it->second.get();
      if (edge.percent_along() == dest_path_edge.percent_along()) {
        return true;
      }
    }
    return false;
  };

  const NodeInfo* closest_ni = nullptr;
  for (const auto& edge : origin.correlation().edges()) {
    if (edge.end_node() && has_other_edges && !super_trivial(edge)) {
      continue;
    }

    const GraphId edgeid(edge.graph_id());
    if (costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())) {
      continue;
    }

    graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
    if (tile == nullptr) {
      continue;
    }
    const DirectedEdge* directededge = tile->directededge(edgeid);

    graph_tile_ptr endtile = graphreader.GetGraphTile(directededge->endnode());
    if (endtile == nullptr) {
      continue;
    }

    const NodeInfo* nodeinfo = endtile->node(directededge->endnode());
    if (closest_ni == nullptr) {
      closest_ni = nodeinfo;
    }

    const auto destonly_restriction_mask =
        costing_->GetExemptedAccessRestrictions(directededge, tile, edgeid);

    const auto add_label = [&](const valhalla::PathEdge* dest_path_edge) {
      const float start = edge.percent_along();
      const float end = dest_path_edge ? dest_path_edge->percent_along() : 1.0f;
      const float percent_traversed = end - start;
      if (percent_traversed < 0.f) {
        return;
      }

      uint8_t flow_sources;
      Cost cost = costing_->PartialEdgeCost(directededge, edgeid, tile, time_info, flow_sources,
                                            start, end);
      cost.cost += edge.distance() + (dest_path_edge ? dest_path_edge->distance() : 0.0f);

      const float sortcost =
          cost.cost + (dest_path_edge ? 0.f : ForwardPotential(graphreader, directededge->endnode()));
      const auto path_distance =
          static_cast<uint32_t>(directededge->length() * percent_traversed + .5f);

      const uint32_t idx = edgelabels_forward_.size();
      if (!dest_path_edge) {
        edgestatus_forward_.Set(edgeid, EdgeSet::kTemporary, idx, tile);
      }

      edgelabels_forward_.emplace_back(kInvalidLabel, edgeid, directededge, cost, sortcost, 0.f, mode_,
                                       kInvalidRestriction, !(costing_->IsClosed(directededge, tile)),
                                       static_cast<bool>(flow_sources & kDefaultFlowMask),
                                       sif::InternalTurn::kNoTurn, 0,
                                       directededge->destonly() ||
                                           (costing_->is_hgv() && directededge->destonly_hgv()),
                                       directededge->forwardaccess() & kTruckAccess,
                                       destonly_restriction_mask);
      auto& edge_label = edgelabels_forward_.back();
      edge_label.Update(kInvalidLabel, cost, sortcost, Cost{}, path_distance, kInvalidRestriction);
      edge_label.set_not_thru(false);
      edge_label.set_origin();
      if (dest_path_edge) {
        edge_label.set_destination();
        if (destination_label_idx_ == kInvalidLabel ||
            cost.cost < edgelabels_forward_[destination_label_idx_].cost().cost) {
          destination_label_idx_ = idx;
        }
      }

      adjacencylist_forward_.add(idx);

      if (expansion_callback_) {
        expansion_callback_(graphreader, edgeid, GraphId{}, name(), Expansion_EdgeStatus_reached,
                            cost.secs, path_distance, cost.cost, Expansion_ExpansionType_forward,
                            flow_sources, TravelMode::TravelMode_INT_MAX_SENTINEL_DO_NOT_USE_);
      }

      pruning_disabled_at_origin_ =
          pruning_disabled_at_origin_ || !edge_label.closure_pruning() ||
          !edge_label.not_thru_pruning() || edge_label.destonly();
    };

    add_label(nullptr);

    const auto dests = destinations_.equal_range(edgeid);
    for (auto it = dests.first; it != dests.second; ++it) {
      add_label(&it->second.get());
    }
  }

  if (closest_ni != nullptr && !origin.date_time().empty() && origin.date_time() == "current") {
    origin.set_date_time(
        DateTime::iso_date_time(DateTime::get_tz_db().from_index(closest_ni->timezone())));
  }
}

bool TimeDependentBidirALT::ExpandForwardInner(GraphReader& graphreader,
                                               const BDEdgeLabel& pred,
                                               const DirectedEdge* /*opp_pred_edge*/,
                                               const NodeInfo* nodeinfo,
                                               const uint32_t pred_idx,
                                               const EdgeMetadata& meta,
                                               const graph_tile_ptr& tile,
                                               const TimeInfo& time_info,
                                               std::pair<int32_t, float>& best_path) {
  if (meta.edge->is_shortcut()) {
    return false;
  }

  if (meta.edge_status->set() == EdgeSet::kPermanent) {
    return true;
  }

  graph_tile_ptr endtile =
      meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
  if (endtile == nullptr) {
    return false;
  }

  uint8_t flow_sources;
  // c(u,v,τ): time-dependent edge weight on G; τ advances via time_info.forward(pred_secs).
  const Cost edge_cost =
      costing_->EdgeCost(meta.edge, meta.edge_id, tile, time_info, flow_sources);
  auto reader_getter = [&graphreader]() { return baldr::LimitedGraphReader(graphreader); };
  const Cost transition_cost =
      costing_->TransitionCost(meta.edge, nodeinfo, pred, tile, reader_getter);

  const auto add_label = [&](const valhalla::PathEdge* dest_path_edge) {
    uint8_t restriction_idx = kInvalidRestriction;
    uint8_t destonly_restriction_mask = pred.destonly_access_restr_mask();
    if (!costing_->Allowed(meta.edge, dest_path_edge, pred, tile, meta.edge_id, time_info.local_time,
                           nodeinfo->timezone(), restriction_idx, destonly_restriction_mask) ||
        costing_->Restricted(meta.edge, pred, edgelabels_forward_, tile, meta.edge_id, true,
                             &edgestatus_forward_, time_info.local_time, nodeinfo->timezone())) {
      return false;
    }

    const auto percent_traversed =
        dest_path_edge ? dest_path_edge->percent_along() : 1.0f;

    auto cost = pred.cost() + transition_cost + edge_cost * percent_traversed;
    cost.cost += dest_path_edge ? dest_path_edge->distance() : 0.0f;

    const float sortcost =
        cost.cost + (dest_path_edge ? 0.f : ForwardPotential(graphreader, meta.edge->endnode()));

    const auto path_distance =
        static_cast<uint32_t>(pred.path_distance() + meta.edge->length() * percent_traversed + .5f);

    const uint32_t idx = edgelabels_forward_.size();

    if (dest_path_edge && (best_path.first == -1 || cost.cost < best_path.second)) {
      best_path.first = static_cast<int32_t>(idx);
      best_path.second = cost.cost;
    }

    GraphId opp_edge_id;
    edgelabels_forward_.emplace_back(pred_idx, meta.edge_id, opp_edge_id, meta.edge, cost, sortcost,
                                     0.f, mode_, transition_cost,
                                     (pred.not_thru_pruning() || !meta.edge->not_thru()),
                                     (pred.closure_pruning() || !(costing_->IsClosed(meta.edge, tile))),
                                     0 != (flow_sources & kDefaultFlowMask),
                                     costing_->TurnType(pred.opp_local_idx(), nodeinfo, meta.edge),
                                     restriction_idx, 0,
                                     meta.edge->destonly() ||
                                         (costing_->is_hgv() && meta.edge->destonly_hgv()),
                                     meta.edge->forwardaccess() & kTruckAccess,
                                     destonly_restriction_mask);

    auto& edge_label = edgelabels_forward_.back();
    edge_label.Update(pred_idx, cost, sortcost, transition_cost, path_distance, restriction_idx);

    if (dest_path_edge) {
      edge_label.set_destination();
    } else {
      *meta.edge_status = {EdgeSet::kTemporary, idx};
    }

    adjacencylist_forward_.add(idx);

    if (expansion_callback_) {
      const GraphId prev_pred =
          pred.predecessor() == kInvalidLabel ? GraphId{} : edgelabels_forward_[pred.predecessor()].edgeid();
      expansion_callback_(graphreader, meta.edge_id, prev_pred, name(), Expansion_EdgeStatus_reached,
                          cost.secs, path_distance, cost.cost, Expansion_ExpansionType_forward,
                          flow_sources, TravelMode::TravelMode_INT_MAX_SENTINEL_DO_NOT_USE_);
    }

    return true;
  };

  bool added = false;

  if (meta.edge_status->set() == EdgeSet::kTemporary) {
    auto update_label = [&]() {
      uint8_t restriction_idx = kInvalidRestriction;
      uint8_t destonly_restriction_mask = pred.destonly_access_restr_mask();
      if (!costing_->Allowed(meta.edge, false, pred, tile, meta.edge_id, time_info.local_time,
                             nodeinfo->timezone(), restriction_idx, destonly_restriction_mask) ||
          costing_->Restricted(meta.edge, pred, edgelabels_forward_, tile, meta.edge_id, true,
                               &edgestatus_forward_, time_info.local_time, nodeinfo->timezone())) {
        return false;
      }

      auto& lab = edgelabels_forward_[meta.edge_status->index()];
      const auto newcost = pred.cost() + transition_cost + edge_cost;

      if (newcost.cost < lab.cost().cost) {
        const float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjacencylist_forward_.decrease(meta.edge_status->index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost, transition_cost, restriction_idx);
      }

      if (expansion_callback_) {
        const GraphId prev_pred =
            pred.predecessor() == kInvalidLabel ? GraphId{} : edgelabels_forward_[pred.predecessor()].edgeid();
        expansion_callback_(graphreader, meta.edge_id, prev_pred, name(), Expansion_EdgeStatus_reached,
                            newcost.secs, lab.path_distance(), newcost.cost,
                            Expansion_ExpansionType_forward, flow_sources,
                            TravelMode::TravelMode_INT_MAX_SENTINEL_DO_NOT_USE_);
      }
      return true;
    };
    added = update_label();
  } else {
    added = add_label(nullptr);
  }

  const auto dests = destinations_.equal_range(meta.edge_id);
  for (auto it = dests.first; it != dests.second; ++it) {
    const auto& dest_path_edge = it->second.get();
    added = add_label(&dest_path_edge) || added;
  }

  return added;
}

void TimeDependentBidirALT::ExpandForward(GraphReader& graphreader,
                                          const GraphId& node,
                                          BDEdgeLabel& pred,
                                          const uint32_t pred_idx,
                                          const TimeInfo& time_info) {
  graph_tile_ptr tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return;
  }
  const NodeInfo* nodeinfo = tile->node(node);

  const auto offset_time =
      time_info.forward(pred.cost().secs, static_cast<int>(nodeinfo->timezone()));

  std::pair<int32_t, float> best_path{-1, 0.f};

  if (!costing_->Allowed(nodeinfo)) {
    const DirectedEdge* opp_edge = nullptr;
    const GraphId opp_edge_id = graphreader.GetOpposingEdgeId(pred.edgeid(), opp_edge, tile);
    pred.set_deadend(true);
    if (opp_edge) {
      ExpandForwardInner(graphreader, pred, opp_edge, nodeinfo, pred_idx,
                         {opp_edge, opp_edge_id, edgestatus_forward_.GetPtr(opp_edge_id, tile)}, tile,
                         offset_time, best_path);
    }
    return;
  }

  EdgeMetadata meta = EdgeMetadata::make(node, nodeinfo, tile, edgestatus_forward_);

  bool disable_uturn = false;
  EdgeMetadata uturn_meta{};

  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++meta) {
    uturn_meta = pred.opp_local_idx() == meta.edge->localedgeidx() ? meta : uturn_meta;

    if (pred.opp_local_idx() != meta.edge->localedgeidx()) {
      disable_uturn =
          ExpandForwardInner(graphreader, pred, nullptr, nodeinfo, pred_idx, meta, tile, offset_time,
                             best_path) ||
          disable_uturn;
    }
  }

  if (!disable_uturn && uturn_meta) {
    pred.set_deadend(true);
    ExpandForwardInner(graphreader, pred, nullptr, nodeinfo, pred_idx, uturn_meta, tile, offset_time,
                       best_path);
  }
}

bool TimeDependentBidirALT::ExpandOneForward(GraphReader& graphreader) {
  const uint32_t pred_idx = adjacencylist_forward_.pop();
  if (pred_idx == kInvalidLabel) {
    return false;
  }

  BDEdgeLabel pred = edgelabels_forward_[pred_idx];

  if (!pred.origin()) {
    edgestatus_forward_.Update(pred.edgeid(), EdgeSet::kPermanent);
  }

  const GraphId node = pred.endnode();
  if (!pred.origin()) {
    OnNodeSettled(node, true, pred_idx);
    MaybeAdvanceCheckpoint(graphreader, node, pred_idx);
  }

  if (pred.destination()) {
    if (destination_label_idx_ == kInvalidLabel ||
        pred.cost().cost < edgelabels_forward_[destination_label_idx_].cost().cost) {
      destination_label_idx_ = pred_idx;
    }
    return true;
  }

  if (phase_ == Phase::kForwardOnly && backward_settled_nodes_.count(node.value) == 0) {
    // Phase 3: skip expansion from nodes outside backward candidate set M.
    return true;
  }

  ExpandForward(graphreader, node, pred, pred_idx, forward_time_info_);
  return true;
}

void TimeDependentBidirALT::SetDestinationBackward(GraphReader& graphreader,
                                                   const valhalla::Location& dest) {
  // Seed backward queue at destination: traverse opposing edges with λ costs (G_λ, no TimeInfo).
  const bool has_other_edges =
      std::any_of(dest.correlation().edges().begin(), dest.correlation().edges().end(),
                  [](const valhalla::PathEdge& e) { return !e.begin_node(); });

  Cost zero_cost;
  for (const auto& edge : dest.correlation().edges()) {
    if (has_other_edges && edge.begin_node()) {
      continue;
    }

    const GraphId edgeid(edge.graph_id());
    if (costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
      continue;
    }

    graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
    if (tile == nullptr) {
      continue;
    }
    const DirectedEdge* directededge = tile->directededge(edgeid);

    graph_tile_ptr opp_tile = tile;
    const DirectedEdge* opp_dir_edge = nullptr;
    const GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgeid, opp_dir_edge, opp_tile);
    if (!opp_dir_edge) {
      continue;
    }

    const float traversed =
        opp_dir_edge->length() * (1.f - edge.percent_along());
    Cost cost;
    cost.secs = LowerBoundCost::seconds(opp_dir_edge, traversed);
    cost.cost = cost.secs + edge.distance();
    // Initial g_λ at destination seed: partial λ along opposing edge, not c(·,τ).

    const GraphId end_node = opp_dir_edge->endnode();
    const float sortcost = cost.cost + BackwardPotential(graphreader, end_node);

    const uint32_t idx = edgelabels_reverse_.size();
    edgestatus_reverse_.Set(opp_edge_id, EdgeSet::kTemporary, idx, opp_tile);
    edgelabels_reverse_.emplace_back(kInvalidLabel, opp_edge_id, edgeid, opp_dir_edge, cost, sortcost,
                                     0.f, mode_, zero_cost, true, true, false,
                                     InternalTurn::kNoTurn, kInvalidRestriction, 0, false, false, 0);
    adjacencylist_reverse_.add(idx);
    edgelabels_reverse_.back().set_not_thru(false);
  }
}

bool TimeDependentBidirALT::ExpandBackwardInner(GraphReader& graphreader,
                                                const BDEdgeLabel& pred,
                                                const DirectedEdge* opp_pred_edge,
                                                const NodeInfo* nodeinfo,
                                                const uint32_t pred_idx,
                                                const EdgeMetadata& meta,
                                                const graph_tile_ptr& tile) {
  if (meta.edge->is_shortcut()) {
    return false;
  }

  graph_tile_ptr endtile = nullptr;
  GraphId opp_edge_id;
  const auto get_opp_edge_data = [&]() {
    endtile = meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
    if (endtile == nullptr) {
      return false;
    }
    opp_edge_id = endtile->GetOpposingEdgeId(meta.edge);
    return true;
  };

  if (!(meta.edge->reverseaccess() & access_mode_)) {
    return false;
  }

  if (meta.edge_status->set() == EdgeSet::kPermanent) {
    return true;
  }

  if (!get_opp_edge_data()) {
    return false;
  }
  const DirectedEdge* opp_edge = endtile->directededge(opp_edge_id);

  Cost newcost = pred.cost();
  // Accumulate g_λ along reverse tree: g_λ(w) = g_λ(u) + λ(u,w), not c(u,w,τ).
  newcost.secs += LowerBoundCost::seconds(opp_edge);
  newcost.cost = pred.cost().cost + LowerBoundCost::seconds(opp_edge);

  if (meta.edge_status->set() == EdgeSet::kTemporary) {
    BDEdgeLabel& lab = edgelabels_reverse_[meta.edge_status->index()];
    if (newcost.cost < lab.cost().cost) {
      const float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
      adjacencylist_reverse_.decrease(meta.edge_status->index(), newsortcost);
      lab.Update(pred_idx, newcost, newsortcost, Cost{}, kInvalidRestriction);
    }
    return true;
  }

  const float sortcost = newcost.cost + BackwardPotential(graphreader, meta.edge->endnode());
  // Backward A* key: g_λ(w) + π*_b(w) — uses λ weights, never c(·,τ).

  const uint32_t idx = edgelabels_reverse_.size();
  edgelabels_reverse_.emplace_back(pred_idx, meta.edge_id, opp_edge_id, meta.edge, newcost, sortcost,
                                   0.f, mode_, Cost{}, true, true, false,
                                   costing_->TurnType(meta.edge->localedgeidx(), nodeinfo, opp_edge,
                                                      opp_pred_edge),
                                   kInvalidRestriction, 0, false, false, 0);
  adjacencylist_reverse_.add(idx);
  *meta.edge_status = {EdgeSet::kTemporary, idx};

  if (expansion_callback_) {
    const GraphId prev_pred =
        pred.predecessor() == kInvalidLabel ? GraphId{} : edgelabels_reverse_[pred.predecessor()].edgeid();
    expansion_callback_(graphreader, opp_edge_id, prev_pred, name(), Expansion_EdgeStatus_reached,
                        newcost.secs, pred.path_distance() + meta.edge->length(), newcost.cost,
                        Expansion_ExpansionType_reverse, kNoFlowMask,
                        TravelMode::TravelMode_INT_MAX_SENTINEL_DO_NOT_USE_);
  }

  return true;
}

void TimeDependentBidirALT::ExpandBackward(GraphReader& graphreader,
                                           const GraphId& node,
                                           const BDEdgeLabel& pred,
                                           const uint32_t pred_idx,
                                           const DirectedEdge* opp_pred_edge) {
  graph_tile_ptr tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return;
  }
  const NodeInfo* nodeinfo = tile->node(node);

  bool disable_uturn = false;
  EdgeMetadata meta = EdgeMetadata::make(node, nodeinfo, tile, edgestatus_reverse_);
  EdgeMetadata uturn_meta{};

  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++meta) {
    const bool is_uturn = pred.opp_local_idx() == meta.edge->localedgeidx();
    uturn_meta = is_uturn ? meta : uturn_meta;

    if (!is_uturn) {
      disable_uturn = ExpandBackwardInner(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx, meta,
                                          tile) ||
                      disable_uturn;
    }
  }

  if (!disable_uturn && uturn_meta) {
    ExpandBackwardInner(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx, uturn_meta, tile);
  }
}

bool TimeDependentBidirALT::ExpandOneBackward(GraphReader& graphreader) {
  const uint32_t pred_idx = adjacencylist_reverse_.pop();
  if (pred_idx == kInvalidLabel) {
    return false;
  }

  BDEdgeLabel pred = edgelabels_reverse_[pred_idx];
  edgestatus_reverse_.Update(pred.edgeid(), EdgeSet::kPermanent);

  const GraphId node = pred.endnode();
  OnNodeSettled(node, false, pred_idx);

  // §7.1: do not expand backward through nodes already settled forward (meet pruning).
  if (forward_settled_nodes_.count(node.value) > 0) {
    return true;
  }

  graph_tile_ptr opp_tile = graphreader.GetGraphTile(pred.opp_edgeid());
  if (opp_tile == nullptr) {
    return true;
  }
  const DirectedEdge* opp_pred_edge = opp_tile->directededge(pred.opp_edgeid());

  ExpandBackward(graphreader, node, pred, pred_idx, opp_pred_edge);
  return true;
}

void TimeDependentBidirALT::OnNodeSettled(const GraphId& node, const bool forward,
                                          const uint32_t pred_idx) {
  if (forward) {
    forward_settled_nodes_.insert(node.value);
    forward_settled_labels_[node.value] = pred_idx;
  } else {
    backward_settled_nodes_.insert(node.value);
    backward_settled_labels_[node.value] = pred_idx;
  }

  if (phase_ != Phase::kMeet) {
    return;
  }

  // Phase 1 meet: node v settled on both trees ⇒ μ = g_fwd(v) + g_bwd(v) ≤ γ_τ₀(optimal).
  if (!forward_settled_nodes_.count(node.value) || !backward_settled_nodes_.count(node.value)) {
    return;
  }

  const auto fwd_it = forward_settled_labels_.find(node.value);
  const auto bwd_it = backward_settled_labels_.find(node.value);
  if (fwd_it == forward_settled_labels_.end() || bwd_it == backward_settled_labels_.end()) {
    return;
  }

  mu_ = edgelabels_forward_[fwd_it->second].cost().cost +
        edgelabels_reverse_[bwd_it->second].cost().cost;
  phase_ = Phase::kBound;
}

std::vector<std::vector<PathInfo>> TimeDependentBidirALT::FormPath(GraphReader& graphreader,
                                                                    const Options& options,
                                                                    const valhalla::Location& origin,
                                                                    const valhalla::Location& dest,
                                                                    const TimeInfo& time_info) {
  // Extract topology from forward labels (g costs); final γ_τ₀ uses c(u,v,τ) via recost_forward.
  if (destination_label_idx_ == kInvalidLabel) {
    return {};
  }

  std::unordered_set<GraphId> recovered_inner_edges;
  std::vector<GraphId> path_edges;

  graph_tile_ptr tile;
  for (auto edgelabel_index = destination_label_idx_; edgelabel_index != kInvalidLabel;
       edgelabel_index = edgelabels_forward_[edgelabel_index].predecessor()) {
    const BDEdgeLabel& edgelabel = edgelabels_forward_[edgelabel_index];
    const DirectedEdge* edge = graphreader.directededge(edgelabel.edgeid(), tile);
    if (edge == nullptr) {
      throw tile_gone_error_t("TimeDependentBidirALT::FormPath failed", edgelabel.edgeid());
    }

    if (edge->is_shortcut()) {
      auto superseded = graphreader.RecoverShortcut(edgelabel.edgeid());
      recovered_inner_edges.insert(superseded.begin() + 1, superseded.end());
      std::move(superseded.rbegin(), superseded.rend(), std::back_inserter(path_edges));
    } else {
      path_edges.push_back(edgelabel.edgeid());
    }

    if (edgelabel.use() == Use::kFerry) {
      has_ferry_ = true;
    }
  }

  std::reverse(path_edges.begin(), path_edges.end());
  if (path_edges.empty()) {
    return {};
  }

  std::vector<PathInfo> path;
  path.reserve(path_edges.size());

  auto edge_itr = path_edges.begin();
  const auto edge_cb = [&edge_itr, &path_edges]() {
    return (edge_itr == path_edges.end()) ? GraphId{} : (*edge_itr++);
  };

  const auto label_cb = [&path, &recovered_inner_edges](const PathEdgeLabel& label) {
    path.emplace_back(label.mode(), label.cost(), label.edgeid(), 0, label.path_distance(),
                      label.restriction_idx(), label.transition_cost(),
                      recovered_inner_edges.count(label.edgeid()));
  };

  float source_pct;
  try {
    source_pct = find_percent_along(origin, path_edges.front());
  } catch (...) {
    throw std::logic_error("Could not find candidate edge used for origin label");
  }

  float target_pct;
  try {
    target_pct = find_percent_along(dest, path_edges.back());
  } catch (...) {
    throw std::logic_error("Could not find candidate edge used for destination label");
  }

  try {
    // γ_τ₀(p): re-sum c(u,v,τ) along the chosen topology; search used c on G, λ only bounded M.
    const bool invariant = options.date_time_type() == Options::invariant;
    const bool ignore_access_on_recost = options.costing_type() != Costing::truck_ban;
    recost_forward(graphreader, *costing_, edge_cb, label_cb, source_pct, target_pct, time_info,
                   invariant, ignore_access_on_recost);
  } catch (const std::exception& e) {
    LOG_ERROR(std::string("TDALT failed to recost final path: ") + e.what());
    return {};
  }

  return {std::move(path)};
}

} // namespace thor
} // namespace valhalla
