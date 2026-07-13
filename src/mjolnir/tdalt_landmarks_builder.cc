#include "mjolnir/tdalt_landmarks_builder.h"

#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/nodetransition.h"
#include "baldr/tdalt_landmarks.h"
#include "midgard/logging.h"
#include "sif/lower_bound_cost.h"

#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace valhalla {
namespace mjolnir {
namespace {

using baldr::DirectedEdge;
using baldr::GraphId;
using baldr::GraphReader;
using baldr::NodeInfo;
using baldr::NodeTransition;
using baldr::Use;

// Finest driving hierarchy level (TileHierarchy "local" = level 2).
constexpr uint8_t kLocalLevel = 2;

// Europe-scale subsampling: keep every Nth level-0 node, capped at kMaxCandidates.
constexpr uint32_t kCandidateSubsampleStride = 100;
constexpr uint32_t kMaxCandidates = 50000;

constexpr float kMaxLandmarkDistance = 1e18f;

using DistMap = std::unordered_map<uint64_t, float>;

bool EdgeTraversableForward(const DirectedEdge* edge) {
  return !edge->is_shortcut() && (edge->forwardaccess() & baldr::kAutoAccess) != 0 &&
         edge->use() != Use::kRailFerry;
}

bool EdgeTraversableReverse(const DirectedEdge* edge) {
  return !edge->is_shortcut() && (edge->reverseaccess() & baldr::kAutoAccess) != 0 &&
         edge->use() != Use::kRailFerry;
}

DistMap RunLambdaDijkstra(GraphReader& reader, const GraphId& source, const bool forward) {
  DistMap dist;
  using QueueItem = std::pair<float, uint64_t>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;

  const uint64_t source_value = source.value;
  dist.emplace(source_value, 0.f);
  queue.emplace(0.f, source_value);

  while (!queue.empty()) {
    const auto [cost, node_value] = queue.top();
    queue.pop();

    const auto found = dist.find(node_value);
    if (found == dist.end() || cost > found->second) {
      continue;
    }

    const GraphId node(node_value);
    if (node.level() != kLocalLevel) {
      continue;
    }

    auto tile = reader.GetGraphTile(node);
    if (tile == nullptr) {
      continue;
    }

    const NodeInfo* nodeinfo = tile->node(node);

    GraphId edge_id(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* edge = tile->directededge(edge_id);
    for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++edge, ++edge_id) {
      if (forward) {
        if (!EdgeTraversableForward(edge)) {
          continue;
        }

        const float edge_cost = sif::LowerBoundCost::seconds(edge);
        const GraphId endnode = edge->endnode();
        if (endnode.level() != kLocalLevel) {
          continue;
        }

        const uint64_t end_value = endnode.value;
        const float new_cost = cost + edge_cost;
        const auto end_it = dist.find(end_value);
        if (end_it == dist.end() || new_cost < end_it->second) {
          dist[end_value] = new_cost;
          queue.emplace(new_cost, end_value);
        }
      } else {
        if (!EdgeTraversableReverse(edge)) {
          continue;
        }

        baldr::graph_tile_ptr endtile =
            edge->leaves_tile() ? reader.GetGraphTile(edge->endnode()) : tile;
        if (endtile == nullptr) {
          continue;
        }

        const GraphId opp_edge_id = endtile->GetOpposingEdgeId(edge);
        if (!opp_edge_id.is_valid()) {
          continue;
        }

        const DirectedEdge* opp_edge = endtile->directededge(opp_edge_id);
        if (opp_edge == nullptr || opp_edge->is_shortcut()) {
          continue;
        }

        const float edge_cost = sif::LowerBoundCost::seconds(opp_edge);
        const GraphId pred_node = opp_edge->endnode();
        if (pred_node.level() != kLocalLevel) {
          continue;
        }

        const uint64_t pred_value = pred_node.value;
        const float new_cost = cost + edge_cost;
        const auto pred_it = dist.find(pred_value);
        if (pred_it == dist.end() || new_cost < pred_it->second) {
          dist[pred_value] = new_cost;
          queue.emplace(new_cost, pred_value);
        }
      }
    }

    // Conservative v1: no upward hierarchy transitions.
    if (nodeinfo->transition_count() > 0) {
      const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
      for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
        if (trans->up()) {
          continue;
        }

        const GraphId trans_node = trans->endnode();
        if (trans_node.level() != kLocalLevel) {
          continue;
        }

        const uint64_t trans_value = trans_node.value;
        const auto trans_it = dist.find(trans_value);
        if (trans_it == dist.end() || cost < trans_it->second) {
          dist[trans_value] = cost;
          queue.emplace(cost, trans_value);
        }
      }
    }
  }

  return dist;
}

std::vector<GraphId> CollectCandidateNodes(GraphReader& reader) {
  std::vector<GraphId> candidates;
  candidates.reserve(kMaxCandidates);

  uint32_t seen = 0;
  for (const auto& tile_id : reader.GetTileSet()) {
    if (tile_id.level() != kLocalLevel) {
      continue;
    }

    auto tile = reader.GetGraphTile(tile_id);
    if (tile == nullptr) {
      continue;
    }

    for (uint32_t node_index = 0; node_index < tile->header()->nodecount(); ++node_index) {
      if (seen++ % kCandidateSubsampleStride != 0) {
        continue;
      }

      candidates.emplace_back(tile_id.tileid(), kLocalLevel, node_index);
      if (candidates.size() >= kMaxCandidates) {
        LOG_INFO("TDALT landmark candidates capped at {} (subsample stride={})", kMaxCandidates,
                 kCandidateSubsampleStride);
        return candidates;
      }
    }
  }

  LOG_INFO("TDALT landmark candidate pool: {} nodes (subsample stride={})", candidates.size(),
           kCandidateSubsampleStride);
  return candidates;
}

float ApproxLambdaSeconds(GraphReader& reader, const GraphId& from, const GraphId& to) {
  const auto from_tile = reader.GetGraphTile(from);
  const auto to_tile = reader.GetGraphTile(to);
  if (from_tile == nullptr || to_tile == nullptr) {
    return kMaxLandmarkDistance;
  }

  const float meters = from_tile->get_node_ll(from).Distance(to_tile->get_node_ll(to));
  constexpr float kMaxHighwaySpeedMps = 130.f * 1000.f / 3600.f;
  return meters / kMaxHighwaySpeedMps;
}

float MinLandmarkDistance(GraphReader& reader,
                          const GraphId& candidate,
                          const std::vector<GraphId>& landmarks,
                          const std::vector<DistMap>& forward_from_landmark) {
  float min_dist = kMaxLandmarkDistance;
  for (size_t landmark = 0; landmark < landmarks.size(); ++landmark) {
    const auto it = forward_from_landmark[landmark].find(candidate.value);
    const float graph_dist =
        (it != forward_from_landmark[landmark].end()) ? it->second : kMaxLandmarkDistance;
    const float approx_dist = ApproxLambdaSeconds(reader, landmarks[landmark], candidate);
    min_dist = std::min(min_dist, std::min(graph_dist, approx_dist));
  }
  return min_dist;
}

std::vector<GraphId> SelectLandmarksMaxCover(GraphReader& reader,
                                             const std::vector<GraphId>& candidates,
                                             const uint32_t landmark_count) {
  if (candidates.empty()) {
    throw std::runtime_error("No level-" + std::to_string(kLocalLevel) +
                             " candidate nodes found for TDALT landmark selection");
  }

  std::vector<GraphId> landmarks;
  landmarks.reserve(landmark_count);

  std::vector<bool> selected(candidates.size(), false);
  std::vector<DistMap> forward_from_landmark;

  // Seed landmark: farthest candidate from the first subsampled node (by λ straight-line proxy).
  size_t first_landmark = 0;
  float farthest = -1.f;
  for (size_t i = 1; i < candidates.size(); ++i) {
    const float dist = ApproxLambdaSeconds(reader, candidates.front(), candidates[i]);
    if (dist > farthest) {
      farthest = dist;
      first_landmark = i;
    }
  }

  landmarks.push_back(candidates[first_landmark]);
  selected[first_landmark] = true;
  forward_from_landmark.push_back(RunLambdaDijkstra(reader, landmarks.back(), true));
  LOG_INFO("TDALT landmark 0: {} (forward reach={} nodes)", landmarks.back().value,
           forward_from_landmark.back().size());

  while (landmarks.size() < landmark_count) {
    float best_min_dist = -1.f;
    size_t best_candidate = 0;

    for (size_t i = 0; i < candidates.size(); ++i) {
      if (selected[i]) {
        continue;
      }

      const float min_dist =
          MinLandmarkDistance(reader, candidates[i], landmarks, forward_from_landmark);
      if (min_dist > best_min_dist) {
        best_min_dist = min_dist;
        best_candidate = i;
      }
    }

    landmarks.push_back(candidates[best_candidate]);
    selected[best_candidate] = true;
    forward_from_landmark.push_back(RunLambdaDijkstra(reader, landmarks.back(), true));
    LOG_INFO("TDALT landmark {}: {} (min-cover dist={:.1f}s, forward reach={} nodes)",
             landmarks.size() - 1, landmarks.back().value, best_min_dist,
             forward_from_landmark.back().size());
  }

  return landmarks;
}

float LookupDistance(const DistMap& dist, const uint64_t node_value) {
  const auto it = dist.find(node_value);
  return (it != dist.end()) ? it->second : kMaxLandmarkDistance;
}

void WriteLandmarkFile(const std::string& landmarks_file,
                       const uint32_t landmark_count,
                       const std::vector<uint64_t>& node_values,
                       const std::vector<DistMap>& dist_to,
                       const std::vector<DistMap>& dist_from) {
  const std::filesystem::path output_path(landmarks_file);
  if (output_path.has_parent_path()) {
    std::filesystem::create_directories(output_path.parent_path());
  }

  std::ofstream outfile(landmarks_file, std::ios::binary | std::ios::trunc);
  if (!outfile) {
    throw std::runtime_error("Unable to open TDALT landmarks output file: " + landmarks_file);
  }

  baldr::TdaltLandmarksHeader header{};
  header.magic = baldr::kTdaltLandmarksMagic;
  header.version = baldr::kTdaltLandmarksVersion;
  header.landmark_count = landmark_count;
  header.node_count = static_cast<uint32_t>(node_values.size());
  header.distances_offset = sizeof(baldr::TdaltLandmarksHeader);

  outfile.write(reinterpret_cast<const char*>(&header), sizeof(header));
  if (!outfile) {
    throw std::runtime_error("Failed writing TDALT landmarks header to: " + landmarks_file);
  }

  const size_t record_stride =
      sizeof(uint32_t) * 2 + 2 * static_cast<size_t>(landmark_count) * sizeof(float);
  std::vector<float> dist_to_row(landmark_count, kMaxLandmarkDistance);
  std::vector<float> dist_from_row(landmark_count, kMaxLandmarkDistance);

  for (const uint64_t node_value : node_values) {
    const uint32_t graphid_lo = static_cast<uint32_t>(node_value & 0xffffffffu);
    const uint32_t graphid_hi = static_cast<uint32_t>(node_value >> 32);

    for (uint32_t landmark = 0; landmark < landmark_count; ++landmark) {
      dist_to_row[landmark] = LookupDistance(dist_to[landmark], node_value);
      dist_from_row[landmark] = LookupDistance(dist_from[landmark], node_value);
    }

    outfile.write(reinterpret_cast<const char*>(&graphid_lo), sizeof(graphid_lo));
    outfile.write(reinterpret_cast<const char*>(&graphid_hi), sizeof(graphid_hi));
    outfile.write(reinterpret_cast<const char*>(dist_to_row.data()),
                  dist_to_row.size() * sizeof(float));
    outfile.write(reinterpret_cast<const char*>(dist_from_row.data()),
                  dist_from_row.size() * sizeof(float));
    if (!outfile) {
      throw std::runtime_error("Failed writing TDALT landmark node record to: " + landmarks_file);
    }
  }

  const size_t expected_size = sizeof(baldr::TdaltLandmarksHeader) + node_values.size() * record_stride;
  LOG_INFO("Wrote TDALT landmarks: {} landmarks, {} nodes, {} bytes", landmark_count,
           node_values.size(), expected_size);
}

} // namespace

void build_tdalt_landmarks(const boost::property_tree::ptree& config) {
  const auto& mjolnir = config.get_child("mjolnir");
  const auto& tdalt = mjolnir.get_child("tdalt");

  const uint32_t landmark_count = tdalt.get<uint32_t>("landmark_count");
  const std::string landmarks_file = tdalt.get<std::string>("landmarks_file");

  if (landmark_count == 0) {
    throw std::runtime_error("mjolnir.tdalt.landmark_count must be greater than zero");
  }

  GraphReader reader(mjolnir);
  const auto candidates = CollectCandidateNodes(reader);
  const auto landmarks = SelectLandmarksMaxCover(reader, candidates, landmark_count);

  std::vector<DistMap> dist_to(landmark_count);
  std::vector<DistMap> dist_from(landmark_count);

  std::unordered_set<uint64_t> reached_nodes;
  for (uint32_t landmark = 0; landmark < landmark_count; ++landmark) {
    LOG_INFO("TDALT Dijkstra from landmark {} ({})", landmark, landmarks[landmark].value);
    dist_from[landmark] = RunLambdaDijkstra(reader, landmarks[landmark], true);
    dist_to[landmark] = RunLambdaDijkstra(reader, landmarks[landmark], false);

    for (const auto& entry : dist_from[landmark]) {
      reached_nodes.insert(entry.first);
    }
    for (const auto& entry : dist_to[landmark]) {
      reached_nodes.insert(entry.first);
    }
  }

  std::vector<uint64_t> node_values(reached_nodes.begin(), reached_nodes.end());
  std::sort(node_values.begin(), node_values.end());

  WriteLandmarkFile(landmarks_file, landmark_count, node_values, dist_to, dist_from);

  baldr::TDALTLandmarkIndex index(landmarks_file);
  if (!index.available()) {
    throw std::runtime_error("TDALT landmark index failed to load after build: " + landmarks_file);
  }

  if (!landmarks.empty()) {
    const GraphId probe(landmarks.front().value);
    const float probe_to = index.distance_to(probe, 0);
    const float probe_from = index.distance_from(probe, 0);
    if (probe_to >= kMaxLandmarkDistance || probe_from >= kMaxLandmarkDistance) {
      throw std::runtime_error("TDALT landmark probe node has unreachable distances after build");
    }
    LOG_INFO("TDALT landmark loader OK (probe node {} dist_to[0]={:.1f}s dist_from[0]={:.1f}s)",
             probe.value, probe_to, probe_from);
  }
}

} // namespace mjolnir
} // namespace valhalla
