#include "thor/cch/cch_graph.h"

#include <valhalla/baldr/admin.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/midgard/pointll.h>

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace thor {
namespace cch {

void CchGraph::build_csr() {
  const size_t n = nodes.size();
  out_offsets.assign(n + 1, 0);
  in_offsets.assign(n + 1, 0);
  for (const auto& e : edges) {
    ++out_offsets[e.u + 1];
    ++in_offsets[e.v + 1];
  }
  for (size_t i = 0; i < n; ++i) {
    out_offsets[i + 1] += out_offsets[i];
    in_offsets[i + 1] += in_offsets[i];
  }
  out_edges.assign(edges.size(), 0);
  in_edges.assign(edges.size(), 0);
  std::vector<uint32_t> ocur(out_offsets.begin(), out_offsets.end() - 1);
  std::vector<uint32_t> icur(in_offsets.begin(), in_offsets.end() - 1);
  for (uint32_t ei = 0; ei < edges.size(); ++ei) {
    out_edges[ocur[edges[ei].u]++] = ei;
    in_edges[icur[edges[ei].v]++] = ei;
  }
}

CchGraph BuildTruckGraph(GraphReader& reader,
                         const std::set<uint32_t>& levels,
                         uint8_t max_roadclass) {
  CchGraph g;

  // Pass 1: nodes (index every node on the requested levels).
  for (const auto& tile_id : reader.GetTileSet()) {
    if (levels.find(tile_id.level()) == levels.end())
      continue;
    auto tile = reader.GetGraphTile(tile_id);
    if (tile == nullptr)
      continue;
    const uint32_t nc = tile->header()->nodecount();
    for (uint32_t idx = 0; idx < nc; ++idx) {
      GraphId nid(tile_id.tileid(), tile_id.level(), idx);
      const NodeInfo* node = tile->node(nid);
      const PointLL ll = tile->get_node_ll(nid);
      CchNode cn;
      cn.graph_id = nid.value;
      cn.lat = ll.lat();
      cn.lon = ll.lng();
      cn.tz_index = node->timezone();
      const Admin* admin = tile->admin(node->admin_index());
      if (admin != nullptr)
        cn.country = admin->country_iso();
      g.set_index(cn.graph_id, static_cast<uint32_t>(g.nodes.size()));
      g.nodes.push_back(std::move(cn));
    }
    reader.Clear();
  }

  // Pass 2: edges (regular directed edges + zero-time transition connectors).
  for (const auto& tile_id : reader.GetTileSet()) {
    if (levels.find(tile_id.level()) == levels.end())
      continue;
    auto tile = reader.GetGraphTile(tile_id);
    if (tile == nullptr)
      continue;
    const uint32_t nc = tile->header()->nodecount();
    for (uint32_t idx = 0; idx < nc; ++idx) {
      GraphId nid(tile_id.tileid(), tile_id.level(), idx);
      const int ui = g.index_of(nid.value);
      if (ui < 0)
        continue;
      const NodeInfo* node = tile->node(nid);
      for (uint32_t i = 0; i < node->edge_count(); ++i) {
        const DirectedEdge* de = tile->directededge(node->edge_index() + i);
        if (de->is_shortcut())
          continue;
        if (static_cast<uint8_t>(de->classification()) > max_roadclass)
          continue;
        const int vi = g.index_of(de->endnode().value);
        if (vi < 0)
          continue;
        const uint32_t speed_kph = de->truck_speed() ? de->truck_speed() : de->speed();
        if (speed_kph == 0)
          continue;
        const uint32_t time_s =
            static_cast<uint32_t>(de->length() / (speed_kph * 1000.0 / 3600.0) + 0.5);
        CchBaseEdge e;
        e.u = static_cast<uint32_t>(ui);
        e.v = static_cast<uint32_t>(vi);
        e.time_s = time_s;
        e.roadclass = static_cast<uint8_t>(de->classification());
        g.edges.push_back(e);
      }
      const uint32_t tc = node->transition_count();
      for (uint32_t i = 0; i < tc; ++i) {
        const NodeTransition* trans = tile->transition(node->transition_index() + i);
        const int vi = g.index_of(trans->endnode().value);
        if (vi < 0)
          continue;
        CchBaseEdge e;
        e.u = static_cast<uint32_t>(ui);
        e.v = static_cast<uint32_t>(vi);
        e.time_s = 0;
        e.roadclass = 255; // transition connector, never banned
        g.edges.push_back(e);
      }
    }
    reader.Clear();
  }

  g.tile_build_hash = reader.GetTileSet().size(); // cheap topology fingerprint for MVP
  g.build_csr();
  return g;
}

} // namespace cch
} // namespace thor
} // namespace valhalla
