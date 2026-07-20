#include "thor/cch/cch_graph.h"

#include <valhalla/baldr/admin.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/midgard/pointll.h>

#include "midgard/logging.h"

#include <algorithm>
#include <atomic>
#include <boost/property_tree/ptree.hpp>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <thread>
#include <vector>

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace thor {
namespace cch {

namespace {

using clock = std::chrono::steady_clock;

double elapsed_s(clock::time_point t0) {
  return std::chrono::duration<double>(clock::now() - t0).count();
}

uint32_t resolve_concurrency(uint32_t concurrency) {
  if (concurrency == 0) {
    const unsigned hc = std::thread::hardware_concurrency();
    return hc == 0 ? 1u : hc;
  }
  return concurrency;
}

std::vector<GraphId> collect_tiles(GraphReader& reader, const std::set<uint32_t>& levels) {
  std::vector<GraphId> tiles;
  for (const auto& tile_id : reader.GetTileSet()) {
    if (levels.find(tile_id.level()) != levels.end())
      tiles.push_back(tile_id);
  }
  std::sort(tiles.begin(), tiles.end());
  return tiles;
}

std::vector<CchNode> extract_nodes(GraphReader& reader, const GraphId& tile_id) {
  std::vector<CchNode> nodes;
  auto tile = reader.GetGraphTile(tile_id);
  if (tile == nullptr)
    return nodes;
  const uint32_t nc = tile->header()->nodecount();
  nodes.reserve(nc);
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
    nodes.push_back(std::move(cn));
  }
  reader.Clear();
  return nodes;
}

std::vector<CchBaseEdge> extract_edges(GraphReader& reader,
                                       const GraphId& tile_id,
                                       const CchGraph& g,
                                       uint8_t max_roadclass,
                                       bool hgv_only) {
  std::vector<CchBaseEdge> edges;
  auto tile = reader.GetGraphTile(tile_id);
  if (tile == nullptr)
    return edges;
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
      if (hgv_only && !(de->forwardaccess() & kTruckAccess))
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
      edges.push_back(e);
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
      edges.push_back(e);
    }
  }
  reader.Clear();
  return edges;
}

void log_tile_progress(const char* pass,
                       size_t tiles_done,
                       size_t tiles_total,
                       size_t count,
                       const char* count_name,
                       clock::time_point t0,
                       clock::time_point* last_log) {
  const auto now = clock::now();
  if (tiles_done != tiles_total &&
      std::chrono::duration<double>(now - *last_log).count() < 15.0 &&
      !(tiles_total > 0 && tiles_done % std::max<size_t>(1, tiles_total / 20) == 0)) {
    return;
  }
  const double pct = tiles_total ? 100.0 * tiles_done / tiles_total : 100.0;
  LOG_INFO(std::string("cch graph: ") + pass + "  tiles=" + std::to_string(tiles_done) + "/" +
           std::to_string(tiles_total) + " (" + std::to_string(static_cast<int>(pct)) + "%)  " +
           count_name + "=" + std::to_string(count) +
           "  elapsed_s=" + std::to_string(elapsed_s(t0)));
  *last_log = now;
}

CchGraph build_from_tiles(const std::vector<GraphId>& tiles,
                          const boost::property_tree::ptree* mjolnir_config,
                          GraphReader* single_reader,
                          uint8_t max_roadclass,
                          uint32_t concurrency,
                          uint64_t tile_build_hash,
                          bool hgv_only) {
  CchGraph g;
  const auto t0 = clock::now();
  const size_t tiles_total = tiles.size();
  const uint32_t threads =
      single_reader ? 1u : resolve_concurrency(concurrency);

  LOG_INFO("cch graph: scanning " + std::to_string(tiles_total) +
           " tiles (pass 1/2: nodes)  threads=" + std::to_string(threads));

  std::vector<std::vector<CchNode>> nodes_by_tile(tiles_total);
  {
    std::atomic<size_t> next{0};
    std::atomic<size_t> tiles_done{0};
    std::atomic<size_t> nodes_seen{0};
    auto last_log = t0;
    std::mutex log_mu;

    auto worker = [&](GraphReader& reader) {
      for (;;) {
        const size_t i = next.fetch_add(1, std::memory_order_relaxed);
        if (i >= tiles_total)
          break;
        nodes_by_tile[i] = extract_nodes(reader, tiles[i]);
        const size_t node_count =
            nodes_seen.fetch_add(nodes_by_tile[i].size(), std::memory_order_relaxed) +
            nodes_by_tile[i].size();
        const size_t done = tiles_done.fetch_add(1, std::memory_order_relaxed) + 1;
        std::lock_guard<std::mutex> lock(log_mu);
        log_tile_progress("pass 1/2 nodes", done, tiles_total, node_count, "nodes", t0, &last_log);
      }
    };

    if (single_reader) {
      worker(*single_reader);
    } else {
      std::vector<std::thread> pool;
      pool.reserve(threads);
      for (uint32_t t = 0; t < threads; ++t) {
        pool.emplace_back([&, mjolnir_config]() {
          GraphReader reader(*mjolnir_config);
          worker(reader);
        });
      }
      for (auto& th : pool)
        th.join();
    }
  }

  size_t node_total = 0;
  for (const auto& bucket : nodes_by_tile)
    node_total += bucket.size();
  g.nodes.reserve(node_total);
  for (auto& bucket : nodes_by_tile) {
    for (auto& cn : bucket) {
      g.set_index(cn.graph_id, static_cast<uint32_t>(g.nodes.size()));
      g.nodes.push_back(std::move(cn));
    }
    bucket.clear();
  }
  nodes_by_tile.clear();

  LOG_INFO("cch graph: pass 1/2 done  nodes=" + std::to_string(g.nodes.size()) +
           "  elapsed_s=" + std::to_string(elapsed_s(t0)));
  LOG_INFO("cch graph: pass 2/2 edges (truck-class + transitions)...");

  std::vector<std::vector<CchBaseEdge>> edges_by_tile(tiles_total);
  {
    std::atomic<size_t> next{0};
    std::atomic<size_t> tiles_done{0};
    std::atomic<size_t> edges_seen{0};
    auto last_log = clock::now();
    std::mutex log_mu;

    auto worker = [&](GraphReader& reader) {
      for (;;) {
        const size_t i = next.fetch_add(1, std::memory_order_relaxed);
        if (i >= tiles_total)
          break;
        edges_by_tile[i] = extract_edges(reader, tiles[i], g, max_roadclass, hgv_only);
        const size_t edge_count =
            edges_seen.fetch_add(edges_by_tile[i].size(), std::memory_order_relaxed) +
            edges_by_tile[i].size();
        const size_t done = tiles_done.fetch_add(1, std::memory_order_relaxed) + 1;
        std::lock_guard<std::mutex> lock(log_mu);
        log_tile_progress("pass 2/2 edges", done, tiles_total, edge_count, "edges", t0, &last_log);
      }
    };

    if (single_reader) {
      worker(*single_reader);
    } else {
      std::vector<std::thread> pool;
      pool.reserve(threads);
      for (uint32_t t = 0; t < threads; ++t) {
        pool.emplace_back([&, mjolnir_config]() {
          GraphReader reader(*mjolnir_config);
          worker(reader);
        });
      }
      for (auto& th : pool)
        th.join();
    }
  }

  size_t edge_total = 0;
  for (const auto& bucket : edges_by_tile)
    edge_total += bucket.size();
  g.edges.reserve(edge_total);
  for (auto& bucket : edges_by_tile) {
    g.edges.insert(g.edges.end(), bucket.begin(), bucket.end());
    bucket.clear();
  }

  g.tile_build_hash = tile_build_hash;
  LOG_INFO("cch graph: building CSR adjacency...");
  const auto t_csr = clock::now();
  g.build_csr();
  LOG_INFO("cch graph: done  nodes=" + std::to_string(g.nodes.size()) +
           " edges=" + std::to_string(g.edges.size()) +
           " csr_s=" + std::to_string(elapsed_s(t_csr)) +
           " total_s=" + std::to_string(elapsed_s(t0)));
  return g;
}

} // namespace

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

CchGraph BuildTruckGraph(const boost::property_tree::ptree& mjolnir_config,
                         const std::set<uint32_t>& levels,
                         uint8_t max_roadclass,
                         uint32_t concurrency,
                         bool hgv_only) {
  GraphReader probe(mjolnir_config);
  auto tiles = collect_tiles(probe, levels);
  const uint64_t hash = probe.GetTileSet().size();
  return build_from_tiles(tiles, &mjolnir_config, nullptr, max_roadclass, concurrency, hash,
                          hgv_only);
}

CchGraph BuildTruckGraph(GraphReader& reader,
                         const std::set<uint32_t>& levels,
                         uint8_t max_roadclass,
                         bool hgv_only) {
  auto tiles = collect_tiles(reader, levels);
  const uint64_t hash = reader.GetTileSet().size();
  return build_from_tiles(tiles, nullptr, &reader, max_roadclass, 1, hash, hgv_only);
}

} // namespace cch
} // namespace thor
} // namespace valhalla
