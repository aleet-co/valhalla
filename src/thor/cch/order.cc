#include "thor/cch/order.h"

#include <cstdio>
#include <fstream>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace valhalla {
namespace thor {
namespace cch {

namespace {

// Edge id space: base edges [0, num_base), shortcuts [num_base, num_base+scount).
// During contraction we track hop-length per connecting edge for edge-difference.
struct DynEdge {
  uint32_t edge_id; // into base or shortcut space
  uint32_t hops;    // number of base edges represented (for unit-weight order)
};

} // namespace

CchOrder BuildOrder(const CchGraph& g) {
  const uint32_t n = static_cast<uint32_t>(g.nodes.size());
  CchOrder order;
  order.num_base_edges = static_cast<uint32_t>(g.edges.size());
  order.rank.assign(n, 0);
  order.tile_build_hash = g.tile_build_hash;

  // Dynamic adjacency: out[u][v] = best DynEdge, in[v] = set of u.
  std::vector<std::unordered_map<uint32_t, DynEdge>> out(n), inc(n);
  std::vector<uint32_t> contracted_neighbors(n, 0);

  auto merge = [](std::unordered_map<uint32_t, DynEdge>& adj, uint32_t key, DynEdge e) {
    auto it = adj.find(key);
    if (it == adj.end() || e.hops < it->second.hops)
      adj[key] = e;
  };

  for (uint32_t ei = 0; ei < g.edges.size(); ++ei) {
    const auto& e = g.edges[ei];
    merge(out[e.u], e.v, DynEdge{ei, 1});
    inc[e.v].emplace(e.u, DynEdge{ei, 1});
  }

  // Propose shortcuts around v (no witness pruning: keep every needed shortcut).
  auto proposed = [&](uint32_t v) {
    std::vector<CchShortcut> props;
    for (const auto& [u, e_uv] : inc[v]) {
      if (u == v)
        continue;
      auto ituv = out[u].find(v);
      if (ituv == out[u].end())
        continue;
      for (const auto& [w, e_vw] : out[v]) {
        if (w == v || w == u)
          continue;
        CchShortcut sc;
        sc.u = u;
        sc.w = w;
        sc.middle = v;
        sc.left = ituv->second.edge_id;
        sc.right = e_vw.edge_id;
        props.push_back(sc);
      }
    }
    return props;
  };

  auto edge_difference = [&](uint32_t v) -> int {
    int added = static_cast<int>(proposed(v).size());
    int removed = static_cast<int>(inc[v].size() + out[v].size());
    return added - removed + static_cast<int>(contracted_neighbors[v]);
  };

  // Lazy priority queue keyed on edge-difference.
  std::priority_queue<std::pair<int, uint32_t>, std::vector<std::pair<int, uint32_t>>,
                      std::greater<>>
      pq;
  for (uint32_t v = 0; v < n; ++v)
    pq.push({edge_difference(v), v});

  std::vector<bool> done(n, false);
  uint32_t rank_counter = 0;

  while (!pq.empty()) {
    auto [key, v] = pq.top();
    pq.pop();
    if (done[v])
      continue;
    int cur = edge_difference(v);
    if (!pq.empty() && cur > pq.top().first) {
      pq.push({cur, v});
      continue;
    }

    // Contract v: materialize shortcuts.
    for (const auto& [u, e_uv] : inc[v]) {
      if (u == v)
        continue;
      auto ituv = out[u].find(v);
      if (ituv == out[u].end())
        continue;
      for (const auto& [w, e_vw] : out[v]) {
        if (w == v || w == u)
          continue;
        CchShortcut sc;
        sc.u = u;
        sc.w = w;
        sc.middle = v;
        sc.left = ituv->second.edge_id;
        sc.right = e_vw.edge_id;
        uint32_t sc_id = order.num_base_edges + static_cast<uint32_t>(order.shortcuts.size());
        uint32_t sc_hops = ituv->second.hops + e_vw.hops;
        order.shortcuts.push_back(sc);
        merge(out[u], w, DynEdge{sc_id, sc_hops});
        inc[w].emplace(u, DynEdge{sc_id, sc_hops});
      }
    }

    // Remove v from the dynamic graph.
    for (const auto& [w, _] : out[v]) {
      inc[w].erase(v);
      ++contracted_neighbors[w];
    }
    for (const auto& [u, _] : inc[v]) {
      out[u].erase(v);
      ++contracted_neighbors[u];
    }
    out[v].clear();
    inc[v].clear();

    order.rank[v] = rank_counter++;
    done[v] = true;
  }

  order.build_adjacency(g);
  return order;
}

void CchOrder::build_adjacency(const CchGraph& g) {
  const uint32_t n = static_cast<uint32_t>(rank.size());
  fwd_adj.assign(n, {});
  bwd_adj.assign(n, {});
  auto add = [&](uint32_t u, uint32_t v, uint32_t eid) {
    if (rank[v] > rank[u])
      fwd_adj[u].push_back({v, eid});
    else if (rank[u] > rank[v])
      bwd_adj[v].push_back({u, eid});
  };
  for (uint32_t ei = 0; ei < g.edges.size(); ++ei)
    add(g.edges[ei].u, g.edges[ei].v, ei);
  for (uint32_t si = 0; si < shortcuts.size(); ++si)
    add(shortcuts[si].u, shortcuts[si].w, num_base_edges + si);
}

namespace {
constexpr uint32_t kMagic = 0x48434356; // "VCCH"
constexpr uint32_t kVersion = 1;

template <typename T> void write_pod(std::ostream& os, const T& v) {
  os.write(reinterpret_cast<const char*>(&v), sizeof(T));
}
template <typename T> void read_pod(std::istream& is, T& v) {
  is.read(reinterpret_cast<char*>(&v), sizeof(T));
}
} // namespace

void CchOrder::save(const std::string& path) const {
  std::ofstream os(path, std::ios::binary);
  if (!os)
    throw std::runtime_error("cch: cannot open artifact for write: " + path);
  write_pod(os, kMagic);
  write_pod(os, kVersion);
  write_pod(os, tile_build_hash);
  write_pod(os, num_base_edges);
  uint64_t nn = rank.size();
  write_pod(os, nn);
  os.write(reinterpret_cast<const char*>(rank.data()), nn * sizeof(uint32_t));
  uint64_t sc = shortcuts.size();
  write_pod(os, sc);
  os.write(reinterpret_cast<const char*>(shortcuts.data()), sc * sizeof(CchShortcut));
}

CchOrder CchOrder::load(const std::string& path) {
  std::ifstream is(path, std::ios::binary);
  if (!is)
    throw std::runtime_error("cch: cannot open artifact for read: " + path);
  uint32_t magic = 0, version = 0;
  read_pod(is, magic);
  read_pod(is, version);
  if (magic != kMagic || version != kVersion)
    throw std::runtime_error("cch: bad artifact magic/version");
  CchOrder o;
  read_pod(is, o.tile_build_hash);
  read_pod(is, o.num_base_edges);
  uint64_t nn = 0;
  read_pod(is, nn);
  o.rank.resize(nn);
  is.read(reinterpret_cast<char*>(o.rank.data()), nn * sizeof(uint32_t));
  uint64_t sc = 0;
  read_pod(is, sc);
  o.shortcuts.resize(sc);
  is.read(reinterpret_cast<char*>(o.shortcuts.data()), sc * sizeof(CchShortcut));
  return o; // caller rebuilds adjacency via build_adjacency(graph)
}

} // namespace cch
} // namespace thor
} // namespace valhalla
