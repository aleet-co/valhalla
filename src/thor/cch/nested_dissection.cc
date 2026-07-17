#include "thor/cch/nested_dissection.h"

#include "midgard/logging.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

namespace valhalla {
namespace thor {
namespace cch {
namespace {

constexpr uint32_t kLeafSize = 16;
constexpr double kTerminalFraction = 0.25;
constexpr double kMinBalance = 0.20;
constexpr uint32_t kInfCap = std::numeric_limits<uint32_t>::max() / 4;

uint32_t resolve_concurrency(uint32_t concurrency) {
  if (concurrency == 0) {
    const unsigned hc = std::thread::hardware_concurrency();
    return hc == 0 ? 1u : hc;
  }
  return concurrency;
}

struct UndirectedGraph {
  uint32_t n = 0;
  std::vector<uint32_t> first_out;
  std::vector<uint32_t> head;
};

// Compact undirected CSR over the whole CchGraph (one undirected edge per pair).
UndirectedGraph build_undirected(const CchGraph& g) {
  const uint32_t n = static_cast<uint32_t>(g.nodes.size());
  std::vector<std::vector<uint32_t>> adj(n);
  for (const auto& e : g.edges) {
    if (e.u == e.v)
      continue;
    adj[e.u].push_back(e.v);
    adj[e.v].push_back(e.u);
  }
  UndirectedGraph ug;
  ug.n = n;
  ug.first_out.assign(n + 1, 0);
  for (uint32_t v = 0; v < n; ++v) {
    auto& a = adj[v];
    std::sort(a.begin(), a.end());
    a.erase(std::unique(a.begin(), a.end()), a.end());
    ug.first_out[v + 1] = ug.first_out[v] + static_cast<uint32_t>(a.size());
  }
  ug.head.resize(ug.first_out[n]);
  for (uint32_t v = 0; v < n; ++v) {
    std::copy(adj[v].begin(), adj[v].end(), ug.head.begin() + ug.first_out[v]);
  }
  return ug;
}

struct Fragment {
  std::vector<uint32_t> nodes; // global ids
  std::vector<uint32_t> first_out;
  std::vector<uint32_t> head; // local ids
};

Fragment make_fragment(const UndirectedGraph& ug, const std::vector<uint32_t>& nodes) {
  Fragment f;
  f.nodes = nodes;
  const uint32_t n = static_cast<uint32_t>(nodes.size());
  std::vector<uint32_t> global_to_local(ug.n, UINT32_MAX);
  for (uint32_t i = 0; i < n; ++i)
    global_to_local[nodes[i]] = i;

  f.first_out.assign(n + 1, 0);
  std::vector<uint32_t> deg(n, 0);
  for (uint32_t i = 0; i < n; ++i) {
    const uint32_t g = nodes[i];
    for (uint32_t ei = ug.first_out[g]; ei < ug.first_out[g + 1]; ++ei) {
      const uint32_t nb = ug.head[ei];
      if (global_to_local[nb] != UINT32_MAX)
        ++deg[i];
    }
  }
  for (uint32_t i = 0; i < n; ++i)
    f.first_out[i + 1] = f.first_out[i] + deg[i];
  f.head.resize(f.first_out[n]);
  std::vector<uint32_t> cursor = f.first_out;
  for (uint32_t i = 0; i < n; ++i) {
    const uint32_t g = nodes[i];
    for (uint32_t ei = ug.first_out[g]; ei < ug.first_out[g + 1]; ++ei) {
      const uint32_t nb = ug.head[ei];
      const uint32_t loc = global_to_local[nb];
      if (loc != UINT32_MAX)
        f.head[cursor[i]++] = loc;
    }
  }
  return f;
}

struct Dinic {
  struct Edge {
    uint32_t to;
    uint32_t rev;
    uint32_t cap;
  };
  std::vector<std::vector<Edge>> g;
  std::vector<int> level;
  std::vector<uint32_t> it;

  explicit Dinic(uint32_t n) : g(n), level(n), it(n) {
  }

  void add_edge(uint32_t u, uint32_t v, uint32_t c) {
    Edge a{v, static_cast<uint32_t>(g[v].size()), c};
    Edge b{u, static_cast<uint32_t>(g[u].size()), 0};
    g[u].push_back(a);
    g[v].push_back(b);
  }

  bool bfs(uint32_t s, uint32_t t) {
    std::fill(level.begin(), level.end(), -1);
    std::queue<uint32_t> q;
    level[s] = 0;
    q.push(s);
    while (!q.empty()) {
      const uint32_t v = q.front();
      q.pop();
      for (const auto& e : g[v]) {
        if (e.cap > 0 && level[e.to] < 0) {
          level[e.to] = level[v] + 1;
          q.push(e.to);
        }
      }
    }
    return level[t] >= 0;
  }

  uint32_t dfs(uint32_t v, uint32_t t, uint32_t f) {
    if (v == t)
      return f;
    for (uint32_t& i = it[v]; i < g[v].size(); ++i) {
      Edge& e = g[v][i];
      if (e.cap > 0 && level[v] < level[e.to]) {
        const uint32_t d = dfs(e.to, t, std::min(f, e.cap));
        if (d > 0) {
          e.cap -= d;
          g[e.to][e.rev].cap += d;
          return d;
        }
      }
    }
    return 0;
  }

  uint32_t max_flow(uint32_t s, uint32_t t) {
    uint32_t flow = 0;
    while (bfs(s, t)) {
      std::fill(it.begin(), it.end(), 0);
      for (;;) {
        const uint32_t f = dfs(s, t, kInfCap);
        if (f == 0)
          break;
        flow += f;
      }
    }
    return flow;
  }

  // Nodes reachable from s in residual graph.
  std::vector<uint8_t> source_side(uint32_t s) const {
    std::vector<uint8_t> reach(g.size(), 0);
    std::queue<uint32_t> q;
    reach[s] = 1;
    q.push(s);
    while (!q.empty()) {
      const uint32_t v = q.front();
      q.pop();
      for (const auto& e : g[v]) {
        if (e.cap > 0 && !reach[e.to]) {
          reach[e.to] = 1;
          q.push(e.to);
        }
      }
    }
    return reach;
  }
};

struct Partition {
  std::vector<uint32_t> side_a;
  std::vector<uint32_t> side_b;
  std::vector<uint32_t> separator;
  bool ok = false;
};

Partition partition_inertial_flow(const Fragment& f,
                                  const CchGraph& g,
                                  const std::vector<double>& angles) {
  const uint32_t n = static_cast<uint32_t>(f.nodes.size());
  Partition best;
  if (n < 3)
    return best;

  const uint32_t n_src = std::max(1u, static_cast<uint32_t>(n * kTerminalFraction));
  const uint32_t n_tgt = n_src;
  if (n_src + n_tgt >= n)
    return best;

  std::vector<uint32_t> order(n);
  for (uint32_t i = 0; i < n; ++i)
    order[i] = i;

  for (double angle : angles) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    std::vector<double> score(n);
    for (uint32_t i = 0; i < n; ++i) {
      const auto& node = g.nodes[f.nodes[i]];
      score[i] = node.lon * c + node.lat * s;
    }
    std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
      if (score[a] != score[b])
        return score[a] < score[b];
      return f.nodes[a] < f.nodes[b];
    });

    // Flow network: local nodes [0,n), super-source n, super-sink n+1.
    // One undirected unit-capacity edge → directed arc + zero reverse (residual).
    const uint32_t S = n;
    const uint32_t T = n + 1;
    Dinic dinic(n + 2);
    for (uint32_t u = 0; u < n; ++u) {
      for (uint32_t ei = f.first_out[u]; ei < f.first_out[u + 1]; ++ei) {
        const uint32_t v = f.head[ei];
        if (u < v)
          dinic.add_edge(u, v, 1);
      }
    }
    for (uint32_t i = 0; i < n_src; ++i)
      dinic.add_edge(S, order[i], kInfCap);
    for (uint32_t i = 0; i < n_tgt; ++i)
      dinic.add_edge(order[n - 1 - i], T, kInfCap);

    dinic.max_flow(S, T);
    auto reach = dinic.source_side(S);

    std::vector<uint8_t> in_S(n, 0);
    uint32_t size_S = 0;
    for (uint32_t i = 0; i < n; ++i) {
      if (reach[i]) {
        in_S[i] = 1;
        ++size_S;
      }
    }
    if (size_S == 0 || size_S == n)
      continue;

    // Node separator = source-side boundary (neighbors outside S).
    std::vector<uint8_t> is_sep(n, 0);
    std::vector<uint32_t> sep, a, b;
    for (uint32_t u = 0; u < n; ++u) {
      if (!in_S[u])
        continue;
      for (uint32_t ei = f.first_out[u]; ei < f.first_out[u + 1]; ++ei) {
        if (!in_S[f.head[ei]]) {
          is_sep[u] = 1;
          break;
        }
      }
    }
    for (uint32_t i = 0; i < n; ++i) {
      if (is_sep[i])
        sep.push_back(f.nodes[i]);
      else if (in_S[i])
        a.push_back(f.nodes[i]);
      else
        b.push_back(f.nodes[i]);
    }

    const double bal =
        static_cast<double>(std::min(a.size(), b.size())) / static_cast<double>(n);
    if (a.empty() || b.empty() || bal < kMinBalance)
      continue;

    const bool better = !best.ok || sep.size() < best.separator.size() ||
                        (sep.size() == best.separator.size() &&
                         std::min(a.size(), b.size()) >
                             std::min(best.side_a.size(), best.side_b.size()));
    if (better) {
      best.side_a = std::move(a);
      best.side_b = std::move(b);
      best.separator = std::move(sep);
      best.ok = true;
    }
  }
  return best;
}

// Geometric fallback: bisect by longitude median band as separator.
Partition partition_geometric(const Fragment& f, const CchGraph& g) {
  Partition p;
  const uint32_t n = static_cast<uint32_t>(f.nodes.size());
  if (n < 3)
    return p;

  std::vector<uint32_t> order(n);
  for (uint32_t i = 0; i < n; ++i)
    order[i] = i;
  std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
    const auto& na = g.nodes[f.nodes[a]];
    const auto& nb = g.nodes[f.nodes[b]];
    if (na.lon != nb.lon)
      return na.lon < nb.lon;
    if (na.lat != nb.lat)
      return na.lat < nb.lat;
    return f.nodes[a] < f.nodes[b];
  });

  const uint32_t mid = n / 2;
  const uint32_t band = std::max(1u, n / 50); // ~2% band
  const uint32_t lo = mid > band / 2 ? mid - band / 2 : 0;
  const uint32_t hi = std::min(n, lo + band);

  std::vector<uint8_t> role(n, 0); // 0=A, 1=sep, 2=B
  for (uint32_t i = 0; i < n; ++i) {
    if (i >= lo && i < hi)
      role[order[i]] = 1;
    else if (i < lo)
      role[order[i]] = 0;
    else
      role[order[i]] = 2;
  }
  for (uint32_t i = 0; i < n; ++i) {
    if (role[i] == 0)
      p.side_a.push_back(f.nodes[i]);
    else if (role[i] == 1)
      p.separator.push_back(f.nodes[i]);
    else
      p.side_b.push_back(f.nodes[i]);
  }
  p.ok = !p.side_a.empty() && !p.side_b.empty() && !p.separator.empty();
  return p;
}

std::vector<uint32_t> order_by_degree(const Fragment& f) {
  const uint32_t n = static_cast<uint32_t>(f.nodes.size());
  std::vector<uint32_t> idx(n);
  for (uint32_t i = 0; i < n; ++i)
    idx[i] = i;
  std::sort(idx.begin(), idx.end(), [&](uint32_t a, uint32_t b) {
    const uint32_t da = f.first_out[a + 1] - f.first_out[a];
    const uint32_t db = f.first_out[b + 1] - f.first_out[b];
    if (da != db)
      return da < db;
    return f.nodes[a] < f.nodes[b];
  });
  std::vector<uint32_t> out;
  out.reserve(n);
  for (uint32_t i : idx)
    out.push_back(f.nodes[i]);
  return out;
}

// Split fragment into connected components (local BFS).
std::vector<std::vector<uint32_t>> connected_components(const Fragment& f) {
  const uint32_t n = static_cast<uint32_t>(f.nodes.size());
  std::vector<uint8_t> seen(n, 0);
  std::vector<std::vector<uint32_t>> comps;
  for (uint32_t s = 0; s < n; ++s) {
    if (seen[s])
      continue;
    std::vector<uint32_t> comp;
    std::queue<uint32_t> q;
    q.push(s);
    seen[s] = 1;
    while (!q.empty()) {
      const uint32_t u = q.front();
      q.pop();
      comp.push_back(f.nodes[u]);
      for (uint32_t ei = f.first_out[u]; ei < f.first_out[u + 1]; ++ei) {
        const uint32_t v = f.head[ei];
        if (!seen[v]) {
          seen[v] = 1;
          q.push(v);
        }
      }
    }
    comps.push_back(std::move(comp));
  }
  for (auto& c : comps)
    std::sort(c.begin(), c.end());
  std::sort(comps.begin(), comps.end(), [](const auto& a, const auto& b) {
    return a.front() < b.front();
  });
  return comps;
}

struct NdState {
  const UndirectedGraph* ug = nullptr;
  const CchGraph* g = nullptr;
  std::vector<double> angles;
  std::atomic<uint32_t>* free_threads = nullptr;
  std::atomic<uint64_t>* bipartitions = nullptr;
};

std::vector<uint32_t> nd_recurse(NdState& st, const std::vector<uint32_t>& nodes);

std::vector<uint32_t> nd_on_connected(NdState& st, const std::vector<uint32_t>& nodes) {
  Fragment f = make_fragment(*st.ug, nodes);
  if (f.nodes.size() <= kLeafSize)
    return order_by_degree(f);

  Partition part = partition_inertial_flow(f, *st.g, st.angles);
  if (!part.ok)
    part = partition_geometric(f, *st.g);
  if (!part.ok)
    return order_by_degree(f);

  st.bipartitions->fetch_add(1, std::memory_order_relaxed);

  std::vector<uint32_t> left, right;
  uint32_t available = st.free_threads->load(std::memory_order_relaxed);
  if (available > 0 && part.side_a.size() > kLeafSize && part.side_b.size() > kLeafSize) {
    st.free_threads->fetch_sub(1, std::memory_order_relaxed);
    std::thread th([&]() { right = nd_recurse(st, part.side_b); });
    left = nd_recurse(st, part.side_a);
    th.join();
    st.free_threads->fetch_add(1, std::memory_order_relaxed);
  } else {
    left = nd_recurse(st, part.side_a);
    right = nd_recurse(st, part.side_b);
  }

  // Separator last (highest ranks). Stable order by node id.
  std::sort(part.separator.begin(), part.separator.end());
  std::vector<uint32_t> out;
  out.reserve(nodes.size());
  out.insert(out.end(), left.begin(), left.end());
  out.insert(out.end(), right.begin(), right.end());
  out.insert(out.end(), part.separator.begin(), part.separator.end());
  return out;
}

std::vector<uint32_t> nd_recurse(NdState& st, const std::vector<uint32_t>& nodes) {
  if (nodes.empty())
    return {};
  if (nodes.size() <= kLeafSize) {
    Fragment f = make_fragment(*st.ug, nodes);
    return order_by_degree(f);
  }

  Fragment f = make_fragment(*st.ug, nodes);
  auto comps = connected_components(f);
  if (comps.size() > 1) {
    std::vector<uint32_t> out;
    out.reserve(nodes.size());
    for (const auto& c : comps) {
      auto part = nd_recurse(st, c);
      out.insert(out.end(), part.begin(), part.end());
    }
    return out;
  }
  return nd_on_connected(st, nodes);
}

} // namespace

std::vector<uint32_t> ComputeNestedDissectionOrder(const CchGraph& g, uint32_t concurrency) {
  const uint32_t threads = resolve_concurrency(concurrency);
  const uint32_t n = static_cast<uint32_t>(g.nodes.size());
  LOG_INFO("cch nd: building undirected graph  nodes=" + std::to_string(n) +
           " threads=" + std::to_string(threads));

  UndirectedGraph ug = build_undirected(g);
  LOG_INFO("cch nd: undirected edges=" + std::to_string(ug.head.size() / 2));

  constexpr double kPi = 3.14159265358979323846;
  std::vector<double> angles = {0.0, kPi / 4.0, kPi / 2.0, 3.0 * kPi / 4.0};
  std::atomic<uint32_t> free_threads{threads > 1 ? threads - 1 : 0};
  std::atomic<uint64_t> bipartitions{0};

  NdState st;
  st.ug = &ug;
  st.g = &g;
  st.angles = angles;
  st.free_threads = &free_threads;
  st.bipartitions = &bipartitions;

  std::vector<uint32_t> all(n);
  for (uint32_t i = 0; i < n; ++i)
    all[i] = i;

  auto order_list = nd_recurse(st, all);
  if (order_list.size() != n) {
    LOG_WARN("cch nd: order size mismatch (" + std::to_string(order_list.size()) + " vs " +
             std::to_string(n) + "); falling back to degree order");
    Fragment f = make_fragment(ug, all);
    order_list = order_by_degree(f);
  }

  std::vector<uint32_t> rank(n);
  std::vector<uint8_t> seen(n, 0);
  for (uint32_t i = 0; i < order_list.size(); ++i) {
    const uint32_t v = order_list[i];
    if (v >= n || seen[v]) {
      LOG_WARN("cch nd: invalid/duplicate node in order; rebuilding degree order");
      Fragment f = make_fragment(ug, all);
      order_list = order_by_degree(f);
      std::fill(seen.begin(), seen.end(), 0);
      for (uint32_t j = 0; j < order_list.size(); ++j) {
        rank[order_list[j]] = j;
        seen[order_list[j]] = 1;
      }
      break;
    }
    seen[v] = 1;
    rank[v] = i;
  }

  LOG_INFO("cch nd: done  bipartitions=" + std::to_string(bipartitions.load()) +
           " nodes=" + std::to_string(n));
  return rank;
}

} // namespace cch
} // namespace thor
} // namespace valhalla
