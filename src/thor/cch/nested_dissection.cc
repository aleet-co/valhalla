#include "thor/cch/nested_dissection.h"

#include "midgard/logging.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

namespace valhalla {
namespace thor {
namespace cch {
namespace {

// Guardrails against the Europe micro-cut death spiral (246k bipartitions /
// depth 70 / 2% ordered in 12h). Stop recursion early, demand real mass on
// both sides of every cut, and fall back to a balanced geometric median.
//
// Speed: Dinic×N angles dominates mid-size fragments. Only run inertial flow
// above kInertialFlowMinSize; below that use cheap geometric median splits.
constexpr uint32_t kLeafSize = 8192;
constexpr uint32_t kMaxDepth = 28;
constexpr uint32_t kInertialFlowMinSize = 100000;
constexpr double kTerminalFraction = 0.25;
constexpr double kMinBalance = 0.20;
constexpr uint32_t kMinSideAbs = 2048; // also enforced as fraction of n
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

// Reject micro-peels (e.g. sides 10+27) that caused pathological depth.
bool partition_acceptable(const Partition& p, uint32_t n) {
  if (!p.ok || p.side_a.empty() || p.side_b.empty() || p.separator.empty())
    return false;
  const uint32_t min_side =
      static_cast<uint32_t>(std::min(p.side_a.size(), p.side_b.size()));
  const uint32_t need =
      std::max(kMinSideAbs, static_cast<uint32_t>(kMinBalance * static_cast<double>(n)));
  if (min_side < need)
    return false;
  // Separator must not dominate the fragment.
  if (p.separator.size() * 2 > n)
    return false;
  return true;
}

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

    Partition cand;
    cand.side_a = std::move(a);
    cand.side_b = std::move(b);
    cand.separator = std::move(sep);
    cand.ok = true;
    if (!partition_acceptable(cand, n))
      continue;

    // Prefer balance (larger min side) over a slightly smaller separator —
    // tiny peels with skinny seps caused the Europe depth explosion.
    const auto min_side = [](const Partition& p) {
      return std::min(p.side_a.size(), p.side_b.size());
    };
    const bool better =
        !best.ok || min_side(cand) > min_side(best) ||
        (min_side(cand) == min_side(best) && cand.separator.size() < best.separator.size());
    if (better)
      best = std::move(cand);
  }
  return best;
}

// Balanced geometric median split along lon or lat (whichever yields the
// smaller acceptable separator). Always aims for ~50/50 sides.
Partition partition_geometric(const Fragment& f, const CchGraph& g) {
  const uint32_t n = static_cast<uint32_t>(f.nodes.size());
  Partition best;
  if (n < 3)
    return best;

  auto try_axis = [&](bool by_lon) {
    std::vector<uint32_t> order(n);
    for (uint32_t i = 0; i < n; ++i)
      order[i] = i;
    std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
      const auto& na = g.nodes[f.nodes[a]];
      const auto& nb = g.nodes[f.nodes[b]];
      const double ka = by_lon ? na.lon : na.lat;
      const double kb = by_lon ? nb.lon : nb.lat;
      if (ka != kb)
        return ka < kb;
      const double ka2 = by_lon ? na.lat : na.lon;
      const double kb2 = by_lon ? nb.lat : nb.lon;
      if (ka2 != kb2)
        return ka2 < kb2;
      return f.nodes[a] < f.nodes[b];
    });

    // Thin median band (~0.5%), at least 1 node, capped so sides stay large.
    uint32_t band = std::max(1u, n / 200);
    const uint32_t max_band = n > 4 ? (n / 5) : 1; // keep >=40% per side possible
    band = std::min(band, max_band);
    const uint32_t mid = n / 2;
    const uint32_t lo = mid > band / 2 ? mid - band / 2 : 0;
    const uint32_t hi = std::min(n, lo + band);
    if (lo == 0 || hi >= n)
      return;

    Partition p;
    p.side_a.reserve(lo);
    p.separator.reserve(hi - lo);
    p.side_b.reserve(n - hi);
    for (uint32_t i = 0; i < lo; ++i)
      p.side_a.push_back(f.nodes[order[i]]);
    for (uint32_t i = lo; i < hi; ++i)
      p.separator.push_back(f.nodes[order[i]]);
    for (uint32_t i = hi; i < n; ++i)
      p.side_b.push_back(f.nodes[order[i]]);
    p.ok = true;
    if (!partition_acceptable(p, n))
      return;
    const auto min_side = [](const Partition& q) {
      return std::min(q.side_a.size(), q.side_b.size());
    };
    if (!best.ok || min_side(p) > min_side(best) ||
        (min_side(p) == min_side(best) && p.separator.size() < best.separator.size()))
      best = std::move(p);
  };

  try_axis(true);
  try_axis(false);
  return best;
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
  std::atomic<uint64_t>* nodes_ordered = nullptr;
  std::atomic<uint32_t>* max_depth = nullptr;
  std::atomic<uint32_t>* last_sep = nullptr;
  std::atomic<uint32_t>* last_side_a = nullptr;
  std::atomic<uint32_t>* last_side_b = nullptr;
  uint32_t total_nodes = 0;
  std::chrono::steady_clock::time_point t0{};
  std::mutex* log_mu = nullptr;
  std::chrono::steady_clock::time_point* last_log = nullptr;
};

void maybe_log_nd_progress(NdState& st,
                           uint32_t depth,
                           uint32_t fragment_n,
                           bool force = false) {
  using clock = std::chrono::steady_clock;
  const auto now = clock::now();
  std::lock_guard<std::mutex> lock(*st.log_mu);
  const double since_log = std::chrono::duration<double>(now - *st.last_log).count();
  const uint64_t ordered = st.nodes_ordered->load(std::memory_order_relaxed);
  const bool done = ordered >= st.total_nodes;
  if (!force && !done && since_log < 15.0)
    return;
  *st.last_log = now;

  const double elapsed = std::chrono::duration<double>(now - st.t0).count();
  const double pct = st.total_nodes ? 100.0 * ordered / st.total_nodes : 100.0;
  const double rate = elapsed > 0.0 ? ordered / elapsed : 0.0;
  const double eta_s =
      rate > 0.0 ? (static_cast<double>(st.total_nodes - ordered) / rate) : 0.0;
  LOG_INFO("cch nd: progress ordered " + std::to_string(ordered) + "/" +
           std::to_string(st.total_nodes) + " (" + std::to_string(static_cast<int>(pct)) +
           "%)  bipartitions=" +
           std::to_string(st.bipartitions->load(std::memory_order_relaxed)) +
           "  depth=" + std::to_string(depth) +
           "  max_depth=" +
           std::to_string(st.max_depth->load(std::memory_order_relaxed)) +
           "  fragment_n=" + std::to_string(fragment_n) + "  last_cut sep=" +
           std::to_string(st.last_sep->load(std::memory_order_relaxed)) +
           " sides=" + std::to_string(st.last_side_a->load(std::memory_order_relaxed)) +
           "+" + std::to_string(st.last_side_b->load(std::memory_order_relaxed)) +
           "  rate_nodes_per_s=" + std::to_string(rate) +
           "  elapsed_s=" + std::to_string(elapsed) + "  eta_s≈" + std::to_string(eta_s));
}

void note_depth(NdState& st, uint32_t depth) {
  uint32_t cur = st.max_depth->load(std::memory_order_relaxed);
  while (depth > cur &&
         !st.max_depth->compare_exchange_weak(cur, depth, std::memory_order_relaxed)) {
  }
}

std::vector<uint32_t> finish_leaf(NdState& st,
                                  const Fragment& f,
                                  uint32_t depth) {
  auto out = order_by_degree(f);
  st.nodes_ordered->fetch_add(out.size(), std::memory_order_relaxed);
  maybe_log_nd_progress(st, depth, static_cast<uint32_t>(f.nodes.size()));
  return out;
}

std::vector<uint32_t> nd_recurse(NdState& st, const std::vector<uint32_t>& nodes, uint32_t depth);

std::vector<uint32_t> nd_on_connected(NdState& st,
                                      const std::vector<uint32_t>& nodes,
                                      uint32_t depth) {
  note_depth(st, depth);
  Fragment f = make_fragment(*st.ug, nodes);
  if (f.nodes.size() <= kLeafSize || depth >= kMaxDepth)
    return finish_leaf(st, f, depth);

  // Large fragments can sit in max-flow for a long time — force a heartbeat.
  constexpr uint32_t kForceLogFragment = 100000;
  maybe_log_nd_progress(st, depth, static_cast<uint32_t>(f.nodes.size()),
                        f.nodes.size() >= kForceLogFragment);

  Partition part;
  const uint32_t fn = static_cast<uint32_t>(f.nodes.size());
  // Expensive max-flow only on large fragments; geometric is enough (and much
  // faster) once pieces are below ~100k.
  if (fn >= kInertialFlowMinSize)
    part = partition_inertial_flow(f, *st.g, st.angles);
  if (!partition_acceptable(part, fn))
    part = partition_geometric(f, *st.g);
  if (!partition_acceptable(part, fn))
    return finish_leaf(st, f, depth);

  st.bipartitions->fetch_add(1, std::memory_order_relaxed);
  st.last_sep->store(static_cast<uint32_t>(part.separator.size()), std::memory_order_relaxed);
  st.last_side_a->store(static_cast<uint32_t>(part.side_a.size()), std::memory_order_relaxed);
  st.last_side_b->store(static_cast<uint32_t>(part.side_b.size()), std::memory_order_relaxed);
  maybe_log_nd_progress(st, depth, static_cast<uint32_t>(f.nodes.size()),
                        f.nodes.size() >= kForceLogFragment);

  std::vector<uint32_t> left, right;
  uint32_t available = st.free_threads->load(std::memory_order_relaxed);
  if (available > 0 && part.side_a.size() > kLeafSize && part.side_b.size() > kLeafSize) {
    st.free_threads->fetch_sub(1, std::memory_order_relaxed);
    std::thread th([&]() { right = nd_recurse(st, part.side_b, depth + 1); });
    left = nd_recurse(st, part.side_a, depth + 1);
    th.join();
    st.free_threads->fetch_add(1, std::memory_order_relaxed);
  } else {
    left = nd_recurse(st, part.side_a, depth + 1);
    right = nd_recurse(st, part.side_b, depth + 1);
  }

  // Separator nodes are finalized here (highest ranks among this fragment).
  std::sort(part.separator.begin(), part.separator.end());
  st.nodes_ordered->fetch_add(part.separator.size(), std::memory_order_relaxed);
  maybe_log_nd_progress(st, depth, static_cast<uint32_t>(f.nodes.size()));

  std::vector<uint32_t> out;
  out.reserve(nodes.size());
  out.insert(out.end(), left.begin(), left.end());
  out.insert(out.end(), right.begin(), right.end());
  out.insert(out.end(), part.separator.begin(), part.separator.end());
  return out;
}

std::vector<uint32_t> nd_recurse(NdState& st, const std::vector<uint32_t>& nodes, uint32_t depth) {
  if (nodes.empty())
    return {};
  note_depth(st, depth);
  if (nodes.size() <= kLeafSize || depth >= kMaxDepth) {
    Fragment f = make_fragment(*st.ug, nodes);
    return finish_leaf(st, f, depth);
  }

  Fragment f = make_fragment(*st.ug, nodes);
  auto comps = connected_components(f);
  if (comps.size() > 1) {
    std::vector<uint32_t> out;
    out.reserve(nodes.size());
    // Connected components are siblings, not deeper ND levels — keep depth.
    for (const auto& c : comps) {
      auto part = nd_recurse(st, c, depth);
      out.insert(out.end(), part.begin(), part.end());
    }
    return out;
  }
  return nd_on_connected(st, nodes, depth);
}

} // namespace

std::vector<uint32_t> ComputeNestedDissectionOrder(const CchGraph& g, uint32_t concurrency) {
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();
  const uint32_t threads = resolve_concurrency(concurrency);
  const uint32_t n = static_cast<uint32_t>(g.nodes.size());
  LOG_INFO("cch nd: building undirected graph  nodes=" + std::to_string(n) +
           " threads=" + std::to_string(threads));

  UndirectedGraph ug = build_undirected(g);
  LOG_INFO("cch nd: undirected edges=" + std::to_string(ug.head.size() / 2) +
           "  leaf_size=" + std::to_string(kLeafSize) +
           "  max_depth=" + std::to_string(kMaxDepth) +
           "  min_side_abs=" + std::to_string(kMinSideAbs) +
           "  if_min_size=" + std::to_string(kInertialFlowMinSize) +
           "  — recursive bipartition starting");

  // Two axis-aligned projections: enough for road graphs, 2× cheaper than 4.
  constexpr double kPi = 3.14159265358979323846;
  std::vector<double> angles = {0.0, kPi / 2.0};
  std::atomic<uint32_t> free_threads{threads > 1 ? threads - 1 : 0};
  std::atomic<uint64_t> bipartitions{0};
  std::atomic<uint64_t> nodes_ordered{0};
  std::atomic<uint32_t> max_depth{0};
  std::atomic<uint32_t> last_sep{0};
  std::atomic<uint32_t> last_side_a{0};
  std::atomic<uint32_t> last_side_b{0};
  std::mutex log_mu;
  auto last_log = t0;

  NdState st;
  st.ug = &ug;
  st.g = &g;
  st.angles = angles;
  st.free_threads = &free_threads;
  st.bipartitions = &bipartitions;
  st.nodes_ordered = &nodes_ordered;
  st.max_depth = &max_depth;
  st.last_sep = &last_sep;
  st.last_side_a = &last_side_a;
  st.last_side_b = &last_side_b;
  st.total_nodes = n;
  st.t0 = t0;
  st.log_mu = &log_mu;
  st.last_log = &last_log;

  std::vector<uint32_t> all(n);
  for (uint32_t i = 0; i < n; ++i)
    all[i] = i;

  auto order_list = nd_recurse(st, all, 0);
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

  const double total_s = std::chrono::duration<double>(clock::now() - t0).count();
  LOG_INFO("cch nd: done  bipartitions=" + std::to_string(bipartitions.load()) +
           "  max_depth=" + std::to_string(max_depth.load()) +
           "  nodes=" + std::to_string(n) + "  elapsed_s=" + std::to_string(total_s));
  return rank;
}

} // namespace cch
} // namespace thor
} // namespace valhalla
