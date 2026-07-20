#include "thor/cch/order.h"

#include "midgard/logging.h"
#include "thor/cch/nested_dissection.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace valhalla {
namespace thor {
namespace cch {

namespace {

struct DynEdge {
  uint32_t edge_id = 0;
  uint32_t hops = 0;
};

// Compact residual adjacency (vector lists). With a METIS ND order, degrees stay
// modest so linear scans beat 167M unordered_maps on RAM and locality.
struct DynNbr {
  uint32_t to = 0;
  uint32_t edge_id = 0;
  uint32_t hops = 0;
};

using CompactAdj = std::vector<std::vector<DynNbr>>;

struct Priority {
  uint32_t degree = 0;
  uint32_t id = 0;
  bool operator<(const Priority& o) const {
    if (degree != o.degree)
      return degree < o.degree;
    return id < o.id;
  }
};

struct Proposed {
  CchShortcut sc;
  uint32_t hops = 0;
};

uint32_t resolve_concurrency(uint32_t concurrency) {
  if (concurrency == 0) {
    const unsigned hc = std::thread::hardware_concurrency();
    return hc == 0 ? 1u : hc;
  }
  return concurrency;
}

template <typename Fn>
void parallel_for(uint32_t nthreads, size_t n, Fn&& fn) {
  if (n == 0)
    return;
  nthreads = std::max(1u, std::min(nthreads, static_cast<uint32_t>(n)));
  if (nthreads == 1) {
    for (size_t i = 0; i < n; ++i)
      fn(i);
    return;
  }
  std::vector<std::thread> threads;
  threads.reserve(nthreads);
  std::atomic<size_t> next{0};
  for (uint32_t t = 0; t < nthreads; ++t) {
    threads.emplace_back([&]() {
      for (;;) {
        const size_t i = next.fetch_add(1, std::memory_order_relaxed);
        if (i >= n)
          break;
        fn(i);
      }
    });
  }
  for (auto& th : threads)
    th.join();
}

DynNbr* find_nbr(std::vector<DynNbr>& a, uint32_t to) {
  for (auto& e : a) {
    if (e.to == to)
      return &e;
  }
  return nullptr;
}

const DynNbr* find_nbr(const std::vector<DynNbr>& a, uint32_t to) {
  for (const auto& e : a) {
    if (e.to == to)
      return &e;
  }
  return nullptr;
}

void merge_nbr(std::vector<DynNbr>& a, uint32_t to, uint32_t edge_id, uint32_t hops) {
  if (auto* p = find_nbr(a, to)) {
    if (hops < p->hops) {
      p->hops = hops;
      p->edge_id = edge_id;
    }
    return;
  }
  a.push_back(DynNbr{to, edge_id, hops});
}

void erase_nbr(std::vector<DynNbr>& a, uint32_t to) {
  a.erase(std::remove_if(a.begin(), a.end(),
                         [to](const DynNbr& e) { return e.to == to; }),
          a.end());
}

void merge_edge(std::unordered_map<uint32_t, DynEdge>& adj, uint32_t key, DynEdge e) {
  auto it = adj.find(key);
  if (it == adj.end() || e.hops < it->second.hops)
    adj[key] = e;
}

bool maybe_add_shortcut_compact(CchOrder& order,
                                CompactAdj& out,
                                CompactAdj& inc,
                                const Proposed& best) {
  if (const auto* existing = find_nbr(out[best.sc.u], best.sc.w)) {
    if (existing->hops <= best.hops)
      return false;
  }
  const uint32_t sc_id = order.num_base_edges + static_cast<uint32_t>(order.shortcuts.size());
  order.shortcuts.push_back(best.sc);
  merge_nbr(out[best.sc.u], best.sc.w, sc_id, best.hops);
  merge_nbr(inc[best.sc.w], best.sc.u, sc_id, best.hops);
  return true;
}

void remove_node_compact(uint32_t v,
                         CompactAdj& out,
                         CompactAdj& inc,
                         std::vector<uint8_t>& active) {
  for (const auto& e : out[v]) {
    if (active[e.to])
      erase_nbr(inc[e.to], v);
  }
  for (const auto& e : inc[v]) {
    if (active[e.to])
      erase_nbr(out[e.to], v);
  }
  out[v].clear();
  out[v].shrink_to_fit();
  inc[v].clear();
  inc[v].shrink_to_fit();
  active[v] = 0;
}

bool maybe_add_shortcut(CchOrder& order,
                        std::vector<std::unordered_map<uint32_t, DynEdge>>& out,
                        std::vector<std::unordered_map<uint32_t, DynEdge>>& inc,
                        const Proposed& best) {
  auto existing = out[best.sc.u].find(best.sc.w);
  if (existing != out[best.sc.u].end() && existing->second.hops <= best.hops)
    return false;
  const uint32_t sc_id = order.num_base_edges + static_cast<uint32_t>(order.shortcuts.size());
  order.shortcuts.push_back(best.sc);
  merge_edge(out[best.sc.u], best.sc.w, DynEdge{sc_id, best.hops});
  merge_edge(inc[best.sc.w], best.sc.u, DynEdge{sc_id, best.hops});
  return true;
}

void remove_node(uint32_t v,
                 std::vector<std::unordered_map<uint32_t, DynEdge>>& out,
                 std::vector<std::unordered_map<uint32_t, DynEdge>>& inc,
                 std::vector<uint8_t>& active) {
  for (const auto& [w, _] : out[v]) {
    if (active[w])
      inc[w].erase(v);
  }
  for (const auto& [u, _] : inc[v]) {
    if (active[u])
      out[u].erase(v);
  }
  out[v].clear();
  inc[v].clear();
  active[v] = 0;
}

// Contract nodes in increasing rank order (low rank first) with compact adj.
CchOrder ContractInOrder(const CchGraph& g, const std::vector<uint32_t>& rank) {
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();
  auto elapsed_s = [&]() {
    return std::chrono::duration<double>(clock::now() - t0).count();
  };

  const uint32_t n = static_cast<uint32_t>(g.nodes.size());
  CchOrder order;
  order.num_base_edges = static_cast<uint32_t>(g.edges.size());
  order.rank = rank;
  order.tile_build_hash = g.tile_build_hash;

  std::vector<uint32_t> by_rank(n);
  for (uint32_t v = 0; v < n; ++v) {
    if (rank[v] >= n)
      throw std::runtime_error("cch: invalid rank in ContractInOrder");
    by_rank[rank[v]] = v;
  }

  CompactAdj out(n), inc(n);
  for (uint32_t ei = 0; ei < g.edges.size(); ++ei) {
    const auto& e = g.edges[ei];
    merge_nbr(out[e.u], e.v, ei, 1);
    merge_nbr(inc[e.v], e.u, ei, 1);
  }
  std::vector<uint8_t> active(n, 1);

  LOG_INFO("cch contract: compact sequential-by-rank  nodes=" + std::to_string(n) +
           " base_edges=" + std::to_string(order.num_base_edges));

  auto last_log = clock::now();
  for (uint32_t ri = 0; ri < n; ++ri) {
    const uint32_t v = by_rank[ri];
    if (!active[v])
      continue;

    for (const auto& e_in : inc[v]) {
      const uint32_t u = e_in.to;
      if (u == v || !active[u])
        continue;
      const auto* uv = find_nbr(out[u], v);
      if (!uv)
        continue;
      // Copy before the inner loop: maybe_add may reallocate out[u].
      const uint32_t uv_edge = uv->edge_id;
      const uint32_t uv_hops = uv->hops;
      for (const auto& e_out : out[v]) {
        const uint32_t w = e_out.to;
        if (w == v || w == u || !active[w])
          continue;
        Proposed p;
        p.sc.u = u;
        p.sc.w = w;
        p.sc.middle = v;
        p.sc.left = uv_edge;
        p.sc.right = e_out.edge_id;
        p.hops = uv_hops + e_out.hops;
        maybe_add_shortcut_compact(order, out, inc, p);
      }
    }
    remove_node_compact(v, out, inc, active);

    const auto now = clock::now();
    const uint32_t done = ri + 1;
    const bool pct_tick =
        n > 0 && (done == n || (done % std::max<uint32_t>(1, n / 20) == 0));
    if (done == n || pct_tick ||
        std::chrono::duration<double>(now - last_log).count() >= 15.0) {
      const double pct = n ? 100.0 * done / n : 100.0;
      const double rate = elapsed_s() > 0.0 ? done / elapsed_s() : 0.0;
      const double eta_s = rate > 0.0 ? (static_cast<double>(n - done) / rate) : 0.0;
      LOG_INFO("cch contract: contracted " + std::to_string(done) + "/" +
               std::to_string(n) + " (" + std::to_string(static_cast<int>(pct)) +
               "%)  shortcuts=" + std::to_string(order.shortcuts.size()) +
               "  rate_nodes_per_s=" + std::to_string(rate) +
               "  elapsed_s=" + std::to_string(elapsed_s()) +
               "  eta_s≈" + std::to_string(eta_s));
      last_log = now;
    }
  }

  LOG_INFO("cch contract: done  shortcuts=" + std::to_string(order.shortcuts.size()) +
           "  elapsed_s=" + std::to_string(elapsed_s()));
  return order;
}

CchOrder BuildOrderIndependentSet(const CchGraph& g, uint32_t concurrency) {
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();
  auto elapsed_s = [&]() {
    return std::chrono::duration<double>(clock::now() - t0).count();
  };

  const uint32_t threads = resolve_concurrency(concurrency);
  const uint32_t n = static_cast<uint32_t>(g.nodes.size());
  CchOrder order;
  order.num_base_edges = static_cast<uint32_t>(g.edges.size());
  order.rank.assign(n, 0);
  order.tile_build_hash = g.tile_build_hash;

  LOG_INFO("cch order: independent-set contraction  nodes=" + std::to_string(n) +
           " base_edges=" + std::to_string(order.num_base_edges) +
           " threads=" + std::to_string(threads));

  std::vector<std::unordered_map<uint32_t, DynEdge>> out(n), inc(n);
  for (uint32_t ei = 0; ei < g.edges.size(); ++ei) {
    const auto& e = g.edges[ei];
    merge_edge(out[e.u], e.v, DynEdge{ei, 1});
    merge_edge(inc[e.v], e.u, DynEdge{ei, 1});
  }
  LOG_INFO("cch order: adjacency ready  elapsed_s=" + std::to_string(elapsed_s()));

  std::vector<uint8_t> active(n, 1);
  uint32_t remaining = n;
  uint32_t rank_counter = 0;
  uint32_t phase = 0;
  auto last_log = clock::now();

  std::vector<Priority> priority(n);
  std::vector<uint8_t> in_is(n, 0);
  std::vector<uint32_t> iset;
  iset.reserve(std::max<uint32_t>(1, n / 8));

  while (remaining > 0) {
    ++phase;

    parallel_for(threads, n, [&](size_t v) {
      if (!active[v])
        return;
      priority[v] = Priority{static_cast<uint32_t>(out[v].size() + inc[v].size()),
                             static_cast<uint32_t>(v)};
    });

    parallel_for(threads, n, [&](size_t v) {
      in_is[v] = 0;
      if (!active[v])
        return;
      const Priority& pv = priority[v];
      for (const auto& [w, _] : out[v]) {
        if (active[w] && !(pv < priority[w]))
          return;
      }
      for (const auto& [u, _] : inc[v]) {
        if (active[u] && !(pv < priority[u]))
          return;
      }
      in_is[v] = 1;
    });

    iset.clear();
    for (uint32_t v = 0; v < n; ++v) {
      if (in_is[v])
        iset.push_back(v);
    }

    if (iset.empty()) {
      uint32_t best = n;
      for (uint32_t v = 0; v < n; ++v) {
        if (!active[v])
          continue;
        if (best == n || priority[v] < priority[best])
          best = v;
      }
      if (best == n)
        break;
      iset.push_back(best);
      in_is[best] = 1;
    }

    std::sort(iset.begin(), iset.end(),
              [&](uint32_t a, uint32_t b) { return priority[a] < priority[b]; });

    std::vector<std::vector<Proposed>> proposals(iset.size());
    parallel_for(threads, iset.size(), [&](size_t ii) {
      const uint32_t v = iset[ii];
      auto& props = proposals[ii];
      for (const auto& [u, e_uv] : inc[v]) {
        if (u == v || !active[u])
          continue;
        auto ituv = out[u].find(v);
        if (ituv == out[u].end())
          continue;
        for (const auto& [w, e_vw] : out[v]) {
          if (w == v || w == u || !active[w])
            continue;
          Proposed p;
          p.sc.u = u;
          p.sc.w = w;
          p.sc.middle = v;
          p.sc.left = ituv->second.edge_id;
          p.sc.right = e_vw.edge_id;
          p.hops = ituv->second.hops + e_vw.hops;
          props.push_back(p);
        }
      }
    });

    size_t total_props = 0;
    for (const auto& bucket : proposals)
      total_props += bucket.size();
    std::vector<Proposed> flat;
    flat.reserve(total_props);
    for (auto& bucket : proposals) {
      flat.insert(flat.end(), bucket.begin(), bucket.end());
      bucket.clear();
    }
    std::sort(flat.begin(), flat.end(), [](const Proposed& a, const Proposed& b) {
      if (a.sc.u != b.sc.u)
        return a.sc.u < b.sc.u;
      if (a.sc.w != b.sc.w)
        return a.sc.w < b.sc.w;
      if (a.hops != b.hops)
        return a.hops < b.hops;
      return a.sc.middle < b.sc.middle;
    });

    for (size_t i = 0; i < flat.size();) {
      size_t j = i + 1;
      while (j < flat.size() && flat[j].sc.u == flat[i].sc.u && flat[j].sc.w == flat[i].sc.w)
        ++j;
      maybe_add_shortcut(order, out, inc, flat[i]);
      i = j;
    }

    for (uint32_t v : iset) {
      remove_node(v, out, inc, active);
      in_is[v] = 0;
      order.rank[v] = rank_counter++;
    }
    remaining -= static_cast<uint32_t>(iset.size());

    const auto now = clock::now();
    const bool pct_tick =
        n > 0 && (rank_counter == n ||
                  (rank_counter % std::max<uint32_t>(1, n / 20) == 0));
    if (rank_counter == n || pct_tick ||
        std::chrono::duration<double>(now - last_log).count() >= 15.0) {
      const double pct = n ? 100.0 * rank_counter / n : 100.0;
      const double rate = elapsed_s() > 0.0 ? rank_counter / elapsed_s() : 0.0;
      const double eta_s =
          rate > 0.0 ? (static_cast<double>(n - rank_counter) / rate) : 0.0;
      LOG_INFO("cch order: phase=" + std::to_string(phase) + " contracted " +
               std::to_string(rank_counter) + "/" + std::to_string(n) + " (" +
               std::to_string(static_cast<int>(pct)) +
               "%)  is_size=" + std::to_string(iset.size()) +
               "  shortcuts=" + std::to_string(order.shortcuts.size()) +
               "  rate_nodes_per_s=" + std::to_string(rate) +
               "  elapsed_s=" + std::to_string(elapsed_s()) +
               "  eta_s≈" + std::to_string(eta_s));
      last_log = now;
    }
  }

  LOG_INFO("cch order: independent-set contraction done  phases=" + std::to_string(phase) +
           " shortcuts=" + std::to_string(order.shortcuts.size()) +
           "  elapsed_s=" + std::to_string(elapsed_s()));
  return order;
}

} // namespace

CchOrder BuildOrder(const CchGraph& g, uint32_t concurrency, OrderMethod method) {
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();
  auto elapsed_s = [&]() {
    return std::chrono::duration<double>(clock::now() - t0).count();
  };

  CchOrder order;
  if (method == OrderMethod::IndependentSet) {
    order = BuildOrderIndependentSet(g, concurrency);
  } else {
    LOG_INFO("cch order: METIS nested-dissection + compact contract-in-order");
    const auto t_nd = clock::now();
    auto rank = ComputeNestedDissectionOrder(g, concurrency);
    LOG_INFO("cch order: nested dissection finished  nd_s=" +
             std::to_string(std::chrono::duration<double>(clock::now() - t_nd).count()));
    order = ContractInOrder(g, rank);
  }

  LOG_INFO("cch order: building up/down adjacency  shortcuts=" +
           std::to_string(order.shortcuts.size()));
  const auto t_adj = clock::now();
  order.build_adjacency(g);
  LOG_INFO("cch order: adjacency built  adj_s=" +
           std::to_string(std::chrono::duration<double>(clock::now() - t_adj).count()) +
           "  total_s=" + std::to_string(elapsed_s()));
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
  return o;
}

} // namespace cch
} // namespace thor
} // namespace valhalla
