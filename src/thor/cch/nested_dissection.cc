#include "thor/cch/nested_dissection.h"

#include "midgard/logging.h"

#include <metis.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace valhalla {
namespace thor {
namespace cch {
namespace {

// Build undirected CSR in METIS format (no self-loops, no duplicate edges).
void build_metis_csr(const CchGraph& g,
                     std::vector<idx_t>& xadj,
                     std::vector<idx_t>& adjncy) {
  const idx_t n = static_cast<idx_t>(g.nodes.size());
  std::vector<std::vector<idx_t>> adj(static_cast<size_t>(n));
  for (const auto& e : g.edges) {
    if (e.u == e.v)
      continue;
    adj[e.u].push_back(static_cast<idx_t>(e.v));
    adj[e.v].push_back(static_cast<idx_t>(e.u));
  }

  xadj.assign(static_cast<size_t>(n) + 1, 0);
  for (idx_t v = 0; v < n; ++v) {
    auto& a = adj[static_cast<size_t>(v)];
    std::sort(a.begin(), a.end());
    a.erase(std::unique(a.begin(), a.end()), a.end());
    xadj[static_cast<size_t>(v) + 1] =
        xadj[static_cast<size_t>(v)] + static_cast<idx_t>(a.size());
  }
  adjncy.resize(static_cast<size_t>(xadj[static_cast<size_t>(n)]));
  for (idx_t v = 0; v < n; ++v) {
    const auto& a = adj[static_cast<size_t>(v)];
    std::copy(a.begin(), a.end(),
              adjncy.begin() + static_cast<size_t>(xadj[static_cast<size_t>(v)]));
  }
}

} // namespace

std::vector<uint32_t> ComputeNestedDissectionOrder(const CchGraph& g,
                                                   uint32_t /*concurrency*/) {
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();
  const uint32_t n_u32 = static_cast<uint32_t>(g.nodes.size());
  if (n_u32 == 0)
    return {};

  LOG_INFO("cch nd: METIS_NodeND  nodes=" + std::to_string(n_u32) +
           "  building undirected CSR...");

  std::vector<idx_t> xadj, adjncy;
  build_metis_csr(g, xadj, adjncy);
  const idx_t n = static_cast<idx_t>(n_u32);
  const idx_t nedges = xadj[static_cast<size_t>(n)] / 2;
  LOG_INFO("cch nd: undirected edges=" + std::to_string(nedges) +
           "  calling METIS_NodeND (this is the heavy step)...");

  std::vector<idx_t> perm(static_cast<size_t>(n));
  std::vector<idx_t> iperm(static_cast<size_t>(n));
  idx_t options[METIS_NOPTIONS];
  METIS_SetDefaultOptions(options);
  options[METIS_OPTION_NUMBERING] = 0; // C-style 0-based
  // Prefer nested-dissection flavour (default for NodeND).
  options[METIS_OPTION_CTYPE] = METIS_CTYPE_SHEM;
  options[METIS_OPTION_RTYPE] = METIS_RTYPE_SEP1SIDED;
  options[METIS_OPTION_IPTYPE] = METIS_IPTYPE_NODE;
  options[METIS_OPTION_NSEPS] = 1;
  options[METIS_OPTION_NITER] = 10;
  options[METIS_OPTION_UFACTOR] = 30; // slight imbalance OK for separators
  options[METIS_OPTION_COMPRESS] = 1;
  options[METIS_OPTION_CCORDER] = 1; // order connected components separately

  idx_t nvtxs = n;
  const int rc =
      METIS_NodeND(&nvtxs, xadj.data(), adjncy.data(), /*vwgt=*/nullptr, options,
                   perm.data(), iperm.data());
  if (rc != METIS_OK) {
    throw std::runtime_error("cch nd: METIS_NodeND failed with code " +
                             std::to_string(rc));
  }

  // METIS: perm[new_pos] = old_vertex, iperm[old_vertex] = new_pos.
  // Fill-reducing ND eliminates small new_pos first → low rank = contract first.
  std::vector<uint32_t> rank(n_u32);
  std::vector<uint8_t> seen(n_u32, 0);
  for (uint32_t v = 0; v < n_u32; ++v) {
    const auto r = static_cast<uint32_t>(iperm[v]);
    if (r >= n_u32 || seen[r]) {
      throw std::runtime_error("cch nd: METIS returned invalid permutation");
    }
    seen[r] = 1;
    rank[v] = r;
  }

  const double secs = std::chrono::duration<double>(clock::now() - t0).count();
  LOG_INFO("cch nd: METIS_NodeND done  nodes=" + std::to_string(n_u32) +
           "  elapsed_s=" + std::to_string(secs));
  return rank;
}

} // namespace cch
} // namespace thor
} // namespace valhalla
