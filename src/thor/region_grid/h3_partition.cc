#include "thor/region_grid/h3_partition.h"

#include "midgard/logging.h"

#include <h3api.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace valhalla {
namespace thor {
namespace region_grid {

namespace {

constexpr uint64_t kInvalidH3 = 0;

struct CountryAgg {
  uint64_t weight = 0;
  std::vector<uint32_t> nodes;
};

uint64_t edge_weight(const cch::CchGraph& graph, const cch::CchBaseEdge& e) {
  // Prefer non-transition truck time; transitions contribute 0.
  if (e.roadclass == 255)
    return 0;
  return e.time_s == 0 ? 1 : static_cast<uint64_t>(e.time_s);
}

std::unordered_map<std::string, CountryAgg> aggregate_by_country(const cch::CchGraph& graph) {
  std::unordered_map<std::string, CountryAgg> by_country;
  by_country.reserve(64);
  for (uint32_t i = 0; i < graph.nodes.size(); ++i) {
    const auto& n = graph.nodes[i];
    std::string iso = n.country.empty() ? "XX" : n.country.substr(0, 2);
    by_country[iso].nodes.push_back(i);
  }
  for (const auto& e : graph.edges) {
    if (e.u >= graph.nodes.size() || e.v >= graph.nodes.size())
      continue;
    const auto& a = graph.nodes[e.u];
    const auto& b = graph.nodes[e.v];
    if (a.country != b.country)
      continue;
    std::string iso = a.country.empty() ? "XX" : a.country.substr(0, 2);
    by_country[iso].weight += edge_weight(graph, e);
  }
  // Ensure every country with nodes has nonzero weight for budgeting.
  for (auto& kv : by_country) {
    if (kv.second.weight == 0)
      kv.second.weight = std::max<size_t>(1, kv.second.nodes.size());
  }
  return by_country;
}

uint32_t allocate_budget(uint64_t country_w, uint64_t total_w, uint32_t target) {
  if (total_w == 0 || target == 0)
    return 1;
  const double share = static_cast<double>(country_w) / static_cast<double>(total_w);
  const uint32_t n = static_cast<uint32_t>(std::llround(share * target));
  return std::max(1u, n);
}

struct CellAcc {
  std::vector<uint32_t> nodes;
  uint64_t weight = 0;
};

std::unordered_map<uint64_t, CellAcc>
build_cells_at_res(const cch::CchGraph& graph,
                   const std::vector<uint32_t>& country_nodes,
                   int res,
                   const std::unordered_map<uint32_t, uint64_t>& node_weight) {
  std::unordered_map<uint64_t, CellAcc> cells;
  cells.reserve(country_nodes.size() / 8 + 1);
  for (uint32_t ni : country_nodes) {
    const auto& n = graph.nodes[ni];
    const uint64_t h3 = LatLngToH3(n.lat, n.lon, res);
    if (h3 == kInvalidH3)
      continue;
    auto& cell = cells[h3];
    cell.nodes.push_back(ni);
    auto wit = node_weight.find(ni);
    cell.weight += wit == node_weight.end() ? 1 : wit->second;
  }
  return cells;
}

std::unordered_map<uint32_t, uint64_t> per_node_weight(const cch::CchGraph& graph) {
  std::unordered_map<uint32_t, uint64_t> w;
  w.reserve(graph.nodes.size());
  for (const auto& e : graph.edges) {
    const uint64_t ew = edge_weight(graph, e);
    if (ew == 0)
      continue;
    w[e.u] += ew;
    w[e.v] += ew;
  }
  return w;
}

std::vector<CellSeed> tune_country_cells(const cch::CchGraph& graph,
                                         const std::string& country,
                                         const std::vector<uint32_t>& country_nodes,
                                         uint32_t target_c,
                                         const RegionGridOptions& options,
                                         const std::unordered_map<uint32_t, uint64_t>& node_weight) {
  if (country_nodes.empty() || target_c == 0)
    return {};

  int res = options.base_h3_res;
  auto cells = build_cells_at_res(graph, country_nodes, res, node_weight);
  LOG_INFO("region_grid: country=" + country + " nodes=" + std::to_string(country_nodes.size()) +
           " cells_before_merge=" + std::to_string(cells.size()) + " target=" +
           std::to_string(target_c) + " res=" + std::to_string(res));

  // Too many cells: merge into parent (coarser) until near target, or neighbor-merge.
  while (cells.size() > target_c && res > 0) {
    std::unordered_map<uint64_t, CellAcc> parents;
    for (auto& kv : cells) {
      const uint64_t parent = H3Parent(kv.first, res - 1);
      if (parent == kInvalidH3) {
        parents[kv.first] = std::move(kv.second);
        continue;
      }
      auto& p = parents[parent];
      p.nodes.insert(p.nodes.end(), kv.second.nodes.begin(), kv.second.nodes.end());
      p.weight += kv.second.weight;
    }
    cells = std::move(parents);
    --res;
  }

  // Still too many: greedily merge smallest into a same-country H3 neighbor that exists.
  while (cells.size() > target_c) {
    // Find lightest cell
    uint64_t lightest = 0;
    uint64_t light_w = UINT64_MAX;
    for (const auto& kv : cells) {
      if (kv.second.weight < light_w) {
        light_w = kv.second.weight;
        lightest = kv.first;
      }
    }
    auto neighbors = H3GridDisk(lightest, 1);
    uint64_t merge_into = kInvalidH3;
    uint64_t best_w = UINT64_MAX;
    for (uint64_t nb : neighbors) {
      if (nb == lightest || nb == kInvalidH3)
        continue;
      auto it = cells.find(nb);
      if (it == cells.end())
        continue;
      if (it->second.weight < best_w) {
        best_w = it->second.weight;
        merge_into = nb;
      }
    }
    if (merge_into == kInvalidH3) {
      // No neighbor: merge into globally lightest other cell
      for (const auto& kv : cells) {
        if (kv.first == lightest)
          continue;
        if (kv.second.weight < best_w) {
          best_w = kv.second.weight;
          merge_into = kv.first;
        }
      }
    }
    if (merge_into == kInvalidH3 || merge_into == lightest)
      break;
    auto& dst = cells[merge_into];
    auto& src = cells[lightest];
    dst.nodes.insert(dst.nodes.end(), src.nodes.begin(), src.nodes.end());
    dst.weight += src.weight;
    cells.erase(lightest);
  }

  // Too few cells: split heaviest at finer resolution.
  while (cells.size() < target_c && res < options.max_h3_res) {
    uint64_t heaviest = 0;
    uint64_t heavy_w = 0;
    for (const auto& kv : cells) {
      if (kv.second.weight > heavy_w && kv.second.nodes.size() > 1) {
        heavy_w = kv.second.weight;
        heaviest = kv.first;
      }
    }
    if (heaviest == kInvalidH3 || heavy_w == 0)
      break;

    auto src = std::move(cells[heaviest]);
    cells.erase(heaviest);
    auto children = build_cells_at_res(graph, src.nodes, res + 1, node_weight);
    if (children.size() <= 1) {
      // Could not split usefully; put back and stop.
      cells[heaviest] = std::move(src);
      break;
    }
    for (auto& ch : children)
      cells[ch.first] = std::move(ch.second);
    ++res;
    // Avoid runaway growth: if we already met/exceeded target, stop splitting.
    if (cells.size() >= target_c)
      break;
  }

  // Final merge pass: splits (and coarse geography) can leave us above target.
  while (cells.size() > target_c) {
    uint64_t lightest = 0;
    uint64_t light_w = UINT64_MAX;
    for (const auto& kv : cells) {
      if (kv.second.weight < light_w) {
        light_w = kv.second.weight;
        lightest = kv.first;
      }
    }
    uint64_t merge_into = kInvalidH3;
    uint64_t best_w = UINT64_MAX;
    auto neighbors = H3GridDisk(lightest, 1);
    for (uint64_t nb : neighbors) {
      if (nb == lightest || nb == kInvalidH3)
        continue;
      auto it = cells.find(nb);
      if (it == cells.end())
        continue;
      if (it->second.weight < best_w) {
        best_w = it->second.weight;
        merge_into = nb;
      }
    }
    if (merge_into == kInvalidH3) {
      for (const auto& kv : cells) {
        if (kv.first == lightest)
          continue;
        if (kv.second.weight < best_w) {
          best_w = kv.second.weight;
          merge_into = kv.first;
        }
      }
    }
    if (merge_into == kInvalidH3 || merge_into == lightest)
      break;
    auto& dst = cells[merge_into];
    auto& src = cells[lightest];
    dst.nodes.insert(dst.nodes.end(), src.nodes.begin(), src.nodes.end());
    dst.weight += src.weight;
    cells.erase(lightest);
  }

  std::vector<CellSeed> out;
  out.reserve(cells.size());
  for (auto& kv : cells) {
    if (kv.second.nodes.empty())
      continue;
    CellSeed seed;
    seed.h3_index = kv.first;
    seed.country = country;
    seed.node_indices = std::move(kv.second.nodes);
    seed.weight = kv.second.weight;
    out.push_back(std::move(seed));
  }
  LOG_INFO("region_grid: country=" + country + " cells_final=" + std::to_string(out.size()));
  return out;
}

} // namespace

uint64_t LatLngToH3(double lat, double lon, int res) {
  LatLng ll;
  ll.lat = degsToRads(lat);
  ll.lng = degsToRads(lon);
  H3Index out = 0;
  if (latLngToCell(&ll, res, &out) != E_SUCCESS)
    return kInvalidH3;
  return out;
}

int H3Resolution(uint64_t h3_index) {
  return getResolution(h3_index);
}

uint64_t H3Parent(uint64_t h3_index, int parent_res) {
  H3Index out = 0;
  if (cellToParent(h3_index, parent_res, &out) != E_SUCCESS)
    return kInvalidH3;
  return out;
}

std::vector<uint64_t> H3GridDisk(uint64_t h3_index, int k) {
  int64_t max_size = 0;
  if (maxGridDiskSize(k, &max_size) != E_SUCCESS || max_size <= 0)
    return {};
  std::vector<H3Index> buf(static_cast<size_t>(max_size), 0);
  if (gridDisk(h3_index, k, buf.data()) != E_SUCCESS)
    return {};
  std::vector<uint64_t> out;
  out.reserve(buf.size());
  for (H3Index h : buf) {
    if (h != 0)
      out.push_back(h);
  }
  return out;
}

std::vector<std::pair<double, double>> H3BoundaryLatLng(uint64_t h3_index) {
  CellBoundary bnd;
  if (cellToBoundary(h3_index, &bnd) != E_SUCCESS)
    return {};
  std::vector<std::pair<double, double>> pts;
  pts.reserve(static_cast<size_t>(bnd.numVerts));
  for (int i = 0; i < bnd.numVerts; ++i) {
    pts.emplace_back(radsToDegs(bnd.verts[i].lat), radsToDegs(bnd.verts[i].lng));
  }
  return pts;
}

std::vector<CellSeed> PartitionH3Cells(const cch::CchGraph& graph, const RegionGridOptions& options) {
  auto by_country = aggregate_by_country(graph);
  uint64_t total_w = 0;
  for (const auto& kv : by_country)
    total_w += kv.second.weight;

  auto node_w = per_node_weight(graph);
  std::vector<CellSeed> all;
  all.reserve(options.target_regions);

  // Stable country order for deterministic budgets.
  std::vector<std::string> countries;
  countries.reserve(by_country.size());
  for (const auto& kv : by_country)
    countries.push_back(kv.first);
  std::sort(countries.begin(), countries.end());

  uint32_t assigned = 0;
  for (size_t i = 0; i < countries.size(); ++i) {
    const auto& iso = countries[i];
    const auto& agg = by_country[iso];
    uint32_t target_c = allocate_budget(agg.weight, total_w, options.target_regions);
    // Last country absorbs rounding remainder toward global target.
    if (i + 1 == countries.size() && assigned < options.target_regions)
      target_c = std::max(target_c, options.target_regions - assigned);
    auto seeds = tune_country_cells(graph, iso, agg.nodes, target_c, options, node_w);
    assigned += static_cast<uint32_t>(seeds.size());
    for (auto& s : seeds)
      all.push_back(std::move(s));
  }

  LOG_INFO("region_grid: partition done  countries=" + std::to_string(countries.size()) +
           " cells=" + std::to_string(all.size()) + " target=" +
           std::to_string(options.target_regions));
  return all;
}

} // namespace region_grid
} // namespace thor
} // namespace valhalla
