#include "argparse_utils.h"
#include "midgard/logging.h"
#include "thor/cch/cch_graph.h"
#include "thor/region_grid/h3_partition.h"
#include "thor/region_grid/network_voronoi.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace valhalla;

namespace {

struct RegionRow {
  uint32_t region_id = 0;
  std::string country;
  uint64_t h3_index = 0;
  uint64_t rep_graph_id = 0;
  double rep_lat = 0;
  double rep_lon = 0;
  uint32_t node_count = 0;
};

std::set<uint32_t> parse_levels(const std::string& s) {
  std::set<uint32_t> levels;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    if (!tok.empty())
      levels.insert(static_cast<uint32_t>(std::stoul(tok)));
  }
  return levels;
}

uint32_t resolve_concurrency(uint32_t concurrency) {
  if (concurrency == 0) {
    const unsigned hc = std::thread::hardware_concurrency();
    return hc == 0 ? 1u : hc;
  }
  return concurrency;
}

std::vector<RegionRow> load_regions_csv(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in)
    throw std::runtime_error("cannot open regions csv: " + path.string());
  std::vector<RegionRow> rows;
  std::string line;
  if (!std::getline(in, line))
    return rows;
  while (std::getline(in, line)) {
    if (line.empty())
      continue;
    std::stringstream ss(line);
    std::string tok;
    std::vector<std::string> cols;
    while (std::getline(ss, tok, ','))
      cols.push_back(tok);
    if (cols.size() < 6)
      continue;
    RegionRow r;
    r.region_id = static_cast<uint32_t>(std::stoul(cols[0]));
    r.country = cols[1];
    r.h3_index = std::stoull(cols[2]);
    r.rep_graph_id = std::stoull(cols[3]);
    r.rep_lat = std::stod(cols[4]);
    r.rep_lon = std::stod(cols[5]);
    if (cols.size() > 6)
      r.node_count = static_cast<uint32_t>(std::stoul(cols[6]));
    rows.push_back(r);
  }
  return rows;
}

void write_regions_csv(const std::filesystem::path& path, const std::vector<RegionRow>& rows) {
  std::ofstream out(path);
  out << "region_id,country,h3,rep_graph_id,rep_lat,rep_lon,node_count\n";
  out << std::setprecision(8);
  for (const auto& r : rows) {
    out << r.region_id << ',' << r.country << ',' << r.h3_index << ',' << r.rep_graph_id << ','
        << r.rep_lat << ',' << r.rep_lon << ',' << r.node_count << '\n';
  }
}

void write_node_regions_csv(const std::filesystem::path& path,
                            const thor::cch::CchGraph& graph,
                            const std::vector<thor::region_grid::NodeAssignment>& node_regions) {
  std::ofstream out(path);
  out << "graph_id,region_id,time_to_rep_s,lat,lon\n";
  out << std::setprecision(8);
  constexpr uint32_t kUnassigned = std::numeric_limits<uint32_t>::max();
  for (size_t i = 0; i < graph.nodes.size() && i < node_regions.size(); ++i) {
    if (node_regions[i].region_id == kUnassigned)
      continue;
    out << graph.nodes[i].graph_id << ',' << node_regions[i].region_id << ','
        << node_regions[i].time_to_rep_s << ',' << graph.nodes[i].lat << ','
        << graph.nodes[i].lon << '\n';
  }
}

void write_meta_json(const std::filesystem::path& path,
                     const thor::cch::CchGraph& graph,
                     size_t n_regions,
                     uint32_t fallback_count,
                     const std::string& levels_str,
                     uint32_t max_class,
                     bool hgv_only) {
  std::ofstream out(path);
  out << "{\n";
  out << "  \"tile_build_hash\": " << graph.tile_build_hash << ",\n";
  out << "  \"levels\": \"" << levels_str << "\",\n";
  out << "  \"max_class\": " << max_class << ",\n";
  out << "  \"hgv_only\": " << (hgv_only ? "true" : "false") << ",\n";
  out << "  \"actual_regions\": " << n_regions << ",\n";
  out << "  \"nodes\": " << graph.nodes.size() << ",\n";
  out << "  \"edges\": " << graph.edges.size() << ",\n";
  out << "  \"euclidean_fallback_nodes\": " << fallback_count << ",\n";
  out << "  \"tool\": \"valhalla_revoronoi_region_grid\",\n";
  out << "  \"version\": \"" << VALHALLA_PRINT_VERSION << "\"\n";
  out << "}\n";
}

void write_geojson(const std::filesystem::path& path, const std::vector<RegionRow>& regions) {
  std::ofstream out(path);
  out << std::setprecision(8);
  out << "{\"type\":\"FeatureCollection\",\"features\":[\n";
  bool first = true;
  for (const auto& r : regions) {
    auto ring = thor::region_grid::H3BoundaryLatLng(r.h3_index);
    if (ring.empty())
      continue;
    if (!first)
      out << ",\n";
    first = false;
    out << "{\"type\":\"Feature\",\"properties\":{"
        << "\"region_id\":" << r.region_id << ",\"country\":\"" << r.country << "\",\"h3\":\""
        << r.h3_index << "\",\"node_count\":" << r.node_count << "},\"geometry\":{"
        << "\"type\":\"Polygon\",\"coordinates\":[[";
    for (size_t i = 0; i < ring.size(); ++i) {
      if (i)
        out << ',';
      out << '[' << ring[i].second << ',' << ring[i].first << ']';
    }
    out << ",[" << ring[0].second << ',' << ring[0].first << ']';
    out << "]]}}";
  }
  out << "\n]}\n";
}

} // namespace

int main(int argc, char* argv[]) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  boost::property_tree::ptree config;
  std::string out_dir, levels_str, regions_path;
  uint32_t max_class = 6;
  uint32_t concurrency = 0;
  bool hgv_only = true;
  float truck_weight_t = 40.f;

  try {
    cxxopts::Options options(program, program + " " + VALHALLA_PRINT_VERSION +
                                          "\n\nRebuilds node_regions.csv via network Voronoi from a "
                                          "pruned regions.csv (no H3/medoid re-pick).\n\n");
    // clang-format off
    options.add_options()
      ("h,help", "Print this help message.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline json config.", cxxopts::value<std::string>())
      ("regions", "Pruned regions.csv path.", cxxopts::value<std::string>(regions_path))
      ("out-dir", "Output directory (writes regions.csv, node_regions.csv, meta).",
        cxxopts::value<std::string>(out_dir))
      ("levels", "Comma-separated hierarchy levels.",
        cxxopts::value<std::string>(levels_str)->default_value("0,1"))
      ("max-class", "Max RoadClass to include.",
        cxxopts::value<uint32_t>(max_class)->default_value("6"))
      ("hgv-only", "Only include truck-accessible edges.",
        cxxopts::value<bool>(hgv_only)->default_value("true")->implicit_value("true"))
      ("truck-weight", "Truck weight in metric tons.",
        cxxopts::value<float>(truck_weight_t)->default_value("40"))
      ("j,concurrency", "Worker threads for subgraph load (0=hardware_concurrency).",
        cxxopts::value<uint32_t>(concurrency)->default_value("0"));
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config))
      return EXIT_SUCCESS;
    if (regions_path.empty() || out_dir.empty()) {
      std::cerr << "required: --regions and --out-dir\n";
      return EXIT_FAILURE;
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse options: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  concurrency = resolve_concurrency(concurrency);
  std::filesystem::create_directories(out_dir);

  auto regions = load_regions_csv(regions_path);
  if (regions.empty()) {
    std::cerr << "no regions loaded from " << regions_path << "\n";
    return EXIT_FAILURE;
  }

  LOG_INFO("valhalla_revoronoi_region_grid: regions=" + std::to_string(regions.size()) +
           " weight_t=" + std::to_string(truck_weight_t) + " out_dir=" + out_dir);

  thor::cch::TruckGraphOptions truck_opts;
  truck_opts.hgv_only = hgv_only;
  truck_opts.exclude_destonly_hgv = true;
  truck_opts.apply_access_restrictions = true;
  truck_opts.vehicle.weight_t = truck_weight_t;
  auto graph =
      thor::cch::BuildTruckGraph(config.get_child("mjolnir"), parse_levels(levels_str),
                                 static_cast<uint8_t>(max_class), concurrency, truck_opts);

  std::vector<uint32_t> medoid_nodes;
  std::vector<uint32_t> region_ids;
  medoid_nodes.reserve(regions.size());
  region_ids.reserve(regions.size());
  for (const auto& r : regions) {
    const int idx = graph.index_of(r.rep_graph_id);
    if (idx < 0) {
      std::cerr << "rep_graph_id " << r.rep_graph_id << " (region_id=" << r.region_id
                << ") missing from truck graph\n";
      return EXIT_FAILURE;
    }
    medoid_nodes.push_back(static_cast<uint32_t>(idx));
    region_ids.push_back(r.region_id);
  }

  auto voronoi = thor::region_grid::ComputeNetworkVoronoi(graph, medoid_nodes, region_ids);

  std::unordered_map<uint32_t, uint32_t> counts;
  constexpr uint32_t kUnassigned = std::numeric_limits<uint32_t>::max();
  for (const auto& a : voronoi.node_regions) {
    if (a.region_id != kUnassigned)
      ++counts[a.region_id];
  }
  for (auto& r : regions)
    r.node_count = counts[r.region_id];

  const std::filesystem::path dir(out_dir);
  write_regions_csv(dir / "regions.csv", regions);
  write_node_regions_csv(dir / "node_regions.csv", graph, voronoi.node_regions);
  write_meta_json(dir / "region_grid_meta.json", graph, regions.size(),
                  voronoi.euclidean_fallback_count, levels_str, max_class, hgv_only);
  // Always rewrite GeoJSON so visualize_region_grid cannot pick up a stale
  // pre-prune FeatureCollection.
  write_geojson(dir / "regions.geojson", regions);

  LOG_INFO("valhalla_revoronoi_region_grid: wrote artifacts under " + out_dir +
           " regions=" + std::to_string(regions.size()) +
           " fallback=" + std::to_string(voronoi.euclidean_fallback_count));
  return EXIT_SUCCESS;
}
