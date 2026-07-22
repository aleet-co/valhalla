#include "argparse_utils.h"
#include "midgard/logging.h"
#include "thor/cch/cch_graph.h"
#include "thor/region_grid/h3_partition.h"
#include "thor/region_grid/region_grid.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <chrono>
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
#include <vector>

using namespace valhalla;

namespace {

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

std::vector<std::string> parse_csv_tokens(const std::string& s) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    // trim spaces
    size_t b = tok.find_first_not_of(" \t");
    if (b == std::string::npos)
      continue;
    size_t e = tok.find_last_not_of(" \t");
    out.push_back(tok.substr(b, e - b + 1));
  }
  return out;
}

uint32_t resolve_concurrency(uint32_t concurrency) {
  if (concurrency == 0) {
    const unsigned hc = std::thread::hardware_concurrency();
    return hc == 0 ? 1u : hc;
  }
  return concurrency;
}

void write_regions_csv(const std::filesystem::path& path,
                       const thor::cch::CchGraph& graph,
                       const thor::region_grid::RegionGridResult& result) {
  std::ofstream out(path);
  out << "region_id,country,h3,rep_graph_id,rep_lat,rep_lon,node_count\n";
  out << std::setprecision(8);
  for (const auto& r : result.regions) {
    uint64_t rep_gid = 0;
    if (r.rep_node_index < graph.nodes.size())
      rep_gid = graph.nodes[r.rep_node_index].graph_id;
    out << r.region_id << ',' << r.country << ',' << r.h3_index << ',' << rep_gid << ','
        << r.rep_lat << ',' << r.rep_lon << ',' << r.node_count << '\n';
  }
}

void write_node_regions_csv(const std::filesystem::path& path,
                            const thor::cch::CchGraph& graph,
                            const thor::region_grid::RegionGridResult& result) {
  std::ofstream out(path);
  out << "graph_id,region_id,time_to_rep_s\n";
  constexpr uint32_t kUnassigned = std::numeric_limits<uint32_t>::max();
  for (size_t i = 0; i < graph.nodes.size() && i < result.node_regions.size(); ++i) {
    if (result.node_regions[i].region_id == kUnassigned)
      continue;
    out << graph.nodes[i].graph_id << ',' << result.node_regions[i].region_id << ','
        << result.node_regions[i].time_to_rep_s << '\n';
  }
}

void write_meta_json(const std::filesystem::path& path,
                     const thor::cch::CchGraph& graph,
                     const thor::region_grid::RegionGridResult& result,
                     const thor::region_grid::RegionGridOptions& options,
                     const std::string& levels_str,
                     uint32_t max_class,
                     bool hgv_only) {
  std::ofstream out(path);
  out << "{\n";
  out << "  \"tile_build_hash\": " << graph.tile_build_hash << ",\n";
  out << "  \"levels\": \"" << levels_str << "\",\n";
  out << "  \"max_class\": " << max_class << ",\n";
  out << "  \"hgv_only\": " << (hgv_only ? "true" : "false") << ",\n";
  out << "  \"target_regions\": " << options.target_regions << ",\n";
  out << "  \"actual_regions\": " << result.regions.size() << ",\n";
  out << "  \"nodes\": " << graph.nodes.size() << ",\n";
  out << "  \"edges\": " << graph.edges.size() << ",\n";
  out << "  \"euclidean_fallback_nodes\": " << result.euclidean_fallback_count << ",\n";
  out << "  \"unassigned_nodes\": " << result.unassigned_nodes << ",\n";
  out << "  \"base_h3_res\": " << options.base_h3_res << ",\n";
  out << "  \"max_h3_res\": " << options.max_h3_res << ",\n";
  out << "  \"dense_max_h3_res\": " << options.dense_max_h3_res << ",\n";
  out << "  \"dense_density_factor\": " << options.dense_density_factor << ",\n";
  out << "  \"exclude_countries\": [";
  for (size_t i = 0; i < options.exclude_countries.size(); ++i) {
    if (i)
      out << ", ";
    out << "\"" << options.exclude_countries[i] << "\"";
  }
  out << "],\n";
  out << "  \"tool\": \"valhalla_build_region_grid\",\n";
  out << "  \"version\": \"" << VALHALLA_PRINT_VERSION << "\"\n";
  out << "}\n";
}

void write_geojson(const std::filesystem::path& path,
                   const thor::region_grid::RegionGridResult& result) {
  std::ofstream out(path);
  out << std::setprecision(8);
  out << "{\"type\":\"FeatureCollection\",\"features\":[\n";
  bool first = true;
  for (const auto& r : result.regions) {
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
      // GeoJSON is lon,lat
      if (i)
        out << ',';
      out << '[' << ring[i].second << ',' << ring[i].first << ']';
    }
    // close ring
    out << ",[" << ring[0].second << ',' << ring[0].first << ']';
    out << "]]}}";
  }
  out << "\n]}\n";
}

} // namespace

int main(int argc, char* argv[]) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  boost::property_tree::ptree config;
  std::string out_dir, levels_str, exclude_countries_str;
  uint32_t max_class = 6;
  uint32_t concurrency = 0;
  uint32_t target_regions = 6000;
  int base_h3_res = 5;
  int max_h3_res = 6;
  int dense_max_h3_res = 5;
  double dense_density_factor = 1.5;
  bool hgv_only = true;
  bool write_geojson_flag = false;

  try {
    cxxopts::Options options(
        program, program + " " + VALHALLA_PRINT_VERSION + "\n\n"
        "Builds a country-clipped H3 truck region grid with network medoids\n"
        "and in-country Voronoi ownership from Valhalla tiles.\n\n");
    // clang-format off
    options.add_options()
      ("h,help", "Print this help message.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline json config.", cxxopts::value<std::string>())
      ("out-dir", "Output directory for region grid artifacts.",
        cxxopts::value<std::string>(out_dir)->default_value("/custom_files/region_grid"))
      ("levels", "Comma-separated hierarchy levels (0=highway,1=arterial,2=local).",
        cxxopts::value<std::string>(levels_str)->default_value("0,1"))
      ("max-class", "Max RoadClass to include (0=motorway .. 7=service).",
        cxxopts::value<uint32_t>(max_class)->default_value("6"))
      ("hgv-only", "Only include truck-accessible edges.",
        cxxopts::value<bool>(hgv_only)->default_value("true"))
      ("target-regions", "Target number of regions (excludes dropped countries).",
        cxxopts::value<uint32_t>(target_regions)->default_value("6000"))
      ("base-h3-res", "Initial H3 resolution before merge/split.",
        cxxopts::value<int>(base_h3_res)->default_value("5"))
      ("max-h3-res", "Hard cap on H3 resolution for any cell.",
        cxxopts::value<int>(max_h3_res)->default_value("6"))
      ("dense-max-h3-res",
        "Cap H3 refinement for dense (high weight/km²) cells; sparse cells may use max-h3-res.",
        cxxopts::value<int>(dense_max_h3_res)->default_value("5"))
      ("dense-density-factor",
        "Cell is dense when weight/km² ≥ factor × country mean density.",
        cxxopts::value<double>(dense_density_factor)->default_value("1.5"))
      ("exclude-countries",
        "Comma-separated ISO2 codes to drop (empty = keep all). Default RU,BY.",
        cxxopts::value<std::string>(exclude_countries_str)->default_value("RU,BY"))
      ("j,concurrency", "Worker threads for subgraph load (0=hardware_concurrency).",
        cxxopts::value<uint32_t>(concurrency)->default_value("0"))
      ("write-geojson", "Also write regions.geojson for QA.",
        cxxopts::value<bool>(write_geojson_flag)->default_value("false"));
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config))
      return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse options: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  const auto levels = parse_levels(levels_str);
  concurrency = resolve_concurrency(concurrency);
  std::filesystem::create_directories(out_dir);

  using clock = std::chrono::steady_clock;
  auto secs = [](clock::time_point t0) {
    return std::chrono::duration<double>(clock::now() - t0).count();
  };

  thor::region_grid::RegionGridOptions grid_opts;
  grid_opts.target_regions = target_regions;
  grid_opts.base_h3_res = base_h3_res;
  grid_opts.max_h3_res = max_h3_res;
  grid_opts.dense_max_h3_res = dense_max_h3_res;
  grid_opts.dense_density_factor = dense_density_factor;
  grid_opts.exclude_countries = parse_csv_tokens(exclude_countries_str);

  LOG_INFO("valhalla_build_region_grid: levels=" + levels_str +
           " max_class=" + std::to_string(max_class) + " hgv_only=" + (hgv_only ? "1" : "0") +
           " target=" + std::to_string(target_regions) +
           " dense_max_h3_res=" + std::to_string(dense_max_h3_res) +
           " exclude=" + exclude_countries_str + " out_dir=" + out_dir);

  LOG_INFO("valhalla_build_region_grid: phase 1/2 Building truck subgraph...");
  const auto t1 = clock::now();
  auto graph = thor::cch::BuildTruckGraph(config.get_child("mjolnir"), levels,
                                          static_cast<uint8_t>(max_class), concurrency, hgv_only);
  LOG_INFO("valhalla_build_region_grid: phase 1/2 done  nodes=" +
           std::to_string(graph.nodes.size()) + " edges=" + std::to_string(graph.edges.size()) +
           " phase_s=" + std::to_string(secs(t1)));

  LOG_INFO("valhalla_build_region_grid: phase 2/2 Building region grid...");
  const auto t2 = clock::now();
  auto grid = thor::region_grid::BuildRegionGrid(graph, grid_opts);
  LOG_INFO("valhalla_build_region_grid: phase 2/2 done  regions=" +
           std::to_string(grid.regions.size()) + " phase_s=" + std::to_string(secs(t2)));

  const std::filesystem::path dir(out_dir);
  write_regions_csv(dir / "regions.csv", graph, grid);
  write_node_regions_csv(dir / "node_regions.csv", graph, grid);
  write_meta_json(dir / "region_grid_meta.json", graph, grid, grid_opts, levels_str, max_class,
                  hgv_only);
  if (write_geojson_flag)
    write_geojson(dir / "regions.geojson", grid);

  LOG_INFO("valhalla_build_region_grid: wrote artifacts under " + out_dir);
  return EXIT_SUCCESS;
}
