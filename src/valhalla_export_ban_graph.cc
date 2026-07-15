#include "argparse_utils.h"
#include "baldr/admin.h"
#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace valhalla::baldr;
using namespace valhalla::midgard;

// Exports a routable subgraph as two CSVs consumable by the tch_ban prototype:
//   nodes.csv : id,lat,lon,country,tz
//   edges.csv : u,v,time_s,roadclass
//
// "id" is the 64-bit GraphId value of a node. Edges are directed (one row per
// directed edge). Regular edges stay within a hierarchy level; transition edges
// are emitted as zero-time connectors so a multi-level export stays connected.
// Two passes guarantee no edge references a node that was filtered out.

namespace {

struct Bbox {
  bool active = false;
  double min_lat = 0, min_lon = 0, max_lat = 0, max_lon = 0;
  bool contains(const PointLL& ll) const {
    if (!active) {
      return true;
    }
    return ll.lat() >= min_lat && ll.lat() <= max_lat && ll.lng() >= min_lon &&
           ll.lng() <= max_lon;
  }
};

std::set<uint32_t> parse_levels(const std::string& s) {
  std::set<uint32_t> levels;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    if (!tok.empty()) {
      levels.insert(static_cast<uint32_t>(std::stoul(tok)));
    }
  }
  return levels;
}

Bbox parse_bbox(const std::string& s) {
  Bbox b;
  if (s.empty()) {
    return b;
  }
  std::stringstream ss(s);
  std::string tok;
  std::vector<double> vals;
  while (std::getline(ss, tok, ',')) {
    vals.push_back(std::stod(tok));
  }
  if (vals.size() != 4) {
    throw std::runtime_error("--bbox needs min_lat,min_lon,max_lat,max_lon");
  }
  b.active = true;
  b.min_lat = vals[0];
  b.min_lon = vals[1];
  b.max_lat = vals[2];
  b.max_lon = vals[3];
  return b;
}

std::string country_of(const graph_tile_ptr& tile, const NodeInfo* node) {
  const Admin* admin = tile->admin(node->admin_index());
  if (admin == nullptr) {
    return "XX";
  }
  std::string iso = admin->country_iso();
  return iso.size() >= 2 ? iso : "XX";
}

} // namespace

int main(int argc, char* argv[]) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  boost::property_tree::ptree config;
  std::string nodes_path, edges_path, bbox_str, levels_str;
  uint32_t max_class = 0;
  bool hgv_only = false;

  try {
    cxxopts::Options options(
        program, program + " " + VALHALLA_PRINT_VERSION + "\n\n"
        "Exports a routable subgraph (nodes.csv/edges.csv) for the\n"
        "bans-only time-dependent CH feasibility prototype.\n\n");
    // clang-format off
    options.add_options()
      ("h,help", "Print this help message.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline json config.", cxxopts::value<std::string>())
      ("nodes", "Output nodes CSV path.", cxxopts::value<std::string>(nodes_path)->default_value("nodes.csv"))
      ("edges", "Output edges CSV path.", cxxopts::value<std::string>(edges_path)->default_value("edges.csv"))
      ("bbox", "Optional bounding box min_lat,min_lon,max_lat,max_lon.", cxxopts::value<std::string>(bbox_str)->default_value(""))
      ("levels", "Comma-separated hierarchy levels to export (0=highway,1=arterial,2=local).", cxxopts::value<std::string>(levels_str)->default_value("0,1"))
      ("max-class", "Max RoadClass to include (0=motorway .. 7=service).", cxxopts::value<uint32_t>(max_class)->default_value("7"))
      ("hgv-only", "Only export truck-accessible edges.", cxxopts::value<bool>(hgv_only)->default_value("false"));
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config)) {
      return EXIT_SUCCESS;
    }
  } catch (std::exception& e) {
    std::cerr << "Failed to parse options: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  valhalla::midgard::logging::Configure({{"type", "std_err"}, {"color", "true"}});

  const Bbox bbox = parse_bbox(bbox_str);
  const std::set<uint32_t> levels = parse_levels(levels_str);

  GraphReader reader(config.get_child("mjolnir"));

  // ---- Pass 1: nodes -----------------------------------------------------
  LOG_INFO("Pass 1/2: exporting nodes...");
  std::unordered_set<uint64_t> emitted;
  emitted.reserve(1u << 20);
  std::ofstream nout(nodes_path);
  nout << "id,lat,lon,country,tz\n";

  for (const auto& tile_id : reader.GetTileSet()) {
    if (levels.find(tile_id.level()) == levels.end()) {
      continue;
    }
    auto tile = reader.GetGraphTile(tile_id);
    if (tile == nullptr) {
      continue;
    }
    const uint32_t n = tile->header()->nodecount();
    for (uint32_t idx = 0; idx < n; ++idx) {
      GraphId nid(tile_id.tileid(), tile_id.level(), idx);
      const PointLL ll = tile->get_node_ll(nid);
      if (!bbox.contains(ll)) {
        continue;
      }
      const NodeInfo* node = tile->node(nid);
      emitted.insert(nid.value);
      nout << nid.value << ',' << ll.lat() << ',' << ll.lng() << ','
           << country_of(tile, node) << ',' << node->timezone() << '\n';
    }
    reader.Clear();
  }
  nout.close();
  LOG_INFO("  wrote " + std::to_string(emitted.size()) + " nodes");

  // ---- Pass 2: edges -----------------------------------------------------
  LOG_INFO("Pass 2/2: exporting edges...");
  std::ofstream eout(edges_path);
  eout << "u,v,time_s,roadclass\n";
  uint64_t edge_rows = 0;

  for (const auto& tile_id : reader.GetTileSet()) {
    if (levels.find(tile_id.level()) == levels.end()) {
      continue;
    }
    auto tile = reader.GetGraphTile(tile_id);
    if (tile == nullptr) {
      continue;
    }
    const uint32_t n = tile->header()->nodecount();
    for (uint32_t idx = 0; idx < n; ++idx) {
      GraphId nid(tile_id.tileid(), tile_id.level(), idx);
      if (emitted.find(nid.value) == emitted.end()) {
        continue;
      }
      const NodeInfo* node = tile->node(nid);

      // regular directed edges
      for (uint32_t i = 0; i < node->edge_count(); ++i) {
        const DirectedEdge* de = tile->directededge(node->edge_index() + i);
        if (de->is_shortcut()) {
          continue;
        }
        if (hgv_only && !(de->forwardaccess() & kTruckAccess)) {
          continue;
        }
        if (static_cast<uint32_t>(de->classification()) > max_class) {
          continue;
        }
        const GraphId endnode = de->endnode();
        if (emitted.find(endnode.value) == emitted.end()) {
          continue;
        }
        const uint32_t speed = de->truck_speed() ? de->truck_speed() : de->speed();
        if (speed == 0 || de->length() == 0) {
          continue;
        }
        const double time_s = de->length() * 3.6 / static_cast<double>(speed);
        eout << nid.value << ',' << endnode.value << ',' << time_s << ','
             << static_cast<uint32_t>(de->classification()) << '\n';
        ++edge_rows;
      }

      // transition edges as zero-time connectors (keep levels connected)
      const uint32_t tcount = node->transition_count();
      for (uint32_t i = 0; i < tcount; ++i) {
        const NodeTransition* trans = tile->transition(node->transition_index() + i);
        const GraphId tnode = trans->endnode();
        if (emitted.find(tnode.value) == emitted.end()) {
          continue;
        }
        eout << nid.value << ',' << tnode.value << ",0.0,255\n";
        ++edge_rows;
      }
    }
    reader.Clear();
  }
  eout.close();
  LOG_INFO("  wrote " + std::to_string(edge_rows) + " edges");
  LOG_INFO("Done: " + nodes_path + " , " + edges_path);
  return EXIT_SUCCESS;
}
