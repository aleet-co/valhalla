#include "argparse_utils.h"
#include "baldr/graphreader.h"
#include "midgard/logging.h"
#include "thor/cch/cch_graph.h"
#include "thor/cch/order.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <set>
#include <sstream>
#include <string>

using namespace valhalla;

// Offline builder for the bans-only CCH matrix artifact. Reads Valhalla tiles,
// builds the truck subgraph, computes a metric-independent (greedy) contraction
// order, and writes the resulting CCH artifact to disk.

namespace {

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

} // namespace

int main(int argc, char* argv[]) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  boost::property_tree::ptree config;
  std::string output_path, levels_str;
  uint32_t max_class = 7;

  try {
    cxxopts::Options options(
        program, program + " " + VALHALLA_PRINT_VERSION + "\n\n"
        "Builds a metric-independent CCH order for the truck subgraph\n"
        "from Valhalla tiles and writes the artifact to disk.\n\n");
    // clang-format off
    options.add_options()
      ("h,help", "Print this help message.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline json config.", cxxopts::value<std::string>())
      ("o,output", "Path to write the CCH artifact.", cxxopts::value<std::string>(output_path)->default_value("/custom_files/cch_truck.bin"))
      ("levels", "Comma-separated hierarchy levels to include (0=highway,1=arterial,2=local).", cxxopts::value<std::string>(levels_str)->default_value("0,1,2"))
      ("max-class", "Max RoadClass to include (0=motorway .. 7=service).", cxxopts::value<uint32_t>(max_class)->default_value("7"));
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config)) {
      return EXIT_SUCCESS;
    }
  } catch (std::exception& e) {
    std::cerr << "Failed to parse options: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  const std::set<uint32_t> levels = parse_levels(levels_str);
  const auto max_roadclass = static_cast<uint8_t>(max_class);

  baldr::GraphReader reader(config.get_child("mjolnir"));

  LOG_INFO("Building truck subgraph...");
  auto graph = thor::cch::BuildTruckGraph(reader, levels, max_roadclass);
  LOG_INFO("  nodes=" + std::to_string(graph.nodes.size()) +
           " edges=" + std::to_string(graph.edges.size()));

  LOG_INFO("Contracting (metric-independent, greedy)...");
  auto order = thor::cch::BuildOrder(graph);
  LOG_INFO("  shortcuts=" + std::to_string(order.shortcuts.size()));

  order.save(output_path);
  LOG_INFO("Wrote " + output_path);
  return EXIT_SUCCESS;
}
