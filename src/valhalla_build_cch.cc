#include "argparse_utils.h"
#include "baldr/graphreader.h"
#include "midgard/logging.h"
#include "thor/cch/cch_graph.h"
#include "thor/cch/order.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <thread>

using namespace valhalla;

// Offline builder for the bans-only CCH matrix artifact. Reads Valhalla tiles,
// builds the truck subgraph, computes a METIS nested-dissection order, contracts
// with compact adjacency, and writes the CCH artifact.

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

uint32_t resolve_concurrency(uint32_t concurrency) {
  if (concurrency == 0) {
    const unsigned hc = std::thread::hardware_concurrency();
    return hc == 0 ? 1u : hc;
  }
  return concurrency;
}

thor::cch::OrderMethod parse_order_method(const std::string& s) {
  if (s == "nested" || s == "nd" || s == "nested-dissection")
    return thor::cch::OrderMethod::NestedDissection;
  if (s == "independent-set" || s == "is")
    return thor::cch::OrderMethod::IndependentSet;
  throw std::runtime_error("unknown --order '" + s +
                           "' (expected nested|independent-set)");
}

} // namespace

int main(int argc, char* argv[]) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  boost::property_tree::ptree config;
  std::string output_path, levels_str, order_str;
  uint32_t max_class = 7;
  uint32_t concurrency = 0;

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
      ("max-class", "Max RoadClass to include (0=motorway .. 7=service).", cxxopts::value<uint32_t>(max_class)->default_value("7"))
      ("order", "Contraction order: nested (METIS_NodeND, default) or independent-set.", cxxopts::value<std::string>(order_str)->default_value("nested"))
      ("j,concurrency", "Worker threads for subgraph load (0=hardware_concurrency). METIS uses its own parallelism.", cxxopts::value<uint32_t>(concurrency)->default_value("0"));
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
  concurrency = resolve_concurrency(concurrency);
  thor::cch::OrderMethod order_method;
  try {
    order_method = parse_order_method(order_str);
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  using clock = std::chrono::steady_clock;
  const auto t_all = clock::now();
  auto secs = [](clock::time_point t0) {
    return std::chrono::duration<double>(clock::now() - t0).count();
  };

  LOG_INFO("valhalla_build_cch: levels=" + levels_str +
           " max_class=" + std::to_string(max_class) +
           " order=" + order_str +
           " concurrency=" + std::to_string(concurrency) +
           " output=" + output_path);

  LOG_INFO("valhalla_build_cch: phase 1/3 Building truck subgraph...");
  const auto t1 = clock::now();
  auto graph = thor::cch::BuildTruckGraph(config.get_child("mjolnir"), levels, max_roadclass,
                                          concurrency);
  LOG_INFO("valhalla_build_cch: phase 1/3 done  nodes=" + std::to_string(graph.nodes.size()) +
           " edges=" + std::to_string(graph.edges.size()) +
           " phase_s=" + std::to_string(secs(t1)));

  LOG_INFO("valhalla_build_cch: phase 2/3 Ordering + contracting...");
  const auto t2 = clock::now();
  auto order = thor::cch::BuildOrder(graph, concurrency, order_method);
  LOG_INFO("valhalla_build_cch: phase 2/3 done  shortcuts=" +
           std::to_string(order.shortcuts.size()) + " phase_s=" + std::to_string(secs(t2)));

  LOG_INFO("valhalla_build_cch: phase 3/3 Writing artifact...");
  const auto t3 = clock::now();
  order.save(output_path);
  LOG_INFO("valhalla_build_cch: phase 3/3 done  wrote " + output_path +
           " phase_s=" + std::to_string(secs(t3)) +
           " total_s=" + std::to_string(secs(t_all)));
  return EXIT_SUCCESS;
}
