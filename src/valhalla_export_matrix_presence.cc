#include "argparse_utils.h"
#include "baldr/graphreader.h"
#include "loki/worker.h"
#include "midgard/logging.h"
#include "sif/costfactory.h"
#include "thor/costmatrix.h"
#include "thor/worker.h"
#include "worker.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::midgard;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

struct Loc {
  double lat = 0;
  double lon = 0;
};

std::vector<Loc> load_locations_csv(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot open locations csv: " + path.string());
  }
  std::vector<Loc> out;
  std::string line;
  // header
  if (!std::getline(in, line)) {
    return out;
  }
  // Accept either lat,lon or region_id,country,...,rep_lat,rep_lon,...
  const bool regions_fmt = line.find("rep_lat") != std::string::npos;
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream ss(line);
    std::string tok;
    std::vector<std::string> cols;
    while (std::getline(ss, tok, ',')) {
      cols.push_back(tok);
    }
    Loc loc;
    if (regions_fmt) {
      // region_id,country,h3,rep_graph_id,rep_lat,rep_lon,node_count
      if (cols.size() < 6) {
        continue;
      }
      loc.lat = std::stod(cols[4]);
      loc.lon = std::stod(cols[5]);
    } else {
      if (cols.size() < 2) {
        continue;
      }
      loc.lat = std::stod(cols[0]);
      loc.lon = std::stod(cols[1]);
    }
    out.push_back(loc);
  }
  return out;
}

std::string build_matrix_json(const std::vector<Loc>& sources,
                              const std::vector<Loc>& targets,
                              float weight_t) {
  std::ostringstream oss;
  oss << std::setprecision(8);
  oss << "{\"sources\":[";
  for (size_t i = 0; i < sources.size(); ++i) {
    if (i) {
      oss << ',';
    }
    oss << "{\"lat\":" << sources[i].lat << ",\"lon\":" << sources[i].lon << "}";
  }
  oss << "],\"targets\":[";
  for (size_t i = 0; i < targets.size(); ++i) {
    if (i) {
      oss << ',';
    }
    oss << "{\"lat\":" << targets[i].lat << ",\"lon\":" << targets[i].lon << "}";
  }
  oss << "],\"costing\":\"truck\",\"costing_options\":{\"truck\":{\"weight\":" << weight_t
      << "}},\"matrix_algorithm\":\"costmatrix\"}";
  return oss.str();
}

void write_jsonl_escape(std::ostream& out, const std::string& s) {
  for (char c : s) {
    if (c == '"' || c == '\\') {
      out << '\\';
    }
    out << c;
  }
}

} // namespace

int main(int argc, char* argv[]) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  boost::property_tree::ptree config;
  std::string locations_path, out_path;
  uint32_t source_begin = 0, source_count = 0, target_begin = 0, target_count = 0;
  float weight_t = 40.f; // metric tons (Valhalla truck weight unit)
  float max_distance = 5000000.f;

  try {
    cxxopts::Options options(
        program, program + " " + VALHALLA_PRINT_VERSION +
                     "\n\nExports CostMatrix times/distances plus per-country "
                     "tin/tout presence for a location block.\n"
                     "Writes JSONL: {i,j,time_s,dist_m,segments:[{country,tin_s,tout_s},...]}\n"
                     "Indices i/j are absolute indices into the locations CSV.\n");
    // clang-format off
    options.add_options()
      ("h,help", "Print this help message.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline json config.", cxxopts::value<std::string>())
      ("locations", "CSV of locations (lat,lon or regions.csv).",
        cxxopts::value<std::string>(locations_path))
      ("out", "Output JSONL path.", cxxopts::value<std::string>(out_path))
      ("source-begin", "First source index (inclusive).",
        cxxopts::value<uint32_t>(source_begin)->default_value("0"))
      ("source-count", "Number of sources (0 = all from begin).",
        cxxopts::value<uint32_t>(source_count)->default_value("0"))
      ("target-begin", "First target index (inclusive).",
        cxxopts::value<uint32_t>(target_begin)->default_value("0"))
      ("target-count", "Number of targets (0 = all from begin).",
        cxxopts::value<uint32_t>(target_count)->default_value("0"))
      ("weight", "Truck weight in metric tons.",
        cxxopts::value<float>(weight_t)->default_value("40"))
      ("max-distance", "Max matrix distance meters.",
        cxxopts::value<float>(max_distance)->default_value("5000000"));
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config, "mjolnir.logging")) {
      return EXIT_FAILURE;
    }
    if (locations_path.empty() || out_path.empty()) {
      std::cerr << "required: --locations and --out\n";
      return EXIT_FAILURE;
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return EXIT_FAILURE;
  }

  auto all = load_locations_csv(locations_path);
  if (all.empty()) {
    std::cerr << "no locations loaded\n";
    return EXIT_FAILURE;
  }
  if (source_count == 0) {
    source_count = static_cast<uint32_t>(all.size() - source_begin);
  }
  if (target_count == 0) {
    target_count = static_cast<uint32_t>(all.size() - target_begin);
  }
  if (source_begin + source_count > all.size() || target_begin + target_count > all.size()) {
    std::cerr << "source/target range out of bounds (n=" << all.size() << ")\n";
    return EXIT_FAILURE;
  }
  if (source_count * target_count > 2500) {
    std::cerr << "block has " << (source_count * target_count)
              << " pairs; keep ≤ 2500 (service limit)\n";
    return EXIT_FAILURE;
  }

  LOG_INFO("valhalla_export_matrix_presence: n=" + std::to_string(all.size()) + " src=[" +
           std::to_string(source_begin) + "," + std::to_string(source_begin + source_count) +
           ") tgt=[" + std::to_string(target_begin) + "," +
           std::to_string(target_begin + target_count) + ") pairs=" +
           std::to_string(source_count * target_count) + " weight_t=" + std::to_string(weight_t) +
           " max_distance=" + std::to_string(max_distance) + " locations=" + locations_path +
           " out=" + out_path);

  std::vector<Loc> sources(all.begin() + source_begin, all.begin() + source_begin + source_count);
  std::vector<Loc> targets(all.begin() + target_begin, all.begin() + target_begin + target_count);

  loki_worker_t loki_worker(config);
  CostMatrix matrix(config.get_child("mjolnir"));
  GraphReader reader(config.get_child("mjolnir"));

  const std::string req_json = build_matrix_json(sources, targets, weight_t);
  Api request;
  ParseApi(req_json, Options::sources_to_targets, request);
  LOG_INFO("valhalla_export_matrix_presence: loki matrix locate…");
  loki_worker.matrix(request);
  thor_worker_t::adjust_locations(request);

  CostFactory factory;
  TravelMode mode;
  auto mode_costing = factory.CreateModeCosting(request.options(), mode);
  LOG_INFO("valhalla_export_matrix_presence: CostMatrix SourceToTarget…");
  matrix.SourceToTarget(request, reader, mode_costing, mode, max_distance);
  LOG_INFO("valhalla_export_matrix_presence: matrix done; writing presence JSONL…");

  std::ofstream out(out_path);
  if (!out) {
    std::cerr << "cannot write " << out_path << "\n";
    return EXIT_FAILURE;
  }

  uint32_t reachable = 0;
  uint32_t empty_presence = 0;
  uint32_t done = 0;
  const uint32_t total = source_count * target_count;
  const uint32_t log_every = std::max<uint32_t>(1, total / 10);
  for (uint32_t si = 0; si < source_count; ++si) {
    for (uint32_t ti = 0; ti < target_count; ++ti) {
      const uint32_t abs_i = source_begin + si;
      const uint32_t abs_j = target_begin + ti;
      const float time_s = matrix.BestTimeSeconds(si, ti);
      const uint32_t dist_m = matrix.BestDistanceMeters(si, ti);
      out << "{\"i\":" << abs_i << ",\"j\":" << abs_j;
      if (time_s >= kMaxCost) {
        out << ",\"time_s\":null,\"dist_m\":null,\"segments\":[]}\n";
        ++done;
        if (done % log_every == 0 || done == total) {
          LOG_INFO("valhalla_export_matrix_presence: progress " + std::to_string(done) + "/" +
                   std::to_string(total) + " reachable=" + std::to_string(reachable) +
                   " empty_presence=" + std::to_string(empty_presence));
        }
        continue;
      }
      ++reachable;
      out << ",\"time_s\":" << time_s << ",\"dist_m\":" << dist_m << ",\"segments\":[";
      auto presence = matrix.FormCountryPresence(reader, request, si, ti);
      if (presence.empty()) {
        ++empty_presence;
      }
      for (size_t k = 0; k < presence.size(); ++k) {
        if (k) {
          out << ',';
        }
        out << "{\"country\":\"";
        write_jsonl_escape(out, presence[k].iso);
        out << "\",\"tin_s\":" << presence[k].tin_s << ",\"tout_s\":" << presence[k].tout_s << "}";
      }
      out << "]}\n";
      ++done;
      if (done % log_every == 0 || done == total) {
        LOG_INFO("valhalla_export_matrix_presence: progress " + std::to_string(done) + "/" +
                 std::to_string(total) + " reachable=" + std::to_string(reachable) +
                 " empty_presence=" + std::to_string(empty_presence));
      }
    }
  }

  LOG_INFO("valhalla_export_matrix_presence: wrote " + std::to_string(total) + " rows (" +
           std::to_string(reachable) + " reachable, " + std::to_string(empty_presence) +
           " reachable-with-empty-presence) → " + out_path);
  return EXIT_SUCCESS;
}
