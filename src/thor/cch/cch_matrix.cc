#include "thor/cch/cch_matrix.h"

#include <fstream>

#include "midgard/logging.h"

namespace valhalla {
namespace thor {

CCHMatrix::CCHMatrix(const boost::property_tree::ptree& config)
    : MatrixAlgorithm(config),
      artifact_path_(config.get<std::string>("cch.artifact", "/custom_files/cch_truck.bin")),
      hops_(config.get<uint32_t>("cch.corridor_hops", 16)) {
}

void CCHMatrix::Clear() {
}

bool CCHMatrix::ensure_customized(baldr::GraphReader& reader) {
  if (ready_)
    return true;

  std::ifstream probe(artifact_path_, std::ios::binary);
  if (!probe.good()) {
    LOG_WARN("cch: artifact not found at " + artifact_path_);
    return false;
  }
  probe.close();

  try {
    order_ = cch::CchOrder::load(artifact_path_);
    graph_ = cch::BuildTruckGraph(reader);
    if (graph_.nodes.size() != order_.rank.size() ||
        graph_.tile_build_hash != order_.tile_build_hash) {
      LOG_WARN("cch: artifact does not match current tiles; disabling cch");
      return false;
    }
    order_.build_adjacency(graph_);
    metric_ = cch::Customize(graph_, order_);
    ready_ = true;
    LOG_INFO("cch: customized metric ready (" + std::to_string(graph_.nodes.size()) + " nodes)");
  } catch (const std::exception& e) {
    LOG_WARN(std::string("cch: customization failed: ") + e.what());
    ready_ = false;
  }
  return ready_;
}

// Minimal stub: the full matrix query is implemented in Task 7. For now this
// only drives the lazy customization lifecycle so the class is instantiable and
// the library links.
bool CCHMatrix::SourceToTarget(Api& /*request*/,
                               baldr::GraphReader& graphreader,
                               const sif::mode_costing_t& /*mode_costing*/,
                               const sif::travel_mode_t /*mode*/,
                               const float /*max_matrix_distance*/) {
  if (!ensure_customized(graphreader)) {
    return false;
  }
  return true;
}

} // namespace thor
} // namespace valhalla
