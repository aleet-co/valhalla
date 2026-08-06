#ifndef VALHALLA_THOR_CCH_CCH_GRAPH_H_
#define VALHALLA_THOR_CCH_CCH_GRAPH_H_

#include <cstdint>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/property_tree/ptree_fwd.hpp>
#include <valhalla/baldr/graphreader.h>

namespace valhalla {
namespace thor {
namespace cch {

struct CchNode {
  uint64_t graph_id = 0;
  double lat = 0;
  double lon = 0;
  std::string country;
  uint32_t tz_index = 0;
};

struct CchBaseEdge {
  uint32_t u = 0;
  uint32_t v = 0;
  uint32_t time_s = 0;
  uint8_t roadclass = 0;
};

// Truck dimensions matching sif::TruckCost / Valhalla API (metric tons, meters).
// Used to apply the same access-restriction checks CostMatrix uses.
struct TruckVehicleParams {
  float weight_t = 21.77f;
  float axle_load_t = 9.07f;
  float height_m = 4.11f;
  float width_m = 2.6f;
  float length_m = 21.64f;
  uint32_t axle_count = 5;
  bool hazmat = false;
};

struct TruckGraphOptions {
  bool hgv_only = false;
  // Drop destonly_hgv edges (CostMatrix first pass cannot enter them from through traffic).
  bool exclude_destonly_hgv = false;
  // Apply dimensional access restrictions (maxweight/height/…) like TruckCost.
  bool apply_access_restrictions = false;
  TruckVehicleParams vehicle;
};

class CchGraph {
public:
  std::vector<CchNode> nodes;
  std::vector<CchBaseEdge> edges;
  std::vector<uint32_t> out_offsets;
  std::vector<uint32_t> out_edges;
  std::vector<uint32_t> in_offsets;
  std::vector<uint32_t> in_edges;
  uint64_t tile_build_hash = 0;

  int index_of(uint64_t graph_id) const {
    auto it = id_to_index_.find(graph_id);
    return it == id_to_index_.end() ? -1 : static_cast<int>(it->second);
  }
  void set_index(uint64_t graph_id, uint32_t idx) {
    id_to_index_[graph_id] = idx;
  }
  void build_csr();

private:
  std::unordered_map<uint64_t, uint32_t> id_to_index_;
};

// Parallel build: one GraphReader per worker thread. concurrency == 0 → hardware_concurrency.
CchGraph BuildTruckGraph(const boost::property_tree::ptree& mjolnir_config,
                         const std::set<uint32_t>& levels,
                         uint8_t max_roadclass,
                         uint32_t concurrency,
                         const TruckGraphOptions& options);

// Convenience: hgv_only only (CCH / legacy callers).
CchGraph BuildTruckGraph(const boost::property_tree::ptree& mjolnir_config,
                         const std::set<uint32_t>& levels = {0, 1, 2},
                         uint8_t max_roadclass = 7,
                         uint32_t concurrency = 0,
                         bool hgv_only = false);

// Single-reader path (runtime customize / tests). Always single-threaded.
CchGraph BuildTruckGraph(baldr::GraphReader& reader,
                         const std::set<uint32_t>& levels,
                         uint8_t max_roadclass,
                         const TruckGraphOptions& options);

CchGraph BuildTruckGraph(baldr::GraphReader& reader,
                         const std::set<uint32_t>& levels = {0, 1, 2},
                         uint8_t max_roadclass = 7,
                         bool hgv_only = false);

} // namespace cch
} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_CCH_CCH_GRAPH_H_
