#ifndef VALHALLA_THOR_CCH_ORDER_H_
#define VALHALLA_THOR_CCH_ORDER_H_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "thor/cch/cch_graph.h"

namespace valhalla {
namespace thor {
namespace cch {

struct CchShortcut {
  uint32_t u = 0;
  uint32_t w = 0;
  uint32_t middle = 0;
  uint32_t left = 0;  // child edge id (base or shortcut)
  uint32_t right = 0; // child edge id
};

class CchOrder {
public:
  uint32_t num_base_edges = 0;
  std::vector<uint32_t> rank; // by node index
  std::vector<CchShortcut> shortcuts;
  uint64_t tile_build_hash = 0;
  std::vector<std::vector<std::pair<uint32_t, uint32_t>>> fwd_adj;
  std::vector<std::vector<std::pair<uint32_t, uint32_t>>> bwd_adj;

  void save(const std::string& path) const;
  static CchOrder load(const std::string& path);
  void build_adjacency(const CchGraph& g);
};

CchOrder BuildOrder(const CchGraph& g);

} // namespace cch
} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_CCH_ORDER_H_
