#ifndef VALHALLA_THOR_CCH_CCH_MATRIX_H_
#define VALHALLA_THOR_CCH_CCH_MATRIX_H_

#include <string>

#include <boost/property_tree/ptree.hpp>

#include "thor/cch/cch_graph.h"
#include "thor/cch/customizer.h"
#include "thor/cch/order.h"
#include "thor/matrixalgorithm.h"

namespace valhalla {
namespace thor {

class CCHMatrix : public MatrixAlgorithm {
public:
  explicit CCHMatrix(const boost::property_tree::ptree& config);

  bool SourceToTarget(Api& request,
                      baldr::GraphReader& graphreader,
                      const sif::mode_costing_t& mode_costing,
                      const sif::travel_mode_t mode,
                      const float max_matrix_distance) override;

  void Clear() override;

  const std::string& name() override {
    return name_;
  }

  // True if the CCH artifact exists and could be customized (used by the worker
  // to decide fallback before dispatch).
  bool available() const {
    return ready_;
  }

private:
  bool ensure_customized(baldr::GraphReader& reader);

  std::string artifact_path_;
  uint32_t hops_;
  bool ready_ = false;
  cch::CchGraph graph_;
  cch::CchOrder order_;
  cch::CustomizedMetric metric_;
  std::string name_ = "cch";
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_CCH_CCH_MATRIX_H_
