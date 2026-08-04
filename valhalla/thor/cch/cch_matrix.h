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

namespace cch {

// Selects the CCH matrix query strategy. Corridor is the production default;
// Contracted is staged behind thor.cch.query_mode for exact-query work.
enum class QueryMode { Corridor, Contracted };

} // namespace cch

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

  // Eagerly customize (if needed) so availability is known at selection time.
  // Wraps the private ensure_customized so the worker can prepare before dispatch.
  bool prepare(baldr::GraphReader& r) {
    return ensure_customized(r);
  }

  cch::QueryMode query_mode() const {
    return query_mode_;
  }

private:
  bool ensure_customized(baldr::GraphReader& reader);

  std::string artifact_path_;
  uint32_t hops_;
  cch::QueryMode query_mode_ = cch::QueryMode::Corridor;
  bool enabled_ = false;
  bool ready_ = false;
  // Negative-cache latch: once we've tried (and failed) to customize, don't
  // re-probe/re-build the truck graph on every subsequent request.
  bool customize_attempted_ = false;
  cch::CchGraph graph_;
  cch::CchOrder order_;
  cch::CustomizedMetric metric_;
  std::string name_ = "cch";
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_CCH_CCH_MATRIX_H_
