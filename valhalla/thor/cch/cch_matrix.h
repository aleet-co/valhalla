#ifndef VALHALLA_THOR_CCH_CCH_MATRIX_H_
#define VALHALLA_THOR_CCH_CCH_MATRIX_H_

#include <cstdint>
#include <memory>
#include <set>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "thor/cch/cch_graph.h"
#include "thor/cch/customizer.h"
#include "thor/cch/order.h"
#include "thor/matrixalgorithm.h"

namespace valhalla {
namespace thor {

namespace cch {

// Selects the CCH matrix query strategy. ContractedPareto is the production
// default; Contracted (single-label) remains selectable via thor.cch.query_mode.
enum class QueryMode { Contracted, ContractedPareto };

// Process-wide customized CCH overlay. Built once; shared read-only by all
// thor worker threads (valhalla_service runs workers as threads in one process).
struct SharedState {
  CchGraph graph;
  CchOrder order;
  CustomizedMetric metric;
};

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
    return static_cast<bool>(shared_);
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
  cch::QueryMode query_mode_ = cch::QueryMode::ContractedPareto;
  bool enabled_ = false;
  // Must match valhalla_build_cch filters used to produce the artifact.
  std::set<uint32_t> levels_{0, 1};
  uint8_t max_class_ = 6;
  cch::TruckGraphOptions truck_opts_;
  // Local handle to the process-wide shared overlay (null until prepare succeeds).
  std::shared_ptr<const cch::SharedState> shared_;
  std::string name_ = "cch";
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_CCH_CCH_MATRIX_H_
