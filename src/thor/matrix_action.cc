#include "thor/worker.h"
#include "tyr/serializers.h"

#include <valhalla/worker.h>

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {
const std::string get_unfound_indices(const google::protobuf::RepeatedField<bool>& result) {
  std::string indices;
  for (int i = 0; i != result.size(); ++i) {
    if (result[i]) {
      indices += std::to_string(i) + ",";
    }
  }
  indices.pop_back();

  return indices;
}

constexpr uint32_t kCostMatrixThreshold = 5;
} // namespace

namespace valhalla {
namespace thor {

MatrixAlgorithm*
thor_worker_t::get_matrix_algorithm(Api& request, const bool has_time, const std::string& costing) {
  if (costing == "bikeshare") {
    return &time_distance_bss_matrix_;
  }

  // Explicit per-request CCH selection (truck-ban, source-side date_time only).
  if (request.options().matrix_algorithm() == Options::matrix_cch) {
    const bool truck = (costing == "truck" || costing == "truck_ban");
    const bool has_depart_time =
        has_time && request.options().date_time_type() != Options::arrive_by;
    if (truck && has_depart_time && cch_matrix_.prepare(*reader)) {
      return &cch_matrix_;
    }
    add_warning(request, 304);
    return &time_distance_matrix_;
  }
  if (request.options().matrix_algorithm() == Options::matrix_timedistancematrix) {
    return &time_distance_matrix_;
  }

  Matrix::Algorithm config_algo = Matrix::CostMatrix;
  switch (source_to_target_algorithm) {
    case SELECT_OPTIMAL:
      // TODO - Do further performance testing to pick the best algorithm for the job
      switch (mode) {
        case travel_mode_t::kPedestrian:
        case travel_mode_t::kBicycle:
          // Use CostMatrix if number of sources and number of targets
          // exceeds some threshold
          if (static_cast<uint32_t>(request.options().sources().size()) <= kCostMatrixThreshold ||
              static_cast<uint32_t>(request.options().targets().size()) <= kCostMatrixThreshold) {
            config_algo = Matrix::TimeDistanceMatrix;
          }
          break;
        case travel_mode_t::kPublicTransit:
          config_algo = Matrix::TimeDistanceMatrix;
          break;
        default:
          break;
      }
      break;
    case COST_MATRIX:
      break;
    case TIME_DISTANCE_MATRIX:
      config_algo = Matrix::TimeDistanceMatrix;
      break;
  }

  // similar to routing: prefer the exact unidirectional algo if not requested otherwise
  // don't use matrix_type, we only need it to set the right warnings for what will be used
  if (has_time && !request.options().prioritize_bidirectional() &&
      source_to_target_algorithm != COST_MATRIX) {
    return &time_distance_matrix_;
  } else if (has_time && request.options().prioritize_bidirectional() &&
             source_to_target_algorithm != TIME_DISTANCE_MATRIX) {
    return &costmatrix_;
  } else if (config_algo == Matrix::CostMatrix) {
    if (has_time && !request.options().prioritize_bidirectional()) {
      add_warning(request, 301);
    }
    return &costmatrix_;
  } else {
    // if this happens, the server config only allows for timedist matrix
    if (has_time && request.options().prioritize_bidirectional()) {
      add_warning(request, 300);
    }
    return &time_distance_matrix_;
  }
}

std::string thor_worker_t::matrix(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  auto& options = *request.mutable_options();
  adjust_locations(request);
  auto costing = parse_costing(request);

  bool has_time =
      check_matrix_time(request, options.prioritize_bidirectional() ? Matrix::CostMatrix
                                                                    : Matrix::TimeDistanceMatrix);

  // allow all algos to be cancelled
  for (auto* alg : std::vector<MatrixAlgorithm*>{
           &costmatrix_,
           &time_distance_matrix_,
           &time_distance_bss_matrix_,
           &cch_matrix_,
       }) {
    alg->set_interrupt(interrupt);
    alg->set_has_time(has_time);
  }

  auto* algo = get_matrix_algorithm(request, has_time, costing);
  if (check_hierarchy_limits(mode_costing[int(mode)]->GetHierarchyLimits(), mode_costing[int(mode)],
                             options.costings().find(options.costing_type())->second.options(),
                             hierarchy_limits_config_costmatrix, allow_hierarchy_limits_modifications,
                             mode_costing[int(mode)]->UseHierarchyLimits())) {
    // maybe warn if we needed to change user provided hierarchy limits
    add_warning(request, allow_hierarchy_limits_modifications ? 210 : 209);
  }
  LOG_INFO("matrix::" + std::string(algo->name()));

  auto max_dist_it = max_matrix_distance.find(costing);
  if (max_dist_it == max_matrix_distance.end()) {
    throw valhalla_exception_t{154, "missing service_limits.max_matrix_distance for " + costing};
  }
  const float matrix_max_distance = max_dist_it->second;

  // TODO(nils): TDMatrix doesn't care about either destonly or no_thru
  if (algo->name() != "costmatrix") {
    algo->SourceToTarget(request, *reader, mode_costing, mode, matrix_max_distance);
    return tyr::serializeMatrix(request);
  }

  // no matrix_locations for CostMatrix
  if (options.matrix_locations() != std::numeric_limits<uint32_t>::max()) {
    add_warning(request, 211);
  }

  // for costmatrix try a second pass if the first didn't work out
  valhalla::sif::cost_ptr_t cost = mode_costing[static_cast<uint32_t>(mode)];
  cost->set_allow_destination_only(false);
  cost->set_pass(0);

  if (!algo->SourceToTarget(request, *reader, mode_costing, mode, matrix_max_distance) &&
      cost->AllowMultiPass() && costmatrix_allow_second_pass) {
    // NOTE: we only look for unfound connections in a second pass; but
    // if A -> B wasn't found and B -> A was, we still expand both for bidirectional efficiency
    // TODO(nils): probably add filtered edges here too?
    algo->Clear();
    cost->set_pass(1);
    cost->RelaxHierarchyLimits(true);
    cost->set_allow_destination_only(true);
    cost->set_allow_conditional_destination(true);
    algo->set_not_thru_pruning(false);
    algo->SourceToTarget(request, *reader, mode_costing, mode, matrix_max_distance);

    // add a warning that we needed to open destonly etc
    add_warning(request, 400, get_unfound_indices(request.matrix().second_pass()));
  };

  return tyr::serializeMatrix(request);
}
} // namespace thor
} // namespace valhalla
