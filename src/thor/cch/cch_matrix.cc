#include "thor/cch/cch_matrix.h"

#include <cstdint>
#include <cerrno>
#include <fcntl.h>
#include <fstream>
#include <set>
#include <sstream>
#include <sys/file.h>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "baldr/datetime.h"
#include "baldr/graphid.h"
#include "baldr/time_info.h"
#include "midgard/logging.h"
#include "thor/cch/contracted_search.h"
#include "thor/matrixalgorithm.h"
#include "thor/pathalgorithm.h"

namespace valhalla {
namespace thor {
namespace {

std::set<uint32_t> parse_cch_levels(const std::string& s) {
  std::set<uint32_t> levels;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    if (!tok.empty())
      levels.insert(static_cast<uint32_t>(std::stoul(tok)));
  }
  if (levels.empty())
    levels = {0, 1};
  return levels;
}

std::string levels_to_string(const std::set<uint32_t>& levels) {
  std::string s;
  for (uint32_t lvl : levels) {
    if (!s.empty())
      s += ",";
    s += std::to_string(lvl);
  }
  return s;
}

} // namespace

CCHMatrix::CCHMatrix(const boost::property_tree::ptree& config)
    : MatrixAlgorithm(config),
      artifact_path_(config.get<std::string>("cch.artifact", "/custom_files/cch_truck.bin")),
      enabled_(config.get<bool>("cch.enabled", false)) {
  const std::string mode = config.get<std::string>("cch.query_mode", "contracted_pareto");
  if (mode == "contracted") {
    query_mode_ = cch::QueryMode::Contracted;
  } else if (mode == "contracted_pareto") {
    query_mode_ = cch::QueryMode::ContractedPareto;
  } else {
    // Unknown strings (including retired "corridor") → contracted_pareto.
    query_mode_ = cch::QueryMode::ContractedPareto;
    LOG_WARN("cch: unknown query_mode '" + mode + "'; defaulting to contracted_pareto");
  }

  // Subgraph filters must match the offline valhalla_build_cch invocation that
  // produced the artifact (region-grid truck defaults).
  levels_ = parse_cch_levels(config.get<std::string>("cch.levels", "0,1"));
  max_class_ = static_cast<uint8_t>(config.get<uint32_t>("cch.max_class", 6));
  truck_opts_.hgv_only = config.get<bool>("cch.hgv_only", true);
  truck_opts_.exclude_destonly_hgv = config.get<bool>("cch.exclude_destonly_hgv", false);
  truck_opts_.apply_access_restrictions =
      config.get<bool>("cch.apply_access_restrictions", false);
}

void CCHMatrix::Clear() {
}

bool CCHMatrix::ensure_customized(baldr::GraphReader& reader) {
  if (ready_)
    return true;

  // WIP kill-switch: keep CCH code paths but always fall back to TDM until the
  // offline artifact build is production-ready.
  if (!enabled_) {
    // Latch so we do not re-log / re-probe every request.
    customize_attempted_ = true;
    LOG_INFO("cch: disabled (thor.cch.enabled=false); matrix requests fall back to TDM");
    return false;
  }

  // Serialize cold-start across prime_server worker *processes*. Without this,
  // td-fill concurrency wakes many workers at once and each builds a full
  // CustomizedMetric (~several GiB) in parallel → OOM. Steady-state RSS is
  // still ~N_workers × metric size — keep VALHALLA_THREADS small for CCH.
  const char* lock_path = "/tmp/valhalla_cch_customize.lock";
  const int lock_fd = ::open(lock_path, O_CREAT | O_RDWR, 0644);
  if (lock_fd >= 0) {
    LOG_INFO("cch: waiting for customize lock pid=" + std::to_string(::getpid()));
    if (::flock(lock_fd, LOCK_EX) != 0) {
      LOG_WARN("cch: flock failed errno=" + std::to_string(errno) + "; continuing without lock");
    }
  } else {
    LOG_WARN(std::string("cch: could not open ") + lock_path + "; continuing without lock");
  }

  struct LockGuard {
    int fd;
    ~LockGuard() {
      if (fd >= 0) {
        ::flock(fd, LOCK_UN);
        ::close(fd);
      }
    }
  } lock_guard{lock_fd};

  // Another thread in this process may have finished while we waited.
  if (ready_)
    return true;

  // Negative-cache latch: a missing/mismatched artifact must not cause the
  // (expensive) truck-graph build to rerun on every request. Try once per worker.
  if (customize_attempted_)
    return false;
  customize_attempted_ = true;

  std::ifstream probe(artifact_path_, std::ios::binary);
  if (!probe.good()) {
    LOG_WARN("cch: artifact not found at " + artifact_path_);
    return false;
  }
  probe.close();

  try {
    order_ = cch::CchOrder::load(artifact_path_);
    LOG_INFO("cch: building truck subgraph pid=" + std::to_string(::getpid()) +
             " levels=[" + levels_to_string(levels_) +
             "] max_class=" + std::to_string(static_cast<unsigned>(max_class_)) +
             " hgv_only=" + (truck_opts_.hgv_only ? "1" : "0"));
    graph_ = cch::BuildTruckGraph(reader, levels_, max_class_, truck_opts_);
    if (graph_.nodes.size() != order_.rank.size() ||
        graph_.tile_build_hash != order_.tile_build_hash) {
      LOG_WARN("cch: artifact does not match current tiles; disabling cch "
               "(graph_nodes=" +
               std::to_string(graph_.nodes.size()) +
               " order_nodes=" + std::to_string(order_.rank.size()) +
               " graph_hash=" + std::to_string(graph_.tile_build_hash) +
               " order_hash=" + std::to_string(order_.tile_build_hash) +
               "). Rebuild cch_truck.bin with the same thor.cch levels/max_class/hgv_only.");
      return false;
    }
    if (graph_.edges.size() != order_.num_base_edges) {
      LOG_WARN("cch: artifact base_edges=" + std::to_string(order_.num_base_edges) +
               " != graph edges=" + std::to_string(graph_.edges.size()) +
               "; filters likely diverge from valhalla_build_cch — disabling cch");
      return false;
    }
    order_.build_adjacency(graph_);
    metric_ = cch::Customize(graph_, order_);
    ready_ = true;
    LOG_INFO("cch: customized metric ready pid=" + std::to_string(::getpid()) + " (" +
             std::to_string(graph_.nodes.size()) + " nodes)");
  } catch (const std::exception& e) {
    LOG_WARN(std::string("cch: customization failed: ") + e.what());
    ready_ = false;
  }
  return ready_;
}

bool CCHMatrix::SourceToTarget(Api& request,
                               baldr::GraphReader& graphreader,
                               const sif::mode_costing_t& /*mode_costing*/,
                               const sif::travel_mode_t /*mode*/,
                               const float /*max_matrix_distance*/) {
  // Report as TimeDistanceMatrix in the PBF payload (MVP: no new enum value).
  request.mutable_matrix()->set_algorithm(Matrix::TimeDistanceMatrix);

  if (!ensure_customized(graphreader)) {
    return false; // worker will have already chosen fallback; defensive.
  }

  const auto& options = request.options();
  const auto& srcs = options.sources();
  const auto& tgts = options.targets();
  const size_t num_elements = static_cast<size_t>(srcs.size()) * tgts.size();

  auto& matrix = *request.mutable_matrix();
  // Reserve/zero the PBF arrays (mirror TimeDistanceMatrix sizing so the
  // serializer sees fully-sized date_time/tz arrays too).
  reserve_pbf_arrays(matrix, num_elements, options.verbose());

  // Snap each source/target to the nearest truck-subgraph base node (MVP).
  auto snap = [&](const valhalla::Location& loc) -> int {
    if (loc.correlation().edges_size() == 0)
      return -1;
    baldr::GraphId edge_id(loc.correlation().edges(0).graph_id());
    auto tile = graphreader.GetGraphTile(edge_id);
    if (!tile)
      return -1;
    const auto* de = tile->directededge(edge_id);
    return graph_.index_of(de->endnode().value);
  };

  std::vector<int> src_idx(srcs.size()), tgt_idx(tgts.size());
  for (int s = 0; s < srcs.size(); ++s)
    src_idx[s] = snap(srcs[s]);
  for (int t = 0; t < tgts.size(); ++t)
    tgt_idx[t] = snap(tgts[t]);

  // Depart second-of-week per source. The tz is derived from the snapped source
  // node exactly as TimeDistanceMatrix::SetTime does (TimeInfo::make, which also
  // normalizes a "current" date_time). The epoch MUST be a PLAIN Unix timestamp
  // to match the customizer's mask clock: DeriveMask indexes bans on
  // utc_second_of_week(kDefaultReferenceWeek + slot*900), a plain-Unix grid.
  // TimeInfo::local_time (and DateTime::seconds_since_epoch) are leap-inclusive
  // (date::to_utc_time), ~27s off, which can flip a ban near a slot boundary --
  // so we take sys_time (get_ldt(...).get_sys_time()) instead of local_time.
  baldr::DateTime::tz_sys_info_cache_t tz_cache;
  auto& mutable_srcs = *request.mutable_options()->mutable_sources();
  const std::string& default_dt = options.date_time();
  std::vector<int64_t> depart_sow(srcs.size(), 0);
  for (int s = 0; s < mutable_srcs.size(); ++s) {
    auto* src = mutable_srcs.Mutable(s);
    if (src->date_time().empty() && !default_dt.empty())
      src->set_date_time(default_dt);
    auto ti = baldr::TimeInfo::make(*src, graphreader, &tz_cache);
    int64_t epoch = 0;
    if (ti.valid) {
      const auto* tz = baldr::DateTime::get_tz_db().from_index(ti.timezone_index);
      if (tz) {
        epoch = baldr::DateTime::get_ldt(baldr::DateTime::get_formatted_date(src->date_time()), tz)
                    .get_sys_time()
                    .time_since_epoch()
                    .count();
      }
    }
    depart_sow[s] = cch::utc_second_of_week(epoch);
  }

  // Ban-aware search on the CCH overlay (consumes shortcut B).
  // Contracted = Stage-1 single-label; ContractedPareto = Stage-2 Pareto labels.
  // Phase A once: union G↓ + ban-free-safe RPHAST buckets across all targets.
  std::unordered_set<uint32_t> g_down_union;
  cch::RphastBuckets buckets;
  std::vector<uint32_t> snapped_targets;
  snapped_targets.reserve(static_cast<size_t>(tgts.size()));
  for (int t = 0; t < tgts.size(); ++t) {
    if (tgt_idx[t] < 0)
      continue;
    snapped_targets.push_back(static_cast<uint32_t>(tgt_idx[t]));
  }
  cch::BuildRphastPhaseA(order_, metric_, snapped_targets, g_down_union, &buckets, interrupt_);

  // Phase B: per-source contracted TD search into the union G↓ with buckets.
  const bool pareto = query_mode_ == cch::QueryMode::ContractedPareto;
  for (int s = 0; s < srcs.size(); ++s) {
    if (src_idx[s] < 0) {
      for (int t = 0; t < tgts.size(); ++t) {
        const size_t pbf = static_cast<size_t>(s) * tgts.size() + t;
        matrix.mutable_from_indices()->Set(pbf, s);
        matrix.mutable_to_indices()->Set(pbf, t);
        matrix.mutable_times()->Set(pbf, kMaxCost);
        matrix.mutable_distances()->Set(pbf, 0);
      }
      continue;
    }
    std::unordered_map<uint32_t, float> arrival;
    if (pareto) {
      cch::ContractedTdPareto(order_, metric_, static_cast<uint32_t>(src_idx[s]), snapped_targets,
                              depart_sow[s], &g_down_union, &buckets, arrival, nullptr,
                              interrupt_);
    } else {
      cch::ContractedTdEarliest(order_, metric_, static_cast<uint32_t>(src_idx[s]), snapped_targets,
                                depart_sow[s], &g_down_union, &buckets, arrival, interrupt_);
    }
    for (int t = 0; t < tgts.size(); ++t) {
      const size_t pbf = static_cast<size_t>(s) * tgts.size() + t;
      matrix.mutable_from_indices()->Set(pbf, s);
      matrix.mutable_to_indices()->Set(pbf, t);
      float secs = kMaxCost;
      if (tgt_idx[t] >= 0) {
        auto it = arrival.find(static_cast<uint32_t>(tgt_idx[t]));
        if (it != arrival.end())
          secs = it->second;
      }
      matrix.mutable_times()->Set(pbf, secs);
      matrix.mutable_distances()->Set(pbf, 0);
    }
  }
  return true;
}

} // namespace thor
} // namespace valhalla
