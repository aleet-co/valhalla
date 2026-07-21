#ifndef VALHALLA_THOR_REGION_GRID_PROGRESS_LOG_H_
#define VALHALLA_THOR_REGION_GRID_PROGRESS_LOG_H_

#include "midgard/logging.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>

namespace valhalla {
namespace thor {
namespace region_grid {

inline double ElapsedSeconds(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

inline std::string FormatDuration(double seconds) {
  if (!(seconds >= 0.0) || !std::isfinite(seconds))
    return "?";
  const long s = static_cast<long>(seconds + 0.5);
  const long h = s / 3600;
  const long m = (s % 3600) / 60;
  const long sec = s % 60;
  if (h > 0)
    return std::to_string(h) + "h" + std::to_string(m) + "m";
  if (m > 0)
    return std::to_string(m) + "m" + std::to_string(sec) + "s";
  return std::to_string(sec) + "s";
}

// Periodic progress with linear ETA. Logs at least every min_interval_s, on
// ~5% milestones, and when done==total. Returns true if a line was emitted.
inline bool MaybeLogProgress(const char* label,
                             size_t done,
                             size_t total,
                             std::chrono::steady_clock::time_point t0,
                             std::chrono::steady_clock::time_point* last_log,
                             double min_interval_s = 30.0,
                             const std::string& extra = {}) {
  const auto now = std::chrono::steady_clock::now();
  const bool finished = total > 0 && done >= total;
  const bool milestone =
      total > 0 && done > 0 && done % std::max<size_t>(1, total / 20) == 0;
  if (!finished && !milestone &&
      std::chrono::duration<double>(now - *last_log).count() < min_interval_s) {
    return false;
  }

  const double elapsed = ElapsedSeconds(t0);
  const int pct = total ? static_cast<int>((100.0 * done) / total) : 100;
  std::string eta = "?";
  if (done > 0 && done < total) {
    eta = FormatDuration(elapsed * static_cast<double>(total - done) /
                         static_cast<double>(done));
  } else if (finished) {
    eta = "0s";
  }

  LOG_INFO(std::string("region_grid: ") + label + "  " + std::to_string(done) + "/" +
           std::to_string(total) + " (" + std::to_string(pct) + "%)  elapsed=" +
           FormatDuration(elapsed) + "  eta=" + eta +
           (extra.empty() ? "" : ("  " + extra)));
  *last_log = now;
  return true;
}

} // namespace region_grid
} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_REGION_GRID_PROGRESS_LOG_H_
