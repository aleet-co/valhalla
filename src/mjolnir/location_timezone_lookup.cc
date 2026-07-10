#include "mjolnir/location_timezone_lookup.h"

#include "baldr/datetime.h"
#include "mjolnir/admin.h"

#include <mutex>

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace mjolnir {
namespace {

constexpr const char* kTimezoneDbPath = "/custom_files/timezones.sqlite";
constexpr double kLookupBufferDegrees = 0.05;

std::optional<AdminDB> g_timezone_db;
std::once_flag g_timezone_db_once;

AdminDB* timezone_db() {
  std::call_once(g_timezone_db_once, []() { g_timezone_db = AdminDB::open(kTimezoneDbPath); });
  return g_timezone_db ? &(*g_timezone_db) : nullptr;
}

} // namespace

int TimezoneIndexForLatLng(double lat, double lng) {
  AdminDB* db = timezone_db();
  if (db == nullptr) {
    return 0;
  }

  const AABB2<PointLL> box({lng - kLookupBufferDegrees, lat - kLookupBufferDegrees},
                           {lng + kLookupBufferDegrees, lat + kLookupBufferDegrees});
  try {
    const auto polys = GetTimeZones(*db, box);
    const PointLL point(lng, lat);
    for (const auto& entry : polys) {
      if (entry.second.intersects(point)) {
        return static_cast<int>(entry.first);
      }
    }
  } catch (...) {
    return 0;
  }

  return 0;
}

} // namespace mjolnir
} // namespace valhalla
