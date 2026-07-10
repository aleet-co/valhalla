#ifndef VALHALLA_MJOLNIR_LOCATION_TIMEZONE_LOOKUP_H_
#define VALHALLA_MJOLNIR_LOCATION_TIMEZONE_LOOKUP_H_

namespace valhalla {
namespace mjolnir {

/**
 * Resolve a Valhalla timezone index for a lat/lng using timezones.sqlite.
 * Returns 0 when the database is missing or the point is not inside any zone.
 */
int TimezoneIndexForLatLng(double lat, double lng);

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_LOCATION_TIMEZONE_LOOKUP_H_
