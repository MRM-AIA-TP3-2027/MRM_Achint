#pragma once
#include <cmath>

// Converts degrees to radians
static constexpr double DEG2RAD = M_PI / 180.0;
// Earth's mean radius in metres
static constexpr double EARTH_RADIUS_M = 6371000.0;

// Represents one GPS fix as a latitude/longitude pair.
// All methods are const — they read data, never modify it.
class GPSCoordinate
{
public:
  double latitude;   // degrees, -90 to +90
  double longitude;  // degrees, -180 to +180

  GPSCoordinate(double lat = 0.0, double lon = 0.0)
    : latitude(lat), longitude(lon) {}

  // Haversine formula — great-circle distance in metres between
  // this coordinate and another one.
  double distanceTo(const GPSCoordinate & other) const
  {
    double dlat = (other.latitude  - latitude)  * DEG2RAD;
    double dlon = (other.longitude - longitude) * DEG2RAD;

    double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0)
             + std::cos(latitude * DEG2RAD) * std::cos(other.latitude * DEG2RAD)
             * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);

    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return EARTH_RADIUS_M * c;
  }

  // Forward azimuth from this point to other, in degrees [0, 360).
  // 0 = North, 90 = East, 180 = South, 270 = West.
  double bearingTo(const GPSCoordinate & other) const
  {
    double lat1 = latitude  * DEG2RAD;
    double lat2 = other.latitude  * DEG2RAD;
    double dlon = (other.longitude - longitude) * DEG2RAD;

    double y = std::sin(dlon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2)
             - std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    double bearing = std::atan2(y, x) / DEG2RAD;
    // Normalise to [0, 360)
    return std::fmod(bearing + 360.0, 360.0);
  }
};