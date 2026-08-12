#pragma once

#include <cmath>
#include <stdexcept>

namespace spar_geodesy {

inline constexpr double kWgs84SemiMajorM = 6378137.0;
inline constexpr double kWgs84EccentricitySquared = 6.69437999014e-3;
inline constexpr double kPi = 3.14159265358979323846;

struct Enu {
  double east = 0.0;
  double north = 0.0;
  double up = 0.0;
};

struct Geodetic {
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double altitude_m = 0.0;
};

class LocalTangent {
public:
  explicit LocalTangent(Geodetic datum) : datum_(datum) {
    if (!std::isfinite(datum.latitude_deg) ||
        !std::isfinite(datum.longitude_deg) ||
        !std::isfinite(datum.altitude_m) ||
        datum.latitude_deg <= -90.0 || datum.latitude_deg >= 90.0 ||
        datum.longitude_deg < -180.0 || datum.longitude_deg > 180.0) {
      throw std::invalid_argument("invalid WGS84 world datum");
    }

    const double latitude = datum.latitude_deg * kPi / 180.0;
    const double sin_latitude = std::sin(latitude);
    const double denominator = std::sqrt(
        1.0 - kWgs84EccentricitySquared * sin_latitude * sin_latitude);
    const double prime_vertical = kWgs84SemiMajorM / denominator;
    const double meridional =
        kWgs84SemiMajorM * (1.0 - kWgs84EccentricitySquared) /
        (denominator * denominator * denominator);
    east_metres_per_radian_ =
        (prime_vertical + datum.altitude_m) * std::cos(latitude);
    north_metres_per_radian_ = meridional + datum.altitude_m;
  }

  const Geodetic& datum() const { return datum_; }

  Geodetic enuToGeodetic(const Enu& point) const {
    return {
        datum_.latitude_deg +
            point.north / north_metres_per_radian_ * 180.0 / kPi,
        datum_.longitude_deg +
            point.east / east_metres_per_radian_ * 180.0 / kPi,
        datum_.altitude_m + point.up,
    };
  }

  Enu geodeticToEnu(const Geodetic& point) const {
    return {
        (point.longitude_deg - datum_.longitude_deg) * kPi / 180.0 *
            east_metres_per_radian_,
        (point.latitude_deg - datum_.latitude_deg) * kPi / 180.0 *
            north_metres_per_radian_,
        point.altitude_m - datum_.altitude_m,
    };
  }

private:
  Geodetic datum_;
  double east_metres_per_radian_ = 0.0;
  double north_metres_per_radian_ = 0.0;
};

}  // namespace spar_geodesy
