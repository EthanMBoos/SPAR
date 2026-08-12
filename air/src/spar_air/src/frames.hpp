#pragma once

// PX4 exposes an earth-fixed local NED estimate plus the WGS84 reference of
// that local origin. SPAR's map is the generated world's ENU tangent plane.
// This file is the single pure-math contract used by feedback, TF, and
// offboard commands; physical spawn coordinates never enter the transform.

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

#include <spar_geodesy/local_tangent.hpp>

namespace spar_air {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

inline bool finite(const Vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

inline Vec3 nedToEnu(double north, double east, double down) {
  return {east, north, -down};
}

inline Vec3 enuToNed(double east, double north, double up) {
  return {north, east, -up};
}

inline double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

// ENU yaw is counter-clockwise from east; NED yaw is clockwise from north.
inline double yawEnuToNed(double yaw_enu) {
  return normalizeAngle(0.5 * spar_geodesy::kPi - yaw_enu);
}

inline std::array<double, 4> multiplyQuaternion(
    const std::array<double, 4>& p, const std::array<double, 4>& q) {
  return {
      p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
      p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
      p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
      p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0],
  };
}

// PX4 q is FRD body -> NED world. ROS needs FLU body -> ENU world.
inline std::array<double, 4> frdNedToFluEnu(
    const std::array<double, 4>& q_frd_to_ned) {
  const double s = std::sqrt(0.5);
  const std::array<double, 4> q_enu_from_ned{0.0, s, s, 0.0};
  const std::array<double, 4> q_frd_from_flu{0.0, 1.0, 0.0, 0.0};
  return multiplyQuaternion(
      multiplyQuaternion(q_enu_from_ned, q_frd_to_ned), q_frd_from_flu);
}

struct ReferenceUpdate {
  bool accepted = false;
  bool reference_changed = false;
  bool xy_reset = false;
  bool z_reset = false;
};

class Px4MapTransform {
public:
  explicit Px4MapTransform(const spar_geodesy::Geodetic& world_datum)
      : world_(world_datum) {}

  ReferenceUpdate updateReference(
      bool xy_valid, bool z_valid, bool xy_global, bool z_global,
      std::uint64_t ref_timestamp, double ref_latitude_deg,
      double ref_longitude_deg, double ref_altitude_m,
      std::uint8_t xy_reset_counter, std::uint8_t z_reset_counter) {
    position_valid_ = xy_valid && z_valid && xy_global && z_global;
    ReferenceUpdate result;
    if (!xy_global || !z_global || !std::isfinite(ref_latitude_deg) ||
        !std::isfinite(ref_longitude_deg) ||
        !std::isfinite(ref_altitude_m)) {
      return result;
    }

    const auto origin = world_.geodeticToEnu(
        {ref_latitude_deg, ref_longitude_deg, ref_altitude_m});
    if (!std::isfinite(origin.east) || !std::isfinite(origin.north) ||
        !std::isfinite(origin.up)) {
      return result;
    }

    result.accepted = true;
    result.reference_changed = !origin_ || ref_timestamp != ref_timestamp_;
    result.xy_reset = have_counters_ && xy_reset_counter != xy_reset_counter_;
    result.z_reset = have_counters_ && z_reset_counter != z_reset_counter_;
    origin_ = Vec3{origin.east, origin.north, origin.up};
    ref_timestamp_ = ref_timestamp;
    xy_reset_counter_ = xy_reset_counter;
    z_reset_counter_ = z_reset_counter;
    have_counters_ = true;
    return result;
  }

  bool ready() const { return origin_.has_value() && position_valid_; }
  bool hasReference() const { return origin_.has_value(); }

  std::optional<Vec3> localNedToMap(const Vec3& local_ned) const {
    if (!ready() || !finite(local_ned)) return std::nullopt;
    const auto local_enu = nedToEnu(local_ned.x, local_ned.y, local_ned.z);
    return Vec3{
        origin_->x + local_enu.x,
        origin_->y + local_enu.y,
        origin_->z + local_enu.z,
    };
  }

  std::optional<Vec3> mapToLocalNed(const Vec3& map) const {
    if (!ready() || !finite(map)) return std::nullopt;
    return enuToNed(
        map.x - origin_->x, map.y - origin_->y, map.z - origin_->z);
  }

private:
  spar_geodesy::LocalTangent world_;
  std::optional<Vec3> origin_;
  std::uint64_t ref_timestamp_ = 0;
  std::uint8_t xy_reset_counter_ = 0;
  std::uint8_t z_reset_counter_ = 0;
  bool have_counters_ = false;
  bool position_valid_ = false;
};

}  // namespace spar_air
