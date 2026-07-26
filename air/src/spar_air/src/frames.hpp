#pragma once

// The one place the PX4 frame conversions live (the classic PX4/ROS bug
// farm). PX4 thinks in NED world / FRD body; this repo's map frame is ENU
// (MuJoCo world axes: +x east, +y north, +z up, matching
// sim/spar_sim/px4_link.py, which owns the sim-side half of this lesson).
//
// ENU <-> NED is the axis swap (n, e, d) = (y, x, -z), applied in both
// directions by the same function since it is its own inverse pair.
//
// The EKF's local origin is the launch pad, which is also the drone's spawn,
// so the ROS map origin is the pad in every world.

namespace spar_air {

struct Vec3 {
  double x = 0, y = 0, z = 0;
};

// PX4 local NED -> map ENU.
inline Vec3 nedToEnu(double n, double e, double d) { return {e, n, -d}; }

// map ENU -> PX4 local NED.
inline Vec3 enuToNed(double x, double y, double z) { return {y, x, -z}; }

// Yaw: ENU measures from +x (east) counterclockwise; NED measures from
// north clockwise. Same formula both ways.
inline double yawEnuNed(double yaw) { return 1.5707963267948966 - yaw; }

}  // namespace spar_air
