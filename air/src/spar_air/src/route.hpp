#pragma once

#include <cstddef>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace spar_air {

struct AirWaypoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
};

inline std::vector<AirWaypoint> parsePatrolWaypoints(
    const std::vector<double>& values) {
  if (values.size() < 12 || values.size() % 4 != 0) {
    throw std::runtime_error(
        "patrol_waypoints must contain at least three x, y, z, yaw tuples");
  }

  std::vector<AirWaypoint> waypoints;
  waypoints.reserve(values.size() / 4);
  for (size_t i = 0; i < values.size(); i += 4) {
    if (!std::isfinite(values[i]) || !std::isfinite(values[i + 1]) ||
        !std::isfinite(values[i + 2]) || !std::isfinite(values[i + 3])) {
      throw std::runtime_error("patrol_waypoints values must be finite");
    }
    if (values[i + 2] <= 0.0) {
      throw std::runtime_error("patrol waypoint altitudes must be positive");
    }
    waypoints.push_back(
        {values[i], values[i + 1], values[i + 2], values[i + 3]});
  }
  return waypoints;
}

}  // namespace spar_air
