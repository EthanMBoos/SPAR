#pragma once
#include <cstdint>

// Target position in SPAR's canonical local metric frame (ENU, meters).
struct Point3 {
    float x_m = 0.0f;
    float y_m = 0.0f;
    float z_m = 0.0f;
};

// A mission waypoint given in global coordinates. Converted to a local Point3
// via GeoDatum at ingestion (lla_to_local) — SPAR never plans in lat/lon.
struct GeoWaypoint {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    float  alt_m   = 0.0f;
};

struct GoalContext {
    uint64_t mission_id   = 0;
    uint64_t timestamp_us = 0;
    Point3   target;
};
