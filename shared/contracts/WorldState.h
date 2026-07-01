#pragma once
#include <cstdint>
#include <vector>

// Vehicle pose in SPAR's canonical local metric frame (ENU, REP-103):
//   x_m  — east   (meters, map frame)
//   y_m  — north  (meters, map frame)
//   z_m  — up     (meters)
//   yaw_rad — heading, CCW from +x (east)
//   speed_ms — body-forward speed
//
// Filled from an external state estimator (e.g. robot_localization) via the
// controller adapter, or from the KinematicBackend in simulation. Georeferencing
// (lat/lon) is a boundary conversion handled by GeoDatum, not stored here.
struct Pose {
    float    x_m          = 0.0f;
    float    y_m          = 0.0f;
    float    z_m          = 0.0f;
    float    yaw_rad      = 0.0f;
    float    speed_ms     = 0.0f;
    uint64_t timestamp_us = 0;
    bool     valid        = false;
};

// Time-coherent snapshot of all observation domains for one tick.
// Built by ObservationAssembler::build() — do not write to this directly.
// BT nodes receive a const ref; nothing above the assembler writes here.
struct WorldState {
    Pose pose;
    // TODO: ObstacleMap  obstacles;
    // TODO: BatteryState battery;
};
