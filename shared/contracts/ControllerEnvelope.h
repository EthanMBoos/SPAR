#pragma once

// The controller's accepted command envelope: the physical limits that the
// normalized [-1, 1] CommandStream maps onto for a specific controller/vehicle.
// The adapter uses it to turn normalized throttle/steering into physical
// set-points; the runtime monitor's invariants are derived from the same limits.
//
// Named per controller because limits are per-vehicle (a Husky's max speed is not
// a Jackal's) even when they share the same Ros2Adapter transport code.
//
// Defaults target the Clearpath Husky (A200).
struct ControllerEnvelope {
    float max_linear_mps   = 1.0f;  // full throttle → forward speed (m/s)
    float max_angular_radps = 2.0f; // full steering → yaw rate (rad/s)
};
