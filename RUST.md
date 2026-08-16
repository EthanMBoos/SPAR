# Rust migration findings

This document records the useful conclusions from the `main_rust` experiment.
The implementation itself was intentionally discarded so the repository can
be renamed and reorganized without carrying an unfinished parallel codebase.

## Feasibility and boundary

A Rust application implementation is feasible without rewriting the robotics
ecosystem underneath it. The clean boundary is:

- Rust owns application behavior, mission policy, coordinate conversion,
  ground odometry and battery simulation, the Nav2 client, and PX4 offboard
  commands.
- ROS continues to own Nav2 and `robot_localization`.
- PX4 continues to own flight estimation, stabilization, control, and its
  native failsafes.
- MuJoCo remains the plant and sensor simulator.
- Python with Ubuntu's apt-installed OpenCV remains a reasonable perception
  implementation when the experiment is specifically comparing C++ and Rust
  application code.

Rewriting Nav2, PX4, or the behavior-tree engine at the same time would change
too many experimental variables. The useful comparison is the application
language and transport adapter, with the world, configuration, algorithms,
and underlying robotics components held constant.

## BehaviorTree.CPP

Keep BehaviorTree.CPP when direct behavioral parity matters. Bonsai and other
Rust behavior-tree libraries do not accept the same BehaviorTree.CPP XML with
identical composite and halt semantics.

The experiment successfully used a small C ABI adapter with generic condition
and stateful-action proxy nodes:

- BehaviorTree.CPP continued to parse and execute the original XML.
- Rust closures implemented every domain-specific leaf.
- Tick and halt calls crossed one narrow unsafe boundary.
- Rust panics were caught before returning through C++.
- The tree remained single-thread owned, matching BehaviorTree.CPP's normal
  synchronous tick model.

The ground and air XML files were byte-for-byte identical to `main`. That is
the right invariant for a future comparison. The adapter should link the same
ROS-distributed BehaviorTree.CPP version used by the C++ application.

Identical XML is not sufficient by itself. The leaf implementations, runtime
parameters, tick clock, action lifecycle, and halt behavior must also match.

## Configuration and comparison inputs

The original behavior-tree XML, autonomy YAML, world YAML, Nav2 YAML,
localization YAML, robot/world assets, and smoke-test stimuli should be shared
inputs between implementations. The Rust experiment copied these files
exactly, but then hard-coded several equivalent values in Rust. That weakens
the comparison because editing the shared YAML no longer changes both
implementations.

A future Rust application should load the same layered autonomy and world
configuration as the C++ application. It should not duplicate battery
thresholds, freshness windows, cooldowns, standoff distances, acceptance
radii, orbit parameters, frame names, or rates as source constants.

The simulator's physics, sensor rates, timestamps, message values, camera
rendering, and world selection should also remain unchanged. Only the
transport-specific construction and publication of messages should differ.

## Hiroz and ROS interoperability

Hiroz was viable as an experimental Rust/Python ROS-compatible transport over
Zenoh. It allowed application and simulator code to avoid `rclrs` and `rclpy`
while communicating with ROS Jazzy processes through `rmw_zenoh_cpp`.

It is nevertheless a high-risk dependency and should be pinned to a reviewed
revision. The experiment used commit
`6b9a966bbbd0d4beae60259c7cbe1e4b93e51b9f`.

The most important interoperability finding was Nav2 action metadata. The
Hiroz-generated Jazzy `NavigateToPose` GetResult and Status metadata matched,
but the SendGoal, CancelGoal, and Feedback hashes did not match the installed
Jazzy endpoints. The Rust client could see the action graph but could not
reach the server until it supplied an explicit local `ZAction` mapping with
the observed Jazzy hashes. This is version-sensitive and belongs in Hiroz's
generator or an upstream compatibility fix, not as unexplained application
magic.

QoS and discovery need explicit attention:

- PX4 output topics require sensor-data-compatible QoS.
- Camera, lidar, IMU, GPS, and wheel data should use the same best-effort
  sensor profile at both ends.
- Static TF depends on transient-local late-join delivery. That path was
  intermittent across Hiroz and `rmw_zenoh_cpp`; a readiness gate and, if
  necessary, transport-safe periodic republication are appropriate.
- Startup should wait for the actual publisher/subscriber/action endpoints
  and required TF chain rather than relying on fixed sleeps.
- Type hashes should be checked on every custom, Nav2, and PX4 endpoint.

These are transport/configuration issues, not evidence that the behavior-tree
XML or perception algorithm is intrinsically slow.

## Perception

Keeping perception in Python was the simplest honest choice. The point of the
branch was to demonstrate replacing C++ application code with Rust, not to
turn a working Python/OpenCV seam into a Rust OpenCV build experiment.
Installing `python3-opencv` with apt avoided compiling OpenCV bindings in
Cargo and matched the current ROS application's algorithm closely.

The Python detector preserved the important contracts:

- exact color/depth timestamp pairing;
- `rgb8` and `bgr8` stride handling;
- `32FC1` metre and `16UC1`/`mono16` millimetre depth formats;
- color-to-depth resolution scaling;
- a 5-by-5 valid-depth neighborhood and the C++ upper-median convention;
- the same two red HSV bands, largest contour, and moment centroid;
- minimum blob area and range rejection;
- optical-to-body camera conventions;
- timestamped projection into the map frame; and
- the same `Detection` schema, label, topic, and RIHS hash.

Fixture tests covered these contracts, but a formal comparison should replay
the same recorded input through both branches and enforce the original gates:
centroids within one pixel and projected detections within five centimetres.

The delayed ground detection observed late in the experiment was more likely
a TF delivery, QoS, startup-order, or configuration issue than an OpenCV
performance problem. The correct diagnostic is to inspect paired images,
camera info, the complete TF chain, detector logs, endpoint hashes, and the
Detection topic concurrently.

## Ground parity details

Several initially small implementation differences materially changed the
behavior:

- `Inspect` must locate the robot with the localized `map -> base_link`
  transform. Raw wheel odometry is in the odom frame and can produce the wrong
  map-frame standoff goal.
- Inspection must use the configured 1.8 metre standoff and stamp the
  configured 45 second cooldown after either success or failure.
- Patrol waypoints advance only after success, or after the configured number
  of failures. They must not advance merely because a goal was requested.
- Nav failures observe the same retry cooldown and maximum retry policy.
- `ReturnToDock` succeeds when Nav2 succeeds, after which the XML selects
  `HoldPosition`.
- `HoldPosition` publishes zero velocity; it is not an empty running leaf.
- Nav2 goals carry the current simulation-time stamp and cancellation mirrors
  the C++ action lifecycle.
- The executive and battery model tick from `/clock`, so pausing or rewinding
  simulation time does not leave wall-clock behavior running ahead.
- BT status should retain the same JSON fields, including battery and anomaly
  ages, so the same observers can compare both implementations.

The Rust Nav2 client did successfully send a real goal through Hiroz to the
C++ Jazzy Nav2 server and move the MuJoCo Husky after the action metadata was
corrected.

## Air parity details

PX4 should remain underneath the Rust application rather than being
reimplemented. The Rust side only needs to convert map ENU targets to PX4
local NED, publish offboard setpoints, issue vehicle commands, and translate
PX4 estimator output back into synchronized map-frame state and TF.

For direct parity with `main`:

- Behavior position comes from `VehicleOdometry`; `VehicleLocalPosition`
  supplies the GPS-backed reference and reset counters.
- Freshness and tree ticks use `/clock`, while synchronized TF uses
  `VehicleOdometry.timestamp_sample`.
- Offboard mode and trajectory setpoints stream at the original 10 Hz, not a
  faster substitute.
- Takeoff repeats offboard and arm commands with the same one-second policy
  until PX4 accepts them.
- Idle preserves the last setpoint rather than creating a different target.
- Landing uses PX4 native Land mode and the original grounded tolerance and
  force-disarm guard.
- Camera mount, route, home, cruise altitude, orbit radius/rate, acceptance
  radii, battery thresholds, and freshness windows come from the same YAML.

Live testing demonstrated arm, Offboard, takeoff, route motion, stop/hold,
native landing, disarm, battery recovery, and rearm. The final air build did
not complete a deterministic live anomaly acquisition/orbit acceptance run.

## Smoke-test parity

Smoke tests should use identical stimuli, while their passive observation and
diagnostics may be made more reliable. In particular:

- Use the same committed world and route.
- Send the same `start` and `stop` mission commands.
- Ground forces `battery/set` to 10 percent, not a different low value.
- Air sets `SIM_BAT_MIN_PCT` to 10, waits for the original return/land path,
  then restores it to 50. Stopping battery telemetry tests a different stale
  sensor failure mode and is not an equivalent input.
- Preserve the same expected phase order and behavior outcomes.
- Retain sensor-data QoS for PX4 CLI observations.

It is fine to add passive continuous BT-status capture so short phases cannot
be missed, endpoint and TF readiness gates, cleanup traps, structured logs,
and latency-tolerant observation windows. Those improve the harness without
changing the experiment's commands or environmental inputs.

At the end of the experiment neither rewritten automated smoke script had a
recorded passing run. The ground script reached real inspection in one run
but exposed inspection leaf mismatches; after partial fixes, a later clean
run waited unexpectedly for detection and was stopped for diagnosis. The air
automated script was not run. Earlier complete ground and air lifecycles were
validated interactively, which is useful feasibility evidence but not a
substitute for automated acceptance.

## Containers and build workflow

Independent images based directly on `ros:jazzy` were clearer than layering
the air image on a local ground image. The ground image installed Rust,
Hiroz, BehaviorTree.CPP, OpenCV, MuJoCo, Nav2, and localization. The air image
installed its own shared dependencies plus pinned PX4 v1.17.0 and matching
`px4_msgs`.

Compose should reference already-built images and use `--no-build` for normal
development. Image construction should remain an explicit setup command.
Named Cargo registry, Git, and target volumes avoid repeatedly downloading or
recompiling dependencies without hiding image inheritance.

The experiment's images were large—approximately 3.8 GB for ground and 6.4 GB
for air—so the independent recipes trade storage for reproducibility and a
clean comparison boundary.

## Validation strategy for a future attempt

Proceed in this order:

1. Assert byte identity for XML, YAML, world assets, and smoke stimuli, and
   prove the Rust processes actually load those files.
2. Compare simulator message fixtures and timestamps before involving
   autonomy.
3. Gate camera streams, endpoint hashes, QoS compatibility, and both TF links;
   then replay perception fixtures and recorded frames.
4. Test every Rust leaf against the C++ leaf's start, running, success,
   failure, retry, cooldown, and halt behavior.
5. Run the unchanged BehaviorTree.CPP XML with deterministic blackboard/event
   traces and compare selected leaves tick by tick.
6. Run the ground smoke test with the original inputs and require a zero exit.
7. Run the air smoke test with the original PX4 battery parameters and require
   a zero exit.
8. Only then compare CPU, memory, startup time, binary/image size, and
   end-to-end latency between C++ and Rust.

The overall conclusion remains positive: the application can be moved to
Rust while retaining the established robotics stack. The hard part is not
Rust or BehaviorTree.CPP. It is keeping every runtime input and leaf semantic
identical while making experimental Hiroz/Zenoh interoperability observable
and deterministic.
