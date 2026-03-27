# ROS2 and ArduPilot Integration Notes

---

## Why the BT Lives in C++ and Not in the ROS Graph

A ROS2-native behavior tree has ambient graph access — it can call services, send action goals, subscribe to anything, trigger sensor reconfiguration mid-mission. SPAR's BTNode is a pure function instead: `WorldState` in, `CommandStream` out, no graph access. The tradeoff is determinism and testability for convenience.

You don't lose capability — you make it explicit. Everything the BT needs to observe becomes a field in `WorldState`, pushed through the assembler from a Zenoh source. Everything the BT needs to actuate becomes a typed field in `CommandStream`, consumed by a dedicated adapter. The interfaces grow; the BT stays pure.

```
input side:   perception_ws topics → ZenohSources → assembler → WorldState fields
output side:  CommandStream fields → typed adapters → MAVLink / ROS2 topics / services
```

For sensor steering specifically: the BT puts an `AttentionHint` in `CommandStream`; perception_ws subscribes and steers its own sensors. The BT expresses intent, the perception layer executes it. The BT never reaches into the graph.

The cost: every new capability requires a named field and a wired adapter. The payoff: any session log fully reconstructs what the BT saw and decided, with no ambient graph state to account for.

---

## Pose: One Source, Used Everywhere

The pose used to transform sensor data into world coordinates in the perception pipeline must be the same pose that lives in `WorldState.pose`. If they differ, the BTNode is comparing "where I am" against obstacle positions that were placed using a different "where I am" — the coordinate frames don't agree and the WorldState is internally inconsistent.

**The rule: one pose source, published in two directions.**

```
ArduPilot EKF3
  │
  ├── MAVLink direct → WorldState.pose          (inbound to SPAR, existing path)
  └── MAVLink direct → outbound TF publisher    (outbound to ROS2 TF tree)
                              │
                              └── perception_ws uses for frame transforms
                                  → obstacle positions are in the same world frame
```

The perception pipeline places obstacles in world coordinates using the rover pose. SPAR reasons about those obstacles relative to `WorldState.pose`. Both must read from the same source or the geometry is wrong.

**Why not MAVROS for the inbound path.** MAVROS is fine for publishing pose outbound to the TF tree — that direction is one-way and latency there doesn't affect WorldState. The inbound path (pose → assembler) must stay on MAVLink direct because MAVROS routes through the ROS2 executor, which bakes receipt-time jitter into what should be a capture timestamp. That jitter is exactly what the assembler exists to remove.

**If you introduce better localization (SLAM, VIO, RTK fusion).** Don't run two parallel pose estimates. Feed the better estimate back into ArduPilot's EKF3 as an external vision input via `VISION_POSITION_ESTIMATE` MAVLink message. ArduPilot fuses it and publishes the result. Everything downstream — `WorldState.pose` and the TF tree — reads one estimate again. A parallel SLAM pose that isn't fed back into ArduPilot creates a control mismatch: you're commanding a controller that believes it's somewhere different from where SPAR thinks it is.

---

## ArduPilot: Stay on MAVLink Direct

For pose and telemetry the existing MAVLink UDP path is correct. ROS2 makes sense for lidar and camera sources where you're already in ROS2 land — perception pipelines, sensor drivers feeding `WorldState.obstacles`.

---

## How ROS2 Is Actually Installed

ROS2 is not a library you submodule or vendor into the repo. It is a system-level installation:

```bash
# On Ubuntu / Jetson Orin
sudo apt install ros-jazzy-desktop
source /opt/ros/jazzy/setup.bash
```

It lands in `/opt/ros/jazzy/`. The repo never contains ROS2 itself.

**On macOS:** ROS2 has no practical native support. Use Docker for development. The Jetson Orin (hardware target) runs Ubuntu and installs ROS2 natively — ROS2 source adapters only matter on the robot anyway.

---

## How It Connects to SPAR

Two separate processes. perception_ws uses `rmw_zenoh_cpp` as its RMW layer — nodes publish natively over Zenoh instead of FastDDS. spar_rover subscribes via zenoh-cpp. Both are in the same Zenoh session. No bridge, no FastDDS anywhere.

```
perception_ws (ROS2 / rmw_zenoh)  → publishes natively over Zenoh
spar_rover                         → Zenoh subscriber thread, links zenoh-cpp only
  ├── tick thread (20 Hz)          → reads assembler.build(t)
  ├── ArduPilot telemetry          → calls assembler.push_pose()       [MAVLink, existing]
  └── Zenoh subscriber thread     → calls assembler.push_obstacles()   [Zenoh, new]
```

Fault isolation is the reason spar_rover doesn't use rclcpp directly. The alternative — embedding rclcpp and FastDDS inside spar_rover — puts unstable middleware in the same address space as the tick loop and monitor. A crash or misbehaving callback takes down the safety-critical process. With this design, if a perception node crashes the feed goes silent, the assembler's staleness bounds emit `degraded`, the fallback kicks in, and spar_rover keeps running.

With shared memory enabled in the Zenoh config, the transport between perception_ws and spar_rover is zero-copy — a lidar node writes a pointcloud to SHM once and spar_rover reads it directly. No intermediate copy.

---

## Directory Layout

```
perception_ws/               ← colcon workspace, lives alongside the spar repo
  src/
    lidar_driver/            (or vendor package — e.g. ros-jazzy-velodyne)
    camera_pipeline/
    radar_driver/
    obstacle_detector/       (fuses sources, publishes /scan, /obstacles, etc.)

spar/                        ← plain CMake, links zenoh-cpp only (no rclcpp)
  spar_rover/
    sources/
      ZenohSources.cpp       ← Zenoh subscriber thread, calls assembler.push_*()
```

### perception_ws colcon workspace

Standard colcon layout. Each sensor or processing package gets its own directory with `package.xml` and `CMakeLists.txt`:

```bash
mkdir -p perception_ws/src
cd perception_ws

# vendor/hardware drivers installed as system packages where available
sudo apt install ros-jazzy-velodyne ros-jazzy-realsense2-camera   # example

# your own packages live in src/
colcon build
source install/setup.bash
```

Run nodes from the perception side independently of SPAR — they publish to Zenoh natively and know nothing about SPAR.

### rmw_zenoh setup

Install `rmw_zenoh_cpp` and set it as the RMW before launching any perception_ws nodes. This replaces FastDDS entirely — no DDS daemon, no bridge process.

```bash
sudo apt install ros-jazzy-rmw-zenoh-cpp

# set before sourcing the workspace or launching nodes
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

ROS2 nodes then publish over Zenoh natively. The wire format is still CDR — the same deserialization code works in `ZenohSources.cpp`.

**Keyexpr format.** rmw_zenoh uses a structured keyexpr for each topic, not the simple `rt/` prefix of the bridge. The format for `/scan` on domain 0 is approximately `0/scan/sensor_msgs::msg::dds_::LaserScan_/_0/_0/UHLC`. Rather than hardcoding this, subscribe with a wildcard in spar_rover:

```cpp
session.declare_subscriber("0/scan/**", callback);
session.declare_subscriber("0/obstacles/**", callback);
```

Nail down the exact keyexprs during initial integration by printing `sample.get_keyexpr()` in the callback — don't guess.

### ZenohSources.cpp inside SPAR

Plain CMake, guarded behind `SPAR_ENABLE_ZENOH`, following the same pattern as `SPAR_ENABLE_MAVLINK`. No rclcpp, no DDS, no executor — just zenoh-cpp linked as a library:

```cmake
option(SPAR_ENABLE_ZENOH "Enable Zenoh sensor sources" OFF)

if(SPAR_ENABLE_ZENOH)
    find_package(zenohcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)   # type definitions only — no rclcpp executor
    target_sources(spar_rover PRIVATE sources/ZenohSources.cpp)
    target_link_libraries(spar_rover PRIVATE zenohcpp::lib sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)
    target_compile_definitions(spar_rover PRIVATE SPAR_HAVE_ZENOH=1)
endif()
```

Build SPAR with plain cmake — no need to source the ROS2 environment for the SPAR build:

```bash
cmake -B build -DSPAR_ENABLE_ZENOH=ON
cmake --build build
```

**CDR deserialization.** Zenoh delivers raw CDR bytes; rclcpp's automatic deserialization is not available. Link against `sensor_msgs` for type definitions only and use a CDR parser to extract `header.stamp` and payload. The subscriber callback must be minimal: deserialize, extract stamp, call `assembler.push_*()`, return. No processing, no allocation, no blocking — heavy work belongs upstream in the perception pipeline.

**Shared memory.** Enable in the Zenoh session config so the bridge and spar_rover exchange data via SHM rather than loopback socket — zero-copy for pointclouds and images:

```json
{ "transport": { "shared_memory": { "enabled": true } } }
```

### Capture timestamp verification — required per driver

`ZenohSources.cpp` extracts `header.stamp` from the deserialized CDR payload and passes it as the sample capture timestamp. `header.stamp` is only correct if the driver stamps at hardware capture, not at publication. This varies and must be verified for each sensor before wiring it into the assembler:

| Source | Typical behavior | Verify |
|--------|-----------------|--------|
| Lidar (Velodyne, Ouster, Livox) | Hardware timestamp in `header.stamp` — usually correct | Confirm with your specific unit |
| Camera | Varies widely — some stamp at frame grab, some at encode/publish | Check driver docs; measure wall time minus `header.stamp` at the ROS2 subscriber |
| Radar | Entirely driver-dependent | Treat as unknown until measured |

Sanity check: add a temporary ROS2 subscriber in the perception_ws and log `rclcpp::now() - msg.header.stamp`. If that value tracks latency and jitter rather than sitting near zero, the driver is stamping at publish time and the timestamp is not safe to use as a capture time without correction.

---

## What Needs to Be Built

**Immediate — blocking SITL integration (P0 bugs):**

- [ ] Fix MAVLink `type_mask` in [spar_rover/adapter/ArdupilotAdapter.cpp:79](spar_rover/adapter/ArdupilotAdapter.cpp#L79) — current value causes ArduPilot to ignore yaw_rate and hold 0° heading; correct value is `0x3F7`
- [ ] Periodic heartbeat in `ArdupilotAdapter` telemetry loop at 1 Hz — currently sent once on connect; ArduPilot stops accepting commands after a few seconds of silence
- [ ] Bind UDP socket before `connect()` — without this `recv()` silently drops all ArduPilot telemetry

**Phase 1 completion:**

- [ ] Wire `third_party/mavlink` submodule behind `SPAR_ENABLE_MAVLINK`
- [ ] Configure ArduPilot stream rates on connect (`SR2_POSITION=10`, `SR2_EXTRA1=50`)
- [ ] TIMESYNC offset measurement at connect — currently receipt time is used as capture timestamp, breaking the assembler's core guarantee
- [ ] Characterize ArduPilot stale/lost-command behavior on the exact SITL config in use (GUIDED Rover, target ArduPilot version): what happens when setpoints go stale or stop arriving — hold last command, ramp to neutral, RTL, or failsafe? The answer is version- and parameter-dependent and must be measured, not assumed. This determines whether "stop writing" is a legitimate fallback or whether the mission-side fallback must guarantee safety without that backstop.
- [ ] Gazebo world for rover mission execution
- [ ] MCAP logging replacing the current TSV session file
- [ ] Temporal-window invariant implementation (oscillation, geofence displacement, jerk)

**Pose outbound to ROS2 — required before perception pipeline can place obstacles in world frame:**

- [ ] Outbound TF publisher: reads ArduPilot pose from the same MAVLink telemetry thread that feeds `assembler.push_pose()`, publishes to ROS2 `/tf` as `map → base_link` transform — this is the pose perception_ws uses for all sensor frame transforms; it must be the same source as `WorldState.pose` or obstacle positions will be in a different coordinate frame than the rover thinks it is
- [ ] Validate consistency: after wiring both paths, check that an obstacle at a known world position appears at the correct relative position in WorldState given the rover's reported pose

**Perception workspace — can be scaffolded now, sources are not on the critical path until Phase 2 extended:**

- [ ] Create `perception_ws/src/` alongside the spar repo; add a placeholder `obstacle_detector` package with `package.xml` and stub `CMakeLists.txt` so the colcon build skeleton exists before sensor drivers are wired in
- [ ] Verify `header.stamp` capture-time correctness for each sensor driver before wiring into the assembler — see verification table above
- [ ] Docker devcontainer config for macOS: perception_ws colcon build + SPAR cmake with `SPAR_ENABLE_ZENOH=ON`

**Zenoh / rmw_zenoh and SPAR source adapters — not on the critical path until Phase 2 extended (obstacles/camera in WorldState):**

- [ ] Install `ros-jazzy-rmw-zenoh-cpp` on the Jetson Orin; confirm perception_ws nodes publish over Zenoh by printing `sample.get_keyexpr()` in a test subscriber — record the exact keyexprs for `/scan` and `/obstacles` before writing `ZenohSources.cpp`
- [ ] Enable shared memory in the Zenoh session config for both perception_ws and spar_rover
- [ ] `spar_rover/sources/ZenohSources.cpp` — Zenoh subscriber thread, subscribes using wildcard keyexprs confirmed above, deserializes CDR, calls `assembler.push_obstacles()` with `header.stamp` as capture timestamp
- [ ] `SPAR_ENABLE_ZENOH` CMake flag wired up as above
- [ ] Transport-degradation layer extended to cover Zenoh sources (delay, jitter, dropout drawn from measured hardware) before running Phase 2 extended catch fraction experiments
