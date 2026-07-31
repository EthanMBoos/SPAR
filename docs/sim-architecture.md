# The simulation architecture

One decision drives everything here: MuJoCo is the sim, full stop. One
Python process (`sim/spar_sim/`) owns one world containing both robots, one
physics state, and one clock. It renders cameras headless through EGL and
publishes hardware-shaped sensor messages directly from MuJoCo. The ground
and air autonomy stacks share its ROS graph but never receive pose truth.

```
                MuJoCo: one world, both robots, one clock
                ========================================
     AIR                                      GROUND
     noisy HIL_SENSOR + HIL_GPS               noisy NavSatFix + LaserScan
                  |                                  |
                  v                                  v
              PX4 EKF2                          tf_from_gps
                  |                          translation map -> odom
                  v                                  |
             tf_from_px4                             v
                  |                                 Nav2
                  v                           TF + live scan only
          PX4 waypoint flight
```

The rule is simple: **the sim emits sensors; the ROS side estimates pose
from them. Nothing downstream is handed truth.**

The air link is lockstep. The sim blocks each physics tick for PX4's actuator
reply and applies rotor forces itself, so PX4 owns flight control inside the
physics loop. Ground control is deliberately asynchronous: Nav2 sends
`cmd_vel` into the skid-steer mixer, and a 0.5 s watchdog stops the wheels
when commands go stale. A statically stable rover can coast through latency;
a quadrotor cannot.

The drone has a camera and detector but no obstacle input. It flies its
waypoints without avoidance. Adding depth-based offboard avoidance is a real
subsystem and remains out of scope.

The same environment runs three ways:

- **Headless:** `make start_sim`, used by the ROS stacks and smoke tests.
- **Watched:** `make view`, a read-only native viewer of the running sim.
- **Training:** the next planned slice drives the same MJCF without ROS
  ([rl.md](rl.md)).

## Worlds and robot configuration

The MJCF file is the world (`sim/worlds/<world>.xml`). It contains geometry
and includes robot definitions from `sim/robots/`. There is no occupancy-map
artifact and no rasterization step.

Sensor geometry belongs to the robot. A planar-scanning robot declares its
site through MuJoCo custom text:

```xml
<custom>
  <text name="husky.scan_site" data="lidar2d_0_laser"/>
</custom>
```

The simulator resolves that entry through `sim/spar_sim/robot_config.py`.
This makes two robots with different mast heights unambiguous and keeps world
generation free of robot-specific sensor knowledge.

Keep collision geometry to primitives where practical and mark decorative
geometry `contype="0" conaffinity="0"`. Visuals can then change without
changing physics or lidar. Static world membership is `body_weldid == 0`;
robots and movers have joints and therefore separate weld groups.

The Husky-looking rover uses two drive spheres and frictionless casters
under its visual shell. An honest four-wheel skid-steer barely pivots in
MuJoCo because its solver enforces lateral tire friction. The exact model
and rationale live beside the parameters in `sim/robots/husky.xml`.

## Ground localization and navigation

The ground TF tree follows REP-105:

```
map --tf_from_gps--> odom --sim odometry--> base_link --> sensor frames
```

The simulator publishes `sensors/gps/fix` as reliable `NavSatFix` at 10 Hz,
using a private geographic datum and RTK-scale Gaussian noise. `tf_from_gps`
averages the first second of fixes as its local origin, converts later fixes
to ENU, and compares them with stamped `odom -> base_link`. It publishes a
translation-only `map -> odom`; heading passes through odometry because a
single-antenna receiver cannot measure yaw.

Transforms are post-dated by 0.5 s, matching the localization tolerance,
so costmaps and stamped camera projection have a current correction
available. Missing stamped odometry drops one correction with a throttled
warning rather than stopping the stack.

Nav2 is mapless. Its 40 m rolling global costmap uses the live scan's obstacle
and inflation layers, and the local rolling costmap uses voxel and inflation
layers. Unknown space is traversable, so global plans are optimistic and
local sensing corrects them as obstacles enter view. Consecutive route legs
should stay near 18 m or less unless the global window grows with the site.

One shortcut remains explicit: `platform/odom` and `odom -> base_link` still
come from MuJoCo pose and do not drift. The next fidelity step is wheel-joint
odometry plus an IMU, followed by `robot_localization`: a local EKF owns
`odom -> base_link`, `navsat_transform_node` converts GPS, and a global EKF
owns `map -> odom`. The sensor and downstream Nav2 interfaces do not change.
SLAM or 3D LIO can replace the same `map -> odom` owner later, but the current
single horizontal scan is intentionally thin input for outdoor SLAM.

## Perception

The camera is real in every run: the sim renders color and depth offscreen,
and the detector turns pixels into a labeled map-frame point on
`perception/detections`. A pretrained vision model can replace the HSV node
and publish the same `Detection` message. The behavior tree selects labels
through `anomaly_label`; it does not depend on the detector implementation.

## Ground topics

Everything below lives under `/husky` except the global clock:

| Topic / TF | Type | Notes |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock` | MuJoCo owns time |
| `platform/odom`, TF `odom -> base_link` | `nav_msgs/Odometry` | drift-free shortcut; twist is in the child frame |
| `sensors/gps/fix` | `sensor_msgs/NavSatFix` | reliable, noisy, 10 Hz |
| TF `map -> odom` | TF | noisy translation correction from `tf_from_gps` |
| TF `base_link -> {lidar2d_0_laser, camera_0_link}` | static TF | published once and latched |
| `sensors/lidar2d_0/scan` | `sensor_msgs/LaserScan` | 720 rays at 15 Hz, 25 m maximum |
| `sensors/camera_0/...` | `sensor_msgs/Image`, `CameraInfo` | color and depth at 10 Hz |
| `perception/detections` | `spar_perception/Detection` | labeled map-frame observations |
| `cmd_vel` | `geometry_msgs/TwistStamped` | Nav2 to skid-steer mixer |

Transport is Zenoh over TCP. The containers share one network namespace, so
ROS nodes and PX4 communicate over localhost.

## Scope

MuJoCo remains the only physics authority. Normal operation targets real time
because ROS planning and vision do not accelerate with the physics clock.
Manipulation, learning from pixels, high-fidelity aerodynamics, and drone
obstacle avoidance are not implemented.

The immediate path is generated MJCF worlds followed by one pure-MuJoCo
training task and one deployed policy leaf. See [rl.md](rl.md).
