# Autonomy ownership

SPAR follows the conventional mission-computer/autopilot split: ROS is the
mission computer that owns perception, world understanding, decisions, and
planning; PX4 is the flight controller that owns air-state estimation,
stabilization, actuators, and flight-critical safety. On the ground, the same
layering keeps mission logic above Nav2 and the base controller.

This is a widely used best-practice pattern for companion-computer robotics,
not a SPAR-specific invention. The source material for the decision is:

- PX4 describes the flight-controller-plus-companion arrangement as a typical
  system, calls the companion a mission computer, and keeps drivers,
  estimators, and controllers in the flight stack.
  [PX4 system architecture](https://docs.px4.io/main/en/concept/px4_systems_architecture)
- ArduPilot independently describes a companion computer receiving autopilot
  telemetry over MAVLink and using it to make higher-level decisions.
  [ArduPilot companion computers](https://ardupilot.org/dev/docs/companion-computers.html)
- Nav2 places application logic above independent planner, controller,
  smoother, route, and recovery servers, and supports using Nav2 as one
  navigation capability inside a larger robot autonomy application.
  [Nav2 navigation concepts](https://docs.nav2.org/concepts/)
- Active ROS guidance assigns long-term localization to `map`, continuous
  short-term motion to `odom`, and robot-relative state to `base_link`, with
  separate authorities for localization and odometry.
  [ROS REP-105](https://reps.openrobotics.org/rep-0105/)
- Autoware uses the same broader separation between localization, perception,
  mission/behavior/motion planning, trajectory validation, control, and the
  vehicle interface.
  [Autoware planning architecture](https://docs.autoware.org/main/design/autoware-architecture-v1/components/planning/),
  [Autoware control architecture](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/components/control/)

This split keeps fast stabilization and failsafes on the vehicle controller
while ROS uses cameras, maps, and higher-level logic to make mission decisions.
It also lets the same ROS autonomy work with the simulator or a real robot.
Other designs are valid—a fixed mission could run entirely in PX4—but SPAR is
built for developing perception-driven ROS autonomy, so this is the right
default here.

## Air ownership

ROS chooses what the X2 should do. It selects the current waypoint, inspection
orbit, home position, or landing behavior. PX4 flies the drone to the target
ROS provides.

PX4 is more than a motor driver. It combines the simulated GPS, IMU,
magnetometer, and barometer readings to estimate the X2's motion. It also keeps
the drone stable, controls the motors, handles arming and landing, and applies
flight-safety checks and failsafes. This is the usual PX4 companion-computer
setup: ROS is the mission computer and PX4 is the flight controller.
[PX4 system architecture](https://docs.px4.io/main/en/concept/px4_systems_architecture)

```text
MuJoCo GPS + IMU + magnetometer + barometer
                       |
                       v
              PX4 estimates the X2
                       |
                       | estimated position
                       v
               ROS chooses a target
                  in map ENU metres
                       |
                       v
          convert map ENU to PX4 local NED
                       |
                       v
                 PX4 flies the X2
                       |
                       | motor outputs
                       v
                     MuJoCo
```

ROS chooses targets in the shared `map` ENU frame. Before sending one to PX4,
the air code converts it into PX4's local NED frame. PX4 then handles the
continuous work of keeping the drone stable and reaching that position through
its [Offboard mode](https://docs.px4.io/main/en/flight_modes/offboard).

The current air demo follows explicit targets and does not avoid obstacles. A
future ROS planner can choose safer paths or stream more detailed trajectories
without taking stabilization away from PX4.

## Ground ownership

Nav2 is part of the ROS autonomy stack. Delegating path planning and motion
control to Nav2 does not move autonomy outside ROS. Nav2 itself documents a
highest-level behavior-tree navigator invoking separate planning, control, and
recovery action servers, and supports being called as one capability inside a
larger application behavior tree.
[Nav2 navigation concepts](https://docs.nav2.org/concepts/)

```text
GPS + wheel/IMU odometry
              |
              v
       ROS localization
              |
              | estimated pose in map
              v
       ROS behavior layer
      mission / exploration / task choice
              |
              | navigation goal in map
              v
             Nav2
       global and local planning
       obstacle avoidance and control
              |
              | cmd_vel
              v
       sim moves the Husky directly
```

The behavior layer chooses the task: patrol, inspect, explore, or return to the
dock. Nav2 chooses a path, uses the simulated lidar scan to avoid obstacles,
and sends `cmd_vel`, which says how fast to drive forward and turn.

The ground simulation does not model the Husky's motors or tires. On every
simulation step, it moves the Husky by the speed and turn rate in `cmd_vel`,
unless that move would overlap an obstacle. That is all “kinematic” means here:
the command changes the robot's position directly instead of going through
simulated motors, tires, and forces. The simulator then calculates wheel
encoder, IMU, and GPS readings from that movement and publishes them as normal
sensor data.

ROS combines the wheel and IMU readings to track the Husky's smooth local
motion in `odom`. GPS keeps that estimate aligned with the world `map`. The
localization code never receives the Husky's true MuJoCo pose or its authored
spawn. This is the standard ROS `map -> odom -> base_link` arrangement described
by [ROS REP-105](https://reps.openrobotics.org/rep-0105/).

## Why SPAR uses both LLA and local ENU

SPAR uses LLA for locations on Earth and local ENU metres for work inside one
site. They answer different questions. REP-105 allows `map` to be tied to a
global reference and recommends ENU alignment for outdoor robots. A flat local
map works well for a small site; over long distances it eventually needs a new
origin or an Earth-aware mapping system.
[ROS REP-105](https://reps.openrobotics.org/rep-0105/)

```text
LLA:
    Where is this location on Earth?

ROS map ENU:
    Where is it within this site, in metres?

PX4 local NED:
    What local setpoint must the flight controller track?
```

Use global LLA for information that should remain meaningful outside one local
run:

- imported GIS terrain and map features;
- survey boundaries and geofences;
- persistent infrastructure locations;
- globally authored waypoints and homes;
- cross-site logging and mission exchange.

Use local ENU metres for geometry and reactive planning:

- lidar, depth, obstacles, and collision checking;
- random exploration and frontier selection;
- local coverage paths;
- distances, velocities, and acceptance radii;
- camera detections and nearby robot coordination.

For example, random exploration should not sample offsets in latitude and
longitude degrees. It should operate in metres around the estimated pose:

```text
current map pose:    (12, 8, 3) m
sampled displacement: (+4, -2, +1) m
candidate target:    (16, 6, 4) m
```

Current SPAR mission files store waypoints in `map` ENU. A future LLA mission
input should be converted to ENU before planning, rather than stored as a
second copy of the same point. Nav2 uses the same general approach for GPS
waypoints. [Nav2 navigation concepts](https://docs.nav2.org/concepts/)

## How SPAR stores locations

A generated world stores locations once:

- one world datum records the LLA of MuJoCo and ROS `map` origin `(0, 0, 0)`;
- robot homes and mission waypoints are stored as local ENU metres from that
  datum.

The live GPS path is separate. MuJoCo converts a robot's ENU position into a
noisy LLA sensor reading. Ground localization converts that GPS reading back
into `map` ENU. PX4 receives the X2's simulated GPS and reports its own local
estimate and global reference; the air adapter uses those to express the X2 in
the same ROS `map`. This is how the robots are localized. It does not require a
second LLA copy of every waypoint.

For a future real-terrain import, the original GIS coordinates remain the
source data. The importer will choose a datum and convert the terrain and
mission points into local ENU for MuJoCo and ROS. That conversion happens when
the world is imported, rather than requiring users to define every point twice.

The literal simulator startup, topics, transformations, and HIL loop are
documented separately in [sim-architecture.md](sim-architecture.md).
