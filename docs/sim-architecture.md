# Simulation architecture

SPAR runs both robots in one MuJoCo world. MuJoCo loads the generated site,
places the robots at their authored spawns, and provides the sensor data used
by ROS and PX4.

MuJoCo world coordinates and the ROS `map` frame use the same local ENU metres:
X is east, Y is north, and Z is up. The world datum records the latitude,
longitude, and altitude (LLA) of `(0, 0, 0)`, allowing the simulator to turn a
robot's local position into simulated GPS.

The X2 flies through PX4 and MuJoCo physics. The Husky moves directly from
Nav2's `cmd_vel` because a physics-based ground model was too slow for quick
behavior development. The reasoning behind the ROS, Nav2, and PX4 roles is in
[autonomy-architecture.md](autonomy-architecture.md).

## What a generated world supplies

The selected world name connects three artifacts:

| Artifact | Purpose |
| --- | --- |
| `sim/worlds/<world>.xml` | Geometry, physical spawn poses, and the world datum |
| `ground/.../config/worlds/<world>.yaml` | Ground datum, dock, and patrol waypoints |
| `air/.../config/worlds/<world>.yaml` | Air datum, landing home, and patrol waypoints |

All three files describe the same world and use the same datum. Spawn poses
only set the robots' initial physical positions; runtime localization comes
from simulated sensors.

## Startup

1. MuJoCo loads `sim/worlds/<world>.xml`, places both robots, reads the datum,
   and starts the simulation clock and sensor interfaces.
2. The ground ROS stack loads the matching ground YAML and starts localization,
   Nav2, and the ground behavior tree.
3. PX4 connects to MuJoCo and begins estimating the X2 from simulated flight
   sensors.
4. The air ROS stack loads the matching air YAML and waits for PX4 to report a
   valid position estimate and global reference before converting poses or
   sending position targets.

MuJoCo must be listening before PX4 connects. If MuJoCo restarts and rewinds
simulation time, restart PX4 and the ROS launches as described in the main
README.

## Ground data flow

```text
MuJoCo camera -> detector -> behavior tree -> target --+
                                                        |
wheel, IMU, GPS -> ROS localization -> robot pose ------+-> Nav2 -> cmd_vel
                                                        |              |
MuJoCo lidar -> obstacle scan --------------------------+              v
                                                          sim moves Husky directly
```

A physics-based Husky was too slow for behavior development, so the simulator
moves it directly from Nav2's `cmd_vel`. This lets patrol, inspection, obstacle
avoidance, and docking tests run quickly. The simulator still prevents the
Husky from moving through obstacles.

After moving the Husky, MuJoCo produces simulated wheel encoder, IMU, GPS,
camera, and lidar data. ROS localizes from those measurements instead of
receiving the true MuJoCo pose. Wheel and IMU data track local movement in
`odom`; GPS keeps that estimate aligned with the world `map`:

```text
map -- GPS correction --> odom -- local odometry --> base_link
```

The lidar checks obstacles at several heights and gives Nav2 one 2D obstacle
scan. This lets Nav2 see low objects such as pallets without requiring a full
3D perception stack. The behavior tree chooses a target in `map`, Nav2 plans a
safe path, and `cmd_vel` returns to the simulator to move the Husky.

## Air data flow

```text
                           MuJoCo X2
                               |
                 simulated GPS, IMU, magnetometer,
                         and barometer data
                               |
                               v
                         PX4 estimation
                               |
                  estimated position and validity
                               |
                               v
                         ROS autonomy
                 behavior and target selection
                               |
                    target in ROS map ENU
                               |
                    ENU -> PX4 local NED
                               |
                               v
                     PX4 flight control
                               |
                          rotor forces
                               |
                               v
                           MuJoCo X2
```

PX4 combines the simulated sensors to estimate and control the X2 in its local
NED frame: north, east, down. That local frame normally begins near zero.

PX4 reports both its local estimate and the geographic reference for that
local frame. The air ROS adapter uses them to express the X2 pose in the same
world-aligned `map` used by the Husky. ROS chooses a target in `map`, the
adapter converts it to PX4-local NED, and PX4 flies the drone there. MuJoCo
applies PX4's rotor outputs, so ROS never moves the X2 directly.

## Current limits

- The Husky moves directly from `cmd_vel`; SPAR does not test ground-vehicle
  physics.
- Simulated sensors use simple, repeatable noise.
- Ground lidar is reduced to a 2D obstacle scan.
- The X2 uses simplified aerodynamics and currently has no obstacle avoidance.

## Implementation pointers

| Concern | Source |
| --- | --- |
| World datum and LLA/ENU conversion | `sim/spar_sim/georeference.py` and `common/src/spar_geodesy` |
| Husky `cmd_vel` movement | `sim/robots/husky.xml` and `sim/spar_sim/husky.py` |
| Ground simulated sensors | `sim/spar_sim/sensors.py` and `sim/spar_sim/sim.py` |
| Wheel integration and EKF configuration | `ground/src/spar_ground/src/wheel_odometry.cpp` and `ground/src/spar_ground/config/localization.yaml` |
| PX4 simulated sensors and rotor loop | `sim/spar_sim/px4_link.py` |
| PX4 NED and ROS ENU conversion | `air/src/spar_air/src/frames.hpp` |
| Air pose publication | `air/src/spar_air/src/tf_from_px4.cpp` |
| ROS targets sent to PX4 | `air/src/spar_air/src/offboard_link.hpp` |

Air topic mappings live in `docker/air/pub.csv` and `docker/air/sub.csv`. The
PX4 HIL connection is in `sim/spar_sim/px4_link.py`. Air and geodesy unit tests
cover the frame conversions.
