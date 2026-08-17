# SPAR

SPAR generates outdoor robotics worlds in Blender, exports them to MuJoCo,
and drives one Clearpath Husky through those worlds with ROS 2 localization,
Nav2, and focused RGB-D perception. The repository is deliberately
ground-only: MuJoCo supplies lidar, GPS, IMU, wheel encoders, an RGB-D camera,
and collision behavior. A compact ground behavior tree owns the mission-level
policy while Nav2 owns planning and motion to standard `NavigateToPose` goals.

## Architecture

```text
family + seed + brief
        |
bounded BlenderMCP stages
        |
MJCF, meshes, Husky spawn, Nav2 demo goals
        |
MuJoCo sensors -> localization ----------------------+
       RGB-D -> red-barrel detector -> map point     |
       battery simulation ---------------------------+-> ground mission BT
generated goals + dock ------------------------------+          |
                                                               Nav2
                                                                |
                                                            cmd_vel
```

Generated worlds share one `map` frame with MuJoCo's ENU coordinates. Each
world config contains its geographic datum, dock pose, and an ordered set of
collision-checked navigation goals. The mission executive uses those goals as
its rounds route and preempts them for inspection or low-battery docking.

## Quick start

Install the host tools as described in [docs/install.md](docs/install.md), then:

```bash
make dev
```

Inside the container, build the ROS workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

In another host terminal, start the simulator:

```bash
make sim WORLD=utility_depot_40_v2
```

Then launch ROS from the container shell. The simulator must be running first
because it owns `/clock`:

```bash
ros2 launch spar navigation.launch.py world:=utility_depot_40_v2
```

Useful commands:

```bash
make shell
make smoke WORLD=utility_depot_40_v2
make view WORLD=utility_depot_40_v2
make rviz
make stop_sim
make down
```

Start or stop the mission manually inside the robot container:

```bash
/ws/scripts/mission.sh start
/ws/scripts/mission.sh stop
```

To send one generated goal directly through Nav2 from the robot container:

```bash
python3 /ws/scripts/nav_goal.py --world utility_depot_40_v2 --index 0
```

The detector publishes localized observations as standard
`geometry_msgs/msg/PointStamped` messages on
`/husky/perception/red_barrel`. `make smoke` verifies the complete autonomy
contract: idle, start, perception-driven inspection, rounds, low-battery
docking, recharge, resume, and stop.

## Generate a world

```bash
make worldgen WORLD=utility_depot_40_v3 SEED=42 \
  BRIEF="a denser west storage yard with a broad central aisle"
```

The final authoring stage creates the Husky navigation-goal sites. Export
writes the MJCF and assets under `sim/worlds/` and the matching datum, dock,
and goals under `ros/src/spar/config/worlds/`. See
[worldgen/README.md](worldgen/README.md) for staged operation and validation.

## Repository layout

```text
ros/src/spar       localization, Nav2, perception, mission BT, battery, RViz
sim/spar_sim       MuJoCo loop and Husky sensor publishers
sim/robots         Husky MJCF
sim/worlds         blank and generated worlds
worldgen           Blender authoring, export, and validation
docker             one ROS/MuJoCo development image and Compose topology
scripts            environment, goal, smoke, and visualization helpers
```

The simulator uses a kinematic Husky base. It validates sensing,
localization, planning, obstacle avoidance, RGB-D perception, and
world-generation contracts;
it does not claim motor, tire, traction, or skid-steer dynamics fidelity.
