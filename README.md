# Worldfile

Worldfile compiles LLM-authored 3D environments into physics-ready MuJoCo
worlds. An environment family, seed, and short brief drive bounded Blender
authoring stages. Deterministic code then owns coordinates, physical metadata,
collision geometry, semantic sites, export, and validation.

The Blender scene is an authoring artifact. The result is MJCF and its
referenced assets, ready to use without Blender. Worldfile does not introduce a
new simulation format or simulation platform. See
[docs/research.md](docs/research.md) for the research direction.

The current utility-depot family is validated with a Clearpath Husky, ROS 2
localization, Nav2, RGB-D perception, and a mission behavior tree. That ground
stack tests whether a generated world supports a complete robot task; it is not
the boundary of the world format or compiler.

## Architecture

```text
environment family + seed + brief
                |
       bounded LLM authoring
                |
          Blender scene
                |
       deterministic compiler
                |
 MJCF + assets + robot and task sites
                |
             validation
                |
 MuJoCo + ROS mission smoke test
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
ros2 launch worldfile_demo navigation.launch.py world:=utility_depot_40_v2
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
and goals under `ros/src/worldfile_demo/config/worlds/`. See
[worldfile/README.md](worldfile/README.md) for staged operation and validation.

## Repository layout

```text
worldfile               Blender authoring, compilation, and validation
ros/src/worldfile_demo  localization, Nav2, perception, mission BT, battery, RViz
sim/worldfile_sim       MuJoCo loop and Husky sensor publishers
sim/robots              Husky MJCF
sim/worlds              blank and generated worlds
docker                  ROS/MuJoCo development image and Compose topology
scripts                 environment, goal, smoke, and visualization helpers
```

The simulator uses a kinematic Husky base. It validates sensing,
localization, planning, obstacle avoidance, RGB-D perception, and
world-generation contracts;
it does not claim motor, tire, traction, or skid-steer dynamics fidelity.
