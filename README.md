# SPAR

SPAR is a small robotics playground for learning ROS2, Nav2, behavior trees,
PX4, and MuJoCo. It is also a test bed for one research direction: generate
many simple worlds with a local LLM, train behaviors across them, then run the
learned behavior inside the ROS2 stack.

It is the starter repository for the
[GT Cloud Robotics](https://www.gtcloudrobotics.com/course-home/) Autonomy
and LLM tracks.

The repository intentionally stops short of an application framework. One
Python process owns MuJoCo and publishes sensor-shaped ROS topics. The ground
track runs a Husky with Nav2 and a C++ behavior tree. The air track runs a
Skydio X2 model controlled by PX4 SITL and a separate behavior tree. Both see
the same world and camera-to-detection interface.

The working behavior is deliberately basic:

- patrol while a mission is active;
- inspect the red anomaly detected from the rendered camera;
- return home when the battery is low;
- resume after recharging.

[Simulation architecture](docs/sim-architecture.md) explains the boundaries.

## Ground quickstart

Requirements are Docker with at least 8 GB assigned and Python 3 for the
optional native MuJoCo viewer.

Start the sim:

```bash
make start_sim
make view                 # optional host viewer
```

In a second terminal:

```bash
make ros2_container

# Inside the container:
colcon build --symlink-install
source install/setup.bash
ros2 launch spar_ground autonomy.launch.py
```

In another container shell:

```bash
make shell
scripts/mission.sh start
ros2 topic echo /husky/bt/status
scripts/mission.sh stop
```

Run the unattended behavior check from the host:

```bash
make smoke
```

The sim owns `/clock`. If the sim restarts, restart the ROS launch too.

## Air track

The air image includes PX4 and takes longer to build:

```bash
make start_sim
make ros2_container_air

# Inside the container:
colcon build --symlink-install
cd /opt/px4/build/px4_sitl_zenoh
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none_iris \
  PX4_SIM_HOSTNAME=localhost ./bin/px4 -d > /tmp/px4.log 2>&1 &
./bin/px4-zenoh start
ros2 launch spar_air air.launch.py
```

After PX4's estimator settles:

```bash
make shell_air
scripts/mission.sh start
ros2 topic echo /skydio/bt/status
```

`make smoke_air` runs takeoff, patrol, inspection, battery return, landing,
disarming, and relaunch.

## World generation

`scripts/generate_world.py` turns a description into a small primitive MJCF
world. A local Ollama model chooses semantic layout fields and reviews its
choice. Python owns coordinates, geometry, robot includes, and linting.

```bash
make lint  # creates .venv and validates the canonical world

.venv/bin/python scripts/generate_world.py \
  --name loading_yard \
  --model gemma3:4b \
  --seed 0 \
  "A compact loading yard with fencing, crates, and a red hazard drum"

make inspect WORLD=loading_yard
```

Generated worlds are local experiments and ignored by git. If one is useful
after visual inspection, select it when the sim starts:

```bash
make start_sim WORLD=loading_yard
```

There are intentionally no Make targets for generation. The script and its
tests are still changing quickly. See [world generation](docs/worldgen.md).

## Editing the autonomy code

The main packages are:

```text
ground/src/spar_ground     ground behavior, localization, battery, launch
air/src/spar_air           air behavior and PX4 transforms
common/src/spar_perception shared Detection message and red-object detector
sim/spar_sim               MuJoCo stepping, rendering, sensors, transport
```

Behavior trees are XML in each robot package. Runtime parameters and launch
files live beside their code. The robot model owns its default spawn, so each
stack treats its own spawn as map `(0, 0)` and generates patrol points around
that origin. Worlds do not contain ROS parameters or waypoint files.

After an initial `colcon build --symlink-install`, a ground-only C++ rebuild
can use:

```bash
cd build/spar_ground && make
```

Stop and relaunch the stack after rebuilding. ROS logs go to `logs/runNNN`
for ground and `logs/air/runNNN` for air. The simulator log is
`logs/sim.log`.

## Useful commands

Run `make` to list all stable commands. The common ones are:

| Task | Command |
| --- | --- |
| Start or stop MuJoCo | `make start_sim`, `make stop_sim` |
| Inspect one world without ROS | `make inspect WORLD=blank` |
| Validate world geometry and route | `make lint WORLD=blank ROBOT=husky` |
| Enter the running ground container | `make shell` |
| Enter the running air container | `make shell_air` |
| Open RViz in a browser | `make rviz` |
| Run ground or air behavior checks | `make smoke`, `make smoke_air` |
| Stop all containers | `make shut_down` |

RL is not implemented yet. [The RL note](docs/rl.md) defines the first narrow
vertical slice without claiming a framework that does not exist.
