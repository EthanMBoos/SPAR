# SPAR - Sim Portable Autonomy Runtime

https://github.com/user-attachments/assets/d33431ac-4db0-448b-80e7-903b282471d2

*The default `utility_depot_40_v2` world, authored with Claude Sonnet via BlenderMCP
and exported for MuJoCo.*

SPAR is a small robotics playground for learning ROS2, Nav2, behavior trees,
PX4, and MuJoCo. It is also a testbed for generating many diverse worlds
through an orchestrated LLM-and-Blender pipeline, then running and evaluating
robot autonomy inside a deterministic ROS 2 stack.

Behavior trees are configured for a Husky ground robot with Nav2 and a Skydio
X2 drone controlled by PX4 SITL. Both see the same world via a shared
camera-to-detection interface.

The working behavior is basic.

- patrol while a mission is active;
- inspect the red anomaly detected from the rendered camera;
- return home when the battery is low;
- resume after recharging.

SPAR simulates outdoor robots in a small local patch of Earth. Every generated
world records the latitude, longitude, and altitude (LLA) of its origin, called
the world datum. From that point, MuJoCo and the ROS `map` frame share the same
east-north-up coordinates in metres. A position therefore has the same XYZ
values in the simulated world and in ROS. Worldgen stores the datum and
physical robot spawns in the MuJoCo XML, then stores the same datum, homes, and
waypoints in the matching ground and air YAML files.

At runtime MuJoCo turns simulated motion into sensor data, including noisy
GPS. ROS owns the mission and selects where each robot should go. Nav2 drives
the Husky. PX4 estimates the X2 from its simulated sensors, then stabilizes and
controls it while following ROS targets. The Husky uses simulated wheel
encoders, IMU, and GPS through ROS `robot_localization`; the X2 runs estimation
and control through PX4. For startup, coordinates, data flow, and current
simulation limits, see the
[simulation architecture](docs/sim-architecture.md). For why responsibilities
are divided between ROS, Nav2, and PX4, see the
[autonomy architecture](docs/autonomy-architecture.md).

## Install

Complete the [installation and BlenderMCP configuration guide](docs/install.md)
before using the commands below. It contains the exact macOS setup used for
SPAR and a shorter Linux equivalent. This README assumes Docker is running,
`uv sync` has completed, and BlenderMCP is connected.

## Generate a world

```text
 WORLD (output identity) + SEED (repo-owned choices) + BRIEF (supported override)
                                      |
                                      v
             recorded family recipe, stage seeds, and source hashes
                                      |
                                      v
       16 LLM-to-BlenderMCP calls, one per stage, sharing one visible Blender scene
          plan -> build -> detail/materials -> render -> ground/air routes
                                      |
                                      v
                         deterministic export
                                      |
                                      v
       MJCF + visual/collision meshes + spawn/home and route YAML
                                      |
                                      v
                   validated MuJoCo world and robot routes
```

Choose a new world name and run the complete visible authoring, export, and
validation pipeline:

```bash
make worldgen WORLD=utility_depot_trial_01 SEED=42 \
  BRIEF='Denser west-side storage and a more weathered shed.'
```

The command opens clean Blender, runs the family's bounded prompts in order,
exports the final scene and routes, and validates the resulting MuJoCo world.
`WORLD` is only the output identity. `SEED` controls repeatable repo-owned
sampling, including default robot spawn/home poses. `BRIEF` supplies supported
family-instance overrides and may request an approximate placement or give
spawn coordinates (for example, `Spawn the Husky at (3, -6) facing north`).
The topology LLM interprets coordinate requests and the exporter validates the
result for safety, but exact equality is not yet enforced by deterministic
code. Both inputs are optional and recorded with the generated artifacts.
World names are normalized to lowercase; other characters remain limited to
letters, digits, and underscores for filesystem and MJCF identifiers.
It does not require manual approval between stages. Family development,
individual stage commands, checkpoint inspection, resume procedures, and
debugging are documented inside the [world-generation subsystem](worldgen/README.md).
The research motivation is in [docs/research.md](docs/research.md).

`utility_depot_40_v2` is the repository default and committed runnable
showcase, including its MJCF, OBJ/PNG assets, and ground and air world configs.

## Ground quickstart: default utility depot

This path runs the tested `utility_depot_40_v2` world and its matching autonomy
configuration by default. `blank.xml` remains available as a small explicit
fixture via `WORLD=blank` and `world:=blank`.

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

The air image includes PX4 and takes longer to build. The current air demo
flies an explicit 3D route from the selected world's YAML file; it does not
avoid obstacles dynamically. Export checks that route against the world's
collision geometry.

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

## Editing the autonomy code

The main packages are:

```text
ground/src/spar_ground     ground behavior, localization, battery, launch
air/src/spar_air           air behavior and PX4 transforms
common/src/spar_perception shared Detection message and red-object detector
sim/spar_sim               MuJoCo stepping, rendering, sensors, transport
```

Behavior trees are XML in each robot package. Runtime parameters and launch
files live beside their code.

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
| Generate, export, and validate a new world | `make worldgen WORLD=<name> [SEED=<n>] [BRIEF='...']` |
| Start or stop the required headless MuJoCo simulation | `make start_sim [WORLD=<name>]`, `make stop_sim` |
| Optionally watch the running simulation in native MuJoCo | `make view [WORLD=<name>]` |
| Optionally inspect a world in native MuJoCo without ROS | `make inspect [WORLD=<name>]` |
| Enter the running ground container | `make shell` |
| Enter the running air container | `make shell_air` |
| Open RViz in a browser | `make rviz` |
| Run ground or air behavior checks | `make smoke`, `make smoke_air` |
| Stop all containers | `make shut_down` |
