# SPAR - Sim Portable Autonomy Runtime

```text
  description   "a gravel utility depot with racks, barrels, and a red drum"
       |
       v
   generate     primitive: Ollama -> Python MJCF        (today)
                fidelity:  orchestrator -> BlenderMCP   (planned)
       |
       v
   world.xml    the site alone; robots compose on at load time
       |
       v
      run       SPAR-GroundNav-v0   pure MuJoCo, PPO, no ROS
                sim/spar_sim        Nav2 + BT (ground), PX4 + BT (air)
       |
       v
  each failure becomes the next description
```

SPAR is a small robotics playground for learning ROS2, Nav2, behavior trees,
PX4, and MuJoCo. It is also a testbed for generating many diverse worlds
through an orchestrated LLM-and-Blender pipeline, training RL behaviors across
them, then running and evaluating the learned behaviors inside a deterministic
ROS 2 stack.

Behavior trees are configured for a Husky ground robot with Nav2 and a Skydio
X2 drone controlled by PX4 SITL. Both see the same world via a shared
camera-to-detection interface.

The working behavior is basic.

- patrol while a mission is active;
- inspect the red anomaly detected from the rendered camera;
- return home when the battery is low;
- resume after recharging.

## World generation

Why generate worlds at all? The leading approach to visuomotor policy learning
is uptraining a large pretrained vision-language or video model into an
action-output model, which inherits broad scene understanding and spends its
data budget mapping that to trajectories. It generalizes well in-distribution
and degrades on the tail: rare geometry, unfamiliar clutter, degenerate
lighting. Those configurations are sparse in the pretraining corpus and rare in
teleop. Field data only ever contains failures already encountered, and
collecting it needs a fleet. This pipeline is a bet that compositional
generation with LLMs in the loop can synthesize those out-of-sample
configurations on purpose, cheaply, before deployment surfaces them. See
[the research direction](docs/research.md).

`scripts/generate_world.py` is the low-fidelity baseline: a description in, a
small primitive MJCF world out. A local Ollama model picks semantic layout
fields; Python owns coordinates, geometry, and linting. `--seed` is both the
semantic variation ID and the coordinate jitter seed.

```bash
make lint  # creates .venv and validates the canonical world

.venv/bin/python scripts/generate_world.py \
  --name loading_yard \
  --model gemma3:4b \
  --seed 0 \
  "A compact loading yard with fencing, crates, and a red hazard drum"

make inspect WORLD=loading_yard
```

Generated worlds are local experiments and ignored by git. Ollama approval and
lint approval are recorded, but human visual inspection is the final gate. If
one is useful after inspection, select it when the sim starts:

```bash
make start_sim WORLD=loading_yard
```

The planned fidelity mode has an orchestrator, designer, and critic make
multiple BlenderMCP calls to build and inspect a richer site, exporting into the
same world contract and passing the same lint gates. Not implemented yet.

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
| Validate world geometry and route | `make lint WORLD=blank ROBOTS=husky` |
| Enter the running ground container | `make shell` |
| Enter the running air container | `make shell_air` |
| Open RViz in a browser | `make rviz` |
| Run ground or air behavior checks | `make smoke`, `make smoke_air` |
| Stop all containers | `make shut_down` |
