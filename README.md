# SPAR - Sim Portable Autonomy Runtime

Robot autonomy is advancing fast on two fronts, RL and VLAs. The
capabilities are real; the reliability needed for long-horizon,
unsupervised work in the wild is not. SPAR is a runtime for hybrid
autonomy: a deterministic behavior tree owns the mission, and learned
components slot in under it (a trained policy as a behavior node, a VLM
as the perception source) without the rest of the stack changing.

The sim is **pure MuJoCo**: one Python process steps the physics,
renders the cameras, and publishes the robot's ROS topics. Nothing is
vendor-locked the way Isaac Sim is. One world file runs three ways on
the same physics and sensors: headless in the container for the full
stack, in a native host viewer window for watching, and as a
pure-MuJoCo Gymnasium env for fast RL training, with the full stack as
the eval harness. The robot is configured as it would be in the real
world. A sim-to-real gap remains, but this is the lowest-friction way to
stress a behavior across a wide variety of scenarios.

The bet is that foundation models and VLAs keep improving every year, so
the hard part becomes systems integration: verifying a policy against the
reliability bar real users actually hold. This repo is that testground.
Build scenarios, run regressions against the edge cases, and be able to
say, with evidence, that a policy is good to go before it leaves sim.

Today there are two concrete tracks in the same world. The ground track
is a ROS2 + Nav2 Husky that makes inspection rounds of a site, inspects
anomalies (a red drum) when perception spots them, and returns to its
dock when the battery runs low. The air track is a PX4-flown Skydio X2
that patrols overhead, orbits the same drum when its camera finds it, and
returns to its pad on low battery ("The air track" below). The substrate
spans domains by swapping dynamics, sensors, and planner, not by forking.

Starter repo for the
[GT Cloud Robotics](https://www.gtcloudrobotics.com/course-home/) Autonomy
and LLM tracks. Design rationale lives in
[docs/sim-architecture.md](docs/sim-architecture.md).

The world is an MJCF file (`sim/worlds/blank.xml`). The sim
(`sim/spar_sim/`) and the autonomy stack run in Docker containers sharing
one network namespace. The sim publishes clock, odometry, noisy GPS, lidar,
and camera data and consumes `cmd_vel`. The ROS side turns sensor data into
the `map` transform used by navigation and perception.

## Quickstart

Requirements: [Docker](https://docs.docker.com/get-docker/) with 8 GB+
memory (Docker Desktop, Settings, Resources) and, for the optional host
viewer window, Python 3 (`make view` creates its own venv at `.venv`).

Start the simulated world first, then the robot's software next to it:

```bash
git clone https://github.com/EthanMBoos/spar.git
cd spar
make start_sim       # builds the image, starts the sim headless in its container
make view            # optional: native viewer window on your host
```

From a second terminal, start the autonomy stack's container and build
the code:

```bash
make ros2_container                # starts the container, drops you into a shell

colcon build --symlink-install
source install/setup.bash          # only needed this once; later shells (make shell) source it for you

ros2 launch spar_bringup autonomy.launch.py
```

Ctrl-C and rerun any time. After editing code, rebuild first (see "Working
on the autonomy code" below), then rerun the launch.

The robot boots idle at its dock. Open another shell (`make shell`) to talk
to the mission layer:

```bash
scripts/mission.sh start
ros2 topic echo /husky/bt/status   # active leaf, mission state, battery
scripts/mission.sh stop
ros2 topic pub --once /husky/battery/set std_msgs/msg/Float32 '{data: 10.0}'  # force low battery
```

From a host terminal:

```bash
make smoke     # end-to-end check (~4 min, ends in PASS)
```

`make` lists every command. Each target is one or two lines in the
[Makefile](Makefile).

## The air track

Everything above is the ground track and works unchanged. The air track
is the same world flown by a different stack: PX4 SITL (the real
autopilot: EKF2, position control, failsafes) in its own container, with
a behavior tree above it speaking offboard setpoints. The `_air` make
targets point at it; the first build compiles PX4 from source and takes
a while.

```bash
make start_sim                     # if not already running
make ros2_container_air
colcon build --symlink-install     # in that shell: builds spar_air

# start PX4, in that shell; give it a few seconds to boot before px4-zenoh
cd /opt/px4/build/px4_sitl_zenoh
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none_iris \
  PX4_SIM_HOSTNAME=localhost ./bin/px4 -d > /tmp/px4.log 2>&1 &
./bin/px4-zenoh start              # joins PX4 to the ROS graph

ros2 launch spar_air air.launch.py   # stays in the foreground, like the ground launch
```

Give EKF2 ~10 s to converge after PX4 starts, then the same mission
controls as the ground robot, drone-flavored:

```bash
make shell_air
scripts/mission.sh start
ros2 topic echo /skydio/bt/status
/opt/px4/build/px4_sitl_zenoh/bin/px4-param set SIM_BAT_MIN_PCT 10  # force low battery
```

`make smoke_air` runs the whole arc unattended (takeoff, inspect,
patrol, battery return, land, disarm, relaunch). A mixed demo is both
containers up and both missions started; nothing else to configure.

One trap the two tracks share: if you restart the sim, restart the ROS
stacks after it (relaunch `autonomy.launch.py`, and for the air track
PX4 too). The sim owns the clock, and both Nav2 and PX4 react badly to
time rewinding under them.

## Working on the autonomy code

`ground/src/spar_ground/`:

```
src/bt/                          BT.CPP node types: conditions, staleness helpers
src/leaves/                      leaves that own Nav2 action calls
src/bt_executive.cpp             registers node types, ticks the tree at 10 Hz
src/anomaly_detector.cpp         camera pixels -> map-frame anomaly point
src/battery_sim.cpp              fake BMS: drains, recharges at the dock
src/tf_from_gps.cpp              noisy GPS fixes -> map-to-odom correction
behavior_trees/main_tree.xml     the tree's shape (BehaviorTree.CPP XML)
test/                            gtest for the node types this repo owns
```

Edit on your host; the workspace is bind-mounted, build artifacts land in
`ground/build/` and `ground/install/` (symlinked, no separate install
step). `colcon
build --symlink-install` (Quickstart) is for the first build; after that,
rebuild faster straight through the generated Makefile, in your `make
ros2_container` shell:

```bash
cd build/spar_ground && make
```

Plain `make` reruns cmake itself if needed. Colcon's build dir already
caches its own source path in `CMakeCache.txt`. Ctrl-C the launch and
rerun it to pick up the change.

Behavior parameters (patrol radius, dock pose, battery thresholds, detector
tuning): `ground/src/spar_bringup/config/autonomy.yaml`. Nav2 (speed
limits, costmaps): `config/nav2.yaml`. Both are symlinked into the install
tree, so edits take effect on the next launch, no rebuild needed.

Logs: `logs/run001, run002, ...` at the repo root, one directory per launch,
one file per node plus `run-info` naming the world. The sim's own log is
`logs/sim.log`.

## The environment

The world is an MJCF file: `sim/worlds/blank.xml` is a small inspection
site with storage racks and a red drum.
Edit it with any text editor; obstacles are a few lines of `<geom>`. The
robots are included files (`sim/robots/husky.xml`, `sim/robots/x2.xml`);
their model files own their default spawns and visual home pads. A new world
is only a new file in `sim/worlds/` that includes them. Inspect a world
without the stack with `make inspect WORLD=blank`.

Or generate one small primitive world from a description with a local Ollama
model, then inspect it before deciding whether to launch it:

```bash
make lint
.venv/bin/python scripts/generate_world.py --name loading_yard --model gemma3:4b \
  "A compact loading yard with fencing, crates, and a red hazard drum"
make inspect WORLD=loading_yard
```

The full constrained workflow is in
[docs/worldgen.md](docs/worldgen.md).

Ground and air behavior use reusable robot configs and generate their patrols
around the model-owned homes. Validate a world's geometry and procedural
ground route against the Husky's declared scan site before running it:

```bash
make lint
```

The navigation costmaps are rolling and built from the live scan; there is
no generated occupancy map.

## Reference

| Thing | Where |
| --- | --- |
| Build the code (first time, or a full rebuild) | `colcon build --symlink-install` inside the shell (then `source install/setup.bash` if that shell predates the build) |
| Bring up the stack | `ros2 launch spar_bringup autonomy.launch.py`, from a `make ros2_container`/`make shell` |
| Start/stop the mission | `scripts/mission.sh start` / `stop`, from a `make shell` |
| Behavior status feed | `ros2 topic echo /husky/bt/status`, from a `make shell`; JSON: active leaf, mission, battery |
| Live node logs | `make tail` (`/rosout`, every node merged), from a host terminal (fails if nothing is launched) |
| rviz2 | `make rviz`, from a host terminal — opens a browser tab (noVNC); Ctrl-C stops it |
| Rebuild after code edits | `cd build/spar_ground && make` inside the shell (fast, after the first build above) |
| End-to-end test | `make smoke` (needs the sim running and `autonomy.launch.py` up) |
| Stop and remove the containers | `make shut_down` |
| Clean rebuild | `make clean` (shuts down, removes `build/` + `install/`), then `make ros2_container` |
| Logs of past runs | `logs/runNNN/`, one per launch |
| The sim | `sim/spar_sim/` (physics, sensors, PX4 link), started by `make start_sim` |
| Worlds and robots | `sim/worlds/*.xml`, `sim/robots/*.xml` (MJCF) |
| Validate a world | `make lint` (`ROBOT=husky` and `WORLD=blank` by default) |
| Inspect a world without ROS | `make inspect WORLD=blank` |
| Watch the sim | `make view` (native viewer on the host, read-only) |
| Perception | cameras render in the sim (EGL, headless); the containerized detector turns pixels into labeled points on `perception/detections` |
| Behavior config | `ground/src/spar_bringup/config/autonomy.yaml` |
| Nav2 config | `ground/src/spar_bringup/config/nav2.yaml` (speed: see `vx_max` comments) |
| The air stack | `air/src/spar_air` (BT + TF + detector), the `make *_air` targets |
| Air behavior config | `air/src/spar_air/config/autonomy.yaml` |
| PX4 pins and the topic mapping | `docker/Dockerfile.air`, `docker/air/{pub,sub}.csv` |
| Why it's built this way | [docs/sim-architecture.md](docs/sim-architecture.md) |
| Using the sim for RL | [docs/rl.md](docs/rl.md) |
