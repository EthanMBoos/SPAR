---
name: verify
description: Run and verify changes to this repo end to end without user help - container C++ build and tests, the sim self-check, robot-aware world lint, headless sim, the smoke test, and a screenshot-based rviz visual check. Use after any code, world, or script change, and for physics experiments in a scratch workspace.
---

# Verifying SPAR autonomously

Every check here runs unattended. Never ask the user to click anything;
every step has a headless path. Run the cheapest check that covers the
change, and always finish a session of edits with the checks in "Which
checks for which change" below.

## Ground rules

- Use absolute paths in shell commands. The repo root is the directory
  containing this file's `.claude/`. A `cd` in one command does not reliably
  persist, and can even reset mid-session.
- Put every temporary file (test worlds, probe output, render output)
  in the session scratchpad directory, never in the repo tree and never in
  `/tmp` directly.
- Read the Makefile before inventing a command. Most workflows are one
  target; the raw commands below exist only where no target does.
- Never commit. Leave everything in the working tree for the user.
- IDE/clangd diagnostics on C++ files are noise on this machine (no ROS
  include paths on the host). The container build is the only authority.
- Never run `make ros2_container` or `make ros2_container_air` from
  here. It ends by exec-ing into an interactive `docker exec -it` shell for
  a human to drive the robot stack from by hand (that's the point of it);
  with no TTY attached, it hangs. Bring the containers up with the raw
  `docker compose` command instead (see check 1), and bring up the robot
  stack with `docker exec -d ... ros2 launch spar_ground
  autonomy.launch.py` (see check 5). This is the one place a fully
  automated, hands-off bring-up belongs at all; the normal dev loop is
  deliberately manual.
- The sim and the host viewer must load the same mujoco. Version skew
  produces subtle model/render drift, and both sides pin 3.10.0 (the
  Dockerfile pip pin and the repo venv). One-line check when in doubt:

  ```bash
  docker exec spar-sim python3 -c 'import mujoco; print(mujoco.__version__)'
  .venv/bin/python -c 'import mujoco; print(mujoco.__version__)'
  ```

## The checks, cheapest first

### 1. C++ build (container)

Required after any change under `src/`. The container must be up:

```bash
docker ps --filter name=spar --format '{{.Names}} {{.Status}}'
```

If the daemon itself is down: `open -a Docker`, then poll `docker info`
until it answers (up to ~60 s). If the containers are down, start them
directly (not `make ros2_container`, see the ground rule above):

```bash
mkdir -p ground/build ground/install logs
docker compose -f docker/compose.yaml up --build -d
```

Nothing auto-launches, the containers just idle (see docker/entrypoint.sh).
Build the same way a user would, with colcon:

```bash
docker exec spar bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ws && colcon --log-base /ws/build/log build --packages-up-to spar_ground --symlink-install'
```

Success includes `Finished <<< spar_perception` and `Finished <<< spar_ground`
with exit 0. Note the flag
order: `--log-base` must come BEFORE the `build`/`test` subcommand.

### 2. C++ unit tests (container)

Required after any change to the BT node types (`src/bt/`) or their tests
(`test/test_bt_nodes.cpp`). The tree engine itself is BehaviorTree.CPP,
tested upstream; these tests cover only the logic this repo still owns
(staleness, hysteresis, cooldowns). Runs in under a second:

```bash
docker exec spar bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ws && colcon --log-base /ws/build/log test --packages-select spar_ground && colcon --log-base /ws/build/log test-result --verbose'
```

Success is `0 errors, 0 failures` in the summary line.

### 3. Sim self-check

Required after any change under `sim/`. Seconds, not minutes: imports every
sim module, loads the world, resolves the named ids the sim and the PX4
link depend on,
and renders one frame per camera through EGL (the same path the sensor
uses). No ports, no ROS spin-up:

```bash
docker exec spar-sim bash -lc 'source /ws/scripts/env.sh && cd /ws/sim && MUJOCO_GL=egl python3 -m spar_sim.check'
```

Success is `[spar] check ok: blank.xml` and exit 0. A rename anywhere in
the MJCF/python contract (body, site, actuator, camera names) fails here
loudly instead of five minutes into a smoke run.

### 4. World lint

Required after any change to `sim/worlds/`, `sim/robots/`, or to
`scripts/lint_world.py` / `sim/spar_sim/robot_config.py`.

Run the repository gate against the selected robot:

```bash
make lint
```

Success is the exact final line
`[lint] OK: blank.xml (4 static solids, 27 geoms total)`.

Changes to robot custom configuration or scan-site lookup also need two
scratchpad probes:

- A compiled model with two custom keys and scan sites at different heights;
  resolve both robot names and prove each selects its own site independent of
  declaration order.
- An undeclared robot; it must fail and name the missing
  `<robot>.scan_site` key.

### 4a. World generator

Required after changes to `scripts/generate_world.py` or its tests:

```bash
.venv/bin/python -m py_compile scripts/generate_world.py scripts/lint_world.py
.venv/bin/python -m unittest scripts/test_generate_world.py
```

Prompt, schema, review-loop, or Ollama integration changes also need a live
run of the same small fixed prompt set with the intended local model. Record
the model tag, elapsed time, successful attempt number, and lint result.
Treat a failure as evidence for a larger model only when it repeats on the
fixed prompts and belongs to model planning or schema following. Missing
meshes, terrain, arbitrary placement, or visual review are pipeline limits,
not evidence that a larger text model is needed.

### 5. Headless sim + smoke test (full end to end)

Required after sim/sensor changes, tree structure changes, or before any
sign-off. This is the one place a fully automated, hands-off bring-up of the
whole stack belongs. The normal dev loop is deliberately manual
(`make ros2_container` drops a human into a shell to run
`autonomy.launch.py` by hand; localization, Nav2, and behavior are all one
launch, one thing to bring up), so this check can't lean on it and has to
bring everything up itself.

Order is load-bearing: the sim FIRST, then the ROS side. The sim owns
/clock, and starting it jumps sim time backward, which would deactivate
Nav2 lifecycle servers ("Action server is inactive") if they were already
up when it happened; starting fresh after the sim is already running
sidesteps that instead of racing it.

**Always recompile before smoke, even with zero C++ changes, and always
relaunch `autonomy.launch.py` fresh: never reuse one left running from a
previous check, and never restart it with a bare `pkill` (it isn't run
under `setsid`; a plain `pkill` orphans its children instead of killing
them). Always go through `make shut_down` for a clean teardown, full
containers included.** A stale binary or a stale process both produce a
false PASS against code that isn't the code you're verifying.

```bash
make shut_down    # clean slate, whatever was running before
mkdir -p ground/build ground/install logs
docker compose -f docker/compose.yaml up --build -d # nothing builds or launches yet (never `make ros2_container`, see ground rules)
make start_sim
sleep 10 && grep -m1 "sim ids ok" logs/sim.log   # retry until it appears
grep -m1 "camera sensor mounted" logs/sim.log
docker exec spar bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ws && colcon --log-base /ws/build/log build --symlink-install'
docker exec -d spar bash -lc 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 launch spar_ground autonomy.launch.py'
# bash -lc matters here, not bash -c: ROS_LOG_DIR is exported via
# /etc/profile.d (see docker/entrypoint.sh), which only login shells source

make smoke         # ~4 min; run in background and poll if doing other work
make stop_sim      # leave the machine as found; the containers are fine to leave running
```

Success is the final line `[smoke] PASS: idle -> start -> rounds -> low
battery -> dock -> recharge -> rounds -> stop`. Any earlier `[smoke] FAIL`
names the step; the per-run logs are in the newest `logs/runNNN/`, and the
sim's own log is `logs/sim.log`.

### 6. rviz visual check (screenshot)

Required after changes to `ground/src/spar_ground/rviz/spar.rviz` or
`scripts/rviz.sh`; worth running after any perception/TF/costmap change too,
since checks 1-5 confirm the topics and logs are right but not that the
picture is. Needs check 5's stack up first (`ros2 launch` and the sim both
alive) — rviz has nothing to draw otherwise.

`imagemagick` (for `import`) is deliberately not in the Dockerfile — it's an
assistant-only verification tool, not something students need (they have
noVNC and a real browser). Install it ad hoc; it's small and doesn't survive
a container recreate:

```bash
docker exec spar bash -lc 'apt-get update -qq && apt-get install -y -qq --no-install-recommends imagemagick >/dev/null 2>&1'
docker exec -d spar /ws/scripts/rviz.sh
sleep 8   # Xvnc + rviz2 + the first costmap swatch
docker exec spar bash -lc 'DISPLAY=:1 import -window root /ws/logs/rviz_check.png'
docker cp spar:/ws/logs/rviz_check.png <scratchpad>/rviz_check.png
```

Read the PNG back with the Read tool (it renders images directly). Success:
the Displays panel's `Global Status` reads `Ok`, not an error like `Frame
[map] does not exist`; and whatever the change should affect is visibly
right — a TF triad that moves between two screenshots a few seconds apart
for a localization/sensor change, a costmap blob in the right place for a
costmap tuning change, a new display actually rendering (not just listed)
for an rviz config change.

## The air track checks

### A1. Air C++ build

Required after any change under `air/src/`. Same shape as check 1, in the
air container (start it with `--profile air`; a bare compose up never
builds PX4):

```bash
mkdir -p air/build air/install logs/air
docker compose -f docker/compose.yaml --profile air up --build -d
docker exec spar-air bash -lc 'source /ws/scripts/env.sh && cd /ws && colcon --log-base /ws/build/log build --symlink-install'
```

Success is `Finished <<< spar_air`. The px4_msgs underlay is baked into
the image; the workspace never builds it.

### A2. Air smoke (full end to end)

Required after changes to `sim/spar_sim/px4_link.py`, `air/src/**`,
`docker/Dockerfile.air`, the air compose service, or the air yaml; and
after world/robot changes that touch the drone or the pad. Order is
load-bearing exactly like check 5: the sim FIRST, then PX4, then the
launch. PX4 must start after the sim is listening (its simulator_mavlink
dials out to the sim's :4560 on localhost, shared network namespace, and
the sim clock must not rewind under it).

```bash
make shut_down
mkdir -p ground/build ground/install logs air/build air/install logs/air
docker compose -f docker/compose.yaml --profile air up --build -d
make start_sim
# wait for "sim ids ok" and "px4 link ids ok" in logs/sim.log
docker exec spar-air bash -lc 'source /ws/scripts/env.sh && cd /ws && colcon --log-base /ws/build/log build --symlink-install'
docker exec -d spar-air bash -c 'cd /opt/px4/build/px4_sitl_zenoh && PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none_iris PX4_SIM_HOSTNAME=localhost ./bin/px4 -d > /tmp/px4.log 2>&1'
# wait for "px4 lockstep engaged" in logs/sim.log, then:
docker exec spar-air bash -c 'cd /opt/px4/build/px4_sitl_zenoh && ./bin/px4-zenoh start'
docker exec -d spar-air bash -lc 'source /ws/scripts/env.sh && ros2 launch spar_air air.launch.py'
make smoke_air         # ~4-6 min; phase 1 alone waits out PX4 boot + EKF2
make stop_sim
```

Success is the final `[smoke] PASS: idle -> start -> takeoff -> ...`
line. The smoke's phase 1 timeout is generous on purpose: a type-hash or
zenoh-protocol drift in the three Dockerfile.air pins shows up here as
phase 1 timing out with the topics still listed.

## Physics experiments in the scratchpad

The repo venv has mujoco (`.venv/bin/python`). To test a hypothesis about
MuJoCo semantics or robot configuration, write a minimal MJCF world in the
scratchpad and run the real lint/config code against it:

```bash
.venv/bin/python - <<'EOF'
import sys
sys.path.insert(0, "<repo>/sim")
import mujoco
from spar_sim.robot_config import scan_site
m = mujoco.MjModel.from_xml_path("<scratchpad>/test_world.xml")
# interrogate m / call scan_site(m, "<robot>")
EOF
```

Two conventions the test worlds must respect: `<robot>.scan_site` names the
site used by that robot, and static world membership is `body_weldid == 0`
(see `static_collision_geoms` in `lint_world.py`). Prove a claim with a
minimal world before changing the scripts, and keep the world as the
regression check after.

## Which checks for which change

| Change | Required checks |
| --- | --- |
| BT node types / leaves (C++) | 1, 2, and 5 if behavior changed |
| behavior_trees/main_tree.xml (tree shape) | 1 (it's installed by CMake), then 5 |
| common/src/spar_perception / battery_sim | 1, then 5 |
| sim/spar_sim/** (sim node, sensors) | 3, then 5 |
| sim/spar_sim/px4_link.py | 3, then A2 |
| sim/worlds/* or sim/robots/* | 3, 4, 5; A2 if the drone or the pad moved |
| sim/viewer.py | 3; no smoke needed, it is read-only |
| lint_world / robot_config | 4 + a scratchpad probe world |
| generate_world.py / test_generate_world.py | 4a; add the live fixed-prompt run when model-facing behavior changed |
| autonomy.launch.py / nav2.yaml / autonomy.yaml / compose.yaml / entrypoint.sh / scripts/*.sh | 5 (its bring-up always tears down and starts the whole stack fresh, so this covers any of these; configs are symlinked, no rebuild needed for yaml-only changes) |
| scripts/rviz.sh / spar.rviz | 6 (needs 5's stack up first) |
| air/src/** (spar_air) | A1, then A2 if behavior changed |
| Dockerfile / Dockerfile.air / air compose service / air yaml / core.sh / smoke_test_air.sh | A2 |
| Docs / comments only | none, but grep that referenced files/names still exist |

A full sign-off pass is 1 through 5 in order. Report results plainly: what
ran, the exact pass/fail evidence (the PASS line and error count), and
anything skipped with the reason.
