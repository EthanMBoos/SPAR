---
name: verify
description: Verify SPAR worldgen, the consolidated ground autonomy package, simulator, perception, Nav2, and the complete mission lifecycle.
---

# Verify SPAR

Run checks unattended and never commit. The simulator must start before the ROS
launch because it owns `/clock`.

## 1. Host tests and syntax

```bash
PYTHONPATH=sim uv run --locked python -m unittest worldgen.test_generation sim.test_husky sim.test_georeference
uv run --locked python -m py_compile worldgen/export.py worldgen/check_export.py sim/spar_sim/*.py scripts/nav_goal.py
uv run --locked python worldgen/check_export.py utility_depot_40_v2
```

## 2. Containers and ROS build

Do not run `make dev` in automation because it opens an interactive shell.

```bash
mkdir -p ros/build ros/install logs
docker compose -f docker/compose.yaml up --build -d
docker exec spar bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ws && colcon --log-base /ws/build/log build --packages-up-to spar --symlink-install'
docker exec spar bash -lc 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && cd /ws && colcon test --packages-select spar && colcon test-result --verbose'
```

## 3. Simulator self-check

```bash
docker exec spar-sim bash -lc 'source /ws/scripts/env.sh && cd /ws/sim && MUJOCO_GL=egl python3 -m spar_sim.check'
```

Success ends with `[spar] check ok`.

## 4. Full mission autonomy smoke test

Use a fresh launch. Do not reuse processes that survived an earlier run.

```bash
make down
mkdir -p ros/build ros/install logs
docker compose -f docker/compose.yaml up --build -d
make sim WORLD=utility_depot_40_v2
docker exec spar bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ws && colcon --log-base /ws/build/log build --symlink-install'
docker exec -d spar bash -lc 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 launch spar navigation.launch.py world:=utility_depot_40_v2'
make smoke WORLD=utility_depot_40_v2
make stop_sim
```

Success ends with `PASS: idle -> start -> inspect -> rounds -> low battery -> dock -> recharge -> rounds -> stop`.

## 5. RViz changes

With the full stack running:

```bash
docker exec -d spar /ws/scripts/rviz.sh
```

Confirm the global frame is `map`, TF is healthy, and local/global costmaps
render around the Husky. Use the browser-backed display or capture the VNC root
window when an automated screenshot tool is available.

## Coverage

- `worldgen/**`, generated MJCF/config/manifest: checks 1 and 3.
- `ros/src/spar/**`, Docker, launch, or scripts: checks 2 and 4.
- `sim/**`: checks 1, 3, and 4.
- RViz configuration: check 5 after check 4.
- Docs only: verify referenced paths and commands with repository search.
