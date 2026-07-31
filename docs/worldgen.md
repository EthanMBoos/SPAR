# World generation

BlenderMCP is the only generated-world authoring path:

```text
description -> BlenderMCP layout and detail -> accepted .blend
            -> explicit collision proxies -> deterministic MJCF export
            -> MuJoCo compile and visual inspection
```

Blender owns layout, visual geometry, materials, and the editable simulation
contract. MuJoCo remains the runtime physics and sensor authority. The
handwritten `sim/worlds/blank.xml` is the minimal default world, not a second
generator.

## Scene contract

An exportable Blender scene contains three collections:

- `SPAR_VISUAL` contains detailed render geometry.
- `SPAR_COLLISION` contains editable box and cylinder proxies with
  `spar_collision_shape` and `spar_source` properties.
- `SPAR_SITES` contains named spawn, dock, and task markers.

All new generated environments are currently 40×40 m. The final waypoint pass
lets BlenderMCP inspect the completed scene and place an ordered ground route
through its aisles and an ordered 3D air route above its obstacles. Nav2 plans
between ground goals. The current air behavior flies directly between its
goals and does not avoid obstacles dynamically, so export rejects an air route
that is not safely above the collision geometry. Behaviors that dynamically
plan their own routes can omit this pass entirely.

Visual geometry never collides in MuJoCo. The exporter converts it to OBJ
chunks grouped by material. Collision objects become native MJCF box and
cylinder geoms instead of collision meshes. This keeps detailed fences, racks,
and clutter from turning into incorrect convex hulls.

## Utility-depot demo

The reusable authoring prompts are in `prompts/utility_depot.md`. The current
40×40 m demo's editable Blender files live locally under
`artifacts/worldgen/utility_depot_40_v1/`. Its runnable MJCF, visual assets, and
ground and air waypoint files are committed as the repository showcase.

On macOS, run Blender through LaunchServices:

```bash
open -n -W -a Blender --args --background \
  "$PWD/artifacts/worldgen/utility_depot_40_v1/final.blend" \
  --python "$PWD/scripts/export_blender_world.py" -- \
  --world utility_depot_40_v1
```

On other platforms, invoke the Blender executable directly with the arguments
after `--args`.

The first export creates conservative proxies, adds the semantic sites, and
saves `simulation.blend` beside `final.blend`. Inspect or edit the green
wireframe proxies there. Re-export from `simulation.blend` to preserve those
edits:

```bash
open -n -W -a Blender --args --background \
  "$PWD/artifacts/worldgen/utility_depot_40_v1/simulation.blend" \
  --python "$PWD/scripts/export_blender_world.py" -- \
  --world utility_depot_40_v1
```

To use Blender-authored waypoints, run the waypoint prompt in the recipe and
export its `waypoints.blend` with the optional flag:

```bash
open -n -W -a Blender --args --background \
  "$PWD/artifacts/worldgen/utility_depot_40_v1/waypoints.blend" \
  --python "$PWD/scripts/export_blender_world.py" -- \
  --world utility_depot_40_v1 --world-waypoints
```

This writes the literal waypoint lists to:

```text
ground/src/spar_ground/config/worlds/utility_depot_40_v1.yaml  [x, y, yaw, ...]
air/src/spar_air/config/worlds/utility_depot_40_v1.yaml        [x, y, z, yaw, ...]
```

Pass the same world name when launching either waypoint-driven stack:

```bash
ros2 launch spar_ground autonomy.launch.py world:=utility_depot_40_v1
ros2 launch spar_air air.launch.py world:=utility_depot_40_v1
```

The committed showcase consists of:

```text
sim/worlds/utility_depot_40_v1.xml
sim/worlds/assets/utility_depot_40_v1/*.obj
sim/worlds/assets/utility_depot_40_v1/*.png
sim/worlds/assets/utility_depot_40_v1/export_manifest.json
ground/src/spar_ground/config/worlds/utility_depot_40_v1.yaml
air/src/spar_air/config/worlds/utility_depot_40_v1.yaml
```

Other generated worlds and all editable `.blend` files remain ignored until a
world is deliberately promoted as another shared showcase or fixture.

Compile and inspect the result:

```bash
uv sync --managed-python
uv run python scripts/check_world_export.py utility_depot_40_v1
uv run mjpython sim/inspect_world.py --world utility_depot_40_v1  # macOS
# On Linux: uv run python sim/inspect_world.py --world utility_depot_40_v1
make start_sim WORLD=utility_depot_40_v1
```

`check_world_export.py` checks referenced files, finite proxy transforms,
supported proxy shapes, required sites, the explicit air route's altitude and
pad clearance, and MuJoCo compilation. It reports scene statistics but enforces
no arbitrary geometry budget.

## Current limits

The exporter carries base colors, UVs, and image textures that are directly
usable by MuJoCo. Blender node graphs, the HDRI, and Eevee lighting do not map
one-to-one to MuJoCo's renderer. The editable Blender render is therefore the
visual target, while the exported view is the runtime approximation.

The v1 world is static. Pallets, the open gate, tire marks, loose gravel, and
small ground clutter are visual-only. Add collision only when an object should
affect robot motion or sensors, then inspect the proxy in `simulation.blend`.
