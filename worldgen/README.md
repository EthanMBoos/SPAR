# World generation

Worldgen turns one environment family, seed, and short brief into a reviewed
Blender scene, MuJoCo assets, a Husky spawn, and collision-checked Nav2 goals.
The authoring stages are narrow and stateless; accepted scene state carries
decisions forward.

## Run the pipeline

```bash
uv run python -m worldgen utility_depot_40_v3 \
  --seed 42 --brief "dense west storage, broad central aisle"
```

Useful controls:

```bash
uv run python -m worldgen.stage --list
uv run python -m worldgen.stage utility_depot_40_v3 05_build_structures
uv run python -m worldgen utility_depot_40_v3 --dry-run
uv run python -m worldgen utility_depot_40_v3 --start-at 11_detail_infrastructure
```

The utility-depot family ends with `15_navigation_goals`. It authors at least
three ordered `navigation_goal_*` sites on connected, clear ground and saves
`artifacts/worldgen/<world>/waypoints.blend`.

## Export and validate

```bash
open -n -W -a Blender --args --background \
  "$PWD/artifacts/worldgen/$WORLD/waypoints.blend" \
  --python-exit-code 1 --python "$PWD/worldgen/export.py" -- \
  --world "$WORLD" --navigation-goals

uv run python worldgen/check_export.py "$WORLD"
```

Export produces:

- `sim/worlds/<world>.xml` and `sim/worlds/assets/<world>/`;
- `ros/src/spar/config/worlds/<world>.yaml` with `navsat_datum`, the authored dock pose,
  and ordered `{name, x, y, yaw}` navigation goals;
- an export manifest recording meshes, colliders, sites, and goal clearance.

Validation requires exactly one Husky spawn, a ground-only MJCF attachment,
matching datum/config/manifest data, finite and in-bounds goals, collision-free
Husky footprints across sampled headings, valid generated OBJ files, a clean
MuJoCo compile, and a finite passive rollout.

## Family contract

Family prompts own creative layout and appearance. Shared helpers own repeated
Blender mechanics. Export code owns file formats and objective checks. Add new
families as ordered prompts first; add family-specific Python only for truly
deterministic logic that prompts should not repeat.

The utility-depot family retains the distinctive red inspection barrel used by
the ground perception and mission smoke test. Its first navigation goal must
provide an early, target-facing view of that barrel. Generated goals feed the
mission's rounds branch, and the authored ground dock feeds ReturnToDock and
charging.
