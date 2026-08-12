# World generation

Generate, export, and validate a world from the repository root:

```bash
make worldgen WORLD=utility_depot_trial_01 SEED=42 \
  BRIEF='Denser west-side storage and a more weathered shed.'
```

The run owns one clean visible Blender process, executes each family prompt in
a fresh Sonnet-medium session, then exports and validates MJCF, assets, and
routes. It refuses to overwrite `artifacts/worldgen/<world>`; pass `--fresh`
only for an intentional replacement.

`WORLD` is identity, not entropy. `SEED` is optional deterministic entropy for
repo-owned choices; exact sampled spawn/home defaults are recorded before the
LLM runs, though the seed cannot make Claude's authored scene identical.
`BRIEF` is an optional supported instance override and may place a robot or
dock approximately or request explicit coordinates. The topology LLM
interprets coordinate requests; export verifies safety and internal
consistency, but deterministic code does not yet enforce coordinate equality.
Safety and family constraints still win. The resolved recipe, rendered
prompts, and raw stage traces are saved under `artifacts/worldgen/<world>/`.
World names are normalized to lowercase; letters, digits, and underscores are
the portable identifier set.

For comparisons, hold the brief and vary seeds to sample a family; hold the
seed and vary the brief to isolate a requested semantic change.

## Debug a family

Run stages individually against one Blender window:

```bash
export WORLD=utility_depot_family_check_01
uv run python -m worldgen.stage "$WORLD" --clean
uv run python worldgen/start_blender.py
uv run python -m worldgen.stage --list --family utility_depot
uv run python -m worldgen.stage "$WORLD" <stage> --family utility_depot \
  --seed 42 --brief 'Denser west-side storage'
```

The first stage records the recipe. Later manual stages reload it; omit the
seed and brief unless verifying that they match. Resume rejects changed inputs,
prompts, or generator source; start a fresh world after tuning them.

Useful controls:

```bash
uv run python -m worldgen.stage "$WORLD" <stage> --dry-run
uv run python -m worldgen "$WORLD" --stop-after <stage>
uv run python -m worldgen "$WORLD" --start-at <stage> --reuse-blender
uv run python -m worldgen "$WORLD" --dry-run
```

`--reuse-blender` uses the scene currently open; it never selects a checkpoint.
A failed run leaves Blender open at the last valid stage.

| Stage | Inspect |
| --- | --- |
| `01_plan_topology` | Boundary, gate, aisles, and seeded/brief-overridden spawn sites |
| `02_plan_infrastructure` | Continuous fence, setbacks, grounded anchors |
| `03_plan_storage` | Traversable density, cluster variation, anomaly placement |
| `04_build_site_shell` | Ground size, gate opening, spawn sites |
| `05_build_structures` | Anchor fit, openings, separate physical elements |
| `06_build_racks` | Local transforms, shelf/load fit, clear aisles |
| `07_build_drums` | Grounding, grouping, one identifiable red target |
| `08_build_pallets_crates` | Grounding, parenting, route clearance |
| `09_build_utility_props` | Anchor fit, plausible assemblies |
| `10_finish_blockout` | Complete composition, circulation, no plan guides |
| `11_detail_infrastructure` | Recognition improved without footprint changes |
| `12_detail_props` | Shape detail without hierarchy/collision regressions |
| `13_materials_lighting` | Full material coverage, neutral light, visible target |
| `14_render_final` | Coherent review renders and `scene_manifest.json` |
| `15_ground_waypoints` | Authored route; waypoint clearance plus early target LOS, range, and yaw |
| `16_air_waypoints` | Collision clearance, target view, return over spawn |

Export an accepted waypoint scene directly:

```bash
/Applications/Blender.app/Contents/MacOS/Blender --background \
  "$PWD/artifacts/worldgen/$WORLD/waypoints.blend" \
  --python-exit-code 1 --python "$PWD/worldgen/export.py" -- \
  --world "$WORLD" --world-waypoints
uv run python worldgen/check_export.py "$WORLD"
```

Use the installed `blender` executable on Linux. After shared-helper changes:

```bash
/Applications/Blender.app/Contents/MacOS/Blender --background \
  --factory-startup --python worldgen/check_blender_helpers.py
```

## Change the right layer

| Failure | Owner |
| --- | --- |
| One unattractive permutation or one stage overreaches | Stage prompt |
| Repeated asset/layout arithmetic within one context | `families/<family>/helpers.py` |
| Transforms, parenting, sites, or mechanics fail across families | `blender_helpers.py` |
| MJCF, collision, asset, or route contract fails | `export.py` / `check_export.py` |
| BlenderMCP is absent, stale, or attached to the wrong window | `start_blender.py` / `mcp.json` |

Do not prompt around deterministic failures. Do not encode subjective visual
taste as an export gate.

## Extend

The shared Blender helpers and SPAR exporter are the generic kernel. A family
owns domain vocabulary, valid topology, recurring assets, appearance, and task
intent. Its variation contract separates invariants from overridable defaults.
A world is one recipe and its generated artifacts.

For a new family, add ordered, narrowly scoped prompts under
`families/<family>/prompts/`. Add family Python only after deterministic logic
repeats. Reuse the exporter unless the simulator contract itself changes.
Qualify several visible variants, export validation, and both robot smoke tests
before treating the family as automatic.

Editable checkpoints live in `artifacts/worldgen/<world>/`; runnable MJCF,
assets, and route YAML remain under their simulator, ground, and air consumers.
The exporter attaches pose-neutral robot models at the authored sites, writes
the shared world datum and mission homes into ground and air configuration,
and keeps both routes in MuJoCo/world-aligned ENU coordinates. Spawn poses
physically live in MJCF and are not duplicated as localization origins. The
autonomy launch always loads the selected world configuration; autonomy does
not choose the physical spawn.

The utility-depot family intentionally omits ground imagery, HDRIs, and
photographic backgrounds until texture scale and Blender/MuJoCo parity receive
a dedicated realism pass.
