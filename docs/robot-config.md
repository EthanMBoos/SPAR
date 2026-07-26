# Dropping indoor assumptions

Status: not started. Blocks `docs/worldgen.md`.

Two cleanups, one theme. Both are places where the indoor era left robot
knowledge and localization knowledge baked into world processing. Part one
stops sensor geometry being discovered from the world. Part two stops
localization depending on a prior map of it.

# Part one: robots should describe themselves

## The problem

Sensor geometry is a property of a robot. Today it is discovered by
scanning the world.

`rasterize_map.py` picks the height to slice the world at by walking every
site in the compiled model and taking the first whose name starts with
`lidar`. `lint_world.py` imports that function and uses the same height for
its stealth-obstacle check. So the map is computed as a function of the
world, when it is really a function of the world and of the robot that will
read it.

It works today for one reason: the husky is the only robot carrying such a
site. Compiling `blank.xml` gives exactly one, `lidar2d_0_laser` at z=0.567,
and the drone contributes only `thrust1` through `thrust4`. Nothing to
collide with, so nothing to notice.

The failure modes are all one robot away.

- **Two ground robots with different masts.** `lidar_z()` returns whichever
  site MuJoCo happens to order first. Arbitrary, silent, and the map is
  quietly sliced at the wrong robot's height.
- **A robot with no lidar.** `sys.exit("no lidar site in the model")`.
- **3D lidar, or any non-planar sensor.** There is no single slice to take,
  so the premise does not apply at all.
- **One world, two robots, two valid maps.** Not expressible. The map is
  named `maps/<world>.yaml` and the launch file resolves it from the world
  alone.

`sim/spar_sim/sim.py` has the same shape from the other direction, resolving
`base_link`, `lidar2d_0_laser`, `camera_0_link`, `act_left`, `act_right`,
and `camera_0` by hardcoded string. Adding a sensor to a robot means editing
the sim process. `sim/robots/husky.xml:33` even documents the coupling, but
backwards: the robot file explains that a world-processing script depends on
its site name.

## Why now

Because it decides how much the world generator has to know.

If robots stay self-describing, a generated world is geometry and nothing
else, and the generator never grows a notion of mast height or sensor
layout. If they do not, that knowledge leaks into the generator, and it
comes back out later at a worse time. Cheaper to fix six lines now than to
unpick assumptions from generated worlds afterward.

New robots and new sensors are also the near-term direction, and every one
of the failure modes above lands the moment a second robot arrives.

## The shape of the fix

Two changes, neither of which needs a new config format.

**The robot declares its own scan geometry.** MuJoCo's `<custom>` block
carries arbitrary named data through compilation, so a robot MJCF can name
the site it scans from without any file outside the robot definition. Having
the robot name a site is better than repeating the height as a number, which
would drift from the site it describes.

**The robot becomes an explicit input, not an inference.** `rasterize_map.py`
and `lint_world.py` take which robot the map is for, rather than guessing
from whatever the world happened to include. Maps are then addressable per
world and robot, and one world can legitimately have several.

The exact spelling of both, and how map naming should change without
breaking the `maps/<world>.yaml` convention the launch file resolves, wants
a prototype rather than a decision made here.

## What it touches

- `scripts/rasterize_map.py`, `lidar_z()` and the CLI.
- `scripts/lint_world.py`, which imports `lidar_z` and `static_collision_geoms`.
- `sim/robots/husky.xml`, to declare its scan site; `x2.xml` if the drone
  ever needs a map.
- `sim/spar_sim/sim.py` and `check.py`, for the hardcoded id resolution.
- `Makefile`, the `map` target.
- `ground/src/spar_bringup/launch/autonomy.launch.py`, which resolves
  `maps/<world>.yaml` from the world alone.
- `.claude/skills/verify/SKILL.md`, check 4 and the change table.

## The bar

The blank world's map must come out byte-identical. It is a pure refactor:
the husky's scan height is 0.567 m before and after, so `blank.pgm` and
`blank.yaml` should not move by a single byte. The verify skill already
treats that as the regression bar for any change to these two scripts.

Beyond that, a scratchpad world with two robots at different mast heights,
proving the map follows the robot you asked for instead of model order.

# Part two: the indoor stack goes

AMCL matching scans against a rasterized occupancy grid is an indoor
technique. We are building outdoor worlds. Assume outdoor, because it is the
harder case and it is the real one, and stop carrying a localization stack
that only works on the easy case we are leaving.

## What goes

- `scripts/rasterize_map.py`, the `map` target in the `Makefile`, and
  `ground/src/spar_bringup/maps/`.
- `amcl` and `map_server` from `localization.yaml`.
- `static_layer` from both costmaps in `nav2.yaml`. The global costmap is
  currently map-sized and map-backed; the local one carries a static layer
  it barely uses.

## What replaces it

**Pose from fusion, not from scan matching.** The air track already has the
pattern: `tf_from_px4.cpp` publishes `map -> base_link` from EKF2,
reconciled to the pad. The ground robot gets the same shape. In sim the
honest first version is publishing `map -> odom` from sim ground truth, a
perfect GPS stand-in, and degrading it with noise once the rest works. That
is a smaller change than it sounds and it removes the scan-matching
dependency immediately.

**A rolling global costmap.** `rolling_window: true` in the `map` frame, fed
by the lidar obstacle layer alone. Standard Nav2 mapless configuration, no
new plugin. Three details are load-bearing and none of them are optional.

*The global costmap currently has no size of its own.* It declares no
`rolling_window` and no `width`/`height`, because the static map supplies
its extent. Removing the static layer without adding both leaves the
costmap with no extent at all and NavFn cannot plan. This is a hard break,
not a degradation, and it is the first thing to get right.

*The obstacle ranges are sized for a world that also had a map.* The global
costmap uses `obstacle_max_range: 2.5` and `raytrace_max_range: 3.0` while
the sim lidar is 720 rays out to 25 m. That split was reasonable when the
static layer carried the structure and the obstacle layer only added nearby
dynamics. Once it is the only input, those numbers throw away most of the
sensor and leave the global planner blind past 2.5 m. Raise them toward
sensor range.

*Unknown space changes meaning.* `track_unknown_space: true` and NavFn's
`allow_unknown: true` are bounded and sensible against a map. Mapless,
almost everything is unknown, so the global plan becomes an optimistic
straight line and obstacles get discovered locally. That is correct mapless
behavior, but it is a real behavioral change: expect different recovery
patterns and more local replanning.

The BT and the anomaly detector need no change. They want a consistent `map`
frame, not a map. `battery_sim` is fine for the same reason: it only does a
`map -> base_link` lookup to decide it is docked. `smoke_test.sh` never
asserts on the map or on AMCL, only on the `navigate_to_pose` action and BT
leaf states, so it stays valid as written.

## What this does to the rest

`lint_world.py`'s stealth check survives and matters more. Without a prior
map, geometry the live sensor cannot see is a collision with no warning at
all, where before it was at least absent from a map you could inspect. The
check stops meaning "will the map miss this" and starts meaning "can the
robot see this", which is the question worth asking. It still needs the
robot's scan height, which is exactly what part one provides. The two halves
of this doc meet here.

The byte-identical map regression in the verify skill goes away with the
map, and check 4 needs rewriting around the lint gate alone.

For worldgen the map contract evaporates entirely. No slice height, no floor
plane sized for map bounds, no `make map`, no per-world autonomy yaml before
you can lint. A generated world becomes geometry, which is the whole point.

## Cost

This is surgery on a working stack, and it is the larger half of this doc.
`smoke_test.sh` drives the full arc through Nav2 and will not pass again
until the new pose source is publishing. Do it deliberately, with the smoke
test as the gate, and expect the pose source and the costmap change to land
together rather than separately.

Heightfields and real terrain are still further out. This work removes the
premise that blocks them (a single lidar plane read once at a fixed z), but
it does not by itself make sloped ground navigable.
