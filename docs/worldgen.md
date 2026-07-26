# Worldgen

Status: prototyping. No code yet. This is the single reference; start here.

A working sketch, not a spec. The parts that describe existing SPAR code are
facts you can check. Everything about the generator itself is a first guess
and should lose to whatever the prototype actually teaches.

## The goal

Generate outdoor worlds (farms, construction sites, logistics yards) for
`sim/worlds/` from a seed, reject the broken ones automatically, and produce
enough of them that coverage becomes a real question instead of one
hand-authored scene. The whole stack under test on every world: behavior
tree, motion stack, perception.

Today there is one world, `blank.xml`, hand-authored. That gap is the work.

## What we already tried, and why it is out

We vendored [SceneSmith](https://github.com/nepfaff/scenesmith), an agentic
indoor scene generator, and tried to run it.

The problem was not that it is heavy. It is that everything heavy is
mandatory and coupled. There is no path to the simplest possible scene that
does not also require the LLM agent triads, the asset pipeline, Drake,
Blender, and a GPU, all present at once. That coupling is what makes it
hard to install, too: `bpy` and Drake are each fine on their own on macOS
arm64, but SceneSmith pins `bpy==4.5.4`, which forces CPython 3.11, and
Drake publishes no macOS arm64 wheel for 3.11. Two individually satisfiable
constraints that are not jointly satisfiable, so the venv does not resolve
on a Mac at all. Then 24 to 32 GB of VRAM, or 50 to 70 GB of asset
downloads, before the first scene.

So the objection is to the coupling, not to the capability. Build the
lightweight core first, and make every heavy thing a layer that switches
on.

## The path

Build it from first principles in MuJoCo, smallest thing that works.

The loop has to be fast. Emitting and critiquing a candidate world has to
cost milliseconds, not seconds, or the generate-and-reject loop is a thought
experiment rather than a search. Pure MJCF gets that; anything that shells
out to an external tool per candidate does not.

```
prompt or seed
  └── placements (JSON: prop, x, y, yaw, static)
      └── build with mujoco.MjSpec, compile
          ├── critic: interpenetration, settling, clearance, lidar visibility
          ├── retry with the critic's complaints fed back
          └── write sim/worlds/<name>.xml + autonomy_<name>.yaml
```

The prop library is primitives first: boxes and cylinders sized and colored
as pallets, cones, containers, hay bales, fence segments. Enough to get the
loop running end to end against the sensors that exist today. Meshes and
materials go in per prop as the visual layer comes up, without the generator
changing. See "The visual layer" below.

Nothing is exported or converted. The generator writes the MJCF that the sim
loads, that `make map` rasterizes, and that the viewer renders. One
representation, no dump step, nothing to keep in sync.

## Dependency floor

The core is `mujoco`, already in `.venv`, plus one LLM SDK. Both are pure
wheels with no build step, on macOS arm64 and x86_64 and on Linux x86_64 and
aarch64. Cloning the repo and generating a world must never need more than
that.

MuJoCo 3.10 covers everything the core needs: `MjSpec` builds models
programmatically and serializes to XML, and `mj_geomDistance` answers the
collision queries the critic runs. Do not reach for PyMJCF (`dm_control`)
for the XML building; `MjSpec` is native and supersedes it.

Everything else is an optional layer, installed only by someone who wants
it. The rule is that a missing layer degrades a feature, never breaks the
core. If the generator cannot emit a world without an extra install, the
layering is wrong.

## What a world needs, per consumer

A world is just MJCF. What it has to contain depends entirely on who reads
it, and the three consumers want different things. Nothing here is a
property of "a generated world" in general.

**Looking at it.** Nothing. `MjSpec`, compile, `to_xml()`,
`mujoco.viewer`. No ROS, no map, no robot, no rules. Prototype here.

**Training on it** (`docs/rl.md`). Geometry and a robot. The Gym env holds
`MjModel`/`MjData` and steps physics directly, so there is no ROS, no AMCL,
no occupancy grid, and no reason to care about anything below. Add the
timestep, since training and deployment reading the same number from the
same file is the point of `docs/research.md` angle 1. Otherwise a world that
loads is a world you can train in.

**Driving it with the ROS stack.** This is the only consumer with real
requirements, and they are mostly the header of `sim/worlds/blank.xml`:

- Robot includes (`../robots/husky.xml`, `x2.xml`) and the `<option>` line.
- A finite floor plane, because the rasterizer sizes the map from it and
  skips planes with `size[0] == 0`.
- Obstacles the robot must avoid, static and crossing the lidar plane at
  0.567 m. Loose freejointed props are fine, they just will not be mapped.
- An `autonomy_<name>.yaml` with a dock pose and waypoints, ideally derived
  from the occupancy grid rather than guessed.

One thing genuinely worth knowing before you meet it: world membership is
`body_weldid == 0`, not "child of worldbody". A generated robot without a
freejoint gets welded to the world and baked into the AMCL map with no error
anywhere. Silent wrong map, an hour of confusion. Everything else in this
section fails loudly or not at all.

## Sequencing: `docs/robot-config.md` comes first

Worldgen proceeds after that work lands.

Sensor geometry is currently discovered by scanning the world:
`rasterize_map.py` finds the map slice height by looking for a site named
`lidar*` in the compiled model. That makes the map a function of the world
when it is really a function of the world and the robot reading it, and it
survives today only because one robot has such a site.

That is not worldgen's bug, but it is worldgen's problem, because it decides
how much a generator has to know. With robots self-describing, a generated
world is geometry and nothing else and the generator never learns what a
mast height is. Without it, that knowledge leaks into the generator and has
to be unpicked later from worlds already written.

It is a small, self-contained refactor with a byte-identical map as its
regression bar. Do it first.

## What `lint_world.py` becomes

Today it catches hand-authoring mistakes. Once a generator exists it should
make its own failure classes unrepresentable, clamping obstacle heights
across the lidar plane and respecting `DOCK_CLEAR_M` (1.2), `WAYPOINT_CLEAR_M`
(0.6), and `GEOM_BUDGET` (300) by construction. Lint then becomes a
regression test on the generator, which is a better job for it.

It already carries the interpenetration and settling checks the critic
needs, so the critic should call into it rather than reimplement them.

One fix landed there from the abandoned attempt: dock and waypoint clearance
used `geom_rbound`, a bounding sphere, so a 10 m wall read as intruding on a
dock 4.9 m clear of it. Every long thin object an outdoor site is built from
would have failed the same way. It measures against the world-axis-aligned
box now.

## The visual layer

Photorealism is on the roadmap, not deferred indefinitely. VLA evaluation
needs frames a pretrained model can actually read, and pixel-based RL needs
them too. Today's stack does not: the 2D lidar reads one slice at z=0.567
and the anomaly detector is HSV color bands, so primitives are genuinely
sufficient for behavior-tree and navigation work. Build for the sensors that
exist, and leave the seam for the ones that are coming.

The prop library is that seam. A prop is a name, a collision geometry, and a
visual. Start with a primitive visual, swap in a mesh and materials later,
and the generator does not change: it places props and never looks at how
they render. Collision geometry stays primitive even after the visual gets
real, which keeps the physics fast and the map honest.

Asset processing is offline and per-prop, not per-scene. Run it once on
whatever machine is convenient, commit the result, and keep it out of the
generation loop. `bpy` installs cleanly from PyPI on macOS arm64 and Linux,
so this is an install someone chooses, not a platform fight.

One ceiling to know before betting on it: MuJoCo's native renderer is
fixed-function OpenGL with no PBR path. MJCF has carried material
sub-elements for occlusion, roughness, and metallic since 3.2.1, but the
native renderer ignores them; they exist for external renderers. So real
photorealism means an external render path (MJCF to USD, or Blender)
alongside the native viewer, not a flag on the viewer. That path serves
offline dataset generation well. Whether it can serve a VLA in the loop
depends on how fast it renders, and that is worth measuring before
committing to it.

## Localization is being handled, not deferred

Outdoor localization used to be the thing that capped how outdoor these
worlds could get. AMCL matching scans against a rasterized grid is an indoor
technique, and its premise is a single lidar plane read once at a fixed z,
which sloped terrain does not have.

That is now part two of `docs/robot-config.md`: the rasterizer, the map, and
AMCL come out, pose comes from fusion the way the air track already does it,
and the global costmap goes rolling and mapless. Worldgen inherits the
result rather than working around it.

The practical consequence here is that the map contract in "What a world
needs" mostly disappears once that lands. What survives is the lint gate,
which stops asking whether the map will miss an obstacle and starts asking
whether the robot can see it. That question needs the robot's scan height,
which is part one of the same doc.

One new constraint arrives with it, and it is a constraint on generated
worlds rather than on the stack. NavFn needs the goal inside the global
costmap, so a rolling window caps how far apart consecutive waypoints can
be. Blank's are at 7 m and 2.5 m and fit anything. A 30 to 100 m site does
not, unless the window is sized past the longest leg. Either the generator
keeps legs inside the window, or the window grows to match the sites, and
that tradeoff should be settled with a real site rather than assumed here.

Heightfields and real terrain are still further out. Removing the
single-plane premise unblocks them; it does not deliver them.

## Open

- Whether the LLM places objects directly, or picks a layout archetype and
  parameters that code then instantiates. The second is cheaper, more
  repeatable, and less likely to produce nonsense; the first is more varied.
- Whether the critic ever needs a render in the loop, or whether geometry
  checks are enough. Assume enough until a scene disproves it.
- Determinism. Seeded generation is easy; the sim's nondeterminism lives at
  the async ROS boundary and is a different problem (see `docs/rl.md`).
