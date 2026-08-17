# CLAUDE.md

## What this is

Worldfile compiles LLM-authored Blender environments into MJCF and referenced
assets. Bounded authoring stages own layout and appearance; deterministic code
owns physical metadata, export, and validation. The Blender scene is an
authoring artifact, not a new simulation format.

The current validation harness uses a Husky with ROS 2 localization,
perception, a mission behavior tree, and Nav2 to run rounds, inspect the
authored red barrel, and return to its dock.

The Python simulator owns MuJoCo state and publishes lidar, GPS, IMU, wheel
encoders, aligned RGB-D images, TF inputs, and `/clock`. The consolidated ROS
package is `ros/src/worldfile_demo`. Architecture decisions live in `docs/`.

## Code style

- Prefer direct functions and small classes. Add an abstraction only after a
  real repeated need exists.
- Delete obsolete code instead of guarding or preserving compatibility.
- The LLM owns composition, layout, object choice, and appearance. Repository
  code owns exact calculations, physical contracts, export, and validation.
- Keep comments next to non-obvious constraints. Do not narrate plain code.
- Keep host tools and runtime behavior separate. Mission leaves use Nav2's
  standard action as their navigation interface.

## Verification

Use `.claude/skills/verify/SKILL.md`. The full pass is host tests, export
validation, a clean container build, the simulator self-check, and the
full mission-lifecycle smoke test. Never commit unless asked.
