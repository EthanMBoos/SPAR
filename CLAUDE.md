# CLAUDE.md

## What this is

SPAR is a ground-only world-generation and autonomy testbed.
BlenderMCP authors worlds, deterministic Python exports them to MuJoCo, and a
Husky uses ROS 2 localization, perception, a mission behavior tree, and Nav2
to run rounds, inspect the authored red barrel, and return to its dock.

The Python simulator owns MuJoCo state and publishes lidar, GPS, IMU, wheel
encoders, aligned RGB-D images, TF inputs, and `/clock`. The consolidated ROS
package is `ros/src/spar`. Architecture decisions live in `docs/`.

## Code style

- Prefer direct functions and small classes. Add an abstraction only after a
  real repeated need exists.
- Delete obsolete code instead of guarding or preserving compatibility.
- Blender owns layout, visuals, collision geometry, semantic sites, and goals.
  Python owns deterministic export and validation.
- Keep comments next to non-obvious constraints. Do not narrate plain code.
- Keep host tools and runtime behavior separate. Mission leaves use Nav2's
  standard action as their navigation interface.

## Verification

Use `.claude/skills/verify/SKILL.md`. The full pass is host tests, export
validation, a clean container build, the simulator self-check, and the
full mission-lifecycle smoke test. Never commit unless asked.
