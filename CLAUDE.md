# CLAUDE.md

## What this is

A teaching template for the GT Cloud Robotics Autonomy and LLM tracks:
ROS2 + Nav2 + a small C++ behavior tree, simulated in pure MuJoCo.
Students fork it and put their own spin on a robotics behavior. The
repo's job is to be read, understood, and modified by a student in a
weekend, not to be a product.

The shape: one Python sim process (`sim/spar_sim/`) steps MuJoCo, renders
the cameras headless, and publishes sensor-shaped ROS topics to containers
running Nav2, PX4, and the behavior trees. The ground stack estimates its
map transform from noisy GPS; the air stack uses PX4 EKF2. All containers
share one network namespace. The MJCF file is the world (`sim/worlds/`,
robots included from `sim/robots/`), and `make lint` validates it against
the selected robot. Design decisions and their reasons are in
`docs/sim-architecture.md`.

## The code I want

- Simple, clean, robust, in that order of visibility: a student should read
  any file top to bottom in one sitting and understand it.
- No enterprise patterns. No factories, no interfaces with one
  implementation, no dependency injection frameworks, no manager classes,
  no config indirection. Reach for a function first, a class second, an
  abstraction only when the third caller exists.
- Prefer deleting code to guarding it. One-time utilities get removed after
  use; git history keeps them.
- Keep experimental world-generation work as directly invoked scripts. It
  will change quickly and need one-off tests; do not add Make targets for it
  until the workflow is stable.
- World generation is a local-model capability probe. Start with the smallest
  practical Ollama model, improve the schema and deterministic Python before
  increasing model size, and escalate only after a repeatable failure on a
  fixed prompt set. Keep heavier systems such as SceneSmith out of the core
  pipeline until a demonstrated need requires assets, visual reasoning, or
  more complex spatial planning.
- Robust means handling the failures that actually happen (stale data,
  unavailable action servers, late callbacks) plainly and locally, not
  wrapping everything in defensive checks.
- Match the conventions already here: one BT node per header, params
  declared and read in constructors, the existing naming.

## Comments and docs

- Comments state non-obvious constraints and hard-won gotchas, next to the
  code they protect, with source links when a claim needs receipts (see
  `sim/robots/husky.xml`). Never narrate what the code does, and never
  explain the same lesson in two files.
- Docs are terse human prose. No em dashes. Implementation rationale
  belongs in code comments, not in markdown; the md docs cover only what
  and why at the architecture level.
- Don't add a doc when a comment will do; don't add a comment when the
  code can say it.

## Verifying changes

The `verify` skill (`.claude/skills/verify/SKILL.md`) is the complete
playbook: which checks each kind of change requires, the exact commands,
and the traps (restart order, clock ownership, log redirection). Use it;
don't improvise the commands. Everything in it runs unattended.

Commit only when asked.
