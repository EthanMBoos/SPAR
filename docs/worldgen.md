# World generation

The implemented v1 worldgen is a constrained local-model experiment:

```text
description -> Ollama plan -> Ollama review -> Python MJCF
            -> MuJoCo compile and lint -> human inspection
```

It is also the primitive baseline for the primary research direction: an
iterative, multi-model BlenderMCP authoring loop. The intended interface keeps
primitive generation as the cheap default and adds an explicit fidelity mode:

```text
--fidelity primitive   structured plan -> deterministic primitive MJCF
--fidelity blender     orchestrator -> BlenderMCP build/query/render/revise
                                    -> site export -> the same MuJoCo lint
```

The fidelity option and Blender path are a target architecture, not implemented
CLI flags yet. See [research.md](research.md) for the golden path and staged
work.

Run it directly from the repository root:

```bash
make lint

.venv/bin/python scripts/generate_world.py \
  --name loading_yard \
  --model gemma3:4b \
  --seed 0 \
  "A compact loading yard with fencing, crates, and a red hazard drum"
```

`OLLAMA_HOST` defaults to `http://localhost:11434`. The generator makes at
most three attempts. Invalid JSON, a semantic review rejection, or a lint
failure becomes feedback for the next attempt. Nothing is published unless
the temporary MJCF compiles and passes lint.

Existing files are protected unless `--force` is passed. Accepted and failed
runs append one JSON object to `logs/worldgen.jsonl`; use `--record PATH` to
change it. Every record includes the description, model, seed, attempts,
elapsed time, and compact Ollama timing/token metadata for every planner and
reviewer call. Accepted records also include the exact plan, review result,
lint counts, and SHA-256 hash of the published MJCF.

Inspect an accepted world before using it:

```bash
make inspect WORLD=loading_yard
make start_sim WORLD=loading_yard
```

Generated worlds are ignored by git. Promote the rare reusable fixture with
`git add -f sim/worlds/<name>.xml`.

## Model versus Python

Ollama chooses only:

- one of four ground appearances;
- three to seven primitive props;
- each prop's kind, non-red color, coarse region, and orientation;
- exactly one red anomaly drum;
- whether its own structured plan matches the description.

Python owns the 16 m world size, prop dimensions, collision geometry, exact
coordinates, seeded position jitter, robot includes, MJCF, and all safety
checks. Red is reserved for the anomaly because the current detector finds
the largest red blob. If the model repeats a coarse region, local validation
rejects the plan and returns that error on the next attempt.

The seed is both a semantic variation ID supplied to the planner and the seed
for Python-owned position jitter. Ollama remains at temperature zero, so a
fixed description and seed are repeatable. The description, model tag, seed,
and exact accepted plan are embedded in the MJCF `<custom>` metadata so an
accepted file remains traceable if copied away from its JSONL record.

The lint currently validates the Husky route and geometry budget. The air
robot is included and can fly in every generated world, but the generator
does not yet validate aerial clearance or avoidance. The drone has no
obstacle-avoidance sensor today.

There are three separate approval stages:

1. Ollama review approval checks the structured plan for semantic fit and
   outdoor plausibility.
2. Programmatic lint approval compiles the temporary MJCF and checks the Husky
   route, geometry, and safety constraints.
3. Human visual approval is the final gate before the world is used.

This version does not generate missions, waypoint files, meshes, terrain,
movable props, or photorealistic assets. It does not render a screenshot for
model review.

## Fidelity mode contract

The high-fidelity path will change how a candidate world is authored, not what
the rest of SPAR consumes. It must preserve these boundaries:

- Blender is the visual authoring and inspection workspace; MuJoCo remains the
  runtime physics and sensor authority.
- The authoring output describes the site. Robot selection and composition are
  outside the Blender loop.
- Detailed visual meshes are separate from simple collision and lidar proxies.
- Multiple model calls and Blender queries are expected. An orchestrator,
  designer, and critic build, inspect, render, and repair the candidate until
  it passes or exhausts a bounded attempt budget.
- Structured scene facts, renders, MuJoCo lint failures, and eventually task
  rollouts may all become repair feedback.
- A Blender-authored world is not published until it compiles and passes the
  same robot-aware programmatic lint as a primitive world.
- Prompts, seeds, model and tool versions, asset identities and hashes, repair
  history, and the final artifact hash are recorded.

The first Blender milestone is deliberately narrow: one static outdoor site,
explicit collision proxies, a small export adapter, and no changes to the
simulator. Articulation, indoor manipulation, and alternative runtime renderers
belong after this path demonstrates reliable value.

## Primitive baseline experiments

Use the fixed descriptions in `scripts/worldgen_prompts.txt`, keep the seed
set constant, and compare the JSONL records. The useful first metrics are:

- accepted worlds per prompt;
- attempts and elapsed time per accepted world;
- invalid structured responses;
- semantic-review and lint rejection reasons;
- inference time, token counts, and generated-token throughput;
- semantic plan differences versus coordinate-only jitter.

Increase model size only after a repeatable failure on this fixed set. That
keeps the baseline experiment about model capability instead of changing
prompts, schema, and model at the same time. These measurements remain useful
as controls for the Blender fidelity track; they are no longer the whole
worldgen research program.
