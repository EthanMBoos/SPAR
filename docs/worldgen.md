# World generation

Worldgen is a constrained local-model experiment:

```text
description -> Ollama plan -> Ollama review -> Python MJCF
            -> MuJoCo compile and lint -> human inspection
```

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
change it. Records include the description, model, seed, attempts, plan,
review reason, lint counts, and elapsed time.

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
the largest red blob. If the model repeats a valid coarse region, Python moves
the duplicate to the next free region and records both plans.

The seed changes only Python-owned position jitter. The description, model
tag, seed, and accepted plan are embedded in the MJCF `<custom>` metadata so
an accepted file remains traceable if copied away from its JSONL record.

The lint currently validates the Husky route and geometry budget. The air
robot is included and can fly in every generated world, but the generator
does not yet validate aerial clearance or avoidance. The drone has no
obstacle-avoidance sensor today.

This version does not generate missions, waypoint files, meshes, terrain,
movable props, or photorealistic assets. It does not render a screenshot for
model review. Human inspection remains the final gate.

## Comparing small models

Use the fixed descriptions in `scripts/worldgen_prompts.txt`, keep the seed
set constant, and compare the JSONL records. The useful first metrics are:

- accepted worlds per prompt;
- attempts and elapsed time per accepted world;
- invalid structured responses;
- semantic-review and lint rejection reasons.

Increase model size only after a repeatable failure on this fixed set. That
keeps the experiment about model capability instead of changing prompts,
schema, and model at the same time.
