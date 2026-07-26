# Research direction

The current hypothesis is narrow: small local language models can create
enough valid, useful variation in primitive MuJoCo worlds to support an RL
curriculum. Deterministic code should own geometry and safety. The model
should only supply semantic choices that are hard to express as a fixed
template.

The first experiment compares local Ollama models on the fixed descriptions
in `scripts/worldgen_prompts.txt`. Hold prompts and seeds constant. Record
acceptance rate, retries, failure reasons, and elapsed time from
`logs/worldgen.jsonl`. Visually inspect accepted scenes for description fit
and useful layout diversity.

This experiment answers three questions in order:

1. Can the smallest model reliably satisfy the structured contract?
2. Does its semantic variation add value beyond seeded procedural placement?
3. What repeatable failure requires a larger model or visual/asset pipeline?

Only the third answer justifies moving toward a heavier approach such as
SceneSmith. Complex assets, screenshot review, and open-ended spatial planning
are not assumed requirements.

The next research milestone is one complete learning loop described in
[rl.md](rl.md): train on generated worlds, export one policy, run it as one
behavior-tree leaf, and compare it with the existing baseline. Until that
exists, claims about policy portability or train-to-ROS performance are
proposals, not results.
