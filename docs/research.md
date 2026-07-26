# Research directions

This repo is a teaching template and a working sim harness, not a research
result. Every architectural pillar it stands on is already well covered in
the literature: BTs as the interpretable container for learned policies,
MuJoCo with ROS2, sim-to-sim validation on matched physics, and
scenario-based regression testing for autonomy. A strong synthesis of known
parts is not, on its own, a top-venue paper.

What a paper needs that this repo does not yet have: a validated claim with
an experiment, a baseline, and a number. The angles below are ordered by how
defensible the novelty is against the smallest lift from the current code.
Angle 1 is the only one where what makes this repo distinctive is itself the
research object, so it does not compete head-on in a crowded space.

## 1. The physics-identity claim, measured

The strongest genuinely underexplored point here is that the *exact same
MJCF file* runs in training and in the deployed sim (docs/rl.md). The
sim-to-real literature calibrates two *separate* engines and reports the
residual gap. Nobody quantifies what remains when the model is byte-identical
on both sides, and where it reappears.

The claim to test: physics is identical, so the residual train-to-eval gap is
entirely elsewhere. Candidates for "elsewhere": rclpy timing jitter, the
10 Hz BT tick, noisy GPS localization, the camera render path.

Shape of the experiment: train a local planner in the pure-MuJoCo Gym env,
deploy it as a behavior node, overlay the two trajectories (the Rerun plan in
docs/rl.md is already built for this), and decompose the residual gap by
source. A clean, honest systems contribution where the repo's architecture is
the method.

Why it is the smallest lift: the substrate, the MJCF dump, and the
overlay-verification design already exist. The missing pieces are the Gym
wrapper, one trained policy, and the measurement.

Venue: sim/benchmark workshop first (CoRL/RSS), main track if the gap
decomposition turns out non-trivial.

## 2. Cross-domain portability, quantified

Ground (mapless Nav2 + GPS + 2D lidar) and air (PX4 + offboard) already share one
world and one sensor layer. The "a domain is three swaps, not a fork" claim
(docs/sim-architecture.md) is currently an assertion in a doc.

To make it a paper: build the marine track on the roadmap, then report a
quantified portability claim. Same BT structure, fraction of code shared, and
the same eval harness catching the same class of behavior bug across three
dynamics regimes.

Effort: high. Needs the third domain built and a metric.

## 3. A reliability-gate benchmark

The README pitch, "say with evidence that a policy is good to go before it
leaves sim," is a benchmark pitch. Benchmarks publish, but only with a suite
of adversarial and edge-case scenarios, baselines, and evidence the suite
catches failures that aggregate success rate hides.

The gap worth aiming at is mechanism-aware evaluation: not one success number
but per-failure-class diagnosis (which capability broke, and why). That is
where recent long-horizon benchmark work is thin.

Effort: high, and the space (scenario-based AV regression testing) is crowded.
The differentiator has to be the per-failure-class angle, not the harness
itself.

## 4. Learned executive vs. hand-written BT

docs/rl.md Tier 2 proposes it directly: learn the patrol / investigate /
return arbitration and race it against the hand-written tree on the same
world. A real experiment, medium effort, and the most natural first thing to
actually run.

On its own it is a small result unless the finding surprises. The publishable
version is a tradeoff with numbers, for example: the learned executive wins on
coverage-per-battery but the BT is strictly more reliable under sensor
dropout. An interpretability-versus-performance result, not just a win.

## Recommendation

Start with Angle 1. It is the smallest jump from the current codebase to a
defensible result, and the property it measures (identical physics on both
sides) is one others assume rather than test. Angles 2 and 3 each need a
domain or a scenario suite built first; Angle 4 is the easiest to run but
needs a surprising result to carry a paper.
