# Research direction

Industry is attacking the robotics data problem from several angles. The
leading one is uptraining a large pretrained model into a robot model: the base
model already has world knowledge, so the data budget goes to mapping that to
actions. It works, and it has no real answer for the long tail — pretrained
priors are thinnest exactly where deployment fails, and the usual fix
(deployment gives you fleet data) requires the robustness you don't have yet.

SPAR is trying the other side: compositional environment generation with LLMs
in the loop, to build the tail cases on purpose instead of waiting to encounter
them. Blender via BlenderMCP is the authoring mechanism, since it's free, the
MCP interface is capable, and agentic generation of robotics worlds is still
greenfield. SPAR is a convenient place to try this because the simulator,
robots, tasks, learning harness, and acceptance gates already exist.

The primitive generator stays as the fast baseline and fallback, not the
ceiling.

## Golden path

Worldgen has one contract and two authoring modes:

```text
                            primitive mode
description -> plan ----------------------------------+
                                                      |
                            Blender fidelity mode     v
description -> orchestrator -> BlenderMCP work loop -> site artifact
                              |  build / query / render / revise
                              +------------------------------^

site artifact -> MuJoCo compose/compile -> SPAR lint -> visual review -> publish
                                      |                        |
                                      +------ repair feedback --+
```

The default primitive mode keeps today's cheap, deterministic path for tests,
fast RL curriculum generation, ablations, and machines without Blender. A
fidelity option will activate the Blender loop. Both modes must ultimately
produce the same kind of site-focused MuJoCo artifact under `sim/worlds/`;
robot selection and composition stay outside the Blender authoring loop.

The Blender loop is intentionally iterative and multi-LLM. A useful world will
normally require multiple model calls and multiple Blender queries rather than
one heroic prompt. The logical roles are:

- an orchestrator that owns the description, seed, constraints, progress, and
  termination decision;
- a designer that creates and revises layout, assets, materials, lighting, and
  collision proxies through BlenderMCP;
- a critic that inspects structured scene state and rendered views, then
  requests concrete repairs;
- deterministic export and lint code that remains the final authority on what
  is safe and runnable.

These are separate LLM-driven workers in the orchestration loop. They may
initially share the same underlying model and differ by prompt, context, and
tool permissions; using different models or concurrent workers is an
experimental choice. The durable boundary is the recorded scene state and
tool results, not an assumption about one particular model provider.

## Authority and artifact contract

Blender is the authoring and visual-inspection workspace. MuJoCo remains the
physics and sensor authority used by the simulator, ROS stack, and training
environment.

High-detail render meshes must be separable from simple collision and lidar
geometry. The export boundary should preserve:

- stable object names, transforms, semantic labels, and source asset IDs;
- visual meshes and materials;
- explicit primitive or convex collision proxies;
- cameras, lights, ground bounds, and site metadata;
- the prompt, seed, model/tool versions, accepted plan, repair history, and
  hashes of external assets and final outputs.

The exporter may use an intermediate scene manifest or emit MJCF directly.
That implementation choice should stay small and repo-owned until a concrete
case proves a larger interchange layer is needed. Existing Blender-to-MJCF
projects are useful references, but SPAR should not inherit a general
articulation format merely to export static outdoor sites.

No model or Blender render can approve its own physics. The current MuJoCo
compile, collision visibility, dock and route clearance, support, settling,
and geometry-budget checks remain mandatory. Blender-specific lint should be
added only for observed failures, likely unapplied transforms, missing asset
files, excessive mesh complexity, invalid collision proxies, and inconsistent
visual/collision bounds.

## Research questions

The central question is not whether an LLM can make an attractive Blender
scene. It is whether an iterative, tool-using system can repeatedly make
diverse robotics worlds that are visually coherent, physically valid, useful
for a task, reproducible enough to study, and cheap enough to generate.

Two things make that specific: a prompt-to-world tool only covers cases someone
already thought of, so coverage has to come from compositional expansion and
from mutating worlds where the current policy actually failed. And the useful
output is failures, not worlds — worth recording per published world which
failure mode it targets, where it came from, and whether it's held out.

The first comparisons should answer:

1. How often does the Blender loop publish a world without human repair?
2. Which feedback is most useful: structured scene queries, renders, MuJoCo
   lint failures, or task rollouts?
3. Does iteration improve semantic fidelity and layout diversity over a
   single Blender call?
4. What fidelity actually changes downstream perception, navigation, or policy
   generalization, and does cheap domain randomization in primitive mode match
   it? Randomization rather than realism is the likelier reason simulated
   locomotion transferred, so primitive-plus-randomization is the baseline.
5. How many distinct failure modes does the pipeline surface per N worlds, and
   does the yield decay?
6. Does hardening on generated cases transfer to a held-out suite authored by a
   different person or model, and does it cost nominal performance?
7. What is the cost in model calls, Blender time, asset downloads, geometry,
   simulation throughput, and human review?

Every high-fidelity result needs a matched primitive baseline where practical.
That keeps the experiment about the value of fidelity and orchestration rather
than merely demonstrating that Blender can produce better screenshots.

## Staged implementation

1. Preserve and measure the current primitive generator.
2. Produce one static outdoor site through BlenderMCP, with visual meshes and
   explicit simple collision proxies, and pass the existing lint unchanged.
3. Record the full multi-call build/query/render/repair trace and make a fixed
   prompt-and-seed run repeatable at the artifact level.
4. Add critic-driven render inspection and automatic repair attempts.
5. Close the failure-driven loop: run a policy or BT over generated worlds,
   collect failures, generate mutations that keep the failure mode, measure
   novel-failure yield across rounds.
6. Generate a small matched primitive/Blender corpus and compare acceptance,
   diversity, runtime, and task outcomes.
7. Only then expand toward movable or articulated assets, indoor scenes,
   alternative renderers, or large-scale parallel generation.

The next system milestone remains one complete learning loop: train on
generated worlds, export one policy, run it as a behavior-tree leaf, and
compare it with the existing scripted and Nav2 baselines. The richer worldgen
track and the learning loop should inform one another, but neither needs to
wait for the other to become fully general.
