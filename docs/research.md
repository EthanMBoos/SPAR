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

## Golden path

Worldgen has one authoring path:

```text
description -> BlenderMCP full layout -> detail / render / revise
            -> visual scene + explicit collision proxies
            -> deterministic MJCF export -> MuJoCo compile -> visual review
```

The first BlenderMCP call owns the whole spatial layout. Later calls add detail,
materials, collision proxies, and bounded repairs without replacing that
layout. The logical responsibilities are:

- an orchestrator that owns the description, seed, constraints, progress, and
  termination decision;
- a designer that creates and revises layout, assets, materials, lighting, and
  collision proxies through BlenderMCP;
- a critic that inspects structured scene state and rendered views, then
  requests concrete repairs;
- deterministic export and MuJoCo compilation code that remains the final
  authority on what is runnable.

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

No model or Blender render can approve its own physics. MuJoCo compilation and
human inspection are the initial gates. Automated validation should be added
only for observed failures.

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
   compile errors, or task rollouts?
3. Does iteration improve semantic fidelity and layout diversity over a
   single Blender call?
4. Which visual and physical variations actually change downstream perception,
   navigation, or policy generalization?
5. How many distinct failure modes does the pipeline surface per N worlds, and
   does the yield decay?
6. Does hardening on generated cases transfer to a held-out suite authored by a
   different person or model, and does it cost nominal performance?
7. What is the cost in model calls, Blender time, asset downloads, geometry,
   simulation throughput, and human review?

## Staged implementation

1. Produce one static outdoor site through BlenderMCP, with visual meshes,
   explicit simple collision proxies, and a deterministic MJCF export.
2. Record the full multi-call build/query/render/repair trace and make a fixed
   prompt-and-seed run repeatable at the artifact level.
3. Add critic-driven render inspection and automatic repair attempts.
4. Close the failure-driven loop: run a policy or BT over generated worlds,
   collect failures, generate mutations that keep the failure mode, measure
   novel-failure yield across rounds.
5. Generate a Blender-authored corpus and compare acceptance, diversity,
   runtime, and task outcomes.
6. Only then expand toward movable or articulated assets, indoor scenes,
   alternative renderers, or large-scale parallel generation.

The next system milestone remains one complete learning loop: train on
generated worlds, export one policy, run it as a behavior-tree leaf, and
compare it with the existing scripted and Nav2 baselines. The richer worldgen
track and the learning loop should inform one another, but neither needs to
wait for the other to become fully general.
