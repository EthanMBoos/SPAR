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
description -> BlenderMCP spatial plan -> construction from plan
            -> detail / materials / waypoints -> completed visual scene
            -> automatic per-element collision -> deterministic MJCF export
            -> MuJoCo compile -> visual observation
```

The spatial-plan prompt owns positions, rotations, zones, paths, and
sightlines. The construction prompt instantiates physical assemblies at those
anchors without solving layout again. Later prompts add detail, materials, and
waypoints without replacing the plan. The logical responsibilities are:

- an orchestrator that eventually submits the tuned prompt sequence and owns
  the description, seed, constraints, progress, and termination decision;
- a designer that plans layout and creates assets, materials, lighting, and
  physical assembly metadata through BlenderMCP;
- deterministic export and MuJoCo compilation code that remains the final
  authority on what is runnable.

The production interface treats a family as the authored distribution and a
stored generation recipe as one instance: world identity, free-text brief,
base seed, stable per-stage seeds, model settings, and source hashes. Seeds
control deterministic repo-owned choices, not the model provider. Rendered
prompts and raw tool traces preserve what the nondeterministic author actually
did.

During prompt research, each designer stage is manually submitted in a fresh
model session so its changes can be watched in Blender and its prompt can be
tuned. This is not a per-world approval workflow. The production orchestrator
should later submit those same prompts automatically. A critic or autonomous
repair loop is not part of the v1 authoring path and should only be introduced
as a separately measured future experiment if repeated failures justify it.
The durable boundary is recorded scene state and tool results, not an
assumption about one particular model provider.

## Authority and artifact contract

Blender is the authoring and visual-inspection workspace. MuJoCo remains the
physics and sensor authority used by the simulator, ROS stack, and training
environment.

High-detail render meshes must remain separable from collision and lidar
geometry where their representations diverge. The export boundary should
preserve:

- stable object names, transforms, semantic labels, and source asset IDs;
- visual meshes and materials;
- per-element automatic convex collision meshes and opt-outs;
- cameras, lights, ground bounds, and site metadata;
- the prompt, seed, model/tool versions, plan checkpoints, repair history, and
  hashes of external assets and final outputs.

The exporter may use an intermediate scene manifest or emit MJCF directly.
That implementation choice should stay small and repo-owned until a concrete
case proves a larger interchange layer is needed. Existing Blender-to-MJCF
projects are useful references, but SPAR should not inherit a general
articulation format merely to export static outdoor sites.

No model or Blender render is the physics authority. Deterministic export and
MuJoCo compilation protect runtime invariants. Human visual observation during
prompt research informs prompt tuning rather than acting as a required
production approval gate. Add further automated validation only for repeated,
observed failures.

## Generality boundary

The strongest early hypothesis was that one shared scene contract plus a
natural-language brief could reliably produce good-enough worlds from unrelated
environment contexts on a lower-cost model. The experiments below show useful
transfer, but the iterative depot work does not support treating that as the
near-term product. Reliable low-cost generation increasingly comes from giving
the model a developed environment-family toolkit rather than asking it to
rewrite geometry and placement algorithms in every call.

The working hypothesis is now bounded family generality:

- A shared Blender kernel owns transforms, hierarchy, primitives, checkpoints,
  and other mechanics that transfer across families.
- The SPAR contract and exporter protect current simulator and robot invariants:
  finite transforms, collision assets, site bounds, route clearance, referenced
  files, and MuJoCo compilation.
- An environment-family kit owns its topology vocabulary, recurring assets,
  legal relationships, visual direction, and deterministic calculations that
  repeatedly fail in prose.
- A small world brief should vary supported choices inside that kit. A major
  change of context can require a new prompt family and new Python helpers while
  still reusing the kernel and exporter.

This is not an argument for a depot-specific exporter. A farm, warehouse,
construction site, and depot should not require different MJCF logic merely
because they contain different static props. It is an argument for explicit
family-owned authoring tools above the exporter. The implementation boundary is
summarized in [`worldgen/README.md`](../worldgen/README.md).

Do not turn every rejected pilot into shared hard-coded policy. Record the
failure first. Tune prompts for subjective one-offs, add family tooling for
repeated domain-specific mechanics, and add shared scaffolding only when a
mechanic transfers across families or protects a simulator/robot invariant.
During prompt research, humans still judge composition, recognizable contents,
plausible scale, and intended free space; that tunes the family and does not add
a manual approval gate to every production world.

Every attempted world, including rejected ones, should record the environment
brief, model and effort, number of model and Blender calls, elapsed time, token
or dollar cost, first failed gate, and human repairs. Compare first-shot
acceptance on several development families and untouched held-out briefs. This
prevents repeatedly tuning one depot prompt and mistaking memorized scaffolding
for general world generation.

The first Sonnet-medium pilot on August 2, 2026 made a complete visible depot
layout in four Blender calls, 4 minutes 5 seconds, and $0.683. It was rejected
because detail meshes were labeled as independent semantic assemblies. A
second fresh attempt with a generic one-root-per-assembly instruction took
5 minutes 16 seconds and $0.909. It fixed the hierarchy but grouped the whole
perimeter fence under one box-shaped collision source, which would fill the
site. These are prompt and representation failures to measure, not evidence
yet for depot-specific exporter rules.

A held-out horticultural field-station brief then used the shared contract
without a fixed object ontology. Sonnet-medium built 260 visual objects in four
calls, 4 minutes 5 seconds, and $0.673. Its hierarchy, ground, sites, cameras,
and anomaly passed the generic scene preflight. A fresh generic collision pass
took 6 minutes 4 seconds and $1.269 to author 49 explicit proxies across new
types such as raised beds, a greenhouse, water tanks, compost bays, and
irrigation equipment. The exported world compiled with 21,214 visual triangles
and 110 MuJoCo geoms. It was programmatically accepted. The collision run also
wasted turns on a large scene dump and denied screenshot and shell attempts,
so the shared prompt now requires a compact in-Blender summary instead.

The first generic template omitted a surfacing stage, so its blockout colors
did not become real Blender materials. A subsequent Sonnet-medium material pass
took 2 minutes 41 seconds and $0.575, covering all 253 visual meshes with 32
procedural material families. A direct Eevee render confirmed the colors. The
template retains separate visual blockout and material passes; collision is now
automatic, and observed hull failures become ordinary targeted scene edits.

The explicit-proxy experiment established that a generic model could recreate
usable collision, but it also added a costly model pass and made ordinary
physics look like a special authoring task. The v1 pipeline therefore uses one
Unity-like rule: every volumetric visual Blender element is exported as its
own MuJoCo convex-mesh collider, decorative elements opt out, and semantic
roots preserve assembly identity. This removes collision authoring from the
default prompt sequence; specific failures are repaired only after they are
observed. The exact mapping lives beside the exporter implementation.

A fresh README-driven depot run on August 3, 2026 tested Sonnet at medium
effort with no scene repair. It produced all four Blender checkpoints and three
renders, but it was rejected at the first deterministic export gate. The
layout put every semantic assembly root at `(0, 0, 0)` and also moved the
required X2 spawn from `(-2, -1, 0)` to the origin. The geometry stage then
created hundreds of details relative to those collapsed anchors; the attempted
export found 787 volumetric collision elements and correctly rejected an
obstructed X2 takeoff corridor. The renders also showed crowded central
geometry, mostly white surfacing, and a poorly framed detail camera. The first
geometry session stopped before editing because it exceeded its inspection
budget, the retry consumed about 21.8k displayed tokens, the material stage
about 20.4k, and the waypoint stage exceeded 34.6k before its post-save response
was interrupted. Human intervention was limited to tool approvals, enabling
Poly Haven, and retrying the no-op geometry session; no visual or physical
repair was made. This is a failed one-shot result and evidence that medium
effort alone does not keep a complex BlenderMCP stage within a small token
budget.

An August 4 split-plan experiment improved the plan itself but exposed a
different failure in construction. Two small plan calls produced plausible,
distinct anchors. The following all-in-one construction prompt ran for about
6 minutes 59 seconds and displayed roughly 41.1k tokens. Its first Blender call
also hit a render-engine detection error and triggered an extra repair call.
The result contained 62 semantic roots and 405 meshes, and the roots remained
at their planned anchors, but keep-world parenting left child meshes with
collapsed or misleading local transforms. Visually, the scene became a dense
web of relationship lines and overlapping geometry. The prompt was still
asking one model turn to inspect a plan, invent a procedural asset library,
instantiate every family, maintain hierarchy, configure presentation, and
save a checkpoint. Splitting only planning from construction was not enough.

The first narrow v1 design therefore used 15 nominally one-call stages grouped
into five passes: two planning calls, seven construction/blockout calls, two
detail calls, two surfacing/render calls, and two waypoint calls. Each stage ran
in a fresh Sonnet-medium session against the same visible Blender GUI. A small
generic helper module owned error-prone mechanics such as local-space parenting,
collections, viewport presentation, cameras, and checkpoint saves; the prompts
still owned environment content.

An August 6 clean README run completed all 15 authoring stages and reached both
waypoint passes, supporting the fresh-context Sonnet hypothesis. It also found
three concrete failures. The ground anchor stored the required complete
`[40, 40, 0.02]` dimensions, but the generic box helper created a unit cube and
then incorrectly divided the requested complete dimensions by two, producing a
20 by 20 m slab; deterministic export correctly stopped. This also meant every
box made through that helper was half its requested size. The pallet/crate stage
briefly reused anchor names for roots, deleted those anchors during its own
recovery, then reloaded the prior checkpoint. The ground-waypoint stage first
used name heuristics that found no storage zones, then made two inspection calls
and a corrective call. Those retries made the previous README's "exactly one
call" statement false. The completed visual scene also contained all 61 roots
but spread many small assemblies too independently across the site instead of
forming convincing work and storage groups.

The revised v1 uses 16 stages: topology, large-infrastructure planning, clustered
storage planning, then the same bounded construction/detail/surfacing/waypoint
families. Generic helpers now read full ground dimensions directly from the
boundary, reserve `ASM_<anchor>` root names, expand rows and grids, provide exact
guide/site/role lookups, compute scene height, and create consistently ordered
route sites. These are construction guarantees, not a critic or repair system.
The runner streams every actual MCP tool invocation so any retry is observable.
There remains no automatic visual critic, complex scene validator, hidden
rerender loop, or family-specific collision pass. Opus at medium remains a
controlled comparison for a stage that repeatedly exceeds Sonnet's reasoning
needs, not the default response to an overloaded prompt.

The first revised partial run exposed another ambiguous mechanic before
construction. The infrastructure planner placed anchor empties at half object
height, treating them as object centers; ground remained at zero. That explained
the visibly mixed floating/non-floating anchors, but left every later builder to
remember the same convention. It also placed three boundary sides at the origin
and generated one negative fence dimension. The helper contract now uses one
base-origin convention for every ground-supported assembly: anchor and semantic
root Z are zero, while child primitive centers use positive local Z. A generic
rectangular-boundary helper now owns five positive fence segments and the gate
at the actual boundary extents.

After that change, a clean stages 01–04 run produced 61 positive-size anchors,
all at base Z=0, with exact counts and purposeful storage groups. The site shell
contained a 40 by 40 by 0.05 m slab spanning Z -0.05 to 0 and fence/gate geometry
spanning Z 0 to 2.2. Infrastructure planning still used three Blender tool calls
despite the one-call instruction; the streamed runner made that visible. The
first storage attempt stopped after one call because it invented a nonexistent
`PLAN_Zone_Storage`; an exact zone-to-family map made the next fresh attempt
succeed in one call. This supports small prompts plus exact mechanical APIs, but
also shows that prose alone cannot guarantee a one-tool-call stage.

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
8. What fraction of held-out environment briefs pass without family-specific
   prompt clauses, validators, or repair calls?

## Staged implementation

1. Produce one static outdoor site through BlenderMCP, with visual meshes,
   automatic per-element collision, and a deterministic MJCF export.
2. Record the full multi-call plan/build/detail/material trace and make a fixed
   prompt-and-seed run repeatable at the artifact level.
3. Chain the tuned prompts automatically without changing their authoring or
   validation behavior.
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
