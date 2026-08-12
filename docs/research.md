# Research direction

SPAR asks whether an LLM working through Blender can build useful outdoor
robotics worlds without a person modeling each world by hand. It demonstrates
the pipeline on accessible hardware by dividing generation into bounded stages
rather than treating scene creation as one large agent task.

An environment family supplies the shared rules, helpers, and validation. A
seed and short brief provide variation, while the LLM handles layout,
appearance, and composition. Repository code owns repeatable choices, exact
calculations, export, and checks. Blender authors the visual scene; MuJoCo runs
it. A scene only counts if it exports, loads, and provides safe robot spawns and
routes.

```text
environment family + seed + brief
                |
    bounded BlenderMCP stages
                |
 contextual detail and materials
                |
     visual review and repair
                |
       deterministic export
                |
 MuJoCo world, assets, and routes
                |
             validation
```

The current utility-depot family proves this boundary for one environment type.
The next step is to improve its visual quality and evaluate the same approach
across fresh generations and, later, a second family.

## Visual-generation approach

[SceneSmith](https://github.com/nepfaff/scenesmith) is a useful reference for
this next step. SPAR can adapt its use of good assets, PBR materials, contextual
object density, and visual correction without reproducing its complete service
stack.

SPAR's procedural racks, pallets, crates, drums, barriers, pipes, and cable
spools will form the base object catalog. These models can gain bevels, UVs,
decals, better materials, and parameterized variation without changing how they
are generated.

[Poly Haven](https://polyhaven.com/models) can supply a small number of detailed
CC0 models for tools, vegetation, and other realistic props.
[AmbientCG](https://ambientcg.com/) will primarily supply CC0 PBR materials and
HDRIs, not the object catalog. Missing specialty equipment, such as generators,
pumps, or electrical cabinets, can be modeled directly or generated once
through SceneSmith's SAM3D path, reviewed, simplified, and stored locally. Asset
generation will remain an offline catalog-building step rather than a
requirement for every world build.

Racks, pallets, walls, and ground areas will expose simple placement zones when
they are created. The detail stages can fill those zones with appropriate props
instead of scattering generic clutter. Small seeded offsets in position,
rotation, color, and wear will make repeated objects less uniform. Task objects,
routes, spawns, and safety clearances will remain exact. Decorative details will
usually be visual-only; MuJoCo collision geometry will be reserved for objects
large enough to affect the robot.

The existing final-render stage will produce a fixed overhead view, four
robot-height views, and a close view of the task area. One visual critique will
check scale, materials, density, repetition, intersections, and access. It may
request one bounded repair pass. Existing checkpoints provide rollback, and the
current deterministic route, collision, and export checks remain authoritative.

This initial version does not need runtime SAM3D generation, CLIP retrieval
services, automatic support-surface extraction, persistent agents, or repeated
critique loops. Those remain optional extensions if evaluation shows a need for
them.

## First evaluation

The first comparison will use the same seed, cameras, and task definition for
the current depot and the enhanced version. It will add roughly ten PBR
materials, five reusable prop families, semantic placement zones, seeded
variation, and one review-and-repair pass.

The comparison will measure visual preference and critic scores alongside
generation time, model calls, asset size, MuJoCo performance, deterministic
validation, and ground and air task completion. Repeated fresh worlds will then
measure unattended success and variation across seeds and briefs. Applying the
same helpers, export path, and review process to a second environment family
will test whether the method generalizes beyond the depot.

See the [SceneSmith paper](https://arxiv.org/abs/2602.09153) and
[repository](https://github.com/nepfaff/scenesmith) for the reference system.
