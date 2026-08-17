# Research direction

Worldfile asks what structure and validation an LLM needs to reliably produce a
physics-ready robot world.

A world build starts with an environment family, seed, and short brief. The LLM
uses bounded Blender stages to choose layout, geometry, and appearance.
Repository code owns exact calculations, naming, physical metadata, export, and
validation.

```text
environment family + seed + brief
                |
       bounded LLM authoring
                |
          Blender scene
                |
       deterministic compiler
                |
        MJCF + referenced assets
                |
             validation
```

The Blender scene is an authoring artifact. The compiled MuJoCo model is the
result. Worldfile does not introduce another simulation format: it emits MJCF
and collects the referenced assets needed to use the world without Blender.
Completed worlds may also be packaged as a single `.mjz` archive.

## Compiler boundary

The LLM handles decisions that are difficult to enumerate in procedural code:
composition, layout, object selection, contextual detail, and appearance. The
compiler handles anything that must be exact.

This includes coordinate conversion, asset paths, visual and collision
geometry, physical properties, semantic names, robot spawns, sensors, and task
sites. A build fails if the result does not compile, remains unstable, contains
invalid collisions, or cannot support the required robot task.

The current utility-depot family covers static geometry, a mobile robot,
perception targets, navigation goals, and a complete ROS mission. Later worlds
can add dynamic actors, articulated objects, and manipulation without changing
the authoring and compilation boundary.

MuJoCo is the initial simulation target and Blender is the initial authoring
tool. Other tools can be considered when they solve a concrete problem.

## Evaluation

The first evaluation should compare one-shot generation, bounded staged
generation, and staged generation with one repair pass. Each condition should
use the same environment families, seeds, briefs, and robot task.

The main measurements are:

- unattended generation success;
- generation time and model calls;
- MJCF export and load success;
- missing or invalid assets;
- collision depth and static stability;
- spawn, route, and task-site validity;
- perception and navigation completion;
- complete mission success.

Repeated builds across held-out seeds and briefs measure variation and
reliability. A second environment family tests whether the authoring contract
generalizes beyond the utility depot.

Policy training is a possible use of the resulting worlds, not a requirement
for the initial evaluation. The current fixed autonomy stack provides a
repeatable test of whether a generated world is usable.

## Related work

[SceneSmith](https://github.com/nepfaff/scenesmith) is the closest reference for
agentic construction of simulation-ready robot environments. Worldfile takes a
narrower implementation path centered on bounded authoring, deterministic
compilation to MuJoCo, procedural or reusable assets, and validation with
existing robot software.
