# Dropping Unity

Status: done, 2026.
This doc is the why. The plan
narrows scope to a hand-authored flat-grid world (the generator and critic
move to a separate scenario repo, and the render test goes with the VLM
work).

## The goal

An outdoor SceneSmith. A generator that produces navigable outdoor worlds from a seed,
a critic that rejects the broken ones, and enough of them that coverage is a real
question. The whole stack under test in every run: behavior tree, motion stack,
perception. Not a policy alone in a physics sandbox.

Today: one scene, one world config, one map, one scripted eval. That gap is the work.

## Why Unity goes

1. **The generate-and-critique loop needs milliseconds per candidate.** Every candidate
   world through a Unity editor batch launch is roughly 30 seconds against milliseconds
   in pure MJCF. That is the difference between an agentic loop and a thought
   experiment. Sceniris (arXiv 2512.16896) reports the same bottleneck independently
   and 234x from fixing it.
2. **The repo's own rule forbids generation, and the rule has to go anyway.** "MJCF
   flows out only" exists because Unity owns the world and MJCF is derived. Once the
   spec is the world, there is one representation, no dump step, and nothing to keep in
   sync. Unity's MJCF importer exists, so this was never a Unity limitation. It was a
   policy, and the policy is wrong for where this is going.
3. **The dependency is dead weight.** ROS-TCP-Connector last shipped a release in 2022
   and was last pushed 2024-05-10, and Unity has since deprecated it toward a product.
   1,263 lines of custom C# plus a vendored plugin plus a vendored bridge, all to run
   physics we already own.
4. **Unity's terms changed 2026-06-30.** Training ML models on Unity Offerings "or data
   derived from them" now needs prior authorization, and AI agents may only touch the
   platform through a Unity-designated framework. Unity staff have said on forums that
   local Editor use and synthetic data from your own scenes are fine. That is a forum
   post, not contract text. Students fork this repo and are bound individually, so the
   blocker is institutional whether or not the legal risk is real. Route to GT legal in
   parallel. It does not gate the work.

## What actually has to be rebuilt

Most of the stack is already MuJoCo math. None of it is already a running process.

| Piece | Today | Rebuild |
|---|---|---|
| ROS publisher: lidar, odom, TF, clock, cmd_vel | `SparRosBridge.cs`, 366 lines | 3-4 days, rclpy |
| Camera RGB + depth | `SparCameraSensor.cs`, 128 lines | 1-2 days |
| World and robot spec to MJCF | the `.unity` scene | 3-5 days |
| Viewer | the editor | 0, `mujoco.viewer` |
| PX4 lockstep | `SparPx4Link.cs`, 483 lines | 3-5 days |
| Verify playbook | `.claude/skills/verify/SKILL.md` | 1 day, and it is not optional |
| Generator + critic | nothing | 1-2 weeks |

Three traps inside that table.

**The clock forces both tracks to move together.** `SparRosBridge.cs:111` is the only
`/clock` publisher in the repo and `air.launch.py:27` runs the whole air stack on
`use_sim_time: True`. The air track cannot stay on Unity. There is no partial port.

**The MJCF has no visual half.** No cameras, no lights, no materials, no textures. The
camera is a bare site with a Unity `Camera` mounted on it. Authoring that is the real
content of the camera row.

**The robots are not in any text file.** Both live in Unity prefabs. The Husky's tuning
in particular has to survive transcription exactly: drive spheres, casters at friction
0.005 with `priority="1"` so MuJoCo resolves the contact pair with the caster's
friction, velocity servos gain 35 and bias -35, ctrl and force clamps. Get `priority`
wrong and the robot stops pivoting. Read them out of `logs/blank.xml` before deleting
anything.

Two hidden contracts the generator must satisfy, currently unwritten: `rasterize_map.py`
needs a site whose name starts with `lidar`, and it defines the world as
`body_weldid == 0`, so a generated robot without a freejoint gets baked into the AMCL
map with no error.

Also, while in there: write `timestep` into the MJCF. It is absent today, so training
and deployment agree at 0.002 by coincidence of two independent defaults. That
coincidence is what `docs/research.md` angle 1 rests on.

## The one real cost, and the test that settles it

MuJoCo's renderer is fixed-function OpenGL with no PBR path. MJCF gained material
sub-elements in 3.2.1 for occlusion, roughness, and metallic, and the native renderer
does not support them. They exist for external renderers. MJWarp's batch renderer is
new and fast but explicitly low fidelity, so it does not change this.

The bet is that it does not matter here, and the evidence leans that way. The result
everyone cites for fidelity mattering (arXiv 2603.22876) is about training pixel
policies, which `docs/rl.md` puts out of scope. The result about pretrained VLMs reading
synthetic frames (ENACT, arXiv 2511.20937) ran a fidelity ablation and found no
significant difference, p>=0.2 across settings.

Test it anyway, in two days, before the port:

Emit one world from a spec. Render ~200 poses spanning it, twice: `mujoco.Renderer` and
the Unity camera, same poses. Run the VLM that will sit at the perception seam over both
sets. Score against ground truth the sim already knows.

Do not run this test with the HSV blob detector. It cannot fail, and it was never why
Unity was here.

If MuJoCo matches Unity, proceed. If it does not, the split is: native renderer in the
closed loop, MJCF to USD to an external renderer for perception datasets, offline and
batched. Be honest that the split does not serve VLM-in-the-loop, which is the goal. It
serves dataset generation, which is a different thing.

## Shape

The spec is the only authored artifact.

```
worlds/<family>.yaml + SEED
  └── emit MJCF                     (PyMJCF, milliseconds)
      ├── critic: clearance, penetration, settling, reachability, feasibility
      ├── rasterize_map.py          unchanged, takes a model path
      ├── lint_world.py             unchanged, takes a model path
      └── derive route + autonomy_<world>.yaml   from the occupancy grid
```

`lint_world.py` changes job. Today it catches hand-authoring mistakes. A generator
should make its failure classes unrepresentable: clamp obstacle heights across the lidar
plane, respect `DOCK_CLEAR_M = 1.2` and `WAYPOINT_CLEAR_M = 0.6`, stay under
`GEOM_BUDGET = 300`. It becomes a regression test on the generator, which is a better
job for it. Add two checks, both short with `mujoco` already imported: static-static
contact at t=0, and step 2 seconds with no controller to see whether anything moves.
Both lifted from SceneSmith.

Build on PyMJCF, not hand-rolled XML. Take heightfields from `artificial_terrains`
(MIT, arXiv 2506.19751). Do not build on madrona-mjx, which is being sunset in favor of
the Warp batch renderer.

## Order

1. Generator, critic, and the two physical checks. Pure Python against files. Nothing
   else blocks this and it is most of the goal.
2. The render test above.
3. Publisher, camera, spec-to-MJCF for the robots. Ground and air together, because of
   the clock.
4. PX4 lockstep. The MAVLink framing is mechanical. The risk is the constants, and every
   one of them is already written down in comments next to the code that earned it.
5. Rewrite the verify playbook. Until this lands there is no automated proof anything
   works.
6. Grader. `smoke_test.sh` is seven steps of string equality on `active_leaf`. Getting
   from there to relative scoring against a recorded reference run per world is the
   thing that makes any of this an eval, and it is entirely independent of the port.

## Outdoor localization is a separate project

AMCL against a rasterized occupancy grid is an indoor technique. Outdoor with 2D lidar
and sparse terrain features needs GPS or 3D lidar. Heightfields break the rasterizer's
premise outright, which is a single lidar plane at a fixed z read once after one
`mj_forward`. On sloped terrain there is no such plane.

Blast radius: `localization.yaml`, both costmaps and the tuned MPPI horizon in
`nav2.yaml`, the rasterizer, `lint_world.py`'s primary check which imports from it,
`make map`, the byte-identical map regression in the verify skill, and `battery_sim`,
which decides "at dock" from a map-frame transform.

The air track already has the answer in the repo: `tf_from_px4.cpp` publishes
`map -> base_link` straight from EKF2, reconciled to the pad. Borrow that pattern. The
BT and the detector need no change, only a consistent `map` frame.

This is bigger than the renderer swap and independent of it. It gets its own doc.

## What stays open

- The render test. Everything else is a port with a known shape.
- Whether the eval layer wants STL robustness (RTAMT) instead of pass/fail, so eroding
  battery-preempt margin from 8.2s to 0.4s is visible before it fails. Probably yes.
- Whether falsification search (Scenic plus VerifAI) is worth an adapter, given the
  world spec is the natural seam for it.
- Determinism. The nondeterminism today is the async ROS boundary at
  `SparRosBridge.cs:196-215` where the ROS thread writes `cmd_vel` under a lock and the
  physics step reads whatever is there. Porting does not fix it. Single-process Tier 1
  does, and that is the pure-MuJoCo Gym env in `docs/rl.md`, which already has no ROS.
