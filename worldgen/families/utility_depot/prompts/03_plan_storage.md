# 03 — Plan storage clusters

Place only storage and safety-prop anchors. Preserve the infrastructure plan
and use exact zone and aisle guide lookups. Do not create mesh geometry.
The only zone names used by this stage are `PLAN_Zone_Barrels`,
`PLAN_Zone_Utilities`, `PLAN_Zone_Staging`, and `PLAN_Zone_Service`; there is no
generic `PLAN_Zone_Storage`.

Always create exactly one `inspection_target` anchor with `role="anomaly"`.
Unless the instance brief requests a supported moderate quantity change, also
create the following defaults:

- fifteen ordinary `barrel` anchors;
- six `pallet`, five `crate`, four `cable_spool`, and three `pipe_stack`
  anchors;
- six `bollard` and four `cone` anchors.

Compose these as a few purposeful groups rather than independent scatter:

- use two or three compact barrel groups in `PLAN_Zone_Barrels`, with the
  inspection target at an accessible group edge visible from an aisle;
- group pallets with crates as loading arrangements near an edge of
  `PLAN_Zone_Staging`, clear of its traffic path;
- group cable spools and pipe stacks in `PLAN_Zone_Utilities`;
- use bollards in `PLAN_Zone_Service` to protect the existing service
  infrastructure and cones in `PLAN_Zone_Staging` to mark one small work area,
  not as random decoration.

Use `spar.add_plan_anchor_row` or `spar.add_plan_anchor_grid` for repeated
groups. Every anchor is still a separate physical assembly. Keep every full
footprint inside its zone and keep aisles, the dock, and the Husky spawn
clear. Nothing except the inspection target is designated red. Use the
recorded stage seed for discretionary group arrangement the brief leaves open.
Every anchor in this stage must use the shared base-origin convention and
therefore have Z=0.

Finish in Solid Material Color mode and save
`<REPO>/artifacts/worldgen/<WORLD>/plan.blend`.
