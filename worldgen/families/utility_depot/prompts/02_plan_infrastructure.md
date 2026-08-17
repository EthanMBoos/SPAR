# 02 — Plan infrastructure

Place only the depot's large, structural assembly anchors. Use exact guide
lookups and the existing guide transforms; do not move topology or create mesh
geometry. This is an empties-only planning stage; do not query mesh bounds or
call `spar.scene_top_z()`.

Always create:

- one `ground` anchor centered on `PLAN_SiteBoundary`, with complete dimensions
  `(40, 40, 0.05)`;
- exactly five `fence` anchors and one `gate` anchor made only with
  `spar.add_rect_boundary_anchors(spar.guide("PLAN_SiteBoundary"),
  gate_side="south", ...)`; choose only the gate offset so it feeds
  `PLAN_Zone_Staging`;

Unless the instance brief requests a supported moderate quantity change, also
create one `structure` shed, three `utility_cabinet` anchors, and six `rack`
anchors. Keep the shed in `PLAN_Zone_Service` near a boundary but open toward
an aisle. Keep cabinets in one coherent service cluster without blocking the
shed. Arrange racks as ordered rows across `PLAN_Zone_RacksWest` and
`PLAN_Zone_RacksEast`, facing usable aisles; prefer
`spar.add_plan_anchor_row`.

Keep the dock, Husky spawn, and the main and cross aisles unobstructed.
Favor clear relationships—rows, service clusters, and entrances—over uniformly
scattered placement. Use the recorded stage seed for discretionary placement
the brief leaves open. Every anchor in this stage must use the shared
base-origin convention and therefore have Z=0.

Finish in Solid Material Color mode and save
`<REPO>/artifacts/worldgen/<WORLD>/02_infrastructure_plan.blend`.
