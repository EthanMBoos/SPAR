# 15 — Navigation goals

Do not change any visual object. Create only the ground navigation goal set.

Inside the single code call, get the center and full dimensions of
`PLAN_Aisle_Main`, `PLAN_Aisle_Cross`, `PLAN_Zone_Staging`,
`PLAN_Zone_RacksWest`, and `PLAN_Zone_RacksEast` only with
`spar.guide_frame(exact_name)`. Plan guides are Blender Empties: never read
their `dimensions`, `bound_box`, `matrix_world`, or display size directly.
Look up `SITE_HuskySpawn` with `spar.site(...)`, and find the one inspection
target with `spar.roots_by_role("anomaly")`. Do not make a separate inspection
call.

Derive the first point from the full main-aisle guide rectangle: use the aisle
edge on the target's X side inset by 0.25 m, the target root's Y coordinate
clamped inside the aisle's Y extent by 0.25 m, and Z=0.05 m. This point must be
3-7 m from the target and no more than 20 m of direct travel from the Husky
spawn. Set its heading with `spar.inspection_yaw(target, point,
min_range=3.0, max_range=7.0)` so detector range, target-facing yaw, and mesh
line of sight are guaranteed. Choose five more points on connected aisle
centers that exercise staging, both rack approaches, and the open gate.

Compute ordinary headings with `spar.route_headings(points, face="arrival")`,
replace the first heading with the verified inspection yaw, and create the
ordered goal sites with
`spar.add_ordered_sites("SITE_NavigationGoal", "navigation_goal", points,
headings=headings)`. The mission behavior tree uses them for its rounds branch.

Finish in Material Preview and save
`<REPO>/artifacts/worldgen/<WORLD>/waypoints.blend`.
