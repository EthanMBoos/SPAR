# 15 — Ground waypoints

Do not change any visual object. Create only the ground route.

Inside the single code call, get the center and full dimensions of
`PLAN_Aisle_Main`, `PLAN_Aisle_Cross`, `PLAN_Zone_Staging`,
`PLAN_Zone_RacksWest`, and `PLAN_Zone_RacksEast` only with
`spar.guide_frame(exact_name)`. Plan guides are Blender Empties: never read
their `dimensions`, `bound_box`, `matrix_world`, or display size directly.
Look up `SITE_HuskySpawn` with `spar.site(...)`, and find the one inspection
target with `spar.roots_by_role("anomaly")`. Do not use `SITE_Dock`, search
names, or make a separate inspection call.

Derive one inspection point from the full main-aisle guide rectangle, not just
its centerline: use the aisle edge on the target's X side inset by 0.25 m, the
target root's Y coordinate clamped inside the aisle's Y extent by 0.25 m, and
Z=0.05 m. This deterministic point must be 3–7 m from the target and no more
than 20 m of direct travel from the Husky spawn. Set
`inspection_yaw = spar.inspection_yaw(target, point, min_range=3.0,
max_range=7.0)`; this must succeed, proving detector range and mesh line of
sight, and returns only the scalar target-facing yaw. Make this inspection
point the first of six XYZ route points. Choose the other five points along
connected aisle centers through both rack approaches and naturally back toward
staging, with useful Husky clearance and no redundant trip to the dock.

Compute ordinary headings with `spar.route_headings(points, face="arrival")`,
replace the first heading with the verified inspection yaw, and create the
route with `spar.add_ordered_sites("SITE_Patrol", "patrol", points,
headings=headings)`. Before creating sites, assert that cumulative XY travel
from the Husky spawn through the first inspection point is no more than 20 m.

Finish in Material Preview and save
`<REPO>/artifacts/worldgen/<WORLD>/15_ground_waypoints.blend`.
