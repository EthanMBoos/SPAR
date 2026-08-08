# 16 — Air waypoints

Do not change any visual object or ground waypoint. Create only the air route.

Use `spar.scene_top_z()` to get the tallest visible mesh inside the single code
call. Choose one common flight altitude at least 0.75 m above it and no higher
than 10 m. Use `spar.guide_frame(exact_name)` for the named zones and
`PLAN_SiteBoundary`, `spar.site("SITE_X2Spawn")`, and
`spar.roots_by_role("anomaly")`; do not make a separate inspection call.

Choose seven XYZ points that visit several distinct zones and stay inside the
site. Derive the boundary's min/max X and Y from its center and full dimensions.
Keep every point at least 0.75 m inside all four edges, and assert those four
inequalities for every point before creating any sites. Keep the straight
segments clear, include a useful view within 12 m of the inspection target,
and make the final point directly above `SITE_X2Spawn` so its takeoff column
remains clear. Create the route with
`spar.add_ordered_sites("SITE_AirPatrol", "air_patrol", points, face="next")`.

Finish in Material Preview and save
`<REPO>/artifacts/worldgen/<WORLD>/waypoints.blend`.
