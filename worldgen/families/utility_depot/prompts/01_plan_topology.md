# 01 — Plan topology

Create the spatial topology for a realistic, asymmetric 40 m by 40 m outdoor
utility depot. This call owns only zones, aisles, sites, and overview framing.
Use the recorded stage seed to vary discretionary zone transforms and framing
that the instance brief does not specify.

In the single call:

1. Run `spar.reset_scene()` and `spar.configure_scene()`.
2. Create `SITE`, `PROPS`, `DETAILS`, `CAMERAS`, `LIGHTS`, `SPAR_SITES`, and
   `SPAR_PLAN`.
3. Add an exactly 40×40 m boundary guide named `PLAN_SiteBoundary`.
4. Choose coherent, non-overlapping transforms for visible guides named:
   `PLAN_Zone_Staging`, `PLAN_Zone_Service`, `PLAN_Zone_RacksWest`,
   `PLAN_Zone_RacksEast`, `PLAN_Zone_Barrels`, `PLAN_Zone_Utilities`,
   `PLAN_Aisle_Main`, and `PLAN_Aisle_Cross`. Use `spar.add_plan_guide`, with
   `spar_plan_kind` values `zone` or `aisle`. Keep both aisles at least 3 m
   wide and connected to the open south gate and central staging area.
5. Use `spar.add_site` to create `SITE_Dock` with site value `dock` and
   `SITE_HuskySpawn` with site value `husky_spawn` at `(0, 0, 0)`, plus
   `SITE_X2Spawn` with site value `x2_spawn` at `(-2, -1, 0)`.
6. Create `CAM_Overview` with `spar.ensure_camera` so the full site is useful
   in a 16:9 viewport.

Do not create assembly anchors or physical mesh geometry. Finish in Solid
Material Color mode and save
`<REPO>/artifacts/worldgen/<WORLD>/01_topology.blend`.
