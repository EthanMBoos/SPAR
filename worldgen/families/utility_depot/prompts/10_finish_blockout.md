# 10 — Finish the blockout checkpoint

Do not create, move, rotate, or reshape any physical assembly.

Create or refine `CAM_Overview`, `CAM_Ground`, and `CAM_Detail` with useful
depot compositions. Keep `CAM_Overview` active. Hide `SPAR_PLAN` from both the
viewport and renders with `spar.set_collection_visibility`; keep `SPAR_SITES`
visible in the viewport but hidden from renders. Ensure relationship lines are
off through `spar.finish_viewport` so the physical scene is easy to inspect.

Finish in Solid Material Color mode and save
`<REPO>/artifacts/worldgen/<WORLD>/layout.blend`.
