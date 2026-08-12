# 14 — Render and save the final visual scene

Do not change scene geometry, transforms, materials, or lighting.

In the single call, use `spar.render_camera` to render:

- `CAM_Overview` to `<REPO>/artifacts/worldgen/<WORLD>/overview.png`;
- `CAM_Ground` to `<REPO>/artifacts/worldgen/<WORLD>/ground.png`;
- `CAM_Detail` to `<REPO>/artifacts/worldgen/<WORLD>/detail.png`.

Use `spar.write_scene_manifest` to write
`<REPO>/artifacts/worldgen/<WORLD>/scene_manifest.json`, pack external
resources, restore `CAM_Overview`, and save
`<REPO>/artifacts/worldgen/<WORLD>/final.blend`.

Finish in Material Preview with scene lights and world.
