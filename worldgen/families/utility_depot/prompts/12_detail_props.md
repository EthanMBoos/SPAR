# 12 — Detail props and ground

Preserve every semantic root transform. Add moderate local detail to drums,
pallets, crates, cable spools, pipe stacks, bollards, and cones: drum ribs and
bungs, pallet slats, crate fasteners, wound cable, pipe end variation, and
safety bands. Keep the inspection drum as the only saturated-red object.

Select assemblies only by exact semantic type with
`spar.roots_by_type("barrel", "inspection_target", "pallet", "crate",
"cable_spool", "pipe_stack", "bollard", "cone")`. Both `barrel` and
`inspection_target` are drums; do not classify them from object-name keywords.

Add restrained unparented visual ground detail in `DETAILS`, such as a few
tire marks, gravel patches, and small imperfections. Mark flat markings and
tiny decorative clutter `spar_no_collision=true`. Do not block aisles or either
spawn site.

Finish in Solid Material Color mode and save
`<REPO>/artifacts/worldgen/<WORLD>/geometry.blend`.
