# Shared visible-authoring contract

Use BlenderMCP only. Do not inspect the repository, use shell tools, take a
screenshot, request a full scene dump, or use a tool other than BlenderMCP.
The prompt has already substituted the absolute `<REPO>` and `<WORLD>` values.

Make exactly one `execute_blender_code` call for the numbered stage below.
Inside that call, import the repository-owned mechanics without opening or
rewriting them:

```python
import sys
sys.path.insert(0, "<REPO>")
from worldgen import blender_helpers as spar
```

The compact helper API is:

```python
spar.ensure_collection(name)
spar.add_plan_guide(name, kind, location, dimensions, rotation=(rx, ry, rz))
spar.add_plan_anchor(name, plan_type, zone, location, dimensions,
                     rotation=(rx, ry, rz), role=None)
spar.add_rect_boundary_anchors(boundary, gate_side="south", gate_offset=0.0,
                               gate_width=4.0)
spar.add_plan_anchor_row(name_prefix, plan_type, zone, center, dimensions,
                         count, spacing, rotation=(rx, ry, rz))
spar.add_plan_anchor_grid(name_prefix, plan_type, zone, center, dimensions,
                          rows, columns, spacing=(x, y), rotation=(rx, ry, rz))
spar.add_site(name, site, location, rotation=(rx, ry, rz), order=None)
spar.add_ordered_sites(name_prefix, site_prefix, points, face="next",
                       headings=None)
spar.route_headings(points, face="next")
spar.yaw_toward(origin_xyz, target_xyz)
spar.inspection_yaw(target_root, location_xyz, min_range=3.0,
                    max_range=7.0, camera_height=0.28)
spar.guide(exact_name)
spar.guide_frame(exact_name)  # returns (center_xyz, full_dimensions_xyz)
spar.site(exact_name)
spar.assembly_anchors(*plan_types)
spar.roots_by_type(*spar_types)
spar.roots_by_role(role)
spar.dimensions_from_anchor(anchor)
spar.semantic_root(anchor, collection="PROPS")
spar.add_box_local(root, name, local_location, dimensions, rotation=...,
                   material=..., no_collision=False, bevel=0.0)
spar.add_ground_from_boundary(root, boundary, thickness=0.05, material=...)
spar.add_cylinder_local(root, name, local_location, radius, depth,
                        rotation=..., material=..., no_collision=False)
spar.add_cone_local(root, name, local_location, radius1, radius2, depth,
                    rotation=..., material=..., no_collision=False)
spar.material(name, rgba, metallic=0.0, roughness=0.5)
spar.ensure_camera(name, location, target, lens=45.0)
spar.finish_viewport(camera="CAM_Overview", shading="SOLID")
spar.save_checkpoint(absolute_path)
```

Locations and dimensions are meters; rotations are XYZ radians. Every
`dimensions` argument is the complete object size, never a half-extent. Row
and grid helpers create numbered, distinct anchors around the supplied center.
Use exact `guide(...)` and `site(...)` lookups rather than name heuristics or
inspection calls. Use ordinary Python loops and small local constructor
functions inside the one call.

For every ground-supported assembly, an anchor is the center of its footprint
at ground contact: set its Z location to `0`, regardless of object height. The
semantic root therefore also sits at Z=0. Primitive locations are their local
centers, so a ground-resting component of height `h` has local Z=`h/2`.

Use `spar.semantic_root(anchor, ...)` for every new physical assembly and only
the `spar.add_*_local(root, ...)` helpers for its component primitives. Never
implement keep-world parenting, assign `matrix_parent_inverse` yourself, or
place a component using world coordinates. Semantic-root names are reserved
and generated automatically as `ASM_<anchor name>`; never choose or rename a
root, and never override its `spar_type`; the helper preserves the anchor's
plan type. The helper deliberately makes each child transform local to its
correctly positioned semantic root.

Do only the listed stage. Do not redesign earlier work, run validators, make a
repair pass, render unless explicitly requested, perform extra inspection
calls, delete existing objects, or reopen an earlier checkpoint. If the single
call fails, report the failure and stop; do not retry. Before the execute call
returns, use `spar.finish_viewport(...)` as specified and save only the listed
checkpoint. Report the saved path and a short factual summary, then stop.
