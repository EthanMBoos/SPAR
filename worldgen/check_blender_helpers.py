"""Focused Blender-side smoke check for local assembly transforms."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import bpy
from mathutils import Matrix, Vector


REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from worldgen import blender_helpers as spar  # noqa: E402


spar.reset_scene()
spar.configure_scene()
spar.ensure_collection("PROPS")
assert spar.scene_top_z() == 0.0
anchor = spar.add_plan_anchor(
    "Anchor_Test", "rack", "test", (11.0, -7.0, 0.0), (4.0, 1.0, 3.0),
    rotation=(0.0, 0.0, math.radians(90.0)),
)
root = spar.semantic_root(anchor, spar_type="rack")
child = spar.add_box_local(root, "rack_test_body", (2.0, 0.0, 1.0), (1.0, 2.0, 2.0))
bpy.context.view_layer.update()

root_anchor_error = max(
    abs(root.matrix_world[row][column] - anchor.matrix_world[row][column])
    for row in range(4)
    for column in range(4)
)
assert root_anchor_error < 1e-6, (root.matrix_world, anchor.matrix_world)
assert child.parent == root
assert child.matrix_parent_inverse == Matrix.Identity(4)
assert child.location == Vector((2.0, 0.0, 1.0))
assert tuple(child.dimensions) == (1.0, 2.0, 2.0)
expected = root.matrix_world @ Vector((2.0, 0.0, 1.0))
assert (child.matrix_world.translation - expected).length < 1e-6
assert spar.dimensions_from_anchor(anchor) == (4.0, 1.0, 3.0)
guide_center, guide_dimensions = spar.guide_frame("Anchor_Test")
assert guide_center == (11.0, -7.0, 0.0)
assert guide_dimensions == (4.0, 1.0, 3.0)
assert root.name == "ASM_Anchor_Test"
assert root["spar_type"] == "rack"

bad_anchor = spar.add_plan_anchor(
    "Anchor_Type_Test", "inspection_target", "test", (0.0, 0.0, 0.0),
    (1.0, 1.0, 1.0), role="featured",
)
try:
    spar.semantic_root(bad_anchor, spar_type="barrel")
except ValueError:
    pass
else:
    raise AssertionError("semantic_root accepted an incorrect anchor type")

boundary = spar.add_plan_guide(
    "PLAN_SiteBoundary", "boundary", (0.0, 0.0, 0.0), (40.0, 40.0, 0.0)
)
fences, gate = spar.add_rect_boundary_anchors(
    boundary, gate_side="south", gate_offset=4.0, gate_width=4.0
)
assert len(fences) == 5
assert tuple(gate.location) == (4.0, -20.0, 0.0)
assert all(tuple(obj.location)[2] == 0.0 for obj in [*fences, gate])
assert all(
    all(value > 0.0 for value in spar.dimensions_from_anchor(obj))
    for obj in [*fences, gate]
)
ground_anchor = spar.add_plan_anchor(
    "Ground_Test", "ground", "PLAN_SiteBoundary", (0.0, 0.0, 0.0),
    (40.0, 40.0, 0.05),
)
ground_root = spar.semantic_root(ground_anchor)
ground = spar.add_ground_from_boundary(ground_root, boundary)
bpy.context.view_layer.update()
ground_dimensions = tuple(round(value, 6) for value in ground.dimensions)
assert ground_dimensions == (40.0, 40.0, 0.05), ground_dimensions
row = spar.add_plan_anchor_row(
    "Row", "crate", "test", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0), 3, 2.0
)
assert [tuple(obj.location) for obj in row] == [
    (-2.0, 0.0, 0.0), (0.0, 0.0, 0.0), (2.0, 0.0, 0.0)
]
grid = spar.add_plan_anchor_grid(
    "Grid", "barrel", "test", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0),
    2, 2, (2.0, 4.0),
)
assert [tuple(obj.location) for obj in grid] == [
    (-1.0, -2.0, 0.0), (1.0, -2.0, 0.0),
    (-1.0, 2.0, 0.0), (1.0, 2.0, 0.0),
]

sites = spar.add_ordered_sites(
    "SITE_Test", "test", [(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], face="next"
)
assert [obj.name for obj in sites] == ["SITE_Test_01", "SITE_Test_02"]
assert [obj["spar_order"] for obj in sites] == [0, 1]
assert spar.guide("PLAN_SiteBoundary") == boundary
assert spar.site("SITE_Test_01") == sites[0]
assert spar.route_headings([(0, 0, 0), (0, 2, 0)]) == [math.pi / 2, math.pi / 2]
assert spar.yaw_toward((0, 0, 0), (-1, 0, 0)) == math.pi
target_anchor = spar.add_plan_anchor(
    "Target_Test", "inspection_target", "test", (5.0, 0.0, 0.0),
    (1.0, 1.0, 1.0), role="featured",
)
target_root = spar.semantic_root(target_anchor)
spar.add_box_local(target_root, "target_body", (0.0, 0.0, 0.5), (1.0, 1.0, 1.0))
bpy.context.view_layer.update()
heading = spar.inspection_yaw(
    target_root, (0.0, 0.0, 0.0), min_range=3.0, max_range=7.0
)
assert heading == 0.0
headed_sites = spar.add_ordered_sites(
    "SITE_Headed", "headed", [(0, 0, 0), (0, 1, 0)],
    headings=[heading, -math.pi / 2],
)
assert math.isclose(headed_sites[1].rotation_euler.z, -math.pi / 2, abs_tol=1e-6)
assert spar.scene_top_z() >= 2.0
print("worldgen helper check passed")
