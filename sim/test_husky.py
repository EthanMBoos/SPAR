"""Host-side structural and kinematic tests for the Husky demo model."""

from __future__ import annotations

import math
import unittest

import mujoco
import numpy as np

from spar_sim.husky import (
    ENCODER_TICKS_PER_M,
    KinematicDrive,
    MAX_ANGULAR_RAD_S,
    MAX_LINEAR_M_S,
    TRACK_WIDTH_M,
    WHEEL_NAMES,
    WHEEL_RADIUS_M,
    clamp_command,
    quantize_wheel_angle,
    wheel_speeds,
    yaw_from_quaternion,
)
from spar_sim.raycast import NavigationRayFan, cast_world_collision_rays


class HuskyModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = mujoco.MjModel.from_xml_path("sim/worlds/blank.xml")

    def ids(self):
        model = self.model
        base_joint = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
        wheel_joints = [
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, f"{wheel}_wheel_joint")
            for wheel in WHEEL_NAMES
        ]
        footprint = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM,
            "husky_kinematic_footprint")
        return base_joint, wheel_joints, footprint

    def test_four_visible_wheels_without_ground_actuators(self) -> None:
        model = mujoco.MjModel.from_xml_path("sim/worlds/blank.xml")
        expected_positions = {
            "front_left": (0.256, 0.2775, 0.03282),
            "front_right": (0.256, -0.2775, 0.03282),
            "rear_left": (-0.256, 0.2775, 0.03282),
            "rear_right": (-0.256, -0.2775, 0.03282),
        }
        for wheel, expected in expected_positions.items():
            body = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, f"{wheel}_wheel_link")
            joint = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, f"{wheel}_wheel_joint")
            visual = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_GEOM, f"{wheel}_wheel_visual")
            actuator = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                f"{wheel}_wheel_actuator")
            collision = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_GEOM,
                f"{wheel}_wheel_collision")
            self.assertGreaterEqual(min(body, joint, visual), 0)
            self.assertEqual(actuator, -1)
            self.assertEqual(collision, -1)
            np.testing.assert_allclose(model.body_pos[body], expected, atol=1e-9)
            self.assertEqual(model.geom_contype[visual], 0)
            self.assertEqual(model.geom_conaffinity[visual], 0)
        _, _, footprint = self.ids()
        self.assertGreaterEqual(footprint, 0)
        self.assertEqual(model.geom_contype[footprint], 1)
        self.assertEqual(model.geom_conaffinity[footprint], 1)

    def test_exact_straight_and_turn_motion(self) -> None:
        model = self.model
        data = mujoco.MjData(model)
        base_joint, wheel_joints, footprint = self.ids()
        drive = KinematicDrive(
            model, data, base_joint, wheel_joints, footprint)
        base_qpos = model.jnt_qposadr[base_joint]
        dt = model.opt.timestep

        for _ in range(round(1.0 / dt)):
            drive.advance(model, data, 2.0, 0.0, dt)
        self.assertAlmostEqual(data.qpos[base_qpos], 2.0, places=9)
        self.assertAlmostEqual(data.qpos[base_qpos + 1], 0.0, places=9)
        self.assertAlmostEqual(
            yaw_from_quaternion(data.qpos[base_qpos + 3:base_qpos + 7]),
            0.0,
            places=9,
        )

        x_before = data.qpos[base_qpos]
        y_before = data.qpos[base_qpos + 1]
        for _ in range(round(0.5 / dt)):
            drive.advance(model, data, 0.0, 2.0, dt)
        self.assertAlmostEqual(data.qpos[base_qpos], x_before, places=9)
        self.assertAlmostEqual(data.qpos[base_qpos + 1], y_before, places=9)
        self.assertAlmostEqual(
            yaw_from_quaternion(data.qpos[base_qpos + 3:base_qpos + 7]),
            1.0,
            places=9,
        )

    def test_world_collision_rejects_translation(self) -> None:
        model = self.model
        data = mujoco.MjData(model)
        base_joint, wheel_joints, footprint = self.ids()
        drive = KinematicDrive(
            model, data, base_joint, wheel_joints, footprint)
        base_qpos = model.jnt_qposadr[base_joint]
        dt = model.opt.timestep

        # Face the rack centred at map (0, -3.9), then drive into it.
        for _ in range(round((math.pi / 2.0) / (2.0 * dt))):
            drive.advance(model, data, 0.0, -2.0, dt)
        for _ in range(round(3.0 / dt)):
            drive.advance(model, data, 2.0, 0.0, dt)
        self.assertTrue(drive.blocked)
        self.assertGreater(data.qpos[base_qpos + 1], -3.2)
        self.assertLess(data.qpos[base_qpos + 1], -2.8)

    def test_lidar_rays_see_invisible_world_collision_meshes(self) -> None:
        model = mujoco.MjModel.from_xml_path(
            "sim/worlds/utility_depot_40_v2.xml")
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        lidar_site = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, "lidar2d_0_laser")
        base_body = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        collision_geoms = model.geom_group == 3
        original_alpha = model.geom_rgba[collision_geoms, 3].copy()

        # Generated worlds separate visible group-2 meshes from transparent
        # group-3 collision meshes. The east-facing ray must return the
        # collision representation rather than the decorative visual mesh.
        geom_ids = np.full(1, -1, dtype=np.int32)
        distances = np.full(1, -1.0)
        groups = np.array([1, 0, 0, 1, 0, 0], dtype=np.uint8)
        cast_world_collision_rays(
            model,
            data,
            data.site_xpos[lidar_site].copy(),
            np.array([1.0, 0.0, 0.0]),
            groups,
            base_body,
            geom_ids,
            distances,
            1,
            25.0,
        )

        self.assertGreaterEqual(geom_ids[0], 0)
        self.assertGreater(distances[0], 0.0)
        self.assertLess(distances[0], 25.0)
        self.assertEqual(model.geom_group[geom_ids[0]], 3)
        np.testing.assert_array_equal(
            model.geom_rgba[collision_geoms, 3], original_alpha)

    def test_navigation_lidar_detects_low_pallet(self) -> None:
        model = mujoco.MjModel.from_xml_path(
            "sim/worlds/utility_depot_40_v2.xml")
        data = mujoco.MjData(model)
        base_joint = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
        base_qpos = int(model.jnt_qposadr[base_joint])
        base_body = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        lidar_site = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, "lidar2d_0_laser")

        # Approach pallet 02 from the east. Its 0.15 m top is well below the
        # puck's 0.50 m horizontal plane, but inside the vertical ray fan.
        yaw = math.pi
        data.qpos[base_qpos:base_qpos + 7] = (
            -6.5,
            -11.72,
            0.13228,
            math.cos(yaw / 2.0),
            0.0,
            0.0,
            math.sin(yaw / 2.0),
        )
        mujoco.mj_forward(model, data)
        ranges, hit_ids = NavigationRayFan(1, 25.0).scan(
            model,
            data,
            data.site_xpos[lidar_site].copy(),
            np.array([yaw]),
            base_body,
        )

        hit_name = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_GEOM, int(hit_ids[0]))
        self.assertEqual(hit_name, "col_auto_slat_0_001")
        self.assertAlmostEqual(ranges[0], 0.85, places=5)
        floor = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.assertEqual(model.geom_group[floor], 4)

    def test_navigation_lidar_ignores_overhead_clearance(self) -> None:
        model = mujoco.MjModel.from_xml_string("""
            <mujoco>
              <worldbody>
                <site name="lidar" pos="0 0 0.5"/>
                <geom name="overhead" type="box" pos="1 0 1.0"
                      size="0.15 0.5 0.1" group="3"/>
                <geom name="low_obstacle" type="box" pos="2 0 0.15"
                      size="0.15 0.5 0.1" group="3"/>
              </worldbody>
            </mujoco>
        """)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        lidar_site = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, "lidar")
        ranges, hit_ids = NavigationRayFan(1, 25.0).scan(
            model,
            data,
            data.site_xpos[lidar_site].copy(),
            np.array([0.0]),
            -1,
        )

        hit_name = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_GEOM, int(hit_ids[0]))
        self.assertEqual(hit_name, "low_obstacle")
        self.assertGreater(ranges[0], 1.0)
        self.assertLess(ranges[0], 2.0)

    def test_encoder_kinematics_and_command_limits(self) -> None:
        angle = 1.234567
        quantized = quantize_wheel_angle(angle)
        distance_error = abs(quantized - angle) * WHEEL_RADIUS_M
        self.assertLessEqual(distance_error, 0.5 / ENCODER_TICKS_PER_M)

        linear, angular = clamp_command(20.0, -20.0)
        self.assertEqual(linear, MAX_LINEAR_M_S)
        self.assertEqual(angular, -MAX_ANGULAR_RAD_S)

        linear = 1.25
        angular = -0.8
        dt = 0.1
        left_rate, right_rate = wheel_speeds(linear, angular)
        left_distance = left_rate * dt * WHEEL_RADIUS_M
        right_distance = right_rate * dt * WHEEL_RADIUS_M
        self.assertAlmostEqual(
            (left_distance + right_distance) / 2.0,
            linear * dt,
        )
        self.assertAlmostEqual(
            (right_distance - left_distance) / TRACK_WIDTH_M,
            angular * dt,
        )


if __name__ == "__main__":
    unittest.main()
