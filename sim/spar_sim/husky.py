"""Deterministic ground-motion and encoder helpers for the Husky demo."""

from __future__ import annotations

import math

import mujoco
import numpy as np


WHEEL_NAMES = (
    "front_left",
    "rear_left",
    "front_right",
    "rear_right",
)
WHEEL_RADIUS_M = 0.1651
TRACK_WIDTH_M = 0.555
ENCODER_TICKS_PER_M = 78000.0

# Ground autonomy is intentionally a fast kinematic demonstration. Nav2's
# velocity smoother owns acceleration limits; the simulator only rejects
# commands outside this generous operating envelope.
MAX_LINEAR_M_S = 2.0
MAX_ANGULAR_RAD_S = 2.0


def quantize_wheel_angle(angle):
    ticks = round(angle * WHEEL_RADIUS_M * ENCODER_TICKS_PER_M)
    return ticks / (WHEEL_RADIUS_M * ENCODER_TICKS_PER_M)


def clamp_command(linear, angular):
    return (
        max(-MAX_LINEAR_M_S, min(MAX_LINEAR_M_S, linear)),
        max(-MAX_ANGULAR_RAD_S, min(MAX_ANGULAR_RAD_S, angular)),
    )


def wheel_speeds(linear, angular):
    """Return exact left/right wheel rates for a differential-drive command."""
    left = linear - angular * TRACK_WIDTH_M / 2.0
    right = linear + angular * TRACK_WIDTH_M / 2.0
    return left / WHEEL_RADIUS_M, right / WHEEL_RADIUS_M


def yaw_from_quaternion(quaternion):
    qw, qx, qy, qz = quaternion
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


class KinematicDrive:
    """Advance the Husky directly from ``cmd_vel`` while MuJoCo steps the X2.

    The free body and four wheel joints remain in MjData so the viewer,
    cameras, lidar, GPS, IMU, and encoder publishers all observe one coherent
    state. Husky contacts, motors, tire forces, and drivetrain dynamics are
    deliberately outside this student autonomy demonstration.
    """

    def __init__(
        self, model, data, base_joint_id, wheel_joint_ids, footprint_geom_id
    ):
        self._base_qpos = int(model.jnt_qposadr[base_joint_id])
        self._base_dof = int(model.jnt_dofadr[base_joint_id])
        self._wheel_qpos = tuple(
            int(model.jnt_qposadr[joint]) for joint in wheel_joint_ids)
        self._wheel_dofs = tuple(
            int(model.jnt_dofadr[joint]) for joint in wheel_joint_ids)
        self._footprint_geom = int(footprint_geom_id)
        # Keep the compiled broad-phase entry but never let this query shape
        # participate in a dynamics step.
        model.geom_contype[self._footprint_geom] = 0
        model.geom_conaffinity[self._footprint_geom] = 0

        root = data.qpos[self._base_qpos:self._base_qpos + 7]
        self._x = float(root[0])
        self._y = float(root[1])
        self._z = float(root[2])
        self._yaw = yaw_from_quaternion(root[3:7])
        self._wheel_angles = [float(data.qpos[address])
                              for address in self._wheel_qpos]
        self._world_velocity = np.zeros(3)
        self.world_acceleration = np.zeros(3)
        self.blocked = False

    def _write_root(self, data, x, y, yaw):
        data.qpos[self._base_qpos:self._base_qpos + 7] = (
            x,
            y,
            self._z,
            math.cos(yaw / 2.0),
            0.0,
            0.0,
            math.sin(yaw / 2.0),
        )

    def _collides(self, model, data):
        """Temporarily enable the footprint and query, never apply, contacts."""
        previous_contype = int(model.geom_contype[self._footprint_geom])
        previous_conaffinity = int(
            model.geom_conaffinity[self._footprint_geom])
        model.geom_contype[self._footprint_geom] = 1
        model.geom_conaffinity[self._footprint_geom] = 1
        try:
            mujoco.mj_forward(model, data)
            return any(
                contact.geom1 == self._footprint_geom
                or contact.geom2 == self._footprint_geom
                for contact in data.contact
            )
        finally:
            model.geom_contype[self._footprint_geom] = previous_contype
            model.geom_conaffinity[
                self._footprint_geom] = previous_conaffinity

    def advance(self, model, data, linear, angular, dt):
        linear, angular = clamp_command(linear, angular)

        # Midpoint integration is exact for straight motion and second-order
        # accurate for a constant-curvature command.
        midpoint_yaw = self._yaw + angular * dt / 2.0
        candidate_x = self._x + linear * math.cos(midpoint_yaw) * dt
        candidate_y = self._y + linear * math.sin(midpoint_yaw) * dt
        candidate_yaw = math.atan2(
            math.sin(self._yaw + angular * dt),
            math.cos(self._yaw + angular * dt),
        )

        self._write_root(data, candidate_x, candidate_y, candidate_yaw)
        self.blocked = self._collides(model, data)
        if self.blocked:
            # Preserve an in-place escape turn when translation is blocked.
            self._write_root(data, self._x, self._y, candidate_yaw)
            if angular != 0.0 and not self._collides(model, data):
                linear = 0.0
                candidate_x = self._x
                candidate_y = self._y
            else:
                linear = angular = 0.0
                candidate_x = self._x
                candidate_y = self._y
                candidate_yaw = self._yaw
        self._x = candidate_x
        self._y = candidate_y
        self._yaw = candidate_yaw

        left, right = wheel_speeds(linear, angular)
        wheel_rates = (left, left, right, right)
        for index, rate in enumerate(wheel_rates):
            self._wheel_angles[index] += rate * dt

        self._write_root(data, self._x, self._y, self._yaw)
        for address, angle in zip(self._wheel_qpos, self._wheel_angles):
            data.qpos[address] = angle

        velocity = np.array([
            linear * math.cos(self._yaw),
            linear * math.sin(self._yaw),
            0.0,
        ])
        self.world_acceleration = (velocity - self._world_velocity) / dt
        self._world_velocity = velocity
        data.qvel[self._base_dof:self._base_dof + 6] = (
            velocity[0], velocity[1], 0.0, 0.0, 0.0, angular)
        for address, rate in zip(self._wheel_dofs, wheel_rates):
            data.qvel[address] = rate

        mujoco.mj_forward(model, data)
        return linear, angular
