"""
SparEnv — Gymnasium-compatible wrapper around the SPAR C++ core.

The monitor is NOT part of the training loop. Policies train on raw kinematics;
the RuntimeMonitor only runs at deployment in spar_rover/main.cpp. Catch fractions
are measured from deployment session logs produced by OnnxNavigateNode.

Build the native extension first:
    cmake -B build -DSPAR_BACKEND=kinematic -DSPAR_ENABLE_PYTHON=ON
    cmake --build build --target spar_bindings

Quick-start:
    import sys; sys.path.insert(0, "build/python")
    from spar_env import SparEnv
    env = SparEnv()
    obs, info = env.reset()
    obs, reward, done, truncated, info = env.step(env.action_space.sample())
"""

from __future__ import annotations
import sys
import os
import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    import gym
    from gym import spaces

try:
    import spar_bindings as _cpp
except ImportError as e:
    raise ImportError(
        "spar_bindings native module not found. "
        "Build with: cmake -B build -DSPAR_BACKEND=kinematic -DSPAR_ENABLE_PYTHON=ON "
        "&& cmake --build build --target spar_bindings"
    ) from e


class SparEnv(gym.Env):
    """Single-rover navigation env backed by SPAR's KinematicBackend.

    Observation space: [dx_m, dy_m, heading_sin, heading_cos, speed_ms]  (5 floats)
    Action space:      [throttle, steering]                               (2 floats in [-1, 1])
    Reward:            -haversine_distance_to_goal (metres, dense)
    Done:              arrival within 1 m of goal waypoint
    Truncated:         step limit (1200 steps = 60 s at 20 Hz) reached
    """

    metadata = {"render_modes": []}

    _OBS_LOW  = np.array([-500.0, -500.0, -1.0, -1.0, -5.0], dtype=np.float32)
    _OBS_HIGH = np.array([ 500.0,  500.0,  1.0,  1.0,  5.0], dtype=np.float32)

    def __init__(
        self,
        kinematic_params:   _cpp.KinematicParams   | None = None,
        goal:               _cpp.GoalContext       | None = None,
        degradation_params: _cpp.DegradationParams | None = None,
    ):
        super().__init__()
        kp  = kinematic_params   or _cpp.KinematicParams()
        g   = goal               or _cpp.GoalContext()
        deg = degradation_params or _cpp.DegradationParams()

        self._core = _cpp.SparCore(kp, g, deg)

        self.observation_space = spaces.Box(
            low=self._OBS_LOW, high=self._OBS_HIGH, dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32
        )

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        obs = np.array(self._core.reset(), dtype=np.float32)
        return obs, {}

    def step(self, action):
        action   = np.asarray(action, dtype=np.float32)
        throttle = float(np.clip(action[0], -1.0, 1.0))
        steering = float(np.clip(action[1], -1.0, 1.0))

        result = self._core.step(throttle, steering)

        obs       = np.array(result.obs, dtype=np.float32)
        reward    = float(result.reward)
        done      = bool(result.done)
        truncated = bool(result.truncated)
        return obs, reward, done, truncated, {}

    def render(self):
        pass

    def close(self):
        pass
