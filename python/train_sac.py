"""
SAC baseline training for SPAR rover navigation.
Trains without the monitor — catch fractions are measured at deployment.

Usage:
    python train_sac.py [--steps N] [--log-dir DIR]

Outputs (both in --log-dir, default tb_logs/):
    sac_spar_nav.zip   — trained policy for ONNX export
    SAC_*/             — tensorboard event files
"""

import argparse
import sys
import os
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..","build", "python"))

from spar_env import SparEnv

try:
    from stable_baselines3 import SAC
    from stable_baselines3.common.callbacks import EvalCallback
except ImportError:
    raise ImportError("pip install stable-baselines3")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps",   type=int, default=300_000)
    parser.add_argument("--log-dir", type=str, default="tb_logs")
    args = parser.parse_args()

    os.makedirs(args.log_dir, exist_ok=True)
    run_ts     = datetime.now().strftime("%Y%m%d_%H%M%S")
    model_path = os.path.join(args.log_dir, f"sac_spar_nav_{run_ts}")

    env      = SparEnv()
    eval_env = SparEnv()

    model = SAC(
        "MlpPolicy",
        env,
        verbose=1,
        tensorboard_log=args.log_dir,
        learning_starts=1000,
        batch_size=256,
    )

    eval_cb = EvalCallback(
        eval_env,
        eval_freq=10_000,
        n_eval_episodes=5,
        verbose=1,
    )

    model.learn(total_timesteps=args.steps, callback=eval_cb)
    model.save(model_path)
    print(f"Saved: {model_path}.zip")


if __name__ == "__main__":
    main()
