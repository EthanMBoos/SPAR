"""
Export a trained SB3 SAC policy to ONNX for deployment via OnnxNavigateNode.

Usage:
    python export_onnx.py [--model sac_spar_nav] [--out sac_spar_nav.onnx]
"""

import argparse
import torch
from stable_baselines3 import SAC


class _DeterministicActor(torch.nn.Module):
    """Wraps the SB3 SAC actor for deterministic (mean) inference."""
    def __init__(self, actor):
        super().__init__()
        self.actor = actor

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        latent = self.actor.latent_pi(obs)
        return torch.tanh(self.actor.mu(latent))  # squashed to [-1, 1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True,
                        help="Path to saved SB3 zip (without .zip), e.g. tb_logs/sac_spar_nav_20260622_143012")
    parser.add_argument("--out",   default=None,
                        help="Output ONNX path (default: same dir/name as model with .onnx)")
    args = parser.parse_args()

    out = args.out or args.model + ".onnx"

    model  = SAC.load(args.model)
    actor  = _DeterministicActor(model.policy.actor).eval()
    dummy  = torch.zeros(1, 5, dtype=torch.float32)  # [batch, obs_dim]

    torch.onnx.export(
        actor,
        dummy,
        out,
        input_names=["obs"],
        output_names=["action"],
        dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
        opset_version=11,
    )
    print(f"Exported: {out}")
    print("  input:  obs    [batch, 5]  — [dx_m, dy_m, heading_sin, heading_cos, speed_ms]")
    print("  output: action [batch, 2]  — [throttle, steering] in [-1, 1]")


if __name__ == "__main__":
    main()
