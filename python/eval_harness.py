"""
eval_harness.py — run all Phase 1 eval scenarios and print catch-fraction table.

Implements the two-variant rule from docs/eval-protocol.md:
  Each scenario runs twice:
    1. clean   — baseline DegradationParams (all zeros), regardless of scenario
    2. degraded — scenario-specific DegradationParams

  The difference in catch_frac between variants isolates transport timing.
  Exception: policy_exploit always uses clean transport (the exploit is in the
  policy node, not the degradation layer).

Usage:
  python eval_harness.py [--build-dir BUILD_DIR] [--binary BINARY_PATH]

Defaults:
  build-dir: ./build
  binary:    <build-dir>/spar_rover/spar_rover

Requirements:
  - spar_rover built in kinematic mode:
      cmake -B build -DSPAR_BACKEND=kinematic [-DSPAR_ENABLE_EXPLOIT_NODE=ON]
      cmake --build build
  - The binary must be the EXPLOIT variant for policy_exploit to work.
    Build a separate binary or rebuild with -DSPAR_ENABLE_EXPLOIT_NODE=ON.
"""

from __future__ import annotations
import argparse
import glob
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from parse_session_log import parse, print_table, SessionSummary

# ---------------------------------------------------------------------------
# Scenario definitions — must match DegradationScenarios.h
# ---------------------------------------------------------------------------

# (scenario_name, is_degraded_variant, needs_exploit_node)
_SCENARIOS = [
    ("baseline",       False, False),
    ("pose_dropout",   False, False),  # clean variant
    ("pose_dropout",   True,  False),  # degraded variant
    ("pose_jitter",    False, False),  # clean variant
    ("pose_jitter",    True,  False),  # degraded variant
    ("policy_exploit", False, True),   # always clean transport
]

_TIMEOUT_S = 90  # max seconds per run (mission is ~60 s at 20 Hz)


def find_latest_log(log_dir: str = "/tmp") -> Path | None:
    pattern = os.path.join(log_dir, "spar_*.log")
    logs = sorted(glob.glob(pattern), key=os.path.getmtime, reverse=True)
    return Path(logs[0]) if logs else None


def run_scenario(binary: Path, scenario: str, is_degraded: bool) -> SessionSummary | None:
    # The degraded flag is encoded in the scenario name for the C++ binary.
    # When is_degraded=False we use SPAR_SCENARIO=baseline so the binary
    # applies zero degradation, but we label the summary with the scenario
    # name + "(clean)" to distinguish the two variants in the table.
    env = os.environ.copy()
    # policy_exploit: transport is always baseline but the scenario name must be
    # "policy_exploit" to activate ExploitNode at runtime.
    # All other scenarios: clean variant uses "baseline" transport.
    if is_degraded or scenario == "policy_exploit":
        env["SPAR_SCENARIO"] = scenario
    else:
        env["SPAR_SCENARIO"] = "baseline"

    # For the clean variant we want the label to still say which scenario
    # this is the clean counterpart of, so we pass it as a comment in the
    # scenario name when the binary supports it. For now we simply record
    # a synthetic label in the summary after parsing.
    timed_out = False
    try:
        t0 = time.monotonic()
        result = subprocess.run(
            [str(binary)],
            env=env,
            timeout=_TIMEOUT_S,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        elapsed = time.monotonic() - t0
    except subprocess.TimeoutExpired:
        timed_out = True
        # subprocess.run() already sends SIGKILL before raising TimeoutExpired.
        # The log file exists but may be missing the last ~40 frames (one OS buffer).
        elapsed = _TIMEOUT_S
        print(f"  [timeout after {_TIMEOUT_S}s — parsing partial log]", flush=True)
    except FileNotFoundError:
        print(f"  [error] binary not found: {binary}", file=sys.stderr)
        return None

    log_path = find_latest_log()
    if log_path is None:
        print(f"  [error] no log found in /tmp after running {scenario}", file=sys.stderr)
        return None

    summary = parse(log_path)
    # Override scenario label so the table shows the variant clearly.
    suffix = " (clean)" if not is_degraded else ""
    if timed_out:
        suffix += " [timeout]"
    summary.scenario = scenario + suffix
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="SPAR Phase 1 eval harness")
    parser.add_argument("--build-dir", default="build",
                        help="CMake build directory (default: build)")
    parser.add_argument("--binary", default=None,
                        help="Override path to spar_rover binary")
    parser.add_argument("--exploit-binary", default=None,
                        help="Separate binary built with SPAR_ENABLE_EXPLOIT_NODE=ON")
    args = parser.parse_args()

    build_dir = Path(args.build_dir)
    default_binary  = build_dir / "spar_rover" / "spar_rover"
    standard_binary = Path(args.binary) if args.binary else default_binary
    exploit_binary  = Path(args.exploit_binary) if args.exploit_binary else standard_binary

    for b in {standard_binary, exploit_binary}:
        if not b.exists():
            print(f"Binary not found: {b}", file=sys.stderr)
            print("Build with: cmake -B build -DSPAR_BACKEND=kinematic "
                  "[-DSPAR_ENABLE_EXPLOIT_NODE=ON] && cmake --build build",
                  file=sys.stderr)
            sys.exit(1)

    summaries: list[SessionSummary] = []
    for scenario, is_degraded, needs_exploit in _SCENARIOS:
        binary = exploit_binary if needs_exploit else standard_binary
        variant_label = "degraded" if is_degraded else "clean"
        print(f"Running {scenario:<20} [{variant_label}]  binary={binary.name} ...",
              flush=True)
        s = run_scenario(binary, scenario, is_degraded)
        if s is not None:
            summaries.append(s)

    if not summaries:
        print("No results collected.", file=sys.stderr)
        sys.exit(1)

    print()
    print_table(summaries)
    print()
    print("Two-variant delta (degraded catch_frac − clean catch_frac):")
    paired = {}
    for s in summaries:
        base = s.scenario.removesuffix(" (clean)")
        paired.setdefault(base, []).append(s)
    for scenario, pair in paired.items():
        if len(pair) == 2:
            clean   = next((p for p in pair if "(clean)" in p.scenario), pair[0])
            degraded = next((p for p in pair if "(clean)" not in p.scenario), pair[1])
            delta = degraded.catch_frac - clean.catch_frac
            print(f"  {scenario:<20}  Δcatch_frac = {delta:+.4f}")


if __name__ == "__main__":
    main()
