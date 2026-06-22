"""
parse_session_log.py — parse a SPAR session log and compute catch fractions.

Session log format (TSV, see docs/mission-design.md):
  # scenario: <name>
  frame  timestamp_us  policy_id  task_id  outcome  triggered_invariant
         invariant_flags  throttle  steering  pose_age_us  pose_degraded  mission_outcome

Catch fraction definitions (docs/architecture.md):
  Architectural-only  — monitor fired (FALL or HALT), behavioral monitor did not
  Behavioral-only     — behavioral monitor fired, monitor did not (Phase 4b)
  Both                — both fired (Phase 4b)
  Neither             — monitor did not fire AND mission failed (the uncatchable residual)

Phase 1 computes two cells from this log:
  catch_frac  = (FALL + HALT) / total_frames
  neither     = frames with outcome==PASS on a failure-terminal session
"""

from __future__ import annotations
import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


@dataclass
class SessionSummary:
    scenario:        str   = ""
    policy_id:       str   = ""
    total_frames:    int   = 0
    pass_count:      int   = 0
    fall_count:      int   = 0
    halt_count:      int   = 0
    mission_outcome: str   = "running"  # terminal frame value
    neither_count:   int   = 0          # PASS frames on a failure-terminal session
    catch_frac:      float = 0.0


def parse(log_path: str | Path) -> SessionSummary:
    path = Path(log_path)
    s = SessionSummary()

    with path.open() as fh:
        # First line: # scenario: <name>
        header = fh.readline().strip()
        if header.startswith("# scenario:"):
            s.scenario = header.split(":", 1)[1].strip()

        reader = csv.DictReader(fh, delimiter="\t")
        rows = list(reader)

    if not rows:
        return s

    s.policy_id = rows[0].get("policy_id", "")

    for row in rows:
        s.total_frames += 1
        outcome = row.get("outcome", "")
        if outcome == "PASS":
            s.pass_count += 1
        elif outcome == "FALL":
            s.fall_count += 1
        elif outcome == "HALT":
            s.halt_count += 1

    # Terminal frame carries the mission outcome oracle.
    # On timeout (SIGKILL), the last row may be partial (None fields). Use the
    # last complete row — if none exists, default to "running".
    complete_rows = [r for r in rows if r.get("mission_outcome") is not None]
    s.mission_outcome = complete_rows[-1]["mission_outcome"] if complete_rows else "running"

    # "Neither" cell: monitor never fired on a session that ended in failure.
    if s.mission_outcome == "failure":
        s.neither_count = s.pass_count  # all PASS frames on a failed mission

    if s.total_frames > 0:
        s.catch_frac = (s.fall_count + s.halt_count) / s.total_frames

    return s


def print_table(summaries: list[SessionSummary]) -> None:
    header = (
        f"{'scenario':<20} {'policy':<10} {'total':>6} "
        f"{'PASS':>6} {'FALL':>6} {'HALT':>6} "
        f"{'catch_frac':>10} {'mission':>10} {'neither':>8}"
    )
    print(header)
    print("-" * len(header))
    for s in summaries:
        print(
            f"{s.scenario:<20} {s.policy_id:<10} {s.total_frames:>6} "
            f"{s.pass_count:>6} {s.fall_count:>6} {s.halt_count:>6} "
            f"{s.catch_frac:>10.4f} {s.mission_outcome:>10} {s.neither_count:>8}"
        )


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <log_file> [log_file ...]")
        sys.exit(1)

    summaries = [parse(p) for p in sys.argv[1:]]
    print_table(summaries)
