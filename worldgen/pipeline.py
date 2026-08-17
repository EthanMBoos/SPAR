"""Run a complete world-family authoring, export, and validation pipeline."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

from . import generation
from . import stage as stage_runner
from .start_blender import blender_executable, launch_visible_blender, wait_until_ready


REPO = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate, export, and validate one complete SPAR world."
    )
    parser.add_argument("world", help="output world name")
    parser.add_argument("--family")
    parser.add_argument("--model")
    parser.add_argument("--effort")
    parser.add_argument("--seed", default=generation.environment_seed())
    parser.add_argument("--brief", default=generation.environment_brief())
    parser.add_argument(
        "--fresh",
        action="store_true",
        help="delete this world's existing authoring artifacts before starting",
    )
    parser.add_argument(
        "--start-at",
        metavar="STAGE",
        help="resume at one stage using the currently open Blender scene",
    )
    parser.add_argument(
        "--stop-after", metavar="STAGE", help="stop after one stage without exporting"
    )
    parser.add_argument(
        "--reuse-blender",
        action="store_true",
        help="use the current Blender scene instead of starting a clean window",
    )
    parser.add_argument(
        "--no-export", action="store_true", help="finish authoring without export checks"
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="print the planned stages without running"
    )
    return parser.parse_args()


def selected_stages(
    all_stages: list[str], start_at: str | None, stop_after: str | None
) -> list[str]:
    if start_at and start_at not in all_stages:
        raise ValueError(f"unknown --start-at stage: {start_at}")
    if stop_after and stop_after not in all_stages:
        raise ValueError(f"unknown --stop-after stage: {stop_after}")
    start = all_stages.index(start_at) if start_at else 0
    stop = all_stages.index(stop_after) + 1 if stop_after else len(all_stages)
    if start >= stop:
        raise ValueError("--start-at must not come after --stop-after")
    return all_stages[start:stop]


def export_and_validate(world: str) -> None:
    blend = REPO / "artifacts" / "worldgen" / world / "waypoints.blend"
    if not blend.is_file():
        raise RuntimeError(f"final waypoint scene does not exist: {blend}")
    export_command = [
        blender_executable(),
        "--background",
        str(blend),
        "--python-exit-code",
        "1",
        "--python",
        str(ROOT / "export.py"),
        "--",
        "--world",
        world,
        "--navigation-goals",
    ]
    print("worldgen: exporting Blender scene to MJCF", flush=True)
    subprocess.run(export_command, cwd=REPO, check=True)
    print("worldgen: validating exported MJCF and rollouts", flush=True)
    subprocess.run(
        [sys.executable, str(ROOT / "check_export.py"), world],
        cwd=REPO,
        check=True,
    )


def main() -> int:
    args = parse_args()
    requested_world = args.world
    try:
        args.world = stage_runner.normalize_world_name(args.world)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    if args.world != requested_world:
        print(f"worldgen: normalized world name to {args.world}", flush=True)

    if args.fresh and args.reuse_blender:
        raise SystemExit("--fresh cannot be combined with --reuse-blender")
    if args.start_at and not args.reuse_blender:
        raise SystemExit("--start-at requires --reuse-blender with the expected scene open")

    try:
        if args.reuse_blender:
            recorded = generation.load_manifest(args.world)
            family = recorded["family"]
        else:
            family = args.family or generation.DEFAULT_FAMILY
        stage_runner.validate_name(family, "family")
        all_stage_paths = stage_runner.available_stages(family)
        all_stages = list(all_stage_paths)
        if not all_stages:
            raise ValueError(f"family has no stages: {family}")
        stages = selected_stages(all_stages, args.start_at, args.stop_after)
    except ValueError as error:
        raise SystemExit(str(error)) from error

    # DevNote: Resume trusts the live Blender scene; choosing a checkpoint here
    # could silently continue from the wrong accepted state.
    artifacts = REPO / "artifacts" / "worldgen" / args.world
    if args.fresh and not args.dry_run:
        stage_runner.clean_world(args.world)
    elif not args.reuse_blender and artifacts.exists() and not args.dry_run:
        raise SystemExit(
            f"authoring artifacts already exist: {artifacts}\n"
            "choose a new world name or rerun explicitly with --fresh"
        )

    try:
        manifest = generation.resolve_manifest(
            args.world,
            all_stage_paths,
            family=args.family,
            seed=args.seed,
            brief=args.brief,
            model=args.model,
            effort=args.effort,
            existing=args.reuse_blender,
            persist=not args.dry_run,
            shared_prompt=stage_runner.SHARED_PROMPT,
        )
    except ValueError as error:
        raise SystemExit(str(error)) from error

    if args.dry_run:
        print(f"world: {manifest['world']}")
        print(f"family: {manifest['family']}")
        print(f"seed: {manifest['seed']}")
        print(f"brief: {manifest['brief'] or '(none)'}")
        for name in stages:
            print(f"{name} seed={manifest['stages'][name]['seed']}")
        if not args.stop_after and not args.no_export:
            print("export")
            print("validate")
        return 0

    print(
        f"worldgen: recipe {generation.manifest_path(args.world)} "
        f"(seed={manifest['seed']})",
        flush=True,
    )

    try:
        if args.reuse_blender:
            wait_until_ready()
        else:
            launch_visible_blender()
            wait_until_ready()

        for name in stages:
            result = stage_runner.run_stage(
                manifest,
                name,
            )
            if result:
                print(
                    f"worldgen stopped at {name}; Blender remains open for inspection",
                    file=sys.stderr,
                )
                return result

        partial = args.stop_after is not None or stages[-1] != all_stages[-1]
        if partial or args.no_export:
            print("worldgen: authoring stages complete; export skipped", flush=True)
            return 0
        export_and_validate(args.world)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        raise SystemExit(f"worldgen failed: {error}") from error

    print(f"worldgen complete: {args.world}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
