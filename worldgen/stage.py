#!/usr/bin/env python3
"""Run one checked-in BlenderMCP family stage in a fresh model session."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess

from . import generation


REPO = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parent
FAMILY_ROOT = ROOT / "families"
SHARED_PROMPT = ROOT / "prompts" / "shared_contract.md"
MCP_CONFIG = ROOT / "mcp.json"
NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_]*$")


def validate_name(value: str, label: str) -> None:
    if not NAME_RE.fullmatch(value):
        raise ValueError(
            f"{label} must contain only letters, digits, and underscores"
        )


def normalize_world_name(value: str) -> str:
    normalized = value.lower()
    validate_name(normalized, "world")
    return normalized


def available_stages(family: str) -> dict[str, Path]:
    validate_name(family, "family")
    directory = FAMILY_ROOT / family / "prompts"
    if not directory.is_dir():
        return {}
    return {path.stem: path for path in sorted(directory.glob("[0-9][0-9]_*.md"))}


def render_prompt(manifest: dict, stage: str) -> str:
    family = manifest["family"]
    world = manifest["world"]
    stages = available_stages(family)
    if stage not in stages:
        choices = ", ".join(stages) or "none"
        raise ValueError(f"unknown stage {stage!r}; available stages: {choices}")
    shared = SHARED_PROMPT.read_text(encoding="utf-8")
    contract_path = generation.family_contract_path(family)
    contract = (
        contract_path.read_text(encoding="utf-8")
        if contract_path.is_file()
        else ""
    )
    prompt = stages[stage].read_text(encoding="utf-8")
    stage_seed = manifest["stages"][stage]["seed"]
    brief = manifest["brief"] or "No instance-specific override."
    spawn_defaults = json.dumps(manifest.get("spawn_defaults", {}), indent=2)
    spawn_context = (
        "Repo-sampled spawn defaults (MuJoCo world coordinates, ENU metres "
        "and yaw radians):\n\n"
        f"```json\n{spawn_defaults}\n```\n"
        if stage == "01_plan_topology"
        else ""
    )
    context = f"""# Run context

- World identity: `{world}`
- Family: `{family}`
- World seed: `{manifest['seed']}`
- Stage seed: `{stage_seed}`

{spawn_context}
Instance brief:

<instance_brief>
{brief}
</instance_brief>

The world name is an output identity, never a source of creative choices. Apply
only supported brief content owned by this numbered stage. Existing scene state
is authoritative for decisions made by earlier stages. For any discretionary
random choices, use `random.Random({stage_seed})`; never use time, process state,
or the world name as entropy.
"""
    return (
        "\n\n".join(part for part in (shared, contract, context, prompt) if part)
        .replace("<REPO>", str(REPO))
        .replace("<WORLD>", world)
        .replace("<FAMILY>", family)
    )


def clean_world(world: str) -> None:
    target = REPO / "artifacts" / "worldgen" / world
    if target.is_dir():
        shutil.rmtree(target)
        print(f"removed {target}")
    else:
        print(f"already clean: {target}")


def run_visible(command: list[str], log_path: Path) -> int:
    """Stream concise boundaries to the terminal and retain the full event log."""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen(
        command,
        cwd=REPO,
        stdout=subprocess.PIPE,
        text=True,
        bufsize=1,
    )
    assert process.stdout is not None
    with log_path.open("w", encoding="utf-8") as log:
        for line in process.stdout:
            log.write(line)
            log.flush()
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                print(line, end="", flush=True)
                continue
            if event.get("type") == "assistant":
                for block in event.get("message", {}).get("content", []):
                    if block.get("type") == "tool_use":
                        print(f"tool: {block.get('name', 'unknown')}", flush=True)
            elif event.get("type") == "user":
                for block in event.get("message", {}).get("content", []):
                    if block.get("type") != "tool_result" or not block.get("is_error"):
                        continue
                    content = block.get("content", "")
                    if isinstance(content, list):
                        content = " ".join(
                            item.get("text", "")
                            for item in content
                            if isinstance(item, dict)
                        )
                    summary = " ".join(str(content).split())[:800]
                    print(f"tool error: {summary}", flush=True)
            elif event.get("type") == "result":
                result = event.get("result")
                if result:
                    print(result, flush=True)
                cost = event.get("total_cost_usd")
                if cost is not None:
                    print(f"reported cost: ${float(cost):.4f}", flush=True)
    return process.wait()


def claude_command() -> list[str]:
    """Resolve Claude Code, including its supported Node-wrapper fallback."""
    claude = shutil.which("claude")
    if claude is None:
        raise RuntimeError("claude executable not found")
    path = Path(claude)
    try:
        header = path.resolve().read_bytes()[:4]
    except OSError:
        header = b""
    if header.startswith(b"#!") or header.startswith(b"\xcf\xfa\xed\xfe"):
        return [claude]

    wrapper = path.resolve().parent.parent / "cli-wrapper.cjs"
    node = shutil.which("node")
    if wrapper.is_file() and node is not None:
        return [node, str(wrapper)]
    raise RuntimeError(
        f"claude launcher is not executable and no Node wrapper was found: {claude}"
    )


def run_stage(
    manifest: dict,
    stage: str,
    *,
    dry_run: bool = False,
) -> int:
    family = manifest["family"]
    world = manifest["world"]
    model = manifest["model"]
    effort = manifest["effort"]
    validate_name(world, "world")
    validate_name(family, "family")
    prompt = render_prompt(manifest, stage)
    if dry_run:
        print(prompt)
        return 0
    if not MCP_CONFIG.is_file():
        raise RuntimeError(f"missing MCP config: {MCP_CONFIG}")

    # DevNote: Model sessions are intentionally stateless between stages. The
    # visible Blender scene and saved checkpoints are the only continuity.
    command = [
        *claude_command(),
        "--print",
        "--model",
        model,
        "--effort",
        effort,
        "--permission-mode",
        "auto",
        "--strict-mcp-config",
        "--mcp-config",
        str(MCP_CONFIG),
        "--tools",
        "",
        "--no-session-persistence",
        "--output-format",
        "stream-json",
        "--verbose",
        prompt,
    ]
    saved_prompt = generation.prompt_path(world, stage)
    saved_prompt.parent.mkdir(parents=True, exist_ok=True)
    saved_prompt.write_text(prompt, encoding="utf-8")
    print(
        f"worldgen: {stage} ({model}, {effort}, world_seed={manifest['seed']}, "
        f"stage_seed={manifest['stages'][stage]['seed']})",
        flush=True,
    )
    return run_visible(command, generation.trace_path(world, stage))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one visible BlenderMCP family stage for debugging."
    )
    parser.add_argument("world", nargs="?", help="output world name")
    parser.add_argument("stage", nargs="?", help="stage name, such as 01_plan_topology")
    parser.add_argument("--family")
    parser.add_argument("--model")
    parser.add_argument("--effort")
    parser.add_argument("--seed", default=generation.environment_seed())
    parser.add_argument("--brief", default=generation.environment_brief())
    parser.add_argument("--list", action="store_true", help="list available stages")
    parser.add_argument(
        "--clean",
        action="store_true",
        help="remove only artifacts/worldgen/<world> before a fresh run",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="print the rendered prompt without Claude"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.world:
        requested_world = args.world
        try:
            args.world = normalize_world_name(args.world)
        except ValueError as error:
            raise SystemExit(str(error)) from error
        if args.world != requested_world:
            print(f"worldgen: normalized world name to {args.world}", flush=True)

    if args.clean:
        if not args.world:
            raise SystemExit("world is required with --clean")
        if args.stage:
            raise SystemExit("--clean does not accept a stage")
        clean_world(args.world)
        return 0

    family = args.family or generation.DEFAULT_FAMILY
    if args.world and generation.manifest_path(args.world).is_file():
        try:
            family = generation.load_manifest(args.world)["family"]
        except ValueError as error:
            raise SystemExit(str(error)) from error
    try:
        stages = available_stages(family)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    if args.list:
        for name in stages:
            print(name)
        return 0
    if not args.world:
        raise SystemExit("world is required unless --list is used")
    if not args.stage:
        raise SystemExit("stage is required unless --list or --clean is used")

    try:
        existing = generation.manifest_path(args.world).is_file()
        artifacts = generation.manifest_path(args.world).parent
        if artifacts.exists() and not existing and not args.dry_run:
            raise ValueError(
                f"authoring artifacts exist without a generation recipe: {artifacts}"
            )
        manifest = generation.resolve_manifest(
            args.world,
            stages,
            family=args.family,
            seed=args.seed,
            brief=args.brief,
            model=args.model,
            effort=args.effort,
            existing=existing,
            persist=not args.dry_run,
            shared_prompt=SHARED_PROMPT,
        )
        return run_stage(manifest, args.stage, dry_run=args.dry_run)
    except (RuntimeError, ValueError) as error:
        raise SystemExit(str(error)) from error


if __name__ == "__main__":
    raise SystemExit(main())
