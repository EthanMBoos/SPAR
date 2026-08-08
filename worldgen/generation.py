"""Immutable inputs and provenance for one generated world instance."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import secrets
import shutil
import subprocess
from typing import Any


REPO = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parent
SCHEMA_VERSION = 1
MAX_BRIEF_CHARS = 1_000
MAX_SEED = (1 << 64) - 1
DEFAULT_FAMILY = "utility_depot"
DEFAULT_MODEL = "sonnet"
DEFAULT_EFFORT = "medium"


@dataclass(frozen=True)
class GenerationInputs:
    family: str
    seed: int
    brief: str
    model: str
    effort: str


def parse_seed(value: int | str | None) -> int | None:
    if value is None or value == "":
        return None
    try:
        seed = int(value)
    except (TypeError, ValueError) as error:
        raise ValueError("seed must be an unsigned 64-bit integer") from error
    if not 0 <= seed <= MAX_SEED:
        raise ValueError("seed must be an unsigned 64-bit integer")
    return seed


def validate_brief(brief: str | None) -> str | None:
    if brief is None:
        return None
    brief = brief.strip()
    if len(brief) > MAX_BRIEF_CHARS:
        raise ValueError(f"brief must not exceed {MAX_BRIEF_CHARS} characters")
    return brief


def derive_seed(seed: int, family: str, namespace: str) -> int:
    """Return an order-independent 64-bit stream seed for one namespace."""
    payload = f"spar-worldgen-v{SCHEMA_VERSION}\0{family}\0{seed}\0{namespace}"
    return int.from_bytes(hashlib.sha256(payload.encode()).digest()[:8], "big")


def manifest_path(world: str) -> Path:
    return REPO / "artifacts" / "worldgen" / world / "generation.json"


def prompt_path(world: str, stage: str) -> Path:
    return manifest_path(world).parent / "prompts" / f"{stage}.md"


def trace_path(world: str, stage: str) -> Path:
    return manifest_path(world).parent / "traces" / f"{stage}.jsonl"


def family_contract_path(family: str) -> Path:
    return ROOT / "families" / family / "variation_contract.md"


def _sha256_files(paths: list[Path]) -> str:
    digest = hashlib.sha256()
    for path in sorted(paths):
        relative = path.relative_to(REPO).as_posix().encode()
        digest.update(len(relative).to_bytes(4, "big"))
        digest.update(relative)
        contents = path.read_bytes()
        digest.update(len(contents).to_bytes(8, "big"))
        digest.update(contents)
    return digest.hexdigest()


def _generator_files(family: str) -> list[Path]:
    files = [path for path in ROOT.glob("*.py") if not path.name.startswith("test_")]
    files.extend((ROOT / "prompts").rglob("*.md"))
    files.extend((ROOT / "families" / family).rglob("*.py"))
    files.extend((ROOT / "families" / family).rglob("*.md"))
    if (ROOT / "mcp.json").is_file():
        files.append(ROOT / "mcp.json")
    return [path for path in files if "__pycache__" not in path.parts]


def _git_provenance() -> dict[str, Any]:
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=REPO,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=REPO,
                check=True,
                capture_output=True,
                text=True,
            ).stdout
        )
    except (OSError, subprocess.CalledProcessError):
        return {"commit": None, "dirty": None}
    return {"commit": commit, "dirty": dirty}


def _claude_version() -> str | None:
    claude = shutil.which("claude")
    if claude is None:
        return None
    try:
        return subprocess.run(
            [claude, "--version"],
            cwd=REPO,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def prompt_source_hash(shared: Path, contract: Path | None, prompt: Path) -> str:
    paths = [shared, prompt]
    if contract is not None and contract.is_file():
        paths.append(contract)
    return _sha256_files(paths)


def build_manifest(
    world: str,
    stages: dict[str, Path],
    *,
    family: str,
    seed: int,
    brief: str,
    model: str,
    effort: str,
    shared_prompt: Path,
) -> dict[str, Any]:
    contract = family_contract_path(family)
    return {
        "schema_version": SCHEMA_VERSION,
        "world": world,
        "family": family,
        "seed": seed,
        "brief": brief,
        "model": model,
        "effort": effort,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "generator": {
            **_git_provenance(),
            "source_sha256": _sha256_files(_generator_files(family)),
            "claude_version": _claude_version(),
        },
        "stages": {
            name: {
                "seed": derive_seed(seed, family, name),
                "prompt_sha256": prompt_source_hash(shared_prompt, contract, path),
            }
            for name, path in stages.items()
        },
    }


def write_manifest(manifest: dict[str, Any]) -> Path:
    path = manifest_path(manifest["world"])
    if path.exists():
        raise ValueError(f"generation recipe already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)
    return path


def load_manifest(world: str) -> dict[str, Any]:
    path = manifest_path(world)
    if not path.is_file():
        raise ValueError(f"generation recipe does not exist: {path}")
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid generation recipe: {path}") from error
    required = {
        "schema_version", "world", "family", "seed", "brief", "model",
        "effort", "generator", "stages",
    }
    if not isinstance(manifest, dict) or not required.issubset(manifest):
        raise ValueError(f"invalid generation recipe: {path}")
    if manifest["schema_version"] != SCHEMA_VERSION or manifest["world"] != world:
        raise ValueError(f"unsupported or mismatched generation recipe: {path}")
    if not all(
        isinstance(manifest[key], str)
        for key in ("family", "brief", "model", "effort")
    ):
        raise ValueError(f"invalid generation recipe: {path}")
    try:
        parse_seed(manifest["seed"])
        validate_brief(manifest["brief"])
        source_hash = manifest["generator"]["source_sha256"]
        stage_records = manifest["stages"]
        valid_stages = isinstance(stage_records, dict) and all(
            isinstance(record, dict)
            and isinstance(record.get("seed"), int)
            and 0 <= record["seed"] <= MAX_SEED
            and isinstance(record.get("prompt_sha256"), str)
            for record in stage_records.values()
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid generation recipe: {path}") from error
    if (
        not isinstance(manifest["seed"], int)
        or not isinstance(source_hash, str)
        or not valid_stages
    ):
        raise ValueError(f"invalid generation recipe: {path}")
    return manifest


def requested_inputs(
    *,
    family: str | None,
    seed: int | str | None,
    brief: str | None,
    model: str | None,
    effort: str | None,
) -> dict[str, Any]:
    return {
        "family": family,
        "seed": parse_seed(seed),
        "brief": validate_brief(brief),
        "model": model,
        "effort": effort,
    }


def resolve_manifest(
    world: str,
    stages: dict[str, Path],
    *,
    family: str | None = None,
    seed: int | str | None = None,
    brief: str | None = None,
    model: str | None = None,
    effort: str | None = None,
    existing: bool = False,
    persist: bool = True,
    shared_prompt: Path,
) -> dict[str, Any]:
    requested = requested_inputs(
        family=family, seed=seed, brief=brief, model=model, effort=effort
    )
    if existing:
        manifest = load_manifest(world)
        for key, value in requested.items():
            if value is not None and value != manifest[key]:
                raise ValueError(
                    f"{key} conflicts with generation recipe: "
                    f"requested {value!r}, recorded {manifest[key]!r}"
                )
        recorded_stages = set(manifest["stages"])
        if recorded_stages != set(stages):
            raise ValueError("stage bank conflicts with the recorded generation recipe")
        contract = family_contract_path(manifest["family"])
        for name, path in stages.items():
            current_hash = prompt_source_hash(shared_prompt, contract, path)
            if manifest["stages"][name]["prompt_sha256"] != current_hash:
                raise ValueError(
                    f"prompt source changed since the recipe was created: {name}"
                )
        current_source_hash = _sha256_files(_generator_files(manifest["family"]))
        if manifest["generator"]["source_sha256"] != current_source_hash:
            raise ValueError("worldgen source changed since the recipe was created")
        return manifest

    inputs = GenerationInputs(
        family=requested["family"] or DEFAULT_FAMILY,
        seed=requested["seed"] if requested["seed"] is not None else secrets.randbits(64),
        brief=requested["brief"] or "",
        model=requested["model"] or DEFAULT_MODEL,
        effort=requested["effort"] or DEFAULT_EFFORT,
    )
    manifest = build_manifest(
        world,
        stages,
        family=inputs.family,
        seed=inputs.seed,
        brief=inputs.brief,
        model=inputs.model,
        effort=inputs.effort,
        shared_prompt=shared_prompt,
    )
    if persist:
        write_manifest(manifest)
    return manifest


def environment_seed() -> str | None:
    return os.environ.get("SPAR_WORLDGEN_SEED") or None


def environment_brief() -> str | None:
    value = os.environ.get("SPAR_WORLDGEN_BRIEF")
    return value if value else None
