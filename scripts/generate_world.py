#!/usr/bin/env python3
"""Generate one small MuJoCo world from a text description using Ollama.

Ollama chooses a constrained semantic layout: ground style, primitive props,
colors, coarse regions, and orientations. Python owns all coordinates, sizes,
MJCF, robot includes, and safety rules. The model reviews its structured plan,
then MuJoCo compiles and programmatically lints a temporary world before it is
published for human inspection.
"""

import argparse
import json
import math
import os
import re
import sys
import tempfile
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLDS = os.path.join(REPO, "sim", "worlds")
GROUND_AUTONOMY = os.path.join(
    REPO, "ground", "src", "spar_bringup", "config", "autonomy.yaml")

sys.path.insert(0, os.path.join(REPO, "scripts"))
from lint_world import validate_world  # noqa: E402

MAX_ATTEMPTS = 3
WORLD_HALF_M = 8.0

# MuJoCo sizes are half-extents. Keeping them here means the model can place
# known objects but can never invent collision geometry.
PROPS = {
    "rack": ("box", (0.8, 0.3, 0.6)),
    "crate": ("box", (0.5, 0.5, 0.5)),
    "container": ("box", (1.5, 0.6, 0.75)),
    "fence": ("box", (1.25, 0.08, 0.6)),
    "barrel": ("cylinder", (0.3, 0.45)),
    "hay_bale": ("cylinder", (0.45, 0.6)),
    "anomaly_drum": ("cylinder", (0.3, 0.45)),
}

COLORS = {
    "red": (1.0, 0.0, 0.0, 1.0),
    "orange": (0.95, 0.35, 0.05, 1.0),
    "yellow": (0.9, 0.7, 0.1, 1.0),
    "green": (0.2, 0.55, 0.25, 1.0),
    "blue": (0.15, 0.35, 0.75, 1.0),
    "gray": (0.5, 0.52, 0.55, 1.0),
    "brown": (0.45, 0.28, 0.12, 1.0),
    "white": (0.85, 0.85, 0.82, 1.0),
}

GROUND_COLORS = {
    "grass": ((0.18, 0.32, 0.16), (0.1, 0.22, 0.1)),
    "dirt": ((0.38, 0.27, 0.16), (0.25, 0.17, 0.1)),
    "concrete": ((0.45, 0.47, 0.48), (0.32, 0.34, 0.35)),
    "gravel": ((0.38, 0.39, 0.38), (0.24, 0.25, 0.24)),
}

REGIONS = {
    "northwest": (-4.5, 4.5),
    "north": (0.0, 4.5),
    "northeast": (4.5, 4.5),
    "west": (-5.0, 0.0),
    "east": (5.0, 0.0),
    "southwest": (-4.5, -4.5),
    "south": (0.0, -4.5),
    "southeast": (4.5, -4.5),
}
ORIENTATIONS = {
    "east_west": 0.0,
    "north_south": math.pi / 2.0,
}

PLAN_SCHEMA = {
    "type": "object",
    "properties": {
        "summary": {"type": "string"},
        "ground": {"type": "string", "enum": list(GROUND_COLORS)},
        "props": {
            "type": "array",
            "minItems": 3,
            "maxItems": 7,
            "items": {
                "type": "object",
                "properties": {
                    "kind": {"type": "string", "enum": list(PROPS)},
                    "color": {"type": "string", "enum": list(COLORS)},
                    "region": {"type": "string", "enum": list(REGIONS)},
                    "orientation": {
                        "type": "string",
                        "enum": list(ORIENTATIONS),
                    },
                },
                "required": ["kind", "color", "region", "orientation"],
                "additionalProperties": False,
            },
        },
    },
    "required": ["summary", "ground", "props"],
    "additionalProperties": False,
}

REVIEW_SCHEMA = {
    "type": "object",
    "properties": {
        "decision": {
            "type": "string",
            "enum": [
                "approve",
                "semantic_mismatch",
                "implausible_layout",
            ],
        },
    },
    "required": ["decision"],
    "additionalProperties": False,
}


class InvalidModelResponse(RuntimeError):
    pass


def ollama_chat(host, model, messages, schema):
    """Return one structured Ollama chat response."""
    if "://" not in host:
        host = f"http://{host}"
    request = urllib.request.Request(
        f"{host.rstrip('/')}/api/chat",
        data=json.dumps({
            "model": model,
            "messages": messages,
            "format": schema,
            "stream": False,
            "options": {"temperature": 0},
        }).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=300) as response:
            body = json.load(response)
        return json.loads(body["message"]["content"])
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode(errors="replace")
        raise RuntimeError(f"Ollama returned HTTP {exc.code}: {detail}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(
            f"cannot reach Ollama at {host}: {exc.reason}") from exc
    except (json.JSONDecodeError, KeyError, TypeError) as exc:
        raise InvalidModelResponse(
            f"Ollama returned an invalid response: {exc}") from exc


def planner_messages(description, feedback=None):
    catalog = ", ".join(PROPS)
    colors = ", ".join(COLORS)
    prompt = f"""Create one compact 16 m by 16 m outdoor robot test world.

User description:
{description}

Use 3-7 grounded props from: {catalog}.
Use only these colors: {colors}.
Place exactly one anomaly_drum and make it red.
Place at most one prop in each named region. Choose east_west or north_south
orientation. Python maps each region to a safe fixed coordinate.
Return only the requested structured data."""
    if feedback:
        prompt += f"\n\nThe previous candidate failed. Fix all of this:\n{feedback}"
    return [
        {
            "role": "system",
            "content": (
                "You plan small, readable outdoor MuJoCo layouts. "
                "Follow the supplied schema and constraints exactly."
            ),
        },
        {"role": "user", "content": prompt},
    ]


def reviewer_messages(description, plan):
    return [
        {
            "role": "system",
            "content": (
                "Review a proposed small outdoor robot world only for "
                "semantic fit, outdoor plausibility, and useful patrol "
                "coverage. It already passed exact programmatic checks for "
                "schema, catalog, counts, colors, bounds, and clearances; do "
                "not second-guess those checks. Catalog props may provide "
                "reasonable scene context even when the user did not name "
                "each one. Return approve unless there is a real semantic "
                "mismatch or the combination is not a plausible outdoor site."
            ),
        },
        {
            "role": "user",
            "content": (
                f"User description:\n{description}\n\n"
                f"Candidate layout:\n{json.dumps(plan, indent=2)}\n\n"
                f"Programmatic facts: {len(plan['props'])} allowed props, "
                "unique outer regions, and exactly one red anomaly drum."
            ),
        },
    ]


def normalize_plan(plan):
    """Move duplicate valid regions to the next free safe anchor."""
    if not isinstance(plan, dict):
        return plan
    props = plan.get("props")
    if not isinstance(props, list):
        return plan
    available = list(REGIONS)
    used = set()
    for prop in props:
        if not isinstance(prop, dict):
            continue
        region = prop.get("region")
        if region in available and region not in used:
            used.add(region)
            continue
        if region in REGIONS:
            replacement = next(
                (candidate for candidate in available if candidate not in used),
                None,
            )
            if replacement:
                prop["region"] = replacement
                used.add(replacement)
    return plan


def validate_plan(plan):
    """Return human-readable errors for a model-produced layout."""
    errors = []
    if not isinstance(plan, dict):
        return ["layout must be a JSON object"]
    if set(plan) != {"summary", "ground", "props"}:
        errors.append("layout must contain only summary, ground, props")
    if not isinstance(plan.get("summary"), str) or not plan.get("summary", "").strip():
        errors.append("summary must be a non-empty string")
    if plan.get("ground") not in GROUND_COLORS:
        errors.append(f"ground must be one of: {', '.join(GROUND_COLORS)}")

    props = plan.get("props")
    if not isinstance(props, list) or not 3 <= len(props) <= 7:
        errors.append("props must contain 3-7 objects")
        props = []
    regions = set()
    anomaly_count = 0
    for index, prop in enumerate(props):
        label = f"prop {index}"
        if not isinstance(prop, dict):
            errors.append(f"{label} must be an object")
            continue
        if set(prop) != {"kind", "color", "region", "orientation"}:
            errors.append(f"{label} has missing or unexpected fields")
            continue
        kind = prop["kind"]
        if kind not in PROPS:
            errors.append(f"{label} has unknown kind: {kind}")
            continue
        if prop["color"] not in COLORS:
            errors.append(f"{label} has unknown color: {prop['color']}")
        if kind == "anomaly_drum":
            anomaly_count += 1
            if prop["color"] != "red":
                errors.append("anomaly_drum must be red")
        region = prop["region"]
        if region not in REGIONS:
            errors.append(f"{label} has unknown region: {region}")
        elif region in regions:
            errors.append(f"duplicate prop region: {region}")
        regions.add(region)
        if prop["orientation"] not in ORIENTATIONS:
            errors.append(
                f"{label} has unknown orientation: {prop['orientation']}")
    if anomaly_count != 1:
        errors.append("props must contain exactly one anomaly_drum")
    return errors


def validate_review(review):
    if not isinstance(review, dict) \
            or set(review) != {"decision"} \
            or review.get("decision") not in REVIEW_SCHEMA[
                "properties"]["decision"]["enum"]:
        return "review must contain one allowed decision"
    return None


def _numbers(values):
    return " ".join(f"{value:.6g}" for value in values)


def render_world(plan, name):
    """Render a validated layout to MJCF text."""
    root = ET.Element("mujoco", {"model": name})
    ET.SubElement(root, "compiler", {"angle": "radian"})
    ET.SubElement(root, "option", {
        "timestep": "0.002",
        "integrator": "implicitfast",
        "magnetic": "0 0 0",
    })
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "headlight", {
        "diffuse": "0.6 0.6 0.6",
        "ambient": "0.3 0.3 0.3",
        "specular": "0 0 0",
    })
    ET.SubElement(visual, "global", {"azimuth": "120", "elevation": "-20"})

    rgb1, rgb2 = GROUND_COLORS[plan["ground"]]
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", {
        "type": "skybox",
        "builtin": "gradient",
        "rgb1": "0.3 0.5 0.7",
        "rgb2": "0 0 0",
        "width": "512",
        "height": "3072",
    })
    ET.SubElement(asset, "texture", {
        "type": "2d",
        "name": "ground_grid",
        "builtin": "checker",
        "mark": "edge",
        "rgb1": _numbers(rgb1),
        "rgb2": _numbers(rgb2),
        "markrgb": "0.8 0.8 0.8",
        "width": "300",
        "height": "300",
    })
    ET.SubElement(asset, "material", {
        "name": "ground",
        "texture": "ground_grid",
        "texuniform": "true",
        "texrepeat": "4 4",
        "reflectance": "0.1",
    })

    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(worldbody, "light", {
        "name": "sun",
        "directional": "true",
        "pos": "0 0 15",
        "dir": "0 0 -1",
    })
    ET.SubElement(worldbody, "camera", {
        "name": "overview",
        "pos": "0 -12 9",
        "xyaxes": "1 0 0 0 0.6 0.8",
    })
    ET.SubElement(worldbody, "geom", {
        "name": "floor",
        "type": "plane",
        "size": f"{WORLD_HALF_M:g} {WORLD_HALF_M:g} 1",
        "material": "ground",
    })

    kind_counts = {}
    for prop in plan["props"]:
        shape, size = PROPS[prop["kind"]]
        z = size[2] if shape == "box" else size[1]
        kind_counts[prop["kind"]] = kind_counts.get(prop["kind"], 0) + 1
        x, y = REGIONS[prop["region"]]
        ET.SubElement(worldbody, "geom", {
            "name": f"prop_{prop['kind']}_{kind_counts[prop['kind']]}",
            "type": shape,
            "size": _numbers(size),
            "pos": _numbers((x, y, z)),
            "euler": _numbers((0, 0, ORIENTATIONS[prop["orientation"]])),
            "rgba": _numbers(COLORS[prop["color"]]),
        })

    ET.SubElement(root, "include", {"file": "../robots/husky.xml"})
    ET.SubElement(root, "include", {"file": "../robots/x2.xml"})
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode") + "\n"


def generate(description, name, model, host, force=False, chat=ollama_chat,
             worlds_dir=WORLDS, autonomy_path=GROUND_AUTONOMY):
    if name == "blank" or not re.fullmatch(r"[a-z][a-z0-9_-]*", name):
        raise ValueError(
            "name must be lowercase letters, numbers, '_' or '-', and not blank")
    world_path = os.path.join(worlds_dir, f"{name}.xml")
    if os.path.exists(world_path) and not force:
        raise FileExistsError(
            f"refusing to replace {world_path}; pass --force to replace")

    feedback = None
    for attempt in range(1, MAX_ATTEMPTS + 1):
        print(f"[worldgen] attempt {attempt}/{MAX_ATTEMPTS}")
        try:
            plan = chat(
                host, model, planner_messages(description, feedback),
                PLAN_SCHEMA)
        except InvalidModelResponse as exc:
            feedback = str(exc)
            print(f"[worldgen] layout response rejected: {feedback}")
            continue
        normalize_plan(plan)
        errors = validate_plan(plan)
        if errors:
            feedback = "\n".join(errors)
            print(f"[worldgen] layout rejected locally: {feedback}")
            continue

        try:
            review = chat(
                host, model, reviewer_messages(description, plan),
                REVIEW_SCHEMA)
        except InvalidModelResponse as exc:
            feedback = str(exc)
            print(f"[worldgen] review response rejected: {feedback}")
            continue
        review_error = validate_review(review)
        if review_error:
            feedback = review_error
            print(f"[worldgen] review rejected locally: {feedback}")
            continue
        if review["decision"] != "approve":
            feedback = f"Ollama review: {review['decision']}"
            print(f"[worldgen] Ollama review rejected layout: {feedback}")
            continue

        os.makedirs(worlds_dir, exist_ok=True)
        world_tmp = None
        try:
            with tempfile.NamedTemporaryFile(
                    mode="w", dir=worlds_dir, prefix=".worldgen_",
                    suffix=".xml", delete=False) as output:
                world_tmp = output.name
                output.write(render_world(plan, name))

            failures, warnings, _ = validate_world(
                "husky", world_tmp, autonomy_path)
            for warning in warnings:
                print(f"[worldgen] lint warning: {warning}")
            if failures:
                feedback = "\n".join(failures)
                print(f"[worldgen] lint rejected layout: {feedback}")
                continue

            os.replace(world_tmp, world_path)
            world_tmp = None
            return world_path, plan
        finally:
            if world_tmp and os.path.exists(world_tmp):
                os.unlink(world_tmp)

    raise RuntimeError(
        f"no valid world after {MAX_ATTEMPTS} attempts"
        + (f": {feedback}" if feedback else ""))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--name", required=True, help="output world name")
    parser.add_argument("--model", required=True, help="installed Ollama model")
    parser.add_argument(
        "--force", action="store_true", help="replace existing output files")
    parser.add_argument(
        "description", help="quoted description of the outdoor scene")
    args = parser.parse_args()
    host = os.environ.get("OLLAMA_HOST", "http://localhost:11434")
    try:
        world, plan = generate(
            args.description, args.name, args.model, host, args.force)
    except (FileExistsError, RuntimeError, ValueError) as exc:
        parser.exit(1, f"[worldgen] ERROR: {exc}\n")

    print(f"[worldgen] created {os.path.relpath(world, REPO)}")
    print(f"[worldgen] {plan['summary']}")
    print("\nReview:")
    print(f"  make inspect WORLD={args.name}")
    print("\nUse with ROS after approval:")
    print(f"  make start_sim WORLD={args.name}")
    print(f"  make view WORLD={args.name}")
    print("  ros2 launch spar_bringup autonomy.launch.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
