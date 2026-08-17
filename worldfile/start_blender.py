"""Start one clean visible Blender window and initialize BlenderMCP."""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parent
SCRIPT = Path(__file__).resolve()


def mcp_endpoint() -> tuple[str, int]:
    config = json.loads((ROOT / "mcp.json").read_text(encoding="utf-8"))
    environment = config["mcpServers"]["blender"].get("env", {})
    return environment.get("BLENDER_HOST", "localhost"), int(
        environment.get("BLENDER_PORT", "9876")
    )


def blender_executable() -> str:
    if sys.platform == "darwin":
        path = Path("/Applications/Blender.app/Contents/MacOS/Blender")
        if path.is_file():
            return str(path)
    executable = shutil.which("blender")
    if executable is None:
        raise RuntimeError("blender executable not found")
    return executable


def launch_visible_blender() -> None:
    # DevNote: A full run owns one Blender process; stale windows can receive
    # MCP edits while the user watches a different scene.
    process_name = "Blender" if sys.platform == "darwin" else "blender"
    subprocess.run(["pkill", "-x", process_name], check=False)

    if sys.platform == "darwin":
        command = [
            "open",
            "-na",
            "Blender",
            "--args",
            "--factory-startup",
            "--python",
            str(SCRIPT),
        ]
        subprocess.run(command, check=True)
    else:
        subprocess.Popen(
            [blender_executable(), "--factory-startup", "--python", str(SCRIPT)],
            start_new_session=True,
        )
    print("launched one clean Blender Worldfile window", flush=True)


def wait_until_ready(timeout_seconds: float = 90.0) -> None:
    host, port = mcp_endpoint()
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=1.0):
                print(f"BlenderMCP ready on {host}:{port}", flush=True)
                return
        except OSError:
            time.sleep(1.0)
    raise RuntimeError(
        f"BlenderMCP did not listen on {host}:{port} within {timeout_seconds:g}s"
    )


def initialize_worldgen() -> None:
    import addon_utils
    import bpy

    bpy.context.preferences.view.show_splash = False
    addon_utils.enable("blender_mcp", default_set=False, persistent=False)
    server = getattr(bpy.types, "blendermcp_server", None)
    if server is None:
        raise RuntimeError(
            "BlenderMCP is not installed for this Blender version; see docs/install.md"
        )

    scene = bpy.context.scene
    if hasattr(bpy.types.Scene, "blendermcp_use_polyhaven"):
        scene.blendermcp_use_polyhaven = False
    if not server.running:
        bpy.ops.blendermcp.start_server()

    try:
        bpy.ops.wm.splash_close()
    except RuntimeError:
        pass
    print("Worldfile authoring ready: BlenderMCP listening (external assets disabled)")


def main() -> int:
    try:
        import bpy  # noqa: F401
    except ModuleNotFoundError:
        launch_visible_blender()
    else:
        initialize_worldgen()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
