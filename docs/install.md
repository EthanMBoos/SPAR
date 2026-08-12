# Installation

macOS is the tested development platform. Ubuntu uses the same repository,
Blender add-on, and MCP configuration.

## macOS

### Install dependencies

Install [Homebrew](https://brew.sh/), then:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
brew install git uv
brew install --cask docker-desktop blender claude
```

Launch macOS applications through LaunchServices:

```bash
open -a Docker
open -a Blender
open -a Claude
```

Complete Docker Desktop setup, allocate at least 8 GB of memory, and verify it:

```bash
docker version
docker compose version
```

### Install SPAR host tools

```bash
git clone https://github.com/EthanMBoos/SPAR.git
cd SPAR
uv python install
uv sync --managed-python
uv run python -c 'import mujoco, yaml; print(mujoco.__version__)'
```

`uv` creates the project `.venv` from `uv.lock`; do not activate or edit it.
This installs the host MuJoCo library and macOS `mjpython` viewer launcher.
ROS 2, Nav2, PX4, and the simulator run in Docker.

### Install BlenderMCP

```bash
curl -L https://raw.githubusercontent.com/ahujasid/blender-mcp/main/addon.py \
  -o /tmp/blender_mcp_addon.py
```

In Blender:

1. Open **Edit → Preferences → Add-ons → Install from Disk**.
2. Select `/tmp/blender_mcp_addon.py`.
3. Enable **Interface: Blender MCP**.
4. In the 3D View, press `N` and open the **BlenderMCP** tab.

Leave **Poly Haven** disabled. The current world-generation families do not use
downloaded assets.

### Configure Claude Code

`worldgen/mcp.json` configures BlenderMCP for Claude Code. The worldgen runner
passes that file explicitly, starts Blender's listener automatically, and
drives each authoring stage in a fresh model-pinned Claude process. Verify that
the installed family bank is available:

```bash
uv run python -m worldgen.stage --list
```

The runner explicitly selects Sonnet at medium effort, overriding a user-level
default such as Opus or high effort. It starts one non-interactive Claude
session per bounded prompt. The MCP config disables telemetry and uses the
local Blender server on port 9876. Continue with the one-command flow in the
[top-level README](../README.md), or use [worldgen/README.md](../worldgen/README.md)
for manual family qualification and debugging.

### Configure Claude Desktop

Get the absolute `uvx` path:

```bash
which uvx
```

Open **Claude → Settings → Developer → Edit Config**. Add this server, replacing
the command with the output of `which uvx`:

```json
{
  "mcpServers": {
    "blender": {
      "command": "/opt/homebrew/bin/uvx",
      "args": ["--python", "3.11", "blender-mcp"],
      "env": {
        "UV_PYTHON_PREFERENCE": "only-managed",
        "BLENDER_HOST": "localhost",
        "BLENDER_PORT": "9876",
        "DISABLE_TELEMETRY": "true"
      }
    }
  }
}
```

Python 3.11 is only for BlenderMCP's isolated server environment. SPAR's host
tools use the Python 3.14 `.venv` created above.

Quit Claude with **Cmd-Q**, reopen it with `open -a Claude`, then click
**Connect to MCP server** in Blender.

Verify the connection in a new Claude conversation:

```text
Use BlenderMCP to inspect the current Blender scene. Do not modify it. Report
the object names, then stop.
```

Continue with the visible world-generation workflow in the
[README](../README.md).

## Ubuntu

Install the base packages:

```bash
sudo apt update
sudo apt install -y git curl docker.io docker-compose-v2
sudo usermod -aG docker "$USER"
sudo snap install blender --classic
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Log out and back in after changing the Docker group. Then clone SPAR and run
the same `uv python install` and `uv sync --managed-python` commands used on
macOS. Download and install the same BlenderMCP add-on, and configure the MCP
client with the absolute path from `which uvx`.

Use `blender` instead of `open -a Blender` and `uv run python` instead of
`uv run mjpython` on Linux.

Upstream: [BlenderMCP](https://github.com/ahujasid/blender-mcp),
[uv](https://docs.astral.sh/uv/getting-started/installation/), and
[Docker](https://docs.docker.com/).
