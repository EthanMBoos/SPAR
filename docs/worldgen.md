# World generation

Worldgen turns one text description into one small outdoor MuJoCo world. It
uses a locally running Ollama model for layout and review, then uses the
repository's existing geometry lint before publishing anything.

```text
description
  -> Ollama layout
  -> Ollama review
  -> temporary MJCF
  -> MuJoCo compile and lint
  -> sim/worlds/<name>.xml
```

Run Ollama with the model you want. `make lint` creates the repository Python
environment on first use. Then generate from the repository root:

```bash
make lint
.venv/bin/python scripts/generate_world.py \
  --name loading_yard \
  --model gemma3:4b \
  "A compact loading yard with fencing, crates, and a red hazard drum"
```

`OLLAMA_HOST` overrides the default `http://localhost:11434`. The generator
tries at most three layouts. Invalid model output, an Ollama review rejection,
or a lint failure is fed into the next attempt. If all attempts fail, no world
is written. Existing output is protected unless `--force` is passed.
Generated worlds are ignored by git by default. Deliberately promote a useful
shared fixture with `git add -f sim/worlds/<name>.xml`.

Inspect the result without ROS:

```bash
make inspect WORLD=loading_yard
```

If it looks right, use it explicitly:

```bash
make start_sim WORLD=loading_yard
make view WORLD=loading_yard

# In the ground ROS container:
ros2 launch spar_bringup autonomy.launch.py
```

## What v1 generates

Every world is a fixed 16 m square with the same lighting, overview camera,
and both robot includes needed by the simulator. The robot models own their
default spawns and home markers. Ollama selects:

- grass, dirt, concrete, or gravel ground colors;
- 3-7 grounded racks, crates, containers, fences, barrels, or hay bales;
- one required red anomaly drum;
- primitive colors, one of eight named outer regions, and horizontal or
  vertical orientation.

Python owns prop sizes, collision geometry, and MJCF. It maps regions to safe
outer coordinates; the reusable ROS configs generate patrols around each
robot's home. The model never writes coordinates, XML, or YAML.

This first version does not generate robot missions, terrain, meshes, movable
props, images, seeds, batches, variations, or photorealistic renders. Model
review reads the structured layout, not a screenshot. Human visual approval is
the final gate before choosing the world for the simulator.
