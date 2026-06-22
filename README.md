# SPAR — Sim Portable Autonomy Runtime

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++](https://img.shields.io/badge/C%2B%2B-20-00599C?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![Status](https://img.shields.io/badge/Status-Phase%202-blue)](docs/architecture.md)

As robot policies scale from hand-coded behaviors to foundation models, a command boundary becomes the last line of defense you can actually reason about: a VLA that decodes motor commands from raw video is opaque, but the commands it produces are not.

**SPAR** is a C++ runtime built around a command authority boundary: an untrusted mission layer proposes commands, and a trusted runtime monitor vets every one before it reaches the vehicle controller. The architecture holds two things fixed by construction across two substitution axes. Swapping the observation source (simulation vs. hardware) and swapping the behavior (hand-coded vs. learned) both leave the surrounding pipeline bit-identical. The policy is the only free variable.

Behaviors are swapped through the boundary from the most predictable to the most adversarial: a hand-coded baseline, a conventional RL policy (SAC via `KinematicBackend`), and foundation-model actors (Cosmos-based VLAs), the hardest free variable. The SAC baseline is built and available; it validates the pipeline and establishes catch fractions before Cosmos data collection begins. Cosmos is the stress test for the boundary, not a separate research track.

## Research Questions

> **Can a fixed runtime monitor reliably catch dangerous commands regardless of what policy produces them and how does that guarantee hold up as the policy changes?**

> **When an operator delegates a behavior to a learned policy, what does the monitor catch that the operator can't anticipate?**

The four-way partition below is how the boundary is evaluated. Every catch is attributed to a layer:

| Class | Caught by |
|---|---|
| Architectural-only | Runtime monitor (bounds, rate-of-change, temporal-window) |
| Behavioral-only | Behavioral monitor: any success-calibrated, conformal-thresholded detector (slot) |
| Both | Both layers |
| Neither | The uncatchable residual: explicit boundary of what monitoring can do |

The boundary is the contribution; the partition is how it's validated. The catch fraction across these classes shows whether the guarantee holds as the policy changes, and where the two layers catch different things.

The behavioral row is a slot. It takes any detector that calibrates on successful rollouts and thresholds with conformal prediction, which is what the recent work (STAC, FAIL-Detect, FIPER) converged on; those ship as reference plug-ins. Phase 4b adds a Cosmos-specific prediction-divergence detector as one more.

## Architecture

```
        sensor sources (async, different rates/latencies)
                              │
                              ▼
┌─────────────────────────────────────────────┐
│  Observation assembler      (TRUSTED)       │
│  ring buffers · latest-as-of-t · staleness  │
└────────────────────┬────────────────────────┘
                     │ time-coherent WorldState (one per tick)
                     ▼
┌─────────────────────────────────────────────┐
│  Mission layer              (UNTRUSTED)     │  ◀── operator selects active behavior
│  Hand-coded behaviors · learned nodes       │
└────────────────────┬────────────────────────┘
                     │ command stream
                     ▼
┌─────────────────────────────────────────────┐
│  Runtime monitor            (TRUSTED)       │
│  bounds · rate-of-change · staleness        │
│  approve → adapter   reject → safe fallback │
└────────────────────┬────────────────────────┘
                     │ approved command, or defined safe fallback on reject
                     ▼
┌─────────────────────────────────────────────┐
│  Adapter → ArduPilot        (TRUSTED)       │
│  single writer to controller                │
└─────────────────────────────────────────────┘
```

**Observation assembler.** The same assembler, the same `latest_at_or_before(t)` lookup, and the same staleness math run in simulation and on the rover; only the source implementation changes. For hand-coded behaviors and the SAC RL baseline, the assembled snapshot is the policy's primary input. For vision-based policies (Cosmos, Phase 4a), raw video frames flow through a separate path, but the assembler still runs, because the monitor needs assembled state (pose age, speed) to gate every command regardless of what produced it.

**Runtime monitor.** Sees every command before the adapter. Enforces per-sample bounds, rate-of-change limits, and temporal-window invariants derived from ArduPilot's accepted command envelope. Every decision is logged with the triggered invariant and an `invariant_flags` bitmask so the post-mortem question is always answerable: did no applicable invariant exist, or did one exist and the monitor miss it?

**Rejection contract.** A rejected command is never silently dropped. The monitor puts a defined safe fallback in its place and logs the rejection with the triggered `invariant_flags`. The fallback is what keeps the vehicle safe; a flagged command does nothing on its own. It's configurable per deployment: hold-last-approved, commanded-stop/zero, or hand-off to an ArduPilot failsafe mode.

**Single-writer adapter.** The only module that links against the controller transport. Bypass is architecturally impossible, not merely prohibited.

## Current State and Roadmap

Single process, no ROS. Each phase has a gate condition; do not start the next phase until the gate is met.

| Phase | What | Gate |
|---|---|---|
| **0** ✓ | SITL bug fixes (type_mask, heartbeat, UDP bind, capture timestamps); dead code cuts | Stable SITL telemetry: pose staleness < 200 ms on consecutive ticks |
| **1** ✓ | `KinematicBackend` + `TransportDegradation` + `OnnxNavigateNode` + SAC training + eval harness | Architectural catch fractions computable from a session log produced by an ONNX policy run |
| **2** | `MissionExecutor` + JSON mission loader + per-task policy assignment | Catch fractions computable per (task, policy) pair from session log |
| **3** | Temporal-window invariants (oscillation, sustained max-rate, jerk) + MCAP logging | At least one temporal invariant catches a failure class per-sample invariants miss |
| **4a** | `VideoRecorder` + `ObservationBuffer` + Cosmos-trained actor via `CosmosNavigateNode` | Architectural catch fractions for Cosmos actor vs. Phase 2 SAC fractions |
| **4b** | Cosmos prediction-divergence behavioral monitor | All four catch fractions computed for a Cosmos-trained policy |

**Phases 0 and 1 are complete.** Phase 1 gate closed 2026-06-22: SAC policy trained via `SparEnv`, exported to ONNX, and run end-to-end through `OnnxNavigateNode`. Verified catch fractions by scenario: hand-coded baseline 0.003, pose_dropout 0.435, pose_jitter 0.053, policy_exploit 0.999, SAC ONNX baseline 0.014 (mission success, neither = 0). The trained policy produces slightly rougher commands than the hand-coded node; the monitor catches them without blocking mission completion.

**Phase 4a prerequisite:** Before starting Phase 4a, benchmark the candidate Cosmos actor's inference latency on target hardware and determine its output format (autoregressive vs. diffusion/chunking). If inference > 30 ms or the model outputs action chunks, `docs/action-chunking.md` describes the required architectural changes and must be implemented first. This decision cannot be deferred past Phase 4a start.

## Out of Scope

These are correct designs for a deployed product. They are not research prerequisites and are not scheduled until Phase 3 produces results worth deploying.

- **Tower integration** (`docs/future/radio_icd.md`): UDP multicast, fleet state, mailbox, multi-robot coordination
- **SAR / high-level behaviors** (`docs/future/high-level-behaviors.md`): CommandBundle, BT composites, coverage planning
- **`CarlaBackend` / `IsaacBackend`**: not needed: the fidelity ladder is `kinematic` → real rover video, not `kinematic` → photorealistic sim
- **Optimal planning, terrain models, probability maps**: SPAR executes plans; it does not compute them
- **Formal verification / safety certification**: SPAR's evaluation is empirical. It is not a formal safety case

## Repository Layout

```
SPAR/
├── shared/
│   ├── contracts/           # BTNode, GoalContext, CommandStream,
│   │                        # MonitorDecision, WorldState, SimulatorBackend
│   └── assembler/           # ObservationAssembler (ring buffers + snapshot)
├── spar_rover/
│   ├── main.cpp             # tick thread (20 Hz)
│   ├── monitor/             # RuntimeMonitor (authority boundary enforcement)
│   ├── mission/             # NavigateNode, OnnxNavigateNode
│   ├── adapter/             # ArdupilotAdapter (MAVLink; disabled in kinematic mode)
│   └── sim/                 # KinematicBackend, TransportDegradation
├── python/                  # pybind11 bindings + SparEnv gym wrapper
└── docs/
    ├── architecture.md      # assembler, authority boundary, data flow, constraints
    ├── mission-design.md    # Task struct, MissionExecutor, session log schema
    └── future/              # radio_icd.md, high-level-behaviors.md (post-Phase-3)
```

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

The venv must be active when building the Python bindings so CMake links against the same interpreter used at runtime.

## Building

Requires CMake 3.22+, a C++20 compiler.

| Flag | Default | Effect |
|---|---|---|
| `SPAR_BACKEND` | `ardupilot` | `ardupilot` = live MAVLink; `kinematic` = in-process bicycle model |
| `SPAR_ENABLE_MAVLINK` | OFF | Enable real MAVLink transport (requires `third_party/mavlink` submodule) |
| `SPAR_ENABLE_ONNXRUNTIME` | OFF | Enable ONNX learned node (`OnnxNavigateNode`) |
| `SPAR_ENABLE_PYTHON` | OFF | Build pybind11 gym wrapper (requires `SPAR_BACKEND=kinematic` + pybind11) |
| `SPAR_ENABLE_EXPLOIT_NODE` | OFF | Build `ExploitNode` for `policy_exploit` scenario (eval only; never on in deployment builds) |

**Runtime environment variables:**

| Variable | Default | Effect |
|---|---|---|
| `SPAR_SCENARIO` | `baseline` | Label written to the session log header (`# scenario: <value>`). Set to the active failure scenario name so logs are unambiguous. See `docs/eval-protocol.md` for defined scenario names. |

**Kinematic mode** (no SITL, no hardware required):
```bash
cmake -B build -DSPAR_BACKEND=kinematic -DSPAR_ENABLE_EXPLOIT_NODE=ON
cmake --build build
SPAR_SCENARIO=baseline ./build/spar_rover/spar_rover   # log written to /tmp/spar_*.log
```

**Kinematic mode + Python bindings** (SAC training):
```bash
cmake -B build -DSPAR_BACKEND=kinematic -DSPAR_ENABLE_PYTHON=ON
cmake --build build --target spar_bindings
python python/train_sac.py
```

**Run all four eval scenarios and print catch-fraction table:**
```bash
cd python && python eval_harness.py --build-dir ../build
```

**ArduPilot stub mode** (adapter runs but writes are no-ops; fixed pose injected):
```bash
cmake -B build
cmake --build build
./build/spar_rover/spar_rover
```

## Running Against ArduPilot SITL

*(MAVLink submodule required: clone `third_party/mavlink`, then build with `SPAR_ENABLE_MAVLINK=ON`)*

```bash
# Start ArduPilot Rover SITL
sim_vehicle.py -v Rover --console

# Run SPAR
./build/spar_rover/spar_rover
# Session log written to /tmp/spar_<session_id>.log
```

## Documentation

- [Architecture](docs/architecture.md): observation assembler, authority boundary, data flow, monitor design, architectural constraints
- [RL Pipeline](docs/rl-pipeline.md): SAC baseline, Cosmos behavior-cloning path, Phase 4a prerequisites (VideoRecorder, ObservationBuffer, action chunking decision gate), build status
- [Eval Protocol](docs/eval-protocol.md): named failure scenarios, eval harness, session log parser, catch-fraction table
- [Mission Design](docs/mission-design.md): Task struct, MissionExecutor design, session log schema
- [Future](docs/future/): Tower radio ICD, high-level behaviors (implement after Phase 3)

## License

MIT. See [LICENSE](LICENSE).