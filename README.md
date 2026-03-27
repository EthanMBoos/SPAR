# SPAR — Sim-Portable Autonomy Runtime

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++](https://img.shields.io/badge/C%2B%2B-20-00599C?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![Status](https://img.shields.io/badge/Status-Phase%201-blue)](docs/plan.md)

A C++ mission execution runtime whose central claim is invariance-by-construction across two substitution axes: swapping the source of an observation (simulation vs. hardware) and swapping the behavior that consumes it (hand-coded vs. learned policy) both leave the surrounding pipeline bit-identical.

The mechanism is a single-reader deterministic snapshot. Asynchronous transport is demoted to dumb stamp-and-push writers feeding per-source ring buffers; a single reader builds one immutable, time-coherent `WorldState` per tick via a `latest_at_or_before(t)` lookup. Because the assembly is a pure function of buffer contents and reference time, scheduler jitter, callback ordering, and "newest-available smear" cannot leak into the observation. The nondeterminism is quarantined to the write side.

---

## The Payoff

The goal is not to make simulation equal reality. The goal is to corner the residual gap rather than close it: confine it to one component (the sample streams themselves), which are injected through a single transport-degradation layer calibrated to delay/jitter/dropout distributions measured on the real hardware. The gap is localized and measurable instead of smeared across the stack.

With the source isolated as the only free variable, SPAR turns "is this policy safe to deploy" from simulation-plus-hope into a quantity you can actually report. For any given policy, four catch fractions partition the failure space: architectural monitor only, behavioral monitor only, both, neither. Measuring those fractions across behavior types (hand-coded, conservative learned, aggressive learned) is what this prototype is built to do.

---

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
│  Mission layer              (UNTRUSTED)     │
│  Hand-coded behaviors · learned nodes       │
└────────────────────┬────────────────────────┘
                     │ command stream
                     ▼
┌─────────────────────────────────────────────┐
│  Runtime monitor            (TRUSTED)       │
│  bounds · rate-of-change · staleness        │
└────────────────────┬────────────────────────┘
                     │ approved commands only
                     ▼
┌─────────────────────────────────────────────┐
│  Adapter → ArduPilot        (TRUSTED)       │
│  single writer to controller                │
└─────────────────────────────────────────────┘
```

**Observation assembler.** The component that makes both substitution axes hold. The same assembler, the same `latest_at_or_before(t)` lookup, and the same staleness math run in simulation and on the rover; only the source implementation changes. Staleness is carried per-component in the snapshot and passed into learned nodes as an input channel, not hidden from them.

**Runtime monitor.** Sees every command before the adapter. Enforces per-sample bounds, rate-of-change limits, and temporal-window invariants derived from ArduPilot's accepted command envelope. Every decision is logged with the full active invariant set so the post-mortem question is always answerable: did no applicable invariant exist, or did one exist and the monitor miss it?

**Single-writer adapter.** The only module that links against the controller transport. Bypass is architecturally impossible, not merely prohibited.

---

## Current State

Working against ArduPilot Rover SITL. Single process, no hardware required for the experiment.

- **Phase 1: Monitor contract in SITL** *(in progress)*: authority boundary end-to-end with a hand-coded behavior node; observation assembler producing time-coherent `WorldState` with per-component staleness; session log that reconstructs every monitor decision from logs alone.
- **Phase 2: Learned node injection**: replace `NavigateNode` with an ONNX policy above the same boundary; transport-degradation layer (delay/jitter/dropout drawn from measured rover distributions) must be in place first; inject shifted inputs and measure catch fractions.
- **Phase 3: Behavioral monitor**: add a behavioral monitor above the architectural boundary consuming the logged observation/action stream on a slower clock; measure the four-way failure partition across behavior types.

---

## Repository Layout

```
SPAR/
├── shared/contracts/        # BTNode, GoalContext, CommandStream,
│                            # MonitorDecision, WorldState
└── spar_rover/
    ├── main.cpp             # tick thread (20 Hz)
    ├── monitor/             # RuntimeMonitor
    ├── mission/             # BehaviorTree, NavigateNode
    └── adapter/             # ArdupilotAdapter
```

---

## Building

Requires CMake 3.16+, a C++20 compiler, and OpenSSL.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

| Flag | Default | Effect |
|---|---|---|
| `SPAR_ENABLE_MAVLINK` | OFF | Enable real MAVLink transport (requires `third_party/mavlink` submodule) |
| `SPAR_ENABLE_ONNXRUNTIME` | OFF | Enable ONNX learned node support (Phase 2) |

Without `SPAR_ENABLE_MAVLINK`, the adapter runs in stub mode: a fixed pose is injected into `WorldState` and command writes are no-ops. The monitor and mission logic run normally.

---

## Running Against ArduPilot SITL

*(MAVLink submodule wiring in progress — see [plan](docs/plan.md))*

```bash
# Start ArduPilot Rover SITL
sim_vehicle.py -v Rover --console

# Run SPAR
./build/spar_rover/spar_rover
# Session log written to /tmp/spar_<session_id>.log
```

---

## Documentation

- [Architecture](docs/architecture.md) — observation assembler, authority boundary, data flow, monitor design
- [Plan](docs/plan.md) — research goal, phase breakdown, exit criteria
- [Mission Design](docs/mission-design.md) — task sequencing, state machine, user roles
- [High-Level Behaviors](docs/high-level-behaviors.md) — behavior composition, multi-actuator output, coverage planning

---

## License

MIT. See [LICENSE](LICENSE).
