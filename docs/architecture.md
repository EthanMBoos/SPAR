# SPAR Architecture

SPAR is a C++ mission and integration runtime built around the authority boundary between high-level autonomy and a trusted low-level controller. The boundary is the central architectural invariant; everything else is organized around enforcing it and making the enforcement auditable.

## System Overview

```
┌──────────────┐   MAVLink    ┌──────────────┐   Protobuf    ┌──────────────┐
│  Controller  │ ◀──────────▶ │     SPAR     │ ◀───────────▶ │ tower-server │
│ (ArduPilot)  │              │  (C++/BT)    │   extension   │     (Go)     │
└──────────────┘              └──────────────┘               └──────────────┘
```

Initial backend: ArduPilot Rover over MAVLink. The boundary and monitor machinery are intended to outlast any particular backend; `spar_ardupilot_adapter` is the first of a planned pluggable adapter family.

## The Observation Assembler

The assembler is what makes both substitution axes hold. The same `latest_at_or_before(t)` lookup and the same staleness math run in simulation and on the rover; only the source implementation changes. Without it, a policy that trained on clean synchronous observations sees something different at runtime, and the gap is misattributed to "sim-to-real" when it's really temporal smear from concatenating "newest available" across sources that were never co-current.

The payoff is not that simulation equals reality. It is that the residual sim-to-real gap is cornered rather than closed: confined to one component (the sample streams), which are injected through a single transport-degradation layer calibrated to delay/jitter/dropout distributions measured on the real hardware. The gap is localized and measurable instead of smeared across the stack. With the source isolated as the only free variable, "is this policy safe to deploy" becomes a quantity you can actually report rather than a judgment call backed by simulation-plus-hope.

**Sources are dumb writers.** Each sensor thread stamps each sample with its capture time and pushes it into a per-source ring buffer. No processing, no triggering downstream work. Writers keep buffers current and nothing else.

**One reader builds the observation.** Once per tick the executor selects a reference time `t` and for each ring takes the most recent sample at or before `t` (`latest_at_or_before(t)`, not "newest available"). The result is a single immutable, time-coherent `WorldState` (async in, synchronous out).

**Staleness is part of the snapshot.** The assembled `WorldState` carries, per source, how old that component was relative to `t`. For learned nodes, staleness channels go into the input vector; hiding staleness is what makes a policy brittle when latency shifts. When a required source exceeds its staleness bound or is missing, the assembler emits an explicit degraded verdict that deterministically routes to the fallback behavior. This is the authoritative first place staleness is computed; node-level and monitor-level staleness checks are redundant defense, not the primary line.

**Capture time, not receipt time.** Samples are stamped with the sensor's own capture timestamp, never the wall-clock time the callback ran. Receipt time has executor and transport jitter baked in; stamping there relaunders the noise the assembler exists to remove.

**The assembler is a standalone library.** It depends only on a timestamped `Sample<T>`, the ring buffers, and the snapshot builder. No SPAR, MAVLink, or ArduPilot types. SPAR wires it in; it does not own SPAR.

---

## The Authority Boundary

The seam that matters runs between high-level autonomy (anything from a hand-coded behavior tree to a whole-layer learned policy) and the trusted low-level controller. Every command crossing that seam passes through two SPAR-owned components:

- **Runtime monitor**: decides whether a command is allowed
- **Adapter**: the only module in the system permitted to write controller-facing commands

```
        ┌─────────────────────────────────────────────┐
        │  High-level autonomy   (UNTRUSTED)          │
        │  • Hand-coded behavior tree                 │
        │  • Single learned BT node                   │
        │  • Whole-layer learned policy               │
        └────────────────────┬────────────────────────┘
                             │ command stream
                             ▼
        ┌─────────────────────────────────────────────┐
        │  Runtime monitor       (TRUSTED)            │
        │  • per-sample bounds, staleness, NaN/Inf    │
        │  • rate-of-change limits                    │
        │  • temporal-window invariants               │
        └────────────────────┬────────────────────────┘
                             │ approved commands only
                             ▼
        ┌─────────────────────────────────────────────┐
        │  spar_ardupilot_adapter (TRUSTED)            │
        │  • single writer to controller              │
        └────────────────────┬────────────────────────┘
                             │ MAVLink
                             ▼
        ┌─────────────────────────────────────────────┐
        │  ArduPilot            (TRUSTED CONTROL)     │
        │  • low-level control, estimation, failsafes │
        └─────────────────────────────────────────────┘
```

The contract doesn't depend on what sits above it. Whether the policy is hand-coded or a 10B-parameter VLA, the monitor, adapter, and logging schema stay the same. The trusted surface stays small while the policy above can scale arbitrarily.

## Process Layout and Data Flow

SPAR runs as a **single process** (`spar_rover`) with a tick thread at fixed rate and sensor threads running asynchronously.

```
                    GPS + IMU + baro + compass
                             │
                             ▼
                      ArduPilot EKF3           ← runs on the controller, not in SPAR
                             │
                    MAVLink telemetry           GLOBAL_POSITION_INT, ATTITUDE, VFR_HUD
                    (EKF output, already fused)
                             │
          ┌──────────────────┼───────────────────────┐
          │  spar_rover       ▼                       │
          │                                          │
          │  telemetry thread ──▶ WorldState.pose    │
          │                                          │
          │  lidar / camera   ──▶ WorldState.obstacles  (SPAR owns this fusion)
          │  (Zenoh / bridge)                        │
          │                            │             │
          │  tick thread (20Hz)        ▼             │
          │  ┌─────────────────────────────────────┐ │
          │  │ GoalContext + WorldState ──▶ BTNode │ │
          │  │                  │                  │ │
          │  │           CommandStream             │ │
          │  │                  │                  │ │
          │  │          RuntimeMonitor             │ │
          │  │          (+ session.log)            │ │
          │  │                  │ approved only    │ │
          │  │                  ▼                  │ │
          │  │              Adapter ───────────────┼─┼──▶ ArduPilot
          │  └─────────────────────────────────────┘ │
          └──────────────────────────────────────────┘
```

Only `spar_ardupilot_adapter` links against the controller transport; no other code path writes to the controller.

### Tick ordering

Each 20 Hz tick executes in fixed order:

```
1. Observation assembler builds WorldState via latest_at_or_before(t) over all source ring buffers
2. BTNode::tick(GoalContext, WorldState) → CommandStream
3. RuntimeMonitor::evaluate(CommandStream) → MonitorDecision
4. If not Halt: Adapter::write(approved CommandStream) → ArduPilot
5. Append MonitorDecision + per-source WorldState staleness to session log
```

Sensor data arrives whenever ArduPilot sends it, not synchronized to the tick cadence. The assembler's `latest_at_or_before(t)` lookup absorbs that jitter; behavior nodes see a clean fixed-rate sequence of coherent observations.

## WorldState and Sensor Sources

`WorldState` is the immutable snapshot the assembler builds once per tick. It holds processed state estimates (never raw sensor readings) plus per-source staleness relative to the tick's reference time `t`. Once built, it is read-only for the duration of that tick: no domain locks, no concurrent writes. Thread safety lives in the ring buffers on the write side; by the time `WorldState` exists it is already a value.

```cpp
struct WorldState {
    Pose         pose;       // from ArduPilot EKF via MAVLink; stamped with capture time
    ObstacleMap  obstacles;  // from lidar / camera pipeline
    BatteryState battery;    // from MAVLink SYS_STATUS
    // per-source staleness carried alongside each domain
};
```

### Navigation state: ArduPilot owns this

`GLOBAL_POSITION_INT` is not raw GPS. It is ArduPilot's EKF3 output, already fusing GPS, IMU, barometer, and compass on the controller. SPAR's telemetry thread unpacks it and writes it to `WorldState.pose`. There is no separate navigation EKF running in SPAR.

Stream rates are configured on the ArduPilot side and matter for staleness. At default rates (often 1–4 Hz) a 20 Hz tick sees pose data up to 250–1000 ms old. Recommended minimums for the rover demo:

```
SR2_POSITION = 10    # GLOBAL_POSITION_INT at 10 Hz
SR2_EXTRA1   = 50    # ATTITUDE at 50 Hz
SR2_EXTRA3   = 10    # VFR_HUD at 10 Hz
```

### Perception state: SPAR owns this

Sensors ArduPilot doesn't see (lidar, camera, any external perception system) write their processed output into `WorldState` directly via their own threads. SPAR is responsible for this processing. It is the only domain where SPAR runs any fusion or detection logic of its own.

### Simulation must inject transport degradation

Every source, whether it originates in SITL or on hardware, feeds the assembler through the same abstract sample-source interface. In SITL, samples arrive perfectly fresh and synchronous, which does not survive contact with a real vehicle. A configurable degradation layer sits in front of each simulated source and injects per-stream delay, jitter, and dropout drawn from distributions measured on the real rover. Without it, the observation the assembler builds in SITL is not the observation it builds on hardware, and any catch fraction measured in simulation describes a world that doesn't exist.

The assembler and the degradation layer are a pair. A clean assembler fed by undegraded SITL streams looks correct immediately; that is exactly the danger. This is why the transport-degradation layer is a Phase 2 gate, not an optional add-on.

### Staleness is the assembler's responsibility first

The assembler carries per-source staleness in the snapshot and emits a degraded verdict when a required source exceeds its bound, routing deterministically to the fallback before the behavior node runs. Node-level staleness checks (returning `Failure` on old pose data) remain as redundant backstops, not the primary line. The monitor's staleness check on commands is the last line of defence.

### ObservationBuffer: learned nodes only

Hand-coded nodes only need `WorldState`. A learned policy requiring temporal context (LSTM, transformer, sequence-to-action) also receives an `ObservationBuffer`, a sliding window of raw sensor readings, passed alongside `WorldState` in the tick call. This keeps high-bandwidth history out of `WorldState` and off the hand-coded node path entirely.

Full tick signature when learned nodes are wired in:

```cpp
virtual NodeStatus tick(const GoalContext&       goal,
                        const WorldState&         world,  // processed state — all nodes
                        const ObservationBuffer*  obs,    // raw window — learned only, else nullptr
                        CommandStream&            out_cmd) = 0;
```

`obs` is `nullptr` for every hand-coded node. The contract remains symmetric regardless.

## Two Levels of Contracts

**1. Authority-boundary contract** *(primary)*
What crosses between high-level autonomy and the controller, what cannot, and what invariants hold regardless of what produces commands above.

**2. BT-node contract** *(module-level)*
Goal-context input and command-stream output. The contract is symmetric: hand-coded and learned nodes satisfy it identically, so individual nodes can be substituted, whole layers swapped, or the entire tree replaced with a monolithic policy without touching the boundary above.

```cpp
class BTNode {
public:
    virtual NodeStatus tick(const GoalContext& goal,
                            const WorldState&  world,
                            CommandStream&     out_cmd) = 0;
    virtual void        reset()   = 0;
    virtual const char* name()    const = 0;
    virtual uint64_t    node_id() const = 0;
};
```

`GoalContext` carries mission-level intent (target waypoint, mode, mission ID). `WorldState` carries current observed state (position, heading, speed, and anything else the sensor/telemetry threads have written). Nodes are free to read as much or as little of `WorldState` as they need; the separation keeps mission intent decoupled from vehicle state.

The core is plain C++, not ROS nodes, so the live middleware graph never becomes the application model.

## The Runtime Monitor

The monitor is the enforcement point. It sees every command before the adapter and either passes it through, substitutes a fallback, or triggers a halt. Three invariant classes cover most of what's catchable at the command interface:

**Per-sample bounds**: NaN/Inf, out-of-range values, stale timestamps, output magnitudes outside the vehicle's safe envelope. Catches buggy policies emitting malformed commands.

**Rate-of-change limits**: Per-step deltas exceeding what the platform can physically execute. Catches discontinuous output and sample-to-sample jumps from numerical instability.

**Temporal-window invariants**: Sequences of individually in-bounds commands that are collectively dangerous: oscillatory heading patterns, sustained max-rate commands past geofences, jerk patterns exceeding mechanical tolerance. Catches distribution-shifted policies emitting well-formed but contextually wrong commands.

Every monitor decision records the full active invariant set alongside the outcome and triggered invariant. This makes the key postmortem question decidable from logs: did no applicable invariant exist for this case, or did one exist and the monitor fail to enforce it?

### What the monitor cannot catch

A policy producing physically valid, sequentially reasonable commands that execute the wrong plan is invisible to the architectural monitor by construction. That failure class hands off to a **behavioral monitor** running at the policy-semantics layer: temporal consistency over action distributions, VLM-based task-progress judgment, out-of-distribution scoring on policy outputs.

```
                        policy command stream
                                 │
               ┌─────────────────┼─────────────────┐
               ▼                                   ▼
┌──────────────────────────────┐   ┌──────────────────────────────┐
│  Architectural monitor       │   │  Behavioral monitor          │
│  (SPAR, this repo)            │   │  (future layer)              │
│                              │   │                              │
│  • per-sample bounds         │   │  • temporal action           │
│  • rate-of-change limits     │   │    consistency               │
│  • temporal-window           │   │  • VLM task-progress         │
│    invariants                │   │  • OOD scoring               │
│                              │   │                              │
│  "is this command            │   │  "is this policy still       │
│   structurally allowed"      │   │   doing the task"            │
└──────────────┬───────────────┘   └──────────────────────────────┘
               │ approved commands only
               ▼
         adapter / controller
```

The partition between layers is a function of the policy's command distribution, not a fixed property of the monitors. For any given policy, four catch fractions (architectural-only, behavioral-only, both, neither) partition the failure space, and the shape of that partition moves as the command distribution changes. Measuring those fractions empirically across policy types is the core research question.

The **uncatchable residual** (failures neither layer can see) is the explicit boundary of runtime monitoring as a category. Closing it requires formal verification of plan structure, interpretability of policy internals, or constrained training above this layer.

## Stack Position

SPAR is the **within-platform** half of the composition story: the authority boundary on each vehicle. The cross-platform half is Tower, which composes vehicles into a fleet through the pidgin protocol seam. The two boundaries are independent: SPAR without Tower, or Tower without SPAR, are both valid configurations.
