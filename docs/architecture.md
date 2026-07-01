# SPAR Architecture

SPAR is a C++ mission and integration runtime built around the authority boundary between high-level autonomy and a trusted low-level controller. The boundary is the central architectural invariant; everything else is organized around enforcing it and making the enforcement auditable.

## System Overview

```
┌──────────────┐  ROS2/Zenoh  ┌──────────────┐   Protobuf    ┌──────────────┐
│  Vehicle     │ ◀──────────▶ │     SPAR     │ ◀───────────▶ │ tower-server │
│  controller  │              │  (C++)       │   extension   │     (Go)     │
└──────────────┘              └──────────────┘               └──────────────┘
```

Two backend modes are supported. `Ros2Adapter` drives a live vehicle over Zenoh — publishing `cmd_vel`, consuming odometry from the vehicle's external state estimator; `KinematicBackend` is an in-process bicycle model that injects observations directly into the assembler. The assembler, monitor, and mission layer are unchanged between them — only the source implementation changes. Build with `-DSPAR_BACKEND=kinematic` for training and catch-fraction experiments without a vehicle.

## The Observation Assembler

The assembler is what makes both substitution axes hold. The same `latest_at_or_before(t)` lookup and the same staleness math run in simulation and on the rover; only the source implementation changes.

**Sources are dumb writers.** Each sensor thread stamps each sample with its capture time and pushes it into a per-source ring buffer. No processing, no triggering downstream work. Writers keep buffers current and nothing else.

**One reader builds the observation.** Once per tick the executor selects a reference time `t` and for each ring takes the most recent sample at or before `t` (`latest_at_or_before(t)`, not "newest available"). The result is a single immutable, time-coherent `WorldState` (async in, synchronous out).

**Staleness is part of the snapshot.** The assembled `WorldState` carries, per source, how old that component was relative to `t`. For learned nodes, staleness channels go into the input vector; hiding staleness is what makes a policy brittle when latency shifts. When a required source exceeds its staleness bound or is missing, the assembler emits an explicit degraded verdict that deterministically routes to the fallback behavior. This is the authoritative first place staleness is computed; node-level and monitor-level staleness checks are redundant defense, not the primary line.

**Capture time, not receipt time.** Samples are stamped with the sensor's own capture timestamp, never the wall-clock time the callback ran. Receipt time has executor and transport jitter baked in; stamping there relaunders the noise the assembler exists to remove.

**The assembler is a standalone library.** It depends only on a timestamped `Sample<T>`, the ring buffers, and the snapshot builder. No SPAR- or transport-specific types. SPAR wires it in; it does not own SPAR.

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
        │  ControllerAdapter      (TRUSTED)           │
        │  • single writer to controller              │
        └────────────────────┬────────────────────────┘
                             │ cmd_vel (ROS 2 / Zenoh)
                             ▼
        ┌─────────────────────────────────────────────┐
        │  Vehicle controller   (TRUSTED CONTROL)     │
        │  • low-level control, estimation, failsafes │
        └─────────────────────────────────────────────┘
```

The contract doesn't depend on what sits above it. Whether the policy is hand-coded or a 10B-parameter VLA, the monitor, adapter, and logging schema stay the same. The trusted surface stays small while the policy above can scale arbitrarily.

## Process Layout and Data Flow

SPAR runs as a **single process** (`spar_rover`) with a tick thread at fixed rate and sensor threads running asynchronously.

```
                    wheel odom + IMU + (GPS)
                             │
                             ▼
             external estimator (robot_localization)  ← runs on the vehicle, not in SPAR
                             │
                    odometry (ROS 2 / Zenoh)    nav_msgs/Odometry, TF
                    (estimate, already fused)
                             │
          ┌──────────────────┼───────────────────────┐
          │  spar_rover       ▼                      │
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
          │  │              Adapter ───────────────┼─┼──▶ vehicle
          │  └─────────────────────────────────────┘ │
          └──────────────────────────────────────────┘
```

Only the `ControllerAdapter` (e.g. `Ros2Adapter`) links against the controller transport; no other code path writes to the controller.

### Tick ordering

Each 20 Hz tick executes in fixed order:

```
1. Observation assembler builds WorldState via latest_at_or_before(t) over all source ring buffers
2. BTNode::tick(GoalContext, WorldState) → CommandStream
3. RuntimeMonitor::evaluate(CommandStream) → MonitorDecision
4. If not Halt: Adapter::write(approved CommandStream) → vehicle controller
5. Append MonitorDecision + per-source WorldState staleness to session log
```

Sensor data arrives whenever the estimator or sensors publish, not synchronized to the tick cadence. The assembler's `latest_at_or_before(t)` lookup absorbs that jitter; behavior nodes see a clean fixed-rate sequence of coherent observations.

## WorldState and Sensor Sources

`WorldState` is the immutable snapshot the assembler builds once per tick. It holds processed state estimates (never raw sensor readings) plus per-source staleness relative to the tick's reference time `t`. Once built, it is read-only for the duration of that tick: no domain locks, no concurrent writes. Thread safety lives in the ring buffers on the write side; by the time `WorldState` exists it is already a value.

```cpp
struct WorldState {
    Pose         pose;       // from the external estimator via odometry; stamped with capture time
    ObstacleMap  obstacles;  // from lidar / camera pipeline
    BatteryState battery;    // from vehicle diagnostics
    // per-source staleness carried alongside each domain
};
```

### Navigation state: the external estimator owns this

The odometry SPAR consumes is not raw GPS. It is the vehicle's state-estimator output (e.g. `robot_localization`), already fusing wheel odometry, IMU, and — when present — GPS on the vehicle, published in a local metric frame. SPAR's ingest thread decodes it and writes it to `WorldState.pose`. There is no separate navigation EKF running in SPAR.

Publish rates are configured on the vehicle side and matter for staleness. At low rates a 20 Hz tick sees pose data hundreds of ms old. Recommended minimum for the rover demo:

```
/odometry/filtered  ≥ 10 Hz    # state estimate → WorldState.pose
```

### Perception state: SPAR owns this

Sensors the state estimator doesn't fuse (lidar, camera, any external perception system) write their processed output into `WorldState` directly via their own threads. SPAR is responsible for this processing. It is the only domain where SPAR runs any fusion or detection logic of its own.

### Simulation must inject transport degradation

`DegradedSource<T>` wraps each simulated source and injects per-stream delay, jitter, and dropout. Always run both a clean variant and a degraded variant — the difference in catch fractions between them isolates timing as a variable. Catch fractions from a clean-only run are not meaningful for deployment comparison.

### Staleness is the assembler's responsibility first

The assembler carries per-source staleness in the snapshot and emits a degraded verdict when a required source exceeds its bound, routing deterministically to the fallback before the behavior node runs. Node-level staleness checks (returning `Failure` on old pose data) remain as redundant backstops, not the primary line. The monitor's staleness check on commands is the last line of defence.

### ObservationBuffer: learned nodes only (Phase 4a)

Hand-coded nodes only need `WorldState`. Learned policies that require a multi-frame context window (Cosmos actor, recurrent policies) also need access to a raw frame history. This is handled via an `ObservationBuffer` added in Phase 4a.

Current tick signature (Phase 1 through Phase 3):

```cpp
virtual NodeStatus tick(const GoalContext& goal,
                        const WorldState&  world,
                        CommandStream&     out_cmd) = 0;
```

Phase 4a evolution — when `CosmosNavigateNode` is introduced, a fourth parameter is added before `out_cmd`:

```cpp
virtual NodeStatus tick(const GoalContext&       goal,
                        const WorldState&         world,
                        const ObservationBuffer*  obs,    // nullptr for all non-video nodes
                        CommandStream&            out_cmd) = 0;
```

Hand-coded nodes receive `nullptr` for `obs` and ignore it. The contract stays symmetric regardless of what sits above.

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

Every monitor decision records the triggered invariant (string) and an `invariant_flags` bitmask encoding which class fired, alongside the outcome. This makes the key postmortem question decidable from logs: did no applicable invariant exist for this case, or did one exist and the monitor fail to enforce it?

### What the monitor cannot catch

A policy producing physically valid, sequentially reasonable commands that execute the wrong plan is invisible to the architectural monitor by construction. That failure class hands off to a **behavioral monitor** running at the policy-semantics layer: temporal consistency over action distributions, VLM-based task-progress judgment, out-of-distribution scoring on policy outputs.

```
                        policy command stream
                                 │
               ┌─────────────────┼─────────────────┐
               ▼                                   ▼
┌──────────────────────────────┐   ┌──────────────────────────────┐
│  Architectural monitor       │   │  Behavioral monitor          │
│  (SPAR, this repo)           │   │  (future layer)              │
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

## Architectural Constraints

These are load-bearing assumptions, not bugs. They are named explicitly so they don't become invisible.

### The reactive-policy assumption

The tick loop calls `node->tick()` every 50 ms and expects a `CommandStream` back. This assumes a **Markovian, reactive** policy: one observation in, one command out, synchronous with the tick rate.

Diffusion policies (π0) and action-chunking systems (ACT) do not work this way. They output trajectory segments — 10 to 100 future commands sampled from a joint distribution. They also run at 1–5 Hz on realistic hardware, not 20 Hz. The `BTNode` contract will need a second evolution before these policy types are first-class:

- **Action chunk buffer** in the tick loop: the policy writes an N-step sequence; the tick loop drains it one command per tick
- **Async inference path**: the policy runs in a background thread; the chunk buffer is the handoff point
- The monitor approves chunks as units, then individual commands as they execute

**The deferral deadline is Phase 4a start, not "later."** If Phase 4a uses an autoregressive Cosmos actor (one token per step, compatible with the current `BTNode` contract), deferral is safe. If Phase 4a uses a diffusion-policy or action-chunking actor — the dominant pattern for current VLAs (π0, GR-00T) — the action chunk buffer and async inference path are blocking prerequisites. Resolve which actor architecture Phase 4a will use before starting it, not during it. See `docs/rl-pipeline.md` for the decision table and `ChunkedBTNode` design.

### The behavioral monitor inference budget

Running Cosmos as a behavioral monitor means running it every tick alongside the policy, not just at training time. Latency and hardware requirements for this path are distinct from running Cosmos as the actor. Establish that the combined inference fits the tick budget before committing to this architecture (Phase 4b).

## Stack Position

SPAR is the **within-platform** half of the composition story: the authority boundary on each vehicle. The cross-platform half is Tower, which composes vehicles into a fleet through the pidgin protocol seam. The two boundaries are independent: SPAR without Tower, or Tower without SPAR, are both valid configurations.
