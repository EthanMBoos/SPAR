# SPAR Mission Design

This doc covers how missions are structured and executed, how behavior switching works, and the session log schema needed to compute catch fractions per (task, policy) pair.

---

## What a BTNode Is

A `BTNode` is the simplest possible unit of robot behavior: **an object with one method that gets called every tick.**

```
Every 50ms:
  node.tick(current goal, current world state) → Success / Failure / Running
                                               + a command to send to the robot
```

`NavigateNode` is the only one that exists right now. Every tick it reads the current pose, computes how far from the target, outputs a throttle and steering command, and returns `Running` (still going), `Success` (arrived), or `Failure` (pose stale or timed out). It does not sequence anything. It does not know about other tasks.

A BTNode can be hand-coded (like `NavigateNode`) or learned (an ONNX model wrapped to satisfy the same interface). Both look identical to everything below the boundary.

**The BT is not the state machine.** The state machine (which task are we on, what comes next, what happens on failure) lives in the `MissionExecutor`. A BTNode just runs *during* a task.

---

## The Problem With GoalContext Today

`GoalContext` is a per-tick message: a goal, a waypoint. It answers "what are we doing right now" but holds no state. There is no sequencing, no record of what step we're on, no mechanism to swap which policy is running. The state machine is implicit: `main.cpp` runs one `NavigateNode` until it returns `Success` and then exits. That is fine for a proof-of-concept but falls apart the moment a mission has more than one step.

Three things are conflated right now and need to be separated:

1. **Mission**: what steps, in what order, under what conditions
2. **Task**: one atomic step (a goal + which policy executes it + how we know it's done)
3. **GoalContext**: the per-tick message from the active task down to the active policy

---

## Layers

```
┌──────────────────────────────────────────────────────────────────┐
│  Mission loader (JSON)                                           │
│  Loads a Mission (task list + transition rules)                  │
└────────────────────────────┬─────────────────────────────────────┘
                             │  Mission (task list + transition rules)
                             ▼
┌──────────────────────────────────────────────────────────────────┐
│  MissionExecutor                                                 │
│  Owns the task list and mission state                            │
│  Tracks which task is active, advances on Success/Failure        │
│  Selects which BTNode runs the active task                       │
│  Exposes hook for behavioral monitor to trigger node fallback    │
└────────────────────────────┬─────────────────────────────────────┘
                             │  GoalContext (current task's goal, this tick)
                             │  + pointer to active BTNode
                             ▼
┌──────────────────────────────────────────────────────────────────┐
│  BTNode / Policy                                                 │
│  Executes the current task goal over one or many ticks           │
│  Returns Success / Failure / Running                             │
│  Hand-coded or learned, same contract, same interface            │
└────────────────────────────┬─────────────────────────────────────┘
                             │  CommandStream
                             ▼
┌──────────────────────────────────────────────────────────────────┐
│  RuntimeMonitor  →  Adapter  →  controller / KinematicBackend    │
│  (authority boundary, unchanged)                                 │
└──────────────────────────────────────────────────────────────────┘
```

The tick loop in `main.cpp` stays simple:

```cpp
const ActiveTask t = mission.active_task();
NodeStatus status  = t.policy->tick(t.goal, world, raw);
MonitorDecision d  = monitor.evaluate(raw, tick_start);
if (d.outcome != Halt) adapter.write(d.output_cmd);
mission.on_result(status, d);
log_frame(...);
```

---

## What Is a Task

A `Task` is the atomic unit of work. It has:

- **A goal**: GoalContext fields (waypoint, timeout, mission_id)
- **A primary node**: which BTNode executes this task (hand-coded or learned)
- **A fallback node**: what to switch to if the behavioral monitor flags a problem; `nullptr` means Halt instead
- **An id**: written into the session log so every monitor decision is attributable to a specific task

```cpp
struct Task {
    uint32_t    task_id;
    GoalContext goal;
    BTNode*     primary_node;
    BTNode*     fallback_node;  // nullptr = no fallback, Halt on behavioral flag
    std::string label;          // human-readable, goes into log
};
```

The node pointer is the substitution point. That's where you swap in a learned node, a distribution-shifted node for experiments, or a different hand-coded behavior. Nothing else in the stack changes.

---

## What Is a Mission

A `Mission` is a list of tasks plus transition rules.

For most rover demos a linear sequence covers it:

```
Task 0: Navigate to waypoint A      node: NavigateNode
Task 1: Navigate to waypoint B      node: OnnxNavigateNode, fallback: NavigateNode
Task 2: Navigate back to origin     node: NavigateNode
```

The mission executor advances from task N to task N+1 when task N returns `Success`. If it returns `Failure`, the executor can retry, skip to a recovery task, or fail the whole mission.

```cpp
struct TransitionRule {
    enum class Trigger { Success, Failure, BehavioralFlag };
    Trigger  on;
    uint32_t next_task_id;  // 0xFFFFFFFF = mission complete/abort
};

struct Mission {
    uint64_t                    mission_id;
    std::vector<Task>           tasks;
    std::vector<TransitionRule> transitions; // keyed by (task_id, trigger)
};
```

For Phase 1 and 2 a simple linear vector with no transition rules is sufficient.

---

## Mission State Machine

The executor is a state machine at two levels: mission state and task state.

**Mission states:**
```
Idle → Running → Completed
                → Failed
                → Aborted
```

**Per-task states:**
```
Pending → Active → Succeeded
                 → Failed
                 → Aborted
```

The executor runs the active task each tick. When the task's policy returns `Success`, the executor marks it `Succeeded` and advances to the next task per the transition table. Every task state transition gets a session log entry with the task_id, the trigger, and the timestamp.

---

## Behavior Switching

Node switching happens at task boundaries and mid-task via the behavioral fallback hook.

**At task boundaries (static):**
The mission config assigns which node runs each task. For research experiments, run the same mission twice: once with `NavigateNode` on every task, once with `OnnxNavigateNode` on some tasks. The monitor sees the same command stream either way. This is the primary experimental variable for measuring catch fractions.

**Mid-task via behavioral monitor flag (dynamic):**
Phase 3 adds a behavioral monitor. When it flags out-of-distribution behavior, the mission executor swaps `primary_node` for `fallback_node` on the current task and resets it.

```cpp
void MissionExecutor::on_behavioral_flag() {
    Task& t = active_task();
    if (t.fallback_node) {
        t.fallback_node->reset();
        t.active_node = t.fallback_node;
        log_node_swap(t.task_id, "behavioral_flag");
    } else {
        mission_state_ = MissionState::Aborted;
    }
}
```

The monitor sees no difference: it still evaluates `CommandStream` through the same invariant checks regardless of which node is upstream.

---

## GoalContext After MissionExecutor

`GoalContext` gains a `task_id` field. It stays a thin per-tick message.

```cpp
struct GoalContext {
    uint64_t mission_id   = 0;
    uint32_t task_id      = 0;   // Phase 2: populated by MissionExecutor
    uint64_t timestamp_us = 0;
    Waypoint target;
};
```

The executor populates `task_id` from the active task before passing `GoalContext` down each tick. Every monitor decision is then attributable to a specific task and a specific policy without relying on operator memory.

---

## Session Log Schema

Log file header (two lines):
```
# scenario: <SPAR_SCENARIO env var, default "baseline">
frame  timestamp_us  policy_id  task_id  outcome  triggered_invariant  invariant_flags  throttle  steering  pose_age_us  pose_degraded  mission_outcome
```

- `policy_id`: `"navigate"` or `"onnx"` — set at compile time by which node is active
- `task_id`: hardcoded `0` for Phase 1; populated by MissionExecutor in Phase 2
- `outcome`: `PASS` / `FALL` / `HALT` — monitor decision for this command
- `triggered_invariant`: empty string on PASS; invariant name string otherwise
- `invariant_flags`: bitmask byte (see `MonitorDecision::kFlag*` constants)
- `mission_outcome`: `running` on every frame except the terminal frame; `success` when the policy returned `NodeStatus::Success`; `failure` when the policy returned `NodeStatus::Failure` (stale pose or timeout)

`mission_outcome` is the oracle signal needed for the "neither" catch-fraction cell. A frame where `outcome == PASS` and `mission_outcome == failure` is a failure invisible to the architectural monitor.

The four catch fractions are computable from this log:
- **Architectural-only**: frames where `outcome != PASS` and behavioral monitor did not flag (Phase 3+)
- **Behavioral-only**: frames where behavioral monitor flagged but `outcome == PASS`
- **Both**: both flagged
- **Neither**: `outcome == PASS` and `mission_outcome == failure` (terminal frame) — policy failed without the monitor catching it

Phase 2 adds `mission_id` column. Phase 3 replaces TSV with MCAP for temporal range queries over command history.
