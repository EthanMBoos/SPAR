# SPAR Mission Design

This doc works through how missions are structured, executed, and how behavior switching fits in. It is a design-in-progress; the goal is to settle on the right shape before wiring more code.

---

## What a BTNode Is

A `BTNode` is the simplest possible unit of robot behavior: **an object with one method that gets called every tick.**

```
Every 20ms:
  node.tick(current goal, current world state) → Success / Failure / Running
                                               + a command to send to the robot
```

`NavigateNode` is the only one that exists right now. Every tick it reads the current pose, computes how far from the target, outputs a throttle and steering command, and returns `Running` (still going), `Success` (arrived), or `Failure` (pose stale or timed out). It does not sequence anything. It does not know about other tasks. It just answers "given where I am right now, what should I do?" once per tick.

A BTNode can be hand-coded (like `NavigateNode`) or learned (an ONNX model wrapped to satisfy the same interface). Both look identical to everything below the boundary.

**BT** (Behavior Tree) is the name for how nodes can be composed within a single task: a `Sequence` runs children left to right and stops if any fail; a `Selector` tries children in order and stops when one succeeds. For the rover demo, each task is a single node so this structure barely matters yet.

**The BT is not the state machine.** The state machine (which task are we on, what comes next, what happens on failure) lives in the `MissionExecutor`. A BTNode just runs *during* a task.

---

## User Roles

There are three distinct users of this system. Getting this wrong produces interfaces that expose implementation details to people who should never see them.

### Role 1: System Integrator

**Who:** The robotics engineer setting up the platform. This is the person writing C++.

**When:** Once at setup, not per deployment.

**What they configure:**
- Which `BTNode` implementations are registered and what task types they handle, initially all hand-coded; learned nodes are a drop-in addition later
- What mode labels map to (e.g., `"careful"` → conservative nav parameters; `"standard"` → default nav; `"autonomous"` → learned node with hand-coded fallback)
- Monitor invariant limits: the safe operating envelope for this specific vehicle
- Default failure handling per task type (retry count, fallback behavior)

Initially "mode" is just a parameter set: max speed, arrival radius, steering gain. Learned nodes are added later as an additional option behind the same mode label interface. The planner and field operator see no difference.

**What they never expose:** BTNode class names, model file paths, monitor threshold values, CommandStream internals. All of this is resolved at integration time, not by operators.

---

### Role 2: Mission Planner

**Who:** The operator or supervisor who defines what the robot should accomplish on a given deployment. May not be technical.

**When:** Before deployment, or during a planning phase.

**What they configure:**

```
Mission: Survey Route Alpha

Task 1  →  Navigate to [33.7756, -84.3963]    mode: normal
Task 2  →  Navigate to [33.7800, -84.3900]    mode: careful
Task 3  →  Return home                         mode: normal

On task failure: retry once, then abort mission
```

That is the full interface. Waypoints, task order, a mode label, and a failure policy. The system integrator has already mapped `mode: careful` to the right BTNode; the mission planner never sees that mapping. They pick *intent*, not implementation.

**What they never see:** BTNode types, ONNX model names, monitor invariant names, GoalContext fields, CommandStream.

**Practical format:** JSON task list is the minimum viable format for loading missions without recompiling. For Phase 1 SITL demos, hardcoded is acceptable. For Phase 2 (cross-policy catch fraction experiments) a JSON loader is required so mission configuration can change without a rebuild.

---

### Role 3: Field Operator

**Who:** The person running the robot on the day. May be the same person as the mission planner, or a separate technician.

**When:** During a live deployment.

**What they interact with:**

```
Mission: Survey Route Alpha          [RUNNING]

  ✓  Task 1  Navigate to checkpoint 1    completed
  ►  Task 2  Navigate to checkpoint 2    in progress — ETA ~40s
     Task 3  Return home                 pending

Monitor: OK  |  Pose: fresh (82ms)  |  Policy: autonomous

[PAUSE]   [SKIP STEP]   [ABORT MISSION]
```

Status in plain language: not "RuntimeMonitor: STALENESS_EXCEEDED on CommandStream" but "pose data stale, holding position." Not "MonitorDecision::Fallback triggered RATE_OF_CHANGE_STEERING" but "steering corrected by monitor."

**What they can do:**
- Start / pause / resume a mission
- Skip the current task (advances to the next)
- Abort the mission (triggers a safe-stop)
- Read current task status and monitor health

**What they cannot do:** Change task parameters mid-mission, swap policies, modify monitor limits. Those are integrator and planner concerns. Giving a field operator access to them is how you get unsafe overrides in the field.

---

### What Each Role Touches

| Component | Integrator | Planner | Field Operator |
|---|---|---|---|
| BTNode implementations | configures | invisible | invisible |
| Behavior-to-mode mapping | defines | picks label only | invisible |
| Monitor invariant limits | sets | invisible | sees plain-language alerts |
| Mission task list | n/a | authors | read-only |
| Task mode / failure policy | sets defaults | overrides per task | invisible |
| Live start / pause / abort | n/a | n/a | full control |
| Session log | reads post-hoc | n/a | n/a |

The key principle: **operators pick intent, the system resolves implementation.** Any interface that surfaces a BTNode name, a policy file path, or a monitor threshold to a planner or field operator is a design error.

---

## The Problem With GoalContext Today

`GoalContext` is a per-tick message: a goal, a mode, a waypoint. It answers "what are we doing right now" but holds no state. There is no sequencing, no record of what step we're on, no mechanism to swap which policy is running. The state machine is implicit: `main.cpp` runs one `NavigateNode` until it returns `Success` and then exits. That is fine for a proof-of-concept but falls apart the moment a mission has more than one step.

Three things are conflated right now and need to be separated:

1. **Mission**: what steps, in what order, under what conditions
2. **Task**: one atomic step (a goal + which policy executes it + how we know it's done)
3. **GoalContext**: the per-tick message from the active task down to the active policy

These are separate concerns. Mixing them produces a monolithic main loop that can only do one thing.

---

## Layers

```
┌──────────────────────────────────────────────────────────────────┐
│  Operator / Tower                                                │
│  Loads a Mission (JSON config, protobuf message, or hardcoded)   │
│  Can pause, abort, override step                                 │
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
│  RuntimeMonitor  →  Adapter  →  ArduPilot                        │
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

- **A goal**: GoalContext fields (waypoint, mode, timeout, mission_id)
- **A primary node**: which BTNode executes this task (hand-coded or learned)
- **A fallback node**: what to switch to if the behavioral monitor flags a problem; `nullptr` means Halt instead
- **A completion condition**: usually NodeStatus::Success, but could be time-bounded
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

The mission executor advances from task N to task N+1 when task N returns `Success`. If it returns `Failure`, the executor can: retry, skip to a recovery task, or fail the whole mission.

More complex missions can use conditional branching ("if task 2 fails, execute task 5 instead of task 3"), but that should be expressed as explicit transition rules, not buried in BT logic. The mission layer owns sequencing. The BT layer owns command generation.

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
                → Aborted   (operator interrupt)
```

**Per-task states:**
```
Pending → Active → Succeeded
                 → Failed
                 → Aborted
```

The executor runs the active task each tick. When the task's policy returns `Success`, the executor marks it `Succeeded` and advances to the next task per the transition table. When it returns `Failure`, the executor looks up the `Failure` transition rule for the current task.

This is all deterministic and fully logged. Every task state transition gets a session log entry with the task_id, the trigger, and the timestamp. This is what makes the mission reconstructable from logs.

---

## Where BTs Fit

**BTs are task-level, not mission-level.** A BT node executes a single task's goal. The mission executor handles sequencing across tasks.

But within a task, BTs earn their complexity budget. A task like "approach and dock" might have a sub-tree:

```
Sequence
  ├─ ApproachNode       (get within 2m of target)
  ├─ AlignNode          (orient to docking face)
  └─ FinalApproachNode  (slow, precise, different policy)
```

That's three BT nodes cooperating on one `Task`. The mission layer doesn't see any of that; it just sees the root of the sub-tree returning `Success` or `Failure`. The BT internal structure is invisible above the task boundary.

For the rover demo, each task is probably a single leaf node. BT sub-trees become relevant as tasks get more complex.

**Contrast with nav2:** nav2 uses a BT as the mission itself; the tree root IS the mission state machine. This works and has good tooling, but it couples mission sequencing logic into the BT, which makes policy substitution harder to reason about. SPAR separates them because the substitution point is the research contribution.

---

## Behavior Switching

Node switching happens at the task boundary and within a task via the behavioral fallback hook.

**At task boundaries (static):**
The mission config assigns which node runs each task. For research experiments, you run the same mission twice: once with `NavigateNode` on every task, once with `OnnxNavigateNode` on some tasks. The monitor sees the same command stream either way. This is the primary experimental variable for measuring catch fractions.

**Mid-task via behavioral monitor flag (dynamic):**
Phase 3 adds a behavioral monitor running on a slower clock. When it flags out-of-distribution behavior or degraded task progress, the mission executor swaps `primary_node` for `fallback_node` on the current task and resets it. The task goal stays the same. The command stream from that point forward comes from the fallback node.

```cpp
void MissionExecutor::on_behavioral_flag() {
    Task& t = active_task();
    if (t.fallback_node) {
        t.fallback_node->reset();
        t.active_node = t.fallback_node;
        log_node_swap(t.task_id, "behavioral_flag");
    } else {
        // no fallback configured — escalate to Halt
        mission_state_ = MissionState::Aborted;
    }
}
```

The monitor sees no difference; it still evaluates `CommandStream` through the same invariant checks regardless of which node is upstream. That is the architectural invariant the research depends on.

---

## GoalContext After This Change

`GoalContext` gains a `task_id` field and nothing else. It stays a thin per-tick message.

```cpp
struct GoalContext {
    uint64_t     mission_id   = 0;
    uint32_t     task_id      = 0;   // added — session log attribution
    uint64_t     timestamp_us = 0;
    Waypoint     target;
    BehaviorMode mode         = BehaviorMode::Stop;
};
```

The executor populates `task_id` from the active task before passing `GoalContext` down each tick. The session log writes `task_id` alongside `frame`, `outcome`, and `triggered_invariant`. Every monitor decision is now attributable to a specific task and a specific policy without relying on operator memory.

---

## Session Log After This Change

Current log columns:
```
frame  timestamp_us  outcome  triggered  throttle  steering
```

After adding mission/task tracking:
```
frame  timestamp_us  mission_id  task_id  policy_id  outcome  triggered  throttle  steering
```

`policy_id` is the `BTNode::node_id()` of whichever node was active this tick. When a mid-task policy swap happens, `policy_id` changes at the exact frame where the behavioral flag fired. This is what makes the four catch fractions computable from the log: architectural-only, behavioral-only, both, neither.

---

## What This Doesn't Cover Yet

- **Mission loading**: hardcoded in `main.cpp` for now. JSON or protobuf loading from Tower is a deployment concern, not a research prerequisite. Design the `Mission` struct cleanly and loading format is mechanical.
- **Operator interrupt**: pause/abort/skip from an operator console. The mission state machine has the `Aborted` state but no input path yet. A simple signal handler or UDP command socket would cover this for SITL demos.
- **Parallel tasks**: running two BT nodes simultaneously (e.g., navigate + scan). Not needed for rover demo. If it comes up, the right answer is compositing two CommandStreams at the mission layer before the monitor, not changing the monitor contract.
- **Task retry**: retry logic belongs in the transition rules, not in the BT node. A node that internally retries produces a command stream the monitor can't inspect over the retry boundary.

---

## Open Questions Before Implementing

1. **Should `BehaviorTree` survive as a class, or does `MissionExecutor` absorb it?**
   Right now `BehaviorTree` is just a `std::vector<BTNode*>` with a `tick()`. If the mission executor owns the active node directly, `BehaviorTree` is redundant for simple missions. It might survive as the mechanism for within-task sub-trees.

2. **Is the fallback policy per-task or per-mission?**
   Per-task is more expressive. But for Phase 1/2 a single mission-wide fallback policy (always `NavigateNode`) is simpler and probably sufficient.

3. **How does the mission get from the operator to the robot for the SITL demo?**
   Hardcoded is fine for Phase 1. For Phase 2 (measuring catch fractions across policy types) we need to be able to swap missions without recompiling; a JSON task list is the minimum.

4. **Does the behavioral monitor flag a specific task, or the whole mission?**
   It should flag the active task. The response is task-scoped: swap to fallback or abort the task. The mission-level state machine decides what happens next.
