# High-Level Behaviors and Composition

A search and rescue mission stress-tests the current design. The core sequencing and substitution machinery holds up, but four gaps appear the moment tasks need to involve more than "navigate to a waypoint." This doc identifies each gap, proposes the fix, and maps each one to the user role that owns it.

---

## The Four Gaps

### Gap 1: CommandStream Is Single-Actuator

The current `CommandStream` carries exactly one thing: throttle and steering. A real mission also needs camera control, gimbal angle, payload triggers, recording state. Every actuator that can cause harm needs to pass through the monitor; you cannot add a side channel for camera commands that skips invariant checking. The monitor's authority boundary is only meaningful if it covers all actuator output.

**Fix: CommandBundle**

Replace `CommandStream` with a `CommandBundle` that carries optional output per actuator family. Each family has its own fields, its own invariants in the monitor, and its own limits set by the system integrator. Adding a new actuator is additive; existing families and their invariants are untouched.

```
CommandBundle
  ├── LocomotionCommand   throttle, steering           (today's RoverCommand)
  ├── CameraCommand       pan_deg, tilt_deg, recording
  └── (future actuators added here)
```

The monitor gains per-family invariant sets. A `CameraCommand` is checked against camera bounds; a `LocomotionCommand` is checked against locomotion bounds. The output of the monitor is still a single approved `CommandBundle`, and the adapter still has single-writer authority, now over all actuators, not just locomotion.

**User role:** System integrator defines the `CommandBundle` schema and the monitor invariants per family. Planners and field operators never see actuator families directly.

---

### Gap 2: No Behavior Composition Within a Task

Right now, a task maps to a single BTNode. A task like "search sector A" requires two things happening simultaneously: a `SearchPatternNode` driving the vehicle through a coverage route, and a `CameraControlNode` keeping the sensor pointed and recording. Neither node knows the other exists. The current tick loop calls exactly one node.

**Fix: BT Composite Nodes**

Three composite types cover most cases:

**Sequence**: run children in order, stop if any fail.
```
Sequence
  ├── ApproachNode       get within 5m of target
  ├── AlignNode          orient to face
  └── ConfirmNode        hold and verify
```

**Selector**: try children in order, stop when one succeeds. Used for fallbacks.
```
Selector
  ├── LearnedNavNode     try first
  └── NavigateNode       hand-coded fallback if learned fails
```

**Parallel**: tick all children every tick, merge their output into one `CommandBundle`.
```
Parallel
  ├── SearchPatternNode  → writes LocomotionCommand
  └── CameraControlNode → writes CameraCommand
```

The `Parallel` node is the key addition for SAR. It collects each child's `CommandBundle`, merges them (locomotion from one, camera from another), and passes the merged bundle up to the monitor. Each family is still checked independently.

**User role:** System integrator wires composite trees at integration time. The mission planner selects task *types* (e.g., "Search" which internally uses a Parallel tree) but never sees the tree structure.

---

### Gap 3: No Inter-Task Data Flow

"Task 2 detected a target at GPS coordinate X. Task 3 should navigate to X." Currently `GoalContext` is populated statically from the mission config. A task has no way to pass computed data forward to a task that hasn't run yet.

**Fix: Mission Blackboard**

A simple key-value store owned by the `MissionExecutor`. Tasks write to it on completion; the executor reads from it when populating `GoalContext` for subsequent tasks.

```
Task 2: SearchNode → on Success, writes "detected_target" = {lat, lon}
Task 3: NavigateNode → executor reads "detected_target", populates GoalContext.target
```

The blackboard connection is declared in the mission config:

```
Task 3: Navigate to detected target
  input: target = blackboard["detected_target"]
```

If the key is absent when Task 3 starts (Task 2 never wrote it), the executor treats it as a task precondition failure and follows the failure transition rule for Task 3.

The integrator defines which keys each task type reads and writes. The planner sees named inputs and outputs ("navigate to detected target"), not key names. The field operator never sees the blackboard.

---

### Gap 4: No Mission-Level Looping or Conditional Branching

A search mission is not a straight line. "Search sector A; if nothing found, search sector B; if found in either sector, orbit for relay; otherwise return to base." The current `TransitionRule` supports only linear advancement. It has no conditional branches and no loop-back.

Two separate problems need separate solutions.

**Coverage routes belong inside the node, not the mission.**

A search area is not 50 sequential waypoints. The mission planner gives an area and a pattern; the `SearchPatternNode` generates the route internally and runs it to completion. The mission executor sees only `Running → Success/Failure`. This is the right boundary: the executor sequences *tasks*, not waypoints within a task.

```
Task 2: Search
  area:    sector-A.geojson
  pattern: lawnmower
  sensor_footprint_m: 15
```

`SearchPatternNode` takes those parameters, computes the coverage route, and drives through it. The planner never touches a waypoint list.

**Conditional branching belongs in the transition rules.**

The found/not-found branch after a search task is a mission-level concern, not a BT concern. Express it explicitly:

```
Task 2: Search sector A
  on Success (target found)  → Task 4: Orbit target
  on Success (area complete) → Task 3: Search sector B
  on Failure                 → Task 5: Return to base
```

This requires the node to distinguish *why* it succeeded: "target found" vs. "area covered with no detection." The node writes a result code to the blackboard alongside any detection data. The transition rule reads the result code.

**Patrol loops** ("repeat route 3 times") are handled with a loop counter in the transition rule:

```
Task 2: Patrol route
  on Success, repeat: 3 → loop back to Task 2
  on Success, after 3   → Task 3: Return to base
```

This keeps loop logic out of the node and in the mission config where the planner can see and change it.

---

## How a SAR Mission Looks Across Roles

### What the system integrator configures (once)

- `SearchPatternNode`: area + pattern → coverage route → locomotion commands
- `CameraControlNode`: recording mode, tilt angle, stabilization
- `Parallel` composite wiring `SearchPatternNode` + `CameraControlNode` as the "Search" task type
- `OrbitNode`: GPS coordinate + radius → orbit locomotion commands
- Monitor invariants for locomotion and camera families
- Blackboard key schema: which keys each task type reads and writes
- Mode labels: `"search-standard"` → Parallel(SearchPatternNode, CameraControlNode)

### What the mission planner authors

```
Mission: Alpha SAR

Task 1  Navigate to search area entry     mode: standard
Task 2  Search sector A                   area: sector-a.geojson, pattern: lawnmower
          on found      → Task 4
          on no-contact → Task 3
Task 3  Search sector B                   area: sector-b.geojson, pattern: lawnmower
          on found      → Task 4
          on no-contact → Task 5
Task 4  Orbit detected target             radius: 30m, source: detected_target
          on operator-release → Task 5
Task 5  Return to base
```

No BTNode names. No actuator families. No coverage waypoints. No blackboard key names. The planner picks task types, sets the parameters each type exposes, and draws the transition graph.

### What the field operator sees

```
Mission: Alpha SAR          [RUNNING]

  ✓  Task 1  Navigate to search area      completed
  ►  Task 2  Search sector A              in progress
             Pattern: 40% covered
             Camera: recording
             No contact yet

     Task 3  Search sector B              pending
     Task 4  Orbit detected target        pending
     Task 5  Return to base               pending

Monitor: OK  |  Pose: fresh (74ms)

[PAUSE]   [SKIP STEP]   [ABORT MISSION]
```

"40% covered" comes from `SearchPatternNode` reporting progress. "No contact yet" comes from the blackboard result code. The operator reads mission progress in task terms, not system terms.

---

## Two Ways to Define a Mission

Users should be able to specify a mission at whatever level of detail they want. These are not two different systems; they are two input modes that converge at the same `Task` struct the `MissionExecutor` consumes.

---

### Mode A: Manual

The user draws the route explicitly. Every waypoint, every parameter, in order. The system executes exactly what was authored.

```
Task 1  Navigate to [33.7751, -84.3963]
Task 2  Navigate to [33.7762, -84.3941]
Task 3  Navigate to [33.7775, -84.3920]
Task 4  Return to base
```

Useful when the operator knows exactly where the robot needs to go: a known inspection path, a pre-surveyed route, a corridor that must be followed precisely.

---

### Mode B: Intent-Based

The user describes the objective. The system uses a per-behavior planner to compute the execution parameters: waypoints, coverage route, orbit geometry, whatever the behavior requires.

```
Task 1  Navigate to search area entry     target: [33.7751, -84.3963]
Task 2  Search                            area: sector-a.geojson
                                          pattern: lawnmower
Task 3  Return to base
```

The planner for "Search" takes the area polygon and computes the coverage route. The user never sees waypoints. If you swap the planner (lawnmower → spiral → probability-weighted), the mission config is unchanged.

---

### How They Converge

Both modes produce the same `Task` struct. The difference is where the task parameters come from:

```
Mode A (manual):   user provides parameters directly → Task
Mode B (intent):   user provides objective → Behavior Planner → Task
                                                                  ↓
                                                         MissionExecutor
                                                                  ↓
                                                              BTNode
                                                                  ↓
                                                    Monitor → Adapter → Robot
```

The `MissionExecutor`, `BTNode`, monitor, and adapter see no difference. Planning is resolved before execution starts.

---

## Per-Behavior Planners

Each behavior type has an optional registered planner. The planner is a pure function: takes the user's intent parameters, returns a fully-specified task (or a sequence of tasks for multi-leg behaviors). It runs before mission execution, not during it.

```
NavigateBehavior   → WaypointPlanner      validate and pass through
SearchBehavior     → CoveragePathPlanner  area + pattern → waypoint sequence
InspectBehavior    → InspectionPlanner    target + standoff → orbit geometry
PatrolBehavior     → PatrolPlanner        route + repeat count → leg sequence
```

Planners are registered by the system integrator. Swapping a planning algorithm (lawnmower to Zamboni pattern, uniform coverage to probability-map-weighted) is an integrator concern. The planner and the behavior node are independent: the planner computes where to go, the BTNode computes the commands to get there.

**The planner is not the BTNode.** A `SearchPatternNode` does not generate its own route. It receives a waypoint list from the planner and executes it. This keeps planning logic (geometry, optimization) separate from execution logic (sensor reads, command output, Success/Failure signaling).

---

### What the Planner Interface Looks Like

```cpp
class BehaviorPlanner {
public:
    // Takes raw user-provided parameters (from mission JSON or UI).
    // Returns a fully-specified task list ready for MissionExecutor.
    virtual std::vector<Task> plan(const PlannerInput& input,
                                   const NodeRegistry&  nodes) = 0;
    virtual const char* behavior_type() const = 0;
};
```

The `NodeRegistry` is passed so the planner can resolve which BTNode handles each generated task; the planner knows about task types, not BTNode internals. This keeps the planner in the integrator's domain.

---

### What the Planner Hides from the User

A `CoveragePathPlanner` for sector search takes:

| User sees | Planner uses internally |
|---|---|
| Area polygon | Decomposed into parallel sweeps |
| Pattern: lawnmower | Sweep spacing = sensor footprint |
| (nothing) | Entry/exit waypoints per sweep leg |
| (nothing) | Turn radius at each end |

The user draws a polygon and picks a pattern label. The planner handles the geometry. If the terrain or sensor changes, the integrator swaps the planner; the mission config is untouched.

---

### Manual Override of Planner Output

Power users (the mission planner role, not the field operator) should be able to inspect and override what the planner generated before committing to a mission. The workflow:

1. Author intent: draw area, pick pattern
2. Planner generates: 23 waypoints shown on map
3. User reviews: adjusts entry point, removes one leg
4. Commit: edited task list saved as the mission

The edited result is Mode A (explicit parameters). The planner was used as a starting point, not as a locked-in algorithm. This gives non-technical users a fast path (just accept the planner output) and experienced operators a way to refine it.

---

## Build Order

Each item is a prerequisite for the ones below it.

**1. CommandBundle** (multi-actuator output + per-family monitor invariants)
Everything else that involves more than locomotion depends on this. Settle the schema first.

**2. BT Parallel composite**
Required for any task that combines locomotion with another actuator (camera, payload). `Sequence` and `Selector` are useful but less urgent: `Sequence` can be approximated with sequential tasks, and `Selector` is only needed when fallback logic lives within a task rather than at the mission transition level.

**3. SearchPatternNode**
Takes area + pattern, generates coverage route, runs it to completion. The most reusable node for SAR and any inspection mission.

**4. Blackboard**
Required the moment any task needs to consume output from a previous task. Small implementation, high leverage.

**5. Conditional transition rules + result codes**
Required for found/not-found branching. Depends on blackboard for result codes.

**6. Mission loop counter in transition rules**
Patrol and repeat-search patterns. Low complexity once transition rules are in place.

**7. JSON mission loader**
Required for Phase 2 experiments and for any non-trivial planner interface. Until then, missions are hardcoded in `main.cpp`.

---

## What This Design Intentionally Does Not Do

**Real-time mission editing mid-flight.** The planner authors a mission before deployment. Operators can pause, skip a step, or abort; they cannot restructure the task graph while the robot is running. Structural changes require a new mission load, which means either a restart or a clean hand-off point. This is a safety constraint, not a capability gap.

**Cross-vehicle coordination.** A multi-vehicle SAR where two rovers cover different sectors simultaneously is Tower's job, not SPAR's. Each vehicle runs its own `MissionExecutor`. Tower assigns missions; SPAR executes them. The boundary is clean.

**Optimal planning.** Per-behavior planners handle geometric coverage (lawnmower, spiral, orbit geometry). They do not solve optimal route planning over terrain models, dynamic obstacle fields, or probability maps. That level of planning sits above the planner layer and feeds it a pre-computed area or waypoint set. SPAR executes what it is given; it does not search for globally optimal plans.
