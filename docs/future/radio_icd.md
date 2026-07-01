# Radio ICD: SPAR ↔ Tower-Server

This document defines the interface between SPAR (on-vehicle) and Tower-Server (ground station). It covers transport, message types, data flow, threading model, coordination patterns, network-degraded operation, and implementation sequence.

---

## 1. Topology

```
Tower-Server (Go)
        │
        │  UDP multicast 239.255.0.1:14550  ← VehicleTelemetry, Heartbeat, Alert, CommandAck
        │  UDP multicast 239.255.0.2:14551  → ServerMessage (Command, ServerHeartbeat)
        │
   TowerRadioLink (C++, new)
        ├─ Telemetry sender thread    : reads WorldState, packages pidgin protobuf, sends at 4 Hz
        ├─ Heartbeat sender thread    : sends Heartbeat at 1 Hz
        ├─ Alert sender               : sends Alert messages from BT nodes on demand
        ├─ Command listener thread    : receives ServerMessage, writes GoalState + WorldState.mailbox
        └─ Heartbeat monitor          : detects ServerHeartbeat loss, triggers failsafe
        │
   GoalState (mutex-protected)              WorldState (per-domain mutexes)
        ├─ mode, target, command_id               ├─ pose      (own vehicle, from estimator)
        └─ failsafe_mode                          ├─ fleet     (other vehicles, from Tower)
                                                  └─ mailbox   (inbound events, from Tower)
        │
   MissionExecutor / main.cpp tick loop
        │
   BehaviorTree → RuntimeMonitor → Ros2Adapter
```

SPAR is **pull-based**. BT nodes never receive external messages. `TowerRadioLink` translates Tower intent into `GoalState` and `WorldState` domains that the tick loop and BT nodes read. The authority boundary is unchanged; no Tower message can bypass the `RuntimeMonitor`.

---

## 2. Protocol

Wire format is **pidgin protobuf** (same proto used by all vehicles in the Tower fleet).

Source: `api/proto/pidgin.proto` in the tower-server repo. Copy or symlink into SPAR at `proto/pidgin.proto` and add to CMakeLists.

| Direction       | Transport              | Address              | Message types                                          |
|-----------------|------------------------|----------------------|--------------------------------------------------------|
| Vehicle → Tower | UDP multicast (send)   | 239.255.0.1 : 14550  | `VehicleTelemetry`, `Heartbeat`, `Alert`, `CommandAck` |
| Tower → Vehicle | UDP multicast (listen) | 239.255.0.2 : 14551  | `Command` (core + extension), `ServerHeartbeat`        |

All messages are wrapped in the `VehicleMessage` / `ServerMessage` oneofs defined in pidgin.proto. SPAR sends `VehicleMessage`; SPAR receives `ServerMessage`.

**Why two separate multicast groups rather than one:**
Vehicle-originated and server-originated traffic are on different groups so each side only joins the groups it needs.

- `239.255.0.1:14550` is the **fleet state fabric**. Every vehicle sends telemetry here. Tower subscribes to relay it to the UI. Other vehicles can also subscribe to receive teammates' positions directly, same data, no Tower relay in the path. This is how `WorldState.fleet` gets populated: the vehicle listens on this group, parses incoming `VehicleTelemetry` for vehicle IDs that aren't its own, and writes their poses locally. The UI does the same thing through Tower's WebSocket feed. Both are subscribers to the same underlying stream; the multicast fabric is the single source of truth.
- `239.255.0.2:14551` is the **server-to-vehicle command channel**. Vehicles subscribe here to receive commands and heartbeats. Tower does not subscribe here, so it does not receive its own commands reflected back.

**Why `ServerHeartbeat` and `Command` share the same group (`239.255.0.2`):**
Both are `ServerMessage` oneofs in pidgin.proto, so they share the same envelope and socket. More importantly, co-locating them means heartbeat reception is a faithful proxy for command receivability, same socket, same network path. If the vehicle stops seeing heartbeats it knows commands are also not getting through, which is exactly the signal the failsafe logic needs. Separate channels would risk split-brain: heartbeats arriving while commands are silently dropped, making the failsafe trigger unreliable.

---

## 3. Messages SPAR Must Send

### 3.1 VehicleTelemetry (4 Hz, from telemetry sender thread)

Populated from `WorldState.pose` after each telemetry thread write. Send rate is independent of the 20 Hz BT tick.

```protobuf
VehicleTelemetry {
    vehicle_id        = <configured vehicle ID, e.g. "ugv-spar-01">
    timestamp_ms      = WorldState.pose.data.timestamp_us / 1000   // vehicle clock, untrusted by Tower
    location {                                   // via local_to_lla(datum, pose) — georeferenced only
        latitude       = geo.lat_deg
        longitude      = geo.lon_deg
        altitude_msl_m = geo.alt_m
    }
    speed_ms          = WorldState.pose.data.speed_ms
    heading_deg       = to_degrees(WorldState.pose.data.yaw_rad)
    sequence_num      = incrementing counter, never reset except on process restart
    environment       = ENV_GROUND
    // No extension namespace initially. supported_extensions and extensions
    // are populated in Phase 1.5 when structured SPAR state (monitor frames,
    // BT status) needs to appear in Tower's UI, or when inbound coordination
    // signals via ExtensionCommand are required.
}
```

**Do not send if pose is invalid** (`WorldState.pose.data.valid == false`). Always increment `sequence_num` even when content is unchanged; Tower's sequence tracker uses it for deduplication.

### 3.2 Heartbeat (1 Hz, from heartbeat sender thread)

Sent independently of telemetry. Advertises what this vehicle can accept. Tower will not show command buttons until a heartbeat is received.

```protobuf
Heartbeat {
    vehicle_id   = <vehicle_id>
    uptime_ms    = ms since TowerRadioLink::connect()
    vehicle_type = "spar-rover"
    capabilities {
        supported_commands = ["goto", "stop", "set_mode"]
        extensions         = []   // empty initially; add "monitor" namespace in Phase 1.5
                                  // for monitor telemetry, and "coordination" namespace in
                                  // Phase 2 when inbound ExtensionCommand signals are needed
    }
}
```

Capabilities can change at runtime, e.g., remove `"goto"` when battery is critical. Resend the updated heartbeat immediately when capabilities change; do not wait for the next 1 Hz tick.

### 3.3 CommandAck (on receipt of any Command)

Two acks per command:

| Timing                    | `AckStatus`  | Notes                                           |
|---------------------------|--------------|-------------------------------------------------|
| On receipt of command     | `ACCEPTED`   | Command parsed, written into GoalState/mailbox  |
| When BT returns `Success` | `COMPLETED`  | Task finished                                   |
| When BT returns `Failure` | `FAILED`     | Task failed (pose stale, timeout, etc.)         |
| On monitor `Halt`         | `FAILED`     | Monitor rejected command execution              |

```protobuf
CommandAck {
    command_id = <copied from Command.command_id>
    vehicle_id = <vehicle_id>
    status     = ACCEPTED | COMPLETED | FAILED
    message    = ""  // human-readable reason on FAILED
}
```

Tower's command tracker times out after 5 seconds without an ack. Always send `ACCEPTED` immediately to prevent synthetic timeout acks from confusing the UI.

### 3.4 Alert (on-demand, from any BT node via TowerRadioLink)

`Alert` is the **upward behavioral event channel**. It is not limited to safety errors; it is the correct mechanism for any event a BT node needs to publish to Tower or the fleet. Tower broadcasts all alerts to UI clients; Tower can also act on them to route coordination signals to other vehicles.

```protobuf
Alert {
    vehicle_id   = <vehicle_id>
    timestamp_ms = now_ms()
    severity     = SEVERITY_INFO | SEVERITY_WARNING | SEVERITY_ERROR | SEVERITY_CRITICAL
    code         = <machine-readable string>   // see table below
    message      = <human-readable description>
    location     = <where the event occurred, if applicable>
}
```

**Defined alert codes (extend as behaviors require):**

| `code`                   | `severity`        | Meaning                                                    |
|--------------------------|-------------------|------------------------------------------------------------|
| `"TARGET_DETECTED"`      | `INFO`            | BT node detected a target of interest at `location`        |
| `"COVERAGE_COMPLETE"`    | `INFO`            | Survey/coverage task finished; area at `location`          |
| `"HANDOFF_REQUEST"`      | `INFO`            | This vehicle requests another vehicle take over at `location` |
| `"CLEARANCE_REQUEST"`    | `WARNING`         | Vehicle is waiting for Tower approval before proceeding    |
| `"POSE_STALE"`           | `WARNING`         | Monitor rejected command due to stale EKF data             |
| `"MONITOR_HALT"`         | `ERROR`           | RuntimeMonitor issued a Halt; vehicle stopped              |
| `"TASK_FAILED"`          | `ERROR`           | BT returned Failure; reason in `message`                   |
| `"TOWER_LINK_LOST"`      | `CRITICAL`        | ServerHeartbeat timeout; entering failsafe                 |
| `"GEOFENCE_BREACH"`      | `CRITICAL`        | Vehicle position outside permitted boundary                |

BT nodes call `TowerRadioLink::send_alert()`. The call is non-blocking; the alert is enqueued and sent by a dedicated sender thread. Nodes do not block on radio delivery.

---

## 4. Messages SPAR Must Receive

### 4.1 ServerMessage: Command

Tower sends commands as `ServerMessage` containing a `Command`. The command listener thread deserializes and acts based on which `oneof payload` is set.

**Core commands → GoalState:**

| `oneof payload` | Required fields                          | Action                                               |
|-----------------|------------------------------------------|------------------------------------------------------|
| `goto`          | `destination {lat, lng, alt}`, speed     | Write `target`, set `mode = Navigate`                |
| `stop`          | none                                     | Set `mode = Stop`                                    |
| `set_mode`      | `mode` enum                              | Map to `BehaviorMode` (see table below)              |

`OperationalMode` → `BehaviorMode` mapping:

| Pidgin `OperationalMode` | `BehaviorMode`  |
|--------------------------|-----------------|
| `MODE_AUTONOMOUS`        | `Navigate`      |
| `MODE_SUPERVISED`        | `Hold`          |
| `MODE_MANUAL`            | `Stop` (SPAR has no manual path; ack ACCEPTED, execute Stop) |

**Extension commands → WorldState.mailbox:**

Commands with `oneof payload = extension` and `namespace = "coordination"` are **coordination signals**, not locomotion commands. They do not touch `GoalState`. Instead, they are written into `WorldState.mailbox` so BT nodes can read them on the next tick.

```protobuf
// Example: Tower grants clearance to proceed
Command {
    command_id = "cmd-xyz"
    vehicle_id = "ugv-spar-01"
    payload.extension = ExtensionCommand {
        namespace = "coordination"
        action    = "clearance_grant"
        version   = 1
        payload   = <CoordinationSignal proto bytes>
    }
}
```

`TowerRadioLink` sends `ACCEPTED` immediately on receipt, then writes the decoded signal into `WorldState.mailbox`. The BT node that issued the `CLEARANCE_REQUEST` alert reads from `WorldState.mailbox` on its next tick and proceeds. Acking `COMPLETED` is the node's responsibility, routed through `TowerRadioLink::on_task_complete()`.

Unknown `namespace` values: ack `FAILED`, log, do not modify any state.

Commands not addressed to this `vehicle_id` must be silently dropped; Tower multicasts to all vehicles on the group; each vehicle filters by ID.

### 4.2 ServerMessage: ServerHeartbeat

Tower broadcasts a `ServerHeartbeat` at 1 Hz to all vehicles on `239.255.0.2:14551`. SPAR uses it to detect Tower connectivity loss and trigger failsafe.

```protobuf
ServerHeartbeat {
    timestamp_ms      = server authoritative clock (display only)
    sequence_num      = monotonic counter; use to detect packet loss
    connected_clients = informational
    tracked_vehicles  = informational
}
```

**Heartbeat monitor behavior:**

| Elapsed since last heartbeat | Action                                                                     |
|------------------------------|----------------------------------------------------------------------------|
| < 5 s                        | Normal operation                                                           |
| ≥ 5 s                        | Send `Alert{code: "TOWER_LINK_LOST", severity: CRITICAL}`; write `GoalState.failsafe_mode` |
| Heartbeat resumes            | Send `Alert{code: "TOWER_LINK_RESTORED", severity: INFO}`; clear failsafe |

Failsafe mode is configured at deployment via `TowerConfig::failsafe_mode`. Default for ground vehicles is `Hold`. The tick loop reads `GoalState.failsafe_mode` and enforces it regardless of the in-flight command; Tower link loss overrides mission state.

---

## 5. GoalState: the Tower/BT Seam

`GoalState` is the thread-safe struct that `TowerRadioLink` writes and the tick loop reads. It replaces the hardcoded `GoalContext` initialization in `main.cpp`.

```cpp
enum class FailsafeMode : uint8_t { None = 0, Hold = 1, Stop = 2, ReturnHome = 3 };

struct GoalState {
    std::mutex    mu;
    BehaviorMode  mode          = BehaviorMode::Stop;
    Waypoint      target        = {};
    std::string   command_id   = "";
    FailsafeMode  failsafe_mode = FailsafeMode::None;  // set on Tower link loss
    uint64_t      updated_us   = 0;
};
```

**Tick loop usage:**

```cpp
GoalContext goal;
std::string cmd_id;
FailsafeMode fsm;
{
    std::lock_guard<std::mutex> lk(goal_state.mu);
    fsm    = goal_state.failsafe_mode;
    goal   = snapshot_goal(goal_state, tick_start);
    cmd_id = goal_state.command_id;
}

// Failsafe overrides mission mode when Tower link is lost
if (fsm != FailsafeMode::None)
    goal.mode = failsafe_to_behavior_mode(fsm);
```

---

## 6. WorldState Additions

Two new domains extend `WorldState` to support fleet awareness and inbound coordination signals. Both follow the existing per-domain mutex pattern.

### 6.1 Fleet Domain

```cpp
struct FleetPose {
    std::string vehicle_id;
    Pose        pose;           // same Pose struct as own-vehicle
    uint64_t    received_us;    // server timestamp of last update
};

// Added to WorldState:
struct { std::mutex mu; std::map<std::string, FleetPose> data; } fleet;
```

`TowerRadioLink`'s telemetry sender receives decoded fleet telemetry from other vehicles relayed by Tower (via a future subscription mechanism or by listening on the multicast group for telemetry not addressed to this vehicle) and writes into `fleet`. BT nodes that need to know where another vehicle is read `world.fleet["ugv-spar-02"].pose` exactly as they read `world.pose`.

### 6.2 Mailbox Domain

```cpp
struct MailboxEntry {
    std::string action;         // "clearance_grant", "coordination_signal", etc.
    std::string from_vehicle;   // empty if from Tower operator, set if relayed from another vehicle
    std::vector<uint8_t> payload;
    uint64_t    received_us;
    bool        consumed = false;
};

// Added to WorldState:
struct { std::mutex mu; std::deque<MailboxEntry> entries; } mailbox;
```

`TowerRadioLink` appends to `mailbox` when it receives an `ExtensionCommand{namespace: "coordination"}`. BT nodes pop entries they consume by marking `consumed = true`. `TowerRadioLink` periodically prunes consumed entries. The mailbox is bounded (max 16 entries); if full, oldest consumed entries are evicted first, then oldest unconsumed with a warning log.

BT nodes check the mailbox exactly as they check `WorldState.pose`: one lock, one read, no blocking:

```cpp
// Inside a BT node tick():
MailboxEntry signal;
bool found = false;
{
    std::lock_guard<std::mutex> lk(world.mailbox.mu);
    for (auto& e : world.mailbox.entries) {
        if (!e.consumed && e.action == "clearance_grant") {
            signal = e;
            e.consumed = true;
            found = true;
            break;
        }
    }
}
if (!found) return NodeStatus::Running;  // still waiting
// proceed
```

---

## 7. Multi-Robot Coordination Pattern

The protocol already provides all the channels needed for coordinated behaviors. BT nodes remain pure pull functions throughout: no messaging inside tick(), no change to the BTNode contract.

### Pattern: Event → Tower → Signal to Other Vehicle

```
Robot A (SPAR)                    Tower-Server              Robot B (SPAR)
─────────────────────────────    ─────────────────────    ─────────────────────────────
BT node detects target
  → TowerRadioLink.send_alert(
      code: "TARGET_DETECTED"
      location: {lat, lon}
    )
  → Alert sent via UDP multicast
                                 Tower receives Alert
                                 UI operator sees it
                                 Tower routes to Robot B:
                                   Command {
                                     vehicle_id: "ugv-spar-02"
                                     extension: {
                                       namespace: "coordination"
                                       action: "coordination_signal"
                                       payload: {target_lat, target_lon}
                                     }
                                   }
                                                            TowerRadioLink receives Command
                                                            Writes to WorldState.mailbox
                                                            BT node reads mailbox next tick
                                                            NavigateNode goal updated
                                                            → Moves to target location
```

### Pattern: Request-Wait-Proceed (Radio-Dependent Behavior)

Used when a BT node must not proceed until Tower or a human confirms it. The node remains fully stateless; it issues the request once, then polls the mailbox on every tick.

```
BT node tick() → Running          // check mailbox: not found yet, re-emit alert if needed
    ...
BT node tick() → Running          // still waiting
    ...
Tower operator approves
Tower sends ExtensionCommand{namespace: "coordination", action: "clearance_grant"}
TowerRadioLink writes to WorldState.mailbox
    ...
BT node tick() → Running          // mailbox hit: found clearance_grant, mark consumed
    proceed with next action
BT node tick() → Success
```

The node does not block a thread. The 20 Hz tick loop continues normally during the wait. Monitor sees zero-throttle `CommandStream` and holds position. When clearance arrives, behavior resumes on the next tick.

### Pattern: Fleet-Aware Navigation

BT nodes that need to know where teammates are read `WorldState.fleet`, no different from reading own pose:

```cpp
NodeStatus FormationNode::tick(const GoalContext& goal, const WorldState& world, CommandStream& out) {
    Pose own;
    FleetPose lead;
    {
        std::lock_guard lk(world.pose.mu);
        own = world.pose.data;
    }
    {
        std::lock_guard lk(world.fleet.mu);
        auto it = world.fleet.data.find(goal.lead_vehicle_id);
        if (it == world.fleet.data.end()) return NodeStatus::Failure;
        lead = it->second;
    }
    // compute offset position behind lead vehicle, navigate there
    ...
}
```

`WorldState.fleet` is updated by `TowerRadioLink` at Tower's relay rate. Formation nodes should check `lead.received_us` and treat stale fleet data (> 500 ms) as a failure condition, matching the same staleness discipline as own-vehicle pose.

---

## 8. Network Degraded Operation

SPAR is designed to operate safely when Tower connectivity degrades or drops entirely. The `TowerRadioLink` heartbeat monitor is the sole mechanism for detecting link loss; no other component needs to know about Tower connectivity.

### Link Loss Behavior

```
Normal operation:
  ServerHeartbeat received at ~1 Hz
  GoalState.failsafe_mode = None
  Tick loop executes current mission

After 5 s without ServerHeartbeat:
  TowerRadioLink writes GoalState.failsafe_mode = Hold (or configured mode)
  TowerRadioLink sends Alert{code: "TOWER_LINK_LOST", severity: CRITICAL}
  Tick loop reads failsafe_mode, overrides goal.mode = Hold
  BT node receives mode = Hold, outputs zero throttle/steering
  Monitor sees valid zero command, passes it, vehicle holds position

When Tower reconnects:
  ServerHeartbeat received
  TowerRadioLink clears GoalState.failsafe_mode = None
  TowerRadioLink sends Alert{code: "TOWER_LINK_RESTORED", severity: INFO}
  Tick loop resumes from last GoalState (mission re-engages from last known goal)
```

### What Continues During Link Loss

| Subsystem              | Behavior during Tower link loss         |
|------------------------|-----------------------------------------|
| Odometry ingest        | Continues normally; WorldState updated  |
| RuntimeMonitor         | Continues normally; all invariants active |
| Session log            | Continues writing to /tmp               |
| BT tick loop           | Continues at 20 Hz; holds position      |
| Mailbox                | Frozen; no new entries arrive           |
| Fleet state            | Frozen at last known poses              |

### Failsafe Modes

Configured per-deployment in `TowerConfig`:

| `FailsafeMode`  | `BehaviorMode` | Appropriate for                           |
|-----------------|----------------|-------------------------------------------|
| `Hold`          | `Hold`         | Ground vehicles (default)                 |
| `Stop`          | `Stop`         | Vehicles where Hold is not meaningful     |
| `ReturnHome`    | `Navigate`     | Vehicles with a known safe home location  |

`ReturnHome` requires `TowerConfig::home_waypoint` to be set at startup. If the vehicle receives a `ReturnHomeCommand` from Tower before link loss, the home waypoint is stored in `GoalState` and `TowerRadioLink` uses it as the failsafe target.

---

## 9. TowerRadioLink Interface

New component, lives at `spar_rover/tower/TowerRadioLink.h/.cpp`.

```cpp
struct TowerConfig {
    std::string  vehicle_id         = "ugv-spar-01";
    std::string  vehicle_type       = "spar-rover";
    std::string  tx_group           = "239.255.0.1";
    uint16_t     tx_port            = 14550;
    std::string  rx_group           = "239.255.0.2";
    uint16_t     rx_port            = 14551;
    FailsafeMode failsafe_mode      = FailsafeMode::Hold;
    Waypoint     home_waypoint      = {};                  // used if failsafe = ReturnHome
    uint32_t     heartbeat_timeout_ms = 5000;
};

class TowerRadioLink {
public:
    explicit TowerRadioLink(TowerConfig cfg, GoalState& goal_state, WorldState& world);

    bool connect();     // Opens sockets, starts threads
    void disconnect();  // Signals stop, joins threads

    // Called from tick loop on task outcome
    void on_task_complete(const std::string& command_id);
    void on_task_failed(const std::string& command_id, const std::string& reason);

    // Called from BT nodes or tick loop to publish behavioral events
    void send_alert(AlertSeverity severity, std::string code,
                    std::string message, std::optional<Location> location = {});

private:
    void telemetry_loop();    // 4 Hz: reads WorldState, sends VehicleTelemetry
    void heartbeat_loop();    // 1 Hz: sends Heartbeat
    void command_loop();      // blocking recvfrom: receives ServerMessage, dispatches
    void alert_loop();        // drains alert queue, sends Alert messages

    void handle_command(const Command& cmd);
    void handle_server_heartbeat(const ServerHeartbeat& hb);
    void send_ack(const std::string& command_id, AckStatus status, const std::string& msg = "");

    TowerConfig            cfg_;
    GoalState&             goal_state_;
    WorldState&            world_;
    std::atomic<bool>      running_{false};
    uint32_t               seq_{0};
    uint64_t               last_server_hb_us_{0};
    std::mutex             alert_mu_;
    std::deque<Alert>      alert_queue_;
    // sockets, threads
};
```

---

## 10. Threading Model

```
Thread                   Owns                                Rate
──────────────────────── ─────────────────────────────────── ────────────────────
main / tick loop         BT tick, monitor, adapter write     20 Hz
Odometry ingest          WorldState.pose write               estimator rate (~10 Hz)
Tower telemetry sender   Reads WorldState, sends UDP         4 Hz
Tower heartbeat sender   Sends Heartbeat UDP                 1 Hz
Tower command listener   Reads UDP, writes GoalState/mailbox event-driven (recvfrom)
Tower heartbeat monitor  Reads last_server_hb_us_           1 Hz check
Tower alert sender       Drains alert_queue_, sends UDP      event-driven
```

All threads share:
- `WorldState`: per-domain mutexes (pose existing; fleet + mailbox new)
- `GoalState`: single mutex
- `TowerRadioLink::alert_queue_`: mutex-protected deque

No thread reads or writes `CommandStream` or `MonitorDecision`; those stay inside the tick loop.

---

## 11. main.cpp Integration Sketch

```cpp
// Startup
GoalState   goal_state;
TowerRadioLink radio(tower_cfg, goal_state, world);
radio.connect();
adapter.connect();

// Tick loop
while (g_running.load()) {
    uint64_t tick_start = now_us();

    GoalContext  goal;
    std::string  cmd_id;
    FailsafeMode fsm;
    {
        std::lock_guard lk(goal_state.mu);
        fsm    = goal_state.failsafe_mode;
        goal   = snapshot_goal(goal_state, tick_start);
        cmd_id = goal_state.command_id;
    }
    if (fsm != FailsafeMode::None)
        goal.mode = failsafe_to_behavior_mode(fsm);

    CommandStream raw{};
    NodeStatus status = bt.tick(goal, world, raw);

    MonitorDecision decision = monitor.evaluate(raw, tick_start);

    if (decision.outcome != MonitorDecision::Outcome::Halt) {
        adapter.write(decision.output_cmd);
    } else if (!cmd_id.empty()) {
        radio.on_task_failed(cmd_id, "monitor_halt");
        radio.send_alert(SEVERITY_ERROR, "MONITOR_HALT",
                         decision.triggered_invariant, std::nullopt);
    }

    log_frame(log, frame_idx++, decision);

    if (status == NodeStatus::Success) {
        if (!cmd_id.empty()) radio.on_task_complete(cmd_id);
        break;
    }
    if (status == NodeStatus::Failure) {
        if (!cmd_id.empty()) radio.on_task_failed(cmd_id, "bt_failure");
        radio.send_alert(SEVERITY_ERROR, "TASK_FAILED", "bt returned failure", std::nullopt);
        break;
    }

    // sleep to 20 Hz
}

radio.disconnect();
```

---

## 12. Build Integration

Add to `CMakeLists.txt`:

```cmake
option(SPAR_ENABLE_TOWER "Enable Tower-Server radio interface" OFF)

if(SPAR_ENABLE_TOWER)
    find_package(Protobuf REQUIRED)
    protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
        proto/pidgin.proto
        proto/monitor_extension.proto  # MonitorTelemetry (Phase 1.5)
        proto/coordination.proto       # CoordinationSignal (Phase 2)
    )
    add_library(spar_tower
        spar_rover/tower/TowerRadioLink.cpp
        ${PROTO_SRCS}
    )
    target_link_libraries(spar_tower PUBLIC protobuf::libprotobuf)
    target_link_libraries(spar_rover PRIVATE spar_tower)
    target_compile_definitions(spar_rover PRIVATE SPAR_HAVE_TOWER=1)
endif()
```

Same pattern as `SPAR_ENABLE_ZENOH`; Tower is optional; SPAR runs standalone in simulation or hardware-only mode with no change to the BT or monitor.

---

## 13. Known Design Gaps

These are not implementation TODOs; they are open architectural problems that the current design cannot solve without structural changes. They are recorded here so that whoever hits them first knows they were anticipated.

### Gap 1: Precisely Synchronized Simultaneous Actions

Most inter-robot coordination works without a new transport. Vehicles subscribe to `239.255.0.1:14550` and receive teammates' `VehicleTelemetry` and `Alert` messages directly at LAN-local latency, the same multicast group they already send on. Position sharing populates `WorldState.fleet` at 4 Hz with no Tower relay. Behavioral signals (target handoffs, clearance requests, coordination triggers) flow peer-to-peer via `Alert` and land in `WorldState.mailbox` on the receiving vehicle. Humans adjust behaviors in real time via Tower's `ExtensionCommand` into the same mailbox. These cases are covered.

What is not covered is **precisely synchronized simultaneous action**: maneuvers where two robots must act within the same control window, not just react to each other's state. Examples: a coordinated gate crossing where both vehicles must commit at exactly the same moment, or a simultaneous multi-vehicle engagement where timing is load-bearing. At 4 Hz telemetry and with non-deterministic UDP delivery, there is no guarantee two robots read a trigger signal within the same 50 ms tick. For most ground robot behaviors this doesn't matter. For aerial or fast-moving synchronized maneuvers it might.

This is a narrow remaining gap. If precise synchronization is required, it needs a dedicated mechanism: a shared clock source, a synchronized trigger signal, or a purpose-built coordination protocol. That is out of scope for the current design and should be designed when a concrete behavior requires it.

> [!IMPORTANT]
> `WorldState.fleet` must not be designed as Tower-only. The struct and the domain mutex are correct; the write path must remain open to a second transport. **Do not couple `WorldState.fleet` writes to `TowerRadioLink` internals.**

---

### Gap 2: Human Intervention Inside Running Behaviors

The current model treats humans as top-level command issuers: `goto`, `stop`, `set_mode`. These replace whatever the vehicle is doing. For complex behaviors, a human needs to reach *inside* a running behavior, not abort and restart it.

Concrete cases that break:
- A survey node detects something and pauses, presenting the human with a binary choice: investigate or continue. Neither option is a new `goto`.
- A behavior reaches a multi-path fork and requires a human to choose before proceeding. The choice has structured parameters (which path, at what speed, with what constraints).
- A behavior is executing correctly but the human wants to modify an in-flight parameter (tighter radius, slower approach) without aborting the mission.

The `Alert` + mailbox mechanism handles the *transport* of this correctly; a human can send `ExtensionCommand{namespace: "coordination", action: "choose_branch"}` and a BT node reads it from `WorldState.mailbox` on the next tick. What doesn't exist is the **protocol for expressing the question upward**. There is no structured way for a BT node to say "I need a human decision between these specific options with this timeout before I can proceed." `Alert.message` is a human-readable string, not a machine-readable decision request.

**What the design needs:** A structured decision-request message type in the upward channel (either an extension to `Alert` or a new `VehicleMessage` payload) that carries the options, the context, and the timeout behavior if no human responds. Tower's UI needs a corresponding panel to present the decision and route the human's answer back as a coordination signal. This is a non-trivial addition to both the protocol and the Tower UI.

> [!IMPORTANT]
> The `Alert` message type must not be extended ad-hoc to carry structured decision data. When this is designed, it should be a first-class message type or a well-defined extension namespace, not a hack on top of severity + code + message. **Do not work around this gap by encoding structured data in `Alert.message`.**

---

## 14. What Is Not Wired Through This ICD

| Concern                        | Where it lives                | Why not here                                                  |
|--------------------------------|-------------------------------|---------------------------------------------------------------|
| Per-tick command to BT nodes   | Not applicable                | Nodes are stateless; read WorldState + GoalContext            |
| Monitor invariant config       | Static `MonitorConfig`        | Invariants are deployment-time, not mission-time              |
| Geofence enforcement           | BTNode or temporal monitor    | Mission-level concern; monitor checks spatial bounds in Phase 3 |
| Inter-task blackboard          | In-process MissionExecutor    | No external write path into within-mission task sequencing    |
| ONNX model distribution        | File transfer (Phase 3)       | Not a real-time command; separate delivery mechanism          |
| Behavioral monitor fallback    | In-process (Phase 3)          | Behavioral monitor calls MissionExecutor directly             |
| Peer-to-peer vehicle comms     | See Gap 1 (Section 13)        | Architectural open problem; not a deferred feature             |
| Human mid-behavior intervention | See Gap 2 (Section 13)       | Requires new protocol design; do not work around with Alert.message |

---

## 14. Implementation Sequence

1. **Copy pidgin.proto** into `proto/`. Add to CMakeLists behind `SPAR_ENABLE_TOWER`.

2. **Add `WorldState.fleet` and `WorldState.mailbox` domains** following the existing per-domain mutex pattern.

3. **Introduce `GoalState`** with `failsafe_mode` field. Replace hardcoded goal initialization in `main.cpp`.

4. **Implement `TowerRadioLink`** in this order:
   - Telemetry sender + heartbeat sender (Tower can see the vehicle)
   - Command listener + ack sender (Tower can control the vehicle)
   - Alert sender (vehicle can publish events)
   - Heartbeat monitor + failsafe write (vehicle survives link loss)

5. **Wire tick loop**: failsafe override, `on_task_complete` / `on_task_failed`, monitor halt alert.

6. **Smoke test** with Tower's `testsender` to confirm telemetry appears in UI, then `testclient` to send `goto` and confirm SPAR acks and navigates. Kill Tower process and verify vehicle holds position and sends `TOWER_LINK_LOST` alert on reconnect.

7. **Phase 1.5: `monitor` extension namespace**: Write `proto/monitor_extension.proto` defining `MonitorTelemetry` (recent `MonitorDecision` frames). Emit in each telemetry packet. Register `monitor` codec in tower-server. UI shows monitor outcome alongside position telemetry.

8. **Phase 2: Coordination**: Write `proto/coordination.proto` defining `CoordinationSignal`. Register `coordination` namespace in Heartbeat capabilities. Implement `WorldState.mailbox` consumption in the first BT node that needs it. Test full Alert → Tower → ExtensionCommand → mailbox → BT node loop.
