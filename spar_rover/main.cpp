#include "monitor/RuntimeMonitor.h"
#include "mission/NavigateNode.h"
#include "../shared/assembler/ObservationAssembler.h"

#ifdef SPAR_HAVE_EXPLOIT_NODE
#include "mission/ExploitNode.h"
#endif

#ifdef SPAR_HAVE_ONNXRUNTIME
#include "mission/OnnxNavigateNode.h"
#endif

#ifdef SPAR_BACKEND_KINEMATIC
#include "sim/KinematicBackend.h"
#include "sim/DegradationScenarios.h"
#else
#include "adapter/ArdupilotAdapter.h"
#endif

#include <chrono>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <atomic>
#include <stdexcept>
#include <unistd.h>

static std::atomic<bool> g_running{true};
static void handle_signal(int) { g_running.store(false); }

static uint64_t now_us() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

// policy_id: written into the session log so catch fractions are attributable.
// For exploit builds the node is selected at runtime based on SPAR_SCENARIO,
// so we use a runtime string rather than a compile-time constant.
#ifdef SPAR_HAVE_ONNXRUNTIME
static constexpr const char* kPolicyIdBase = "onnx";
#else
static constexpr const char* kPolicyIdBase = "navigate";
#endif

static void log_frame(std::ofstream& log,
                      uint64_t       frame_idx,
                      const MonitorDecision& d,
                      const SourceStatus&    pose_status,
                      const char*            policy_id,
                      const char*            mission_outcome = "running")
{
    const char* outcome = d.outcome == MonitorDecision::Outcome::Passed   ? "PASS"
                        : d.outcome == MonitorDecision::Outcome::Fallback ? "FALL"
                                                                          : "HALT";
    log << frame_idx
        << '\t' << d.timestamp_us
        << '\t' << policy_id
        << '\t' << 0                         // task_id: hardcoded 0 until MissionExecutor (Phase 2)
        << '\t' << outcome
        << '\t' << d.triggered_invariant
        << '\t' << static_cast<unsigned>(d.invariant_flags)
        << '\t' << d.output_cmd.cmd.throttle
        << '\t' << d.output_cmd.cmd.steering
        << '\t' << pose_status.age_us
        << '\t' << pose_status.degraded
        << '\t' << mission_outcome
        << '\n';
}

int main() {
    std::signal(SIGINT,  handle_signal);
    std::signal(SIGTERM, handle_signal);

    const char* scenario = std::getenv("SPAR_SCENARIO");
    if (!scenario) scenario = "baseline";

    ObservationAssembler assembler;
    NavigateNode nav_node;
    RuntimeMonitor monitor;

#ifdef SPAR_HAVE_ONNXRUNTIME
    const char* onnx_model_path = std::getenv("SPAR_ONNX_MODEL");
    if (!onnx_model_path)
        throw std::runtime_error("SPAR_ONNX_MODEL env var required when built with ONNX support");
    OnnxNavigateNode onnx_node(onnx_model_path);
    BTNode* base_node = &onnx_node;
#else
    BTNode* base_node = &nav_node;
#endif

#ifdef SPAR_HAVE_EXPLOIT_NODE
    ExploitNode exploit_node;
    const bool use_exploit = (std::string_view(scenario) == "policy_exploit");
    BTNode* active_node  = use_exploit ? static_cast<BTNode*>(&exploit_node)
                                       : base_node;
    const char* policy_id = use_exploit ? "exploit" : kPolicyIdBase;
#else
    BTNode* active_node  = base_node;
    const char* policy_id = kPolicyIdBase;
#endif

#ifdef SPAR_BACKEND_KINEMATIC
    KinematicBackend backend(assembler, {}, degradation_for_scenario(scenario));
    backend.reset();
    std::fprintf(stdout, "[spar_rover] backend=kinematic scenario=%s policy=%s\n",
                 scenario, policy_id);
#else
    ArdupilotAdapter adapter(assembler);
    if (!adapter.connect())
        std::fprintf(stderr, "[spar_rover] ArduPilot not reachable — stub mode\n");
#endif

    const uint64_t session_id = now_us();
    std::ofstream  log("/tmp/spar_" + std::to_string(session_id) + ".log");
    log << "# scenario: " << scenario << '\n';
    log << "frame\ttimestamp_us\tpolicy_id\ttask_id\toutcome\ttriggered_invariant\tinvariant_flags\tthrottle\tsteering\tpose_age_us\tpose_degraded\tmission_outcome\n";

    uint64_t frame_idx = 0;
    constexpr uint64_t TICK_US = 50'000;  // 20 Hz

    GoalContext goal;
    goal.mission_id = 1;
    goal.target     = {33.7756, -84.3959, 0.0f};  // ~37 m east of start — reachable in ~33 s at throttle 0.6
    // TODO: load goal from a mission file or receive it from Tower over pidgin

    std::fprintf(stdout, "[spar_rover] session=%llu\n", (unsigned long long)session_id);

    while (g_running.load()) {
        uint64_t tick_start = now_us();
        goal.timestamp_us   = tick_start;

        // Step 1: build a time-coherent WorldState snapshot from the ring buffers.
        // In kinematic mode the sim clock and wall clock share the same epoch (both start
        // from wall_now_us() at reset), but usleep imprecision means the sim timestamp of
        // the last injected pose may be marginally ahead of now_us().  Use the sim clock as
        // the reference so latest_at_or_before() reliably finds the most recent pose.
#ifdef SPAR_BACKEND_KINEMATIC
        AssembledSnapshot snap = assembler.build(backend.sim_time_us());
#else
        AssembledSnapshot snap = assembler.build(tick_start);
#endif

        // Step 2: if any required source is degraded, skip the mission layer and
        // command the trusted fallback directly.
        // The assembler's staleness gate IS the catch — log it as FALL so the
        // session log captures the correct catch fraction for degraded scenarios.
        if (snap.any_degraded) {
            // TODO: invoke the per-task hand-coded fallback (decelerate-and-hold or RTL)
            // instead of a bare zero command. The fallback is a BTNode that satisfies
            // the same interface — wire it here once it exists.
            CommandStream safe{};
            MonitorDecision decision;
            decision.outcome             = MonitorDecision::Outcome::Fallback;
            decision.triggered_invariant = "staleness.pose_age";
            decision.invariant_flags     = MonitorDecision::kFlagStalenessPose;
            decision.output_cmd          = safe;
            decision.timestamp_us        = tick_start;
#ifdef SPAR_BACKEND_KINEMATIC
            backend.step(safe);
#else
            adapter.write(safe);
#endif
            log_frame(log, frame_idx++, decision, snap.pose_status, policy_id);

            uint64_t elapsed = now_us() - tick_start;
            if (elapsed < TICK_US)
                ::usleep(static_cast<useconds_t>(TICK_US - elapsed));
            continue;
        }

        // Step 3: tick the active behavior node with the assembled snapshot.
        CommandStream raw{};
        NodeStatus status = active_node->tick(goal, snap.state, raw);

        // Step 4: monitor enforces structural invariants before the adapter sees the command.
        MonitorDecision decision = monitor.evaluate(raw, tick_start);

        // Step 5: approved commands reach the controller / advance the sim.
#ifdef SPAR_BACKEND_KINEMATIC
        backend.step(decision.output_cmd);
#else
        if (decision.outcome != MonitorDecision::Outcome::Halt)
            adapter.write(decision.output_cmd);
#endif

        // Step 6: log frame including per-source staleness and mission outcome.
        // mission_outcome on the terminal frame tells the eval harness the oracle result
        // needed to compute the "neither" catch-fraction cell.
        const char* mission_outcome = (status == NodeStatus::Success) ? "success"
                                    : (status == NodeStatus::Failure)  ? "failure"
                                                                       : "running";
        log_frame(log, frame_idx++, decision, snap.pose_status, policy_id, mission_outcome);

        if (status == NodeStatus::Success) {
            std::fprintf(stdout, "[spar_rover] mission complete\n");
            break;
        }
        if (status == NodeStatus::Failure) {
            std::fprintf(stdout, "[spar_rover] mission failed\n");
            break;
        }

        uint64_t elapsed = now_us() - tick_start;
        if (elapsed < TICK_US)
            ::usleep(static_cast<useconds_t>(TICK_US - elapsed));
    }

#ifndef SPAR_BACKEND_KINEMATIC
    adapter.disconnect();
#endif
    return 0;
}
