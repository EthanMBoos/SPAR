#include "monitor/RuntimeMonitor.h"
#include "adapter/ArdupilotAdapter.h"
#include "mission/BehaviorTree.h"
#include "mission/NavigateNode.h"
#include "../shared/assembler/ObservationAssembler.h"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <atomic>
#include <unistd.h>

static std::atomic<bool> g_running{true};
static void handle_signal(int) { g_running.store(false); }

static uint64_t now_us() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

static void log_frame(std::ofstream& log,
                      uint64_t       frame_idx,
                      const MonitorDecision& d,
                      const SourceStatus&    pose_status)
{
    const char* outcome = d.outcome == MonitorDecision::Outcome::Passed   ? "PASS"
                        : d.outcome == MonitorDecision::Outcome::Fallback ? "FALL"
                                                                          : "HALT";
    log << frame_idx
        << '\t' << d.timestamp_us
        << '\t' << outcome
        << '\t' << d.triggered_invariant
        << '\t' << d.output_cmd.cmd.throttle
        << '\t' << d.output_cmd.cmd.steering
        << '\t' << pose_status.age_us       // per-source staleness now in every log line
        << '\t' << pose_status.degraded
        << '\n';
}

int main() {
    std::signal(SIGINT,  handle_signal);
    std::signal(SIGTERM, handle_signal);

    ObservationAssembler assembler;

    BehaviorTree bt;
    bt.add_node(std::make_unique<NavigateNode>());

    RuntimeMonitor monitor;

    ArdupilotAdapter adapter(assembler);
    if (!adapter.connect())
        std::fprintf(stderr, "[spar_rover] ArduPilot not reachable — stub mode\n");

    const uint64_t session_id = now_us();
    std::ofstream  log("/tmp/spar_" + std::to_string(session_id) + ".log");
    log << "frame\ttimestamp_us\toutcome\ttriggered\tthrottle\tsteering\tpose_age_us\tpose_degraded\n";

    uint64_t frame_idx = 0;
    constexpr uint64_t TICK_US = 50'000;  // 20 Hz

    GoalContext goal;
    goal.mission_id = 1;
    goal.mode       = BehaviorMode::Navigate;
    goal.target     = {33.7756, -84.3963, 0.0f};
    // TODO: load goal from a mission file or receive it from Tower over pidgin

    std::fprintf(stdout, "[spar_rover] session=%llu\n", (unsigned long long)session_id);

    while (g_running.load()) {
        uint64_t tick_start = now_us();
        goal.timestamp_us   = tick_start;

        // Step 1: build a time-coherent WorldState snapshot from the ring buffers.
        AssembledSnapshot snap = assembler.build(tick_start);

        // Step 2: if any required source is degraded, skip the mission layer and
        // command the trusted fallback directly.
        if (snap.any_degraded) {
            // TODO: invoke the per-task hand-coded fallback (decelerate-and-hold or RTL)
            // instead of a bare zero command. The fallback is a BTNode that satisfies
            // the same interface — wire it here once it exists.
            CommandStream safe{};
            MonitorDecision decision = monitor.evaluate(safe, tick_start);
            adapter.write(decision.output_cmd);
            log_frame(log, frame_idx++, decision, snap.pose_status);

            uint64_t elapsed = now_us() - tick_start;
            if (elapsed < TICK_US)
                ::usleep(static_cast<useconds_t>(TICK_US - elapsed));
            continue;
        }

        // Step 3: tick the active behavior node with the assembled snapshot.
        CommandStream raw{};
        NodeStatus status = bt.tick(goal, snap.state, raw);

        // Step 4: monitor enforces structural invariants before the adapter sees the command.
        MonitorDecision decision = monitor.evaluate(raw, tick_start);

        // Step 5: approved commands only reach the controller.
        if (decision.outcome != MonitorDecision::Outcome::Halt)
            adapter.write(decision.output_cmd);

        // Step 6: log frame including per-source staleness.
        log_frame(log, frame_idx++, decision, snap.pose_status);

        if (status == NodeStatus::Success) {
            std::fprintf(stdout, "[spar_rover] mission complete\n");
            break;
        }

        uint64_t elapsed = now_us() - tick_start;
        if (elapsed < TICK_US)
            ::usleep(static_cast<useconds_t>(TICK_US - elapsed));
    }

    adapter.disconnect();
    return 0;
}
