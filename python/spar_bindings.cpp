// pybind11 bindings for the SPAR C++ core.
// Exposes SparCore: a gym-compatible step/reset interface backed by KinematicBackend.
// The RuntimeMonitor is NOT part of the training loop — it runs at deployment only
// (spar_rover/main.cpp). Catch fractions are measured from deployment session logs.
//
// Build with: cmake -B build -DSPAR_BACKEND=kinematic -DSPAR_ENABLE_PYTHON=ON
//             cmake --build build --target spar_bindings

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "assembler/ObservationAssembler.h"
#include "contracts/GoalContext.h"
#include "contracts/CommandStream.h"
#include "contracts/NavigationObs.h"

#include "../spar_rover/sim/KinematicBackend.h"

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace py = pybind11;

// ── Haversine distance (duplicated from NavigateNode; keep bindings self-contained) ──
static constexpr double kR = 6371000.0;  // Earth radius, metres
static constexpr double kPiB = 3.14159265358979323846;
static float haversine_m(double lat1, double lon1, double lat2, double lon2) {
    auto rad = [](double d) { return d * kPiB / 180.0; };
    double dlat = rad(lat2 - lat1), dlon = rad(lon2 - lon1);
    double a = std::sin(dlat / 2) * std::sin(dlat / 2)
             + std::cos(rad(lat1)) * std::cos(rad(lat2))
             * std::sin(dlon / 2) * std::sin(dlon / 2);
    return static_cast<float>(2.0 * kR * std::asin(std::sqrt(a)));
}

// ── SparCore ─────────────────────────────────────────────────────────────────

struct StepResult {
    std::vector<float> obs;
    float              reward    = 0.0f;
    bool               done      = false;
    bool               truncated = false;
};

class SparCore {
public:
    static constexpr uint32_t kMaxSteps       = 1200;  // 60 s at 20 Hz
    static constexpr float    kArrivalRadiusM = 1.0f;

    explicit SparCore(KinematicParams   kparams     = {},
                      GoalContext       goal        = {},
                      DegradationParams degradation = {})
        : backend_(assembler_, kparams, degradation)
        , goal_(goal)
        , kparams_(kparams)
    {
        if (goal_.mission_id == 0) {
            goal_.mission_id = 1;
            goal_.target.lat_deg = kparams_.start_lat_deg;
            goal_.target.lon_deg = kparams_.start_lon_deg + 0.001;  // ~100 m east
        }
    }

    std::vector<float> reset() {
        backend_.reset();
        step_count_ = 0;

        // Advance one tick with a zero command so the assembler has an initial sample.
        CommandStream zero{};
        zero.cmd.timestamp_us = 1;
        backend_.step(zero);

        AssembledSnapshot snap = assembler_.build(backend_.sim_time_us());
        return make_nav_obs(snap.state, goal_);
    }

    StepResult step(float throttle, float steering) {
        ++step_count_;
        uint64_t t = backend_.sim_time_us();

        CommandStream cmd{};
        cmd.cmd.throttle     = throttle;
        cmd.cmd.steering     = steering;
        cmd.cmd.timestamp_us = t;

        // Pass command directly to the sim — no monitor in the training loop.
        backend_.step(cmd);

        uint64_t t_next        = backend_.sim_time_us();
        AssembledSnapshot snap = assembler_.build(t_next);
        std::vector<float> obs = make_nav_obs(snap.state, goal_);

        float dist = haversine_m(snap.state.pose.lat_deg, snap.state.pose.lon_deg,
                                 goal_.target.lat_deg,    goal_.target.lon_deg);

        StepResult r;
        r.obs       = obs;
        r.reward    = -dist;
        r.done      = dist < kArrivalRadiusM;
        r.truncated = (step_count_ >= kMaxSteps) && !r.done;
        return r;
    }

    const GoalContext& goal() const { return goal_; }

private:
    ObservationAssembler assembler_;
    KinematicBackend     backend_;
    GoalContext          goal_;
    KinematicParams      kparams_;
    uint32_t             step_count_ = 0;
};

// ── pybind11 module ───────────────────────────────────────────────────────────

PYBIND11_MODULE(spar_bindings, m) {
    m.doc() = "SPAR C++ core bindings — KinematicBackend gym interface (monitor runs at deployment only)";

    py::class_<KinematicParams>(m, "KinematicParams")
        .def(py::init<>())
        .def_readwrite("wheelbase_m",       &KinematicParams::wheelbase_m)
        .def_readwrite("max_speed_ms",      &KinematicParams::max_speed_ms)
        .def_readwrite("max_accel_ms2",     &KinematicParams::max_accel_ms2)
        .def_readwrite("max_steer_rate_rs", &KinematicParams::max_steer_rate_rs)
        .def_readwrite("dt_s",              &KinematicParams::dt_s)
        .def_readwrite("start_lat_deg",     &KinematicParams::start_lat_deg)
        .def_readwrite("start_lon_deg",     &KinematicParams::start_lon_deg)
        .def_readwrite("start_heading_deg", &KinematicParams::start_heading_deg);

    py::class_<Waypoint>(m, "Waypoint")
        .def(py::init<>())
        .def_readwrite("lat_deg", &Waypoint::lat_deg)
        .def_readwrite("lon_deg", &Waypoint::lon_deg)
        .def_readwrite("alt_m",   &Waypoint::alt_m);

    py::class_<GoalContext>(m, "GoalContext")
        .def(py::init<>())
        .def_readwrite("mission_id",   &GoalContext::mission_id)
        .def_readwrite("timestamp_us", &GoalContext::timestamp_us)
        .def_readwrite("target",       &GoalContext::target);

    py::class_<DegradationParams>(m, "DegradationParams")
        .def(py::init<>())
        .def_readwrite("mean_delay_us", &DegradationParams::mean_delay_us)
        .def_readwrite("jitter_stddev", &DegradationParams::jitter_stddev)
        .def_readwrite("dropout_rate",  &DegradationParams::dropout_rate);

    py::class_<StepResult>(m, "StepResult")
        .def_readonly("obs",       &StepResult::obs)
        .def_readonly("reward",    &StepResult::reward)
        .def_readonly("done",      &StepResult::done)
        .def_readonly("truncated", &StepResult::truncated);

    py::class_<SparCore>(m, "SparCore")
        .def(py::init<KinematicParams, GoalContext, DegradationParams>(),
             py::arg("kparams")     = KinematicParams{},
             py::arg("goal")        = GoalContext{},
             py::arg("degradation") = DegradationParams{})
        .def("reset", &SparCore::reset,
             "Reset the environment and return the initial observation vector.")
        .def("step", &SparCore::step,
             py::arg("throttle"), py::arg("steering"),
             "Advance one tick. Returns a StepResult with obs, reward, done, truncated.")
        .def_property_readonly("goal", &SparCore::goal);
}
