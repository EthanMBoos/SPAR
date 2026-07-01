#pragma once
#ifdef SPAR_HAVE_ONNXRUNTIME

#include "../../shared/contracts/BTNode.h"
#include <string>
#include <vector>
#include <onnxruntime_cxx_api.h>

// Learned navigate-to-waypoint node backed by an ONNX actor.
// Satisfies the same BTNode contract as NavigateNode — the monitor sees identical
// CommandStream regardless of which node produced it.
//
// Actor input:  make_nav_obs(world, goal) (3 floats: gx_body, gy_body, speed_ms)
// Actor output: [throttle, steering] in [-1, 1]
//
// Returns Running each tick (stateless reactive policy); Success/Failure come from
// the assembler's degraded verdict routing upstream in main.cpp.
class OnnxNavigateNode final : public BTNode {
public:
    explicit OnnxNavigateNode(const std::string& model_path);

    NodeStatus  tick(const GoalContext& goal,
                     const WorldState&  world,
                     CommandStream&     out_cmd) override;

    void        reset()    override { start_us_ = 0; }
    const char* name()     const override { return "OnnxNavigateNode"; }
    uint64_t    node_id()  const override { return id_; }

private:
    static constexpr float    kArrivalRadiusM = 1.0f;
    static constexpr uint64_t kTimeoutUs      = 60'000'000;  // 60 s

    Ort::Env          env_;
    Ort::Session      session_;
    std::string       input_name_;
    std::string       output_name_;
    uint64_t          id_;
    uint64_t          start_us_ = 0;

    inline static uint64_t next_id_ = 1000;
};

#endif // SPAR_HAVE_ONNXRUNTIME
