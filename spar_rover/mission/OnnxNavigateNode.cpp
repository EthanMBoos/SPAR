#ifdef SPAR_HAVE_ONNXRUNTIME
#include "OnnxNavigateNode.h"
#include "../../shared/contracts/NavigationObs.h"
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <array>

static uint64_t now_us_onnx() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

OnnxNavigateNode::OnnxNavigateNode(const std::string& model_path)
    : env_(ORT_LOGGING_LEVEL_WARNING, "spar_onnx")
    , session_(env_, model_path.c_str(), Ort::SessionOptions{})
    , id_(next_id_++)
{
    Ort::AllocatorWithDefaultOptions alloc;
    // Expect exactly one input and one output.
    if (session_.GetInputCount() == 0 || session_.GetOutputCount() == 0)
        throw std::runtime_error("OnnxNavigateNode: model has no inputs or outputs");

    input_name_  = session_.GetInputNameAllocated(0, alloc).get();
    output_name_ = session_.GetOutputNameAllocated(0, alloc).get();
}

NodeStatus OnnxNavigateNode::tick(const GoalContext& goal,
                                  const WorldState&  world,
                                  CommandStream&     out_cmd)
{
    std::vector<float> obs = make_nav_obs(world, goal);

    std::array<int64_t, 2> input_shape{1, static_cast<int64_t>(obs.size())};
    Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        mem, obs.data(), obs.size(), input_shape.data(), input_shape.size());

    const char* input_names[]  = {input_name_.c_str()};
    const char* output_names[] = {output_name_.c_str()};

    auto outputs = session_.Run(Ort::RunOptions{nullptr},
                                input_names, &input_tensor, 1,
                                output_names, 1);

    const float* out_data = outputs[0].GetTensorData<float>();
    out_cmd.cmd.throttle     = std::max(-1.0f, std::min(1.0f, out_data[0]));
    out_cmd.cmd.steering     = std::max(-1.0f, std::min(1.0f, out_data[1]));
    out_cmd.cmd.timestamp_us = now_us_onnx();
    out_cmd.source_id        = id_;

    if (start_us_ == 0) start_us_ = goal.timestamp_us;
    if (goal.timestamp_us - start_us_ > kTimeoutUs) return NodeStatus::Failure;

    float dist = std::hypot(goal.target.x_m - world.pose.x_m,
                            goal.target.y_m - world.pose.y_m);
    if (dist < kArrivalRadiusM) return NodeStatus::Success;

    return NodeStatus::Running;
}

#endif // SPAR_HAVE_ONNXRUNTIME
