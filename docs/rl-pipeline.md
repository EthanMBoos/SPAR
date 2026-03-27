# RL Pipeline

## Design principle

The policy always trains inside the same assembler → BTNode → monitor loop it deploys into. What changes between training and deployment is exactly one thing: the **SimulatorBackend** that provides world state and executes commands. Everything above that seam is identical in both modes.

---

## The SimulatorBackend abstraction

SPAR has two ArduPilot-specific seams. A `SimulatorBackend` replaces both simultaneously:

- **Input side** — instead of MAVLink telemetry and Zenoh sensor feeds pushing into the assembler, the backend pushes simulator state after each step
- **Output side** — instead of `ArduPilotAdapter` sending MAVLink commands, the backend receives the approved command and steps the world forward

```
training mode:
  SimulatorBackend → assembler.push_*()
  assembler.build(t) → WorldState → BTNode → CommandStream → monitor → approved command
  SimulatorBackend.step(approved_command) → (loop)

deployment mode:
  MAVLink telemetry + Zenoh/rmw_zenoh → assembler.push_*()
  assembler.build(t) → WorldState → BTNode → CommandStream → monitor → approved command
  ArduPilotAdapter → MAVLink → controller
```

The assembler, BTNode contract, and monitor are not aware of which mode is active.

---

## Build modes

Training mode is a compile-time configuration. Turn off the hardware transports and select a backend:

```bash
# training — no MAVLink, no Zenoh, kinematic physics
cmake -B build \
  -DSPAR_ENABLE_MAVLINK=OFF \
  -DSPAR_ENABLE_ZENOH=OFF \
  -DSPAR_BACKEND=kinematic

# deployment — full hardware stack
cmake -B build \
  -DSPAR_ENABLE_MAVLINK=ON \
  -DSPAR_ENABLE_ZENOH=ON \
  -DSPAR_BACKEND=ardupilot
```

---

## The Gymnasium wrapper

Training happens through a standard `gym.Env` that maps one `step()` call to exactly one SPAR tick. The policy never sees SPAR internals — it sees an observation vector and produces an action.

```
gym.reset()
  → SimulatorBackend.reset()
  → assembler.build(t)
  → WorldState.to_observation_vector()

gym.step(action)
  → monitor.check(action_as_command)   # monitor runs during training
  → SimulatorBackend.step(approved)
  → assembler.push_*(new_state)
  → assembler.build(t + dt)
  → (next_obs, reward, done, info)
```

The wrapper is implemented via pybind11 bindings over the SPAR C++ core. For the kinematic backend the bindings are thin — the entire physics step is a few lines of C++.

---

## The fidelity ladder

Each rung is a backend swap. The assembler, monitor, and BTNode contract do not change.

| Backend | What it provides to WorldState | Policy architecture | Notes |
|---|---|---|---|
| `KinematicBackend` | pose, heading, speed, staleness | SAC (stable-baselines3) | In-process C++, very fast iteration |
| `CarlaBackend` | pose + camera + lidar + obstacles | DreamerV3 | CARLA Python API, realistic physics |
| `IsaacBackend` | GPU-parallelized, any of the above | DreamerV3 or parallel SAC | Many envs simultaneously |

WorldState complexity grows with the backend — that is what drives policy architecture choice, not an arbitrary preference. SAC is appropriate for pose-only because it is a simple continuous control problem. DreamerV3 is appropriate when WorldState includes images or obstacle maps because the policy must reason over high-dimensional, temporally-delayed inputs; a world model is justified by the observation space, not by complexity preference.

The `ObservationBuffer` (sliding window of raw samples) is the bridge between WorldState and sequence-input policies. It belongs in `shared/contracts/` so both the training wrapper and the deployed `OnnxNavigateNode` use the same windowing logic.

---

## The monitor runs during training

This is not incidental — it is load-bearing. During training, commands the monitor rejects do not get executed. The policy learns within the same command envelope it will face at deployment. Catch fractions are measurable during training, not only at evaluation time.

The four-way failure partition (architectural-only, behavioral-only, both, neither) can be estimated across training runs at each fidelity level. A policy that produces many monitor rejections during training is telling you something about the distribution mismatch before you ever touch hardware.

---

## Transport degradation

A clean kinematic backend produces training results that describe a world that does not exist — observations arrive perfectly fresh and synchronous, a condition that does not survive contact with real hardware. The transport-degradation layer sits in front of each simulated source and injects per-stream delay, jitter, and dropout drawn from distributions measured on the real rover.

**This is a hard gate.** Do not run catch fraction experiments or report policy performance numbers until the degradation layer is active. A policy trained or evaluated without degradation describes a clean-sim world; the gap to hardware is unmeasured and unquantified.

Train two policy variants at each fidelity level: one against clean simulator outputs, one with degradation injected. The difference in catch fractions between them is the quantity that characterizes how much timing shift alone costs you.

---

## Training to deployment: the ONNX path

The policy trained via the Gymnasium wrapper is an actor network. Export it to ONNX and insert it as `OnnxNavigateNode`, which satisfies the standard `BTNode` contract:

```cpp
class OnnxNavigateNode : public BTNode {
    NodeStatus tick(const GoalContext&, const WorldState&, CommandStream&) override;
    // loads ONNX actor, feeds WorldState.to_observation_vector(), writes command
};
```

Nothing below the command boundary changes. Nothing above the observation boundary changes. The monitor sees the same interface it saw during training.

The evaluation harness runs a fixed mission set against ArduPilot SITL with degradation active and parses the session log for catch fractions. This is the deployment validation step — not a replacement for training, but the final measurement before hardware.

---

## DreamerV4 / offline RL (later)

Once MCAP session logging is in place, logs from SITL and hardware runs become a training dataset — training data is the deployment distribution by construction. This is not a near-term concern and does not affect the architecture above.

---

## What Needs to Be Built

**Blocked on Phase 1 (do not start training infrastructure until these exist):**

- [ ] Transport-degradation layer: configurable per-source delay, jitter, dropout in the simulator backend — hard gate for any catch fraction measurement
- [ ] `OnnxNavigateNode` satisfying the `BTNode` contract (blocked on `SPAR_ENABLE_ONNXRUNTIME`)

**Phase 2 initial — kinematic backend, pose-only WorldState:**

- [ ] `KinematicBackend` in C++: bicycle/diff-drive kinematic rover, parameters calibrated against measured ArduPilot GUIDED mode step responses; satisfies the SimulatorBackend interface (push pose into assembler, accept approved command, step)
- [ ] `SPAR_BACKEND=kinematic` CMake flag wired up; `SPAR_ENABLE_MAVLINK=OFF` + `SPAR_ENABLE_ZENOH=OFF` builds and runs cleanly
- [ ] pybind11 bindings exposing the SPAR tick loop to Python
- [ ] `SparEnv(gym.Env)` wrapper: `reset()`, `step()`, `WorldState.to_observation_vector()`, reward function for rover navigation
- [ ] SAC training run (stable-baselines3): clean-obs variant and degradation-injected variant; ONNX actor export for both
- [ ] Evaluation harness: load ONNX actor into `OnnxNavigateNode`, run fixed mission against ArduPilot SITL with degradation active, parse session log for catch fractions

**Phase 2 extended — richer WorldState (obstacles, camera):**

- [ ] `ObservationBuffer` in `shared/contracts/` — sliding window of raw samples, used by both training wrapper and `OnnxNavigateNode`
- [ ] `CarlaBackend`: CARLA Python API, pushes pose + camera + lidar into assembler after each step
- [ ] DreamerV3 training once WorldState includes images or obstacle maps; ONNX actor export
- [ ] `IsaacBackend` (optional): GPU-parallelized parallel envs via Isaac Lab

**Later:**

- [ ] DreamerV4 offline RL on MCAP session logs — not a concern until MCAP logging is in place and real run data exists
