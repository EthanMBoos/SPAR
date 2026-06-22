# RL Pipeline

## How the seam works

The policy always trains inside the **same assembler → monitor loop it deploys into.** Only the data source changes between training and deployment.

```
DEPLOYMENT:
  MAVLink telemetry ──▶ assembler ──▶ WorldState ──▶ policy ──▶ CommandStream ──▶ monitor ──▶ ArduPilotAdapter ──▶ rover

TRAINING:
  SimulatorBackend ──▶ assembler ──▶ WorldState ──▶ policy ──▶ CommandStream ──▶ monitor ──▶ SimulatorBackend.step()
                  ↑                                                                                    │
                  └────────────────────────── (loop) ─────────────────────────────────────────────────┘
```

Everything between `assembler` and `monitor` is identical in both modes. The policy contract doesn't know which side of the seam it's on.

---

## Phase 1 — SAC baseline (KinematicBackend)

Conventional RL. Establishes baseline catch fractions before Cosmos data collection begins.

```
1. Reset the world
2. Observe: assembler builds WorldState from KinematicBackend
3. Act: SAC outputs [throttle, steering]
4. Step physics 50 ms — command passed directly, no monitor in the training loop
5. Reward: negative distance to goal
6. Repeat
```

```python
from spar_env import SparEnv
env = SparEnv()
obs, info = env.reset()      # obs: [dx_m, dy_m, heading_sin, heading_cos, speed_ms]
obs, reward, done, trunc, info = env.step([throttle, steering])
```

Export to ONNX. Deploy via `OnnxNavigateNode`.

---

## Phase 4a — Cosmos (behavior cloning)

**What Cosmos is:** a World Foundation Model (WFM) pre-trained on large-scale video. It does not output motor commands out of the box. Fine-tuning adds an action head via behavior cloning on paired (video, action) data.

> **Not latent-space RL.** DreamerV4 trains actor-critic via imagined rollouts in a learned latent space — a different pipeline requiring MCAP logs from Phase 3. Do not conflate the two.

**Training path:**
```
Phase A:  real rover driving ──▶ VideoRecorder ──▶ video + session log (paired by timestamp)
Phase B:  (video, action) pairs ──▶ Cosmos Tokenizer ──▶ WFM backbone (frozen) + action head (BC loss)
Phase C:  action head ──▶ torch.onnx.export() ──▶ cosmos_actor.onnx ──▶ CosmosNavigateNode
```

**Phase B:** supervised regression against ground-truth (throttle, steering) from the session log. Freeze the WFM backbone initially; only the action head is trained. Unfreeze top backbone layers if performance plateaus.

**Minimum dataset:** 30–60 min of diverse rover footage from the target environment, including failure cases. KinematicBackend trajectories are not usable — Cosmos needs real video.

**`CosmosNavigateNode` is not `OnnxNavigateNode`.** Cosmos takes a tokenized video context window, not the 5-float pose obs. A new BTNode variant is required. `OnnxNavigateNode` stays for the SAC path.

---

## Phase 4a prerequisites

### VideoRecorder (hard gate)

No Cosmos fine-tuning without paired video + action data. `VideoRecorder` writes time-aligned (timestamp_us, frame) pairs using the same monotonic clock as the session log. Do not start Phase 4a until VideoRecorder is implemented and its output is verified time-aligned with the session log.

### ObservationBuffer

N-frame sliding window built by the assembler via `latest_at_or_before(t - k*dt)` for k=0..N-1. Passed to learned nodes as `const ObservationBuffer*`; hand-coded nodes receive `nullptr`. Default: N=8, dt=100 ms.

### Action chunking — resolve before starting Phase 4a

The current `BTNode` contract assumes synchronous, Markovian policy (one obs in, one command out, 50 ms). Most capable VLAs violate this:

| Policy | Latency | Output | Compatible? |
|---|---|---|---|
| SAC / NavigateNode | < 1 ms | single command | Yes |
| Cosmos autoregressive | ~5–50 ms | single command | Possibly |
| π0, GR-00T, ACT | 50–500 ms | action chunk (N steps) | No |

**Before starting Phase 4a: benchmark the candidate Cosmos actor's inference latency on target hardware.**
- < 30 ms → autoregressive adapter, existing `BTNode` works
- > 30 ms or chunked output → implement `ChunkedBTNode` + `ChunkQueue` first

`ChunkedBTNode` runs in a background inference thread; the tick loop drains one command per tick from an SPSC queue. The monitor checks the full chunk on arrival, then each drained command individually.

---

## What both approaches share

**The monitor is deployment-only.** Policies train on raw kinematics without monitor interference — catch fractions reflect the policy's natural command distribution, not one already shaped by the monitor. Catch fractions are measured from session logs produced by `OnnxNavigateNode` through the full assembler → monitor → adapter runtime.

**Run degraded variants.** `DegradedSource<T>` injects delay, jitter, and dropout. Always compare clean vs. degraded catch fractions. Clean-only results are not meaningful for deployment comparison.

**ONNX deployment path.** Both SAC and Cosmos export to ONNX and deploy through a BTNode variant. Nothing below the command boundary changes.

---

## Build modes

```bash
# Phase 1 — training + eval
cmake -B build -DSPAR_BACKEND=kinematic -DSPAR_ENABLE_PYTHON=ON -DSPAR_ENABLE_EXPLOIT_NODE=ON
cmake --build build
cd python && python eval_harness.py --build-dir ../build

# Deployment — hardware
cmake -B build -DSPAR_BACKEND=ardupilot -DSPAR_ENABLE_MAVLINK=ON
```

---

## Build status

**Done — Phase 1 boilerplate (2026-06-22):**

- [x] `KinematicBackend` + `DegradedSource<T>` wired end-to-end (`spar_rover/sim/`)
- [x] `DegradationScenarios.h` — four named scenarios via `SPAR_SCENARIO` env var
- [x] `NavigationObs.h` — goal-relative 5-float obs via `make_nav_obs()`
- [x] `OnnxNavigateNode` — `BTNode` contract; guarded by `SPAR_ENABLE_ONNXRUNTIME`
- [x] `ExploitNode` — rate-of-change fixture; guarded by `SPAR_ENABLE_EXPLOIT_NODE`
- [x] `SparEnv(gym.Env)` pybind11 wrapper + eval harness (`python/`)
- [x] Verified catch fractions: baseline 0.003, pose_dropout 0.435, pose_jitter 0.053, policy_exploit 0.999

**Phase 1 gate closed (2026-06-22):**

- [x] SAC training run; ONNX export (`python/train_sac.py` → `python/export_onnx.py`)
- [x] `OnnxNavigateNode` end-to-end baseline run: catch fraction 0.014, mission success, neither = 0

**Phase 2:**

- [ ] `MissionExecutor` + JSON mission loader (see `docs/mission-design.md`)

**Phase 4a:**

- [ ] `VideoRecorder` (hard gate — no Cosmos without this)
- [ ] `ObservationBuffer` in `shared/contracts/`
- [ ] `CosmosNavigateNode`
- [ ] Real rover video dataset (30–60 min)
- [ ] Benchmark inference latency; implement `ChunkedBTNode` + `ChunkQueue` if needed

**Later:**

- [ ] MCAP logging (Phase 3 prerequisite)
- [ ] DreamerV4 offline RL on MCAP logs
