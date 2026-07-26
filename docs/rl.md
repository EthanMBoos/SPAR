# Reinforcement learning

RL is the next vertical slice, not a finished feature. There is currently no
Gym environment, trainer, exported policy, or learned ROS behavior node in
this repository.

The intended path is:

```text
generated MJCF worlds
  -> pure MuJoCo training loop, without ROS
  -> one exported policy
  -> one behavior-tree leaf
  -> evaluation in the existing ROS smoke harness
```

Training should load the same `sim/worlds/<name>.xml` files as the deployed
sim. ROS, Nav2, cameras, and the behavior tree do not belong in the fast
step loop unless they are the system being evaluated.

## First experiment

Implement one learned local-planner leaf before adding reusable RL
infrastructure:

- observation: downsampled planar lidar plus a relative local goal;
- action: linear and angular velocity;
- task: reach the goal without collision;
- worlds: the fixed prompts and seeds recorded by worldgen;
- baseline: the existing Nav2 behavior on the same start and goal pairs;
- deployment: a BT leaf with the same halt and stale-input behavior as the
  existing navigation leaves.

Measure success rate, collision rate, elapsed simulation time, and behavior
under stale sensor input. Keep training dependencies outside the base ROS
image until this first policy proves which library and artifact format are
actually needed.

The implementation is complete only when an exported policy runs through the
ROS stack and a repeatable check compares it with the baseline. Do not build a
generic policy manager or model registry first.

## Perception boundary

The current detector publishes `spar_perception/Detection`. A pretrained
vision model can replace that node without changing either behavior tree.
Learning from camera pixels is a separate experiment; the first RL slice
should use low-dimensional state so world variety and control can be tested
without rendering cost.
