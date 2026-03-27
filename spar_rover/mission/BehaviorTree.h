#pragma once
#include "../../shared/contracts/BTNode.h"
#include <memory>
#include <vector>

// Minimal sequential behavior tree: ticks nodes in order until one returns
// Running (continues that node next tick) or Failure (restarts from the top).
// Sufficient for Phase 1 SITL demos; replace with a full BT library when
// the node set grows large enough to need composites/decorators.
class BehaviorTree {
public:
    void add_node(std::unique_ptr<BTNode> node);
    void reset();

    // Tick the active node. Returns the node's status.
    // Populates out_cmd with whatever the active node emitted (caller passes
    // this to the monitor regardless of status).
    NodeStatus tick(const GoalContext& goal,
                    const WorldState&  world,
                    CommandStream&     out_cmd);

    const BTNode* active_node() const;

private:
    std::vector<std::unique_ptr<BTNode>> nodes_;
    size_t                               active_idx_ = 0;
};
