#include "BehaviorTree.h"
#include <stdexcept>

void BehaviorTree::add_node(std::unique_ptr<BTNode> node) {
    nodes_.push_back(std::move(node));
}

void BehaviorTree::reset() {
    active_idx_ = 0;
    for (auto& n : nodes_) n->reset();
}

NodeStatus BehaviorTree::tick(const GoalContext& goal,
                               const WorldState&  world,
                               CommandStream&     out_cmd)
{
    if (nodes_.empty()) return NodeStatus::Failure;

    NodeStatus status = nodes_[active_idx_]->tick(goal, world, out_cmd);

    switch (status) {
        case NodeStatus::Running:
            break;
        case NodeStatus::Success:
            active_idx_ = (active_idx_ + 1) % nodes_.size();
            if (active_idx_ == 0) {
                reset();
                return NodeStatus::Success;
            }
            break;
        case NodeStatus::Failure:
            reset();
            break;
    }
    return status;
}

const BTNode* BehaviorTree::active_node() const {
    if (nodes_.empty()) return nullptr;
    return nodes_[active_idx_].get();
}
