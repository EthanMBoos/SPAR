#pragma once
#include "GoalContext.h"
#include "CommandStream.h"
#include "WorldState.h"
#include <cstdint>

enum class NodeStatus : uint8_t {
    Running = 0,
    Success = 1,
    Failure = 2,
};

class BTNode {
public:
    virtual ~BTNode() = default;

    virtual NodeStatus  tick(const GoalContext& goal,
                             const WorldState&  world,
                             CommandStream&     out_cmd) = 0;
    virtual void        reset()    = 0;
    virtual const char* name()     const = 0;
    virtual uint64_t    node_id()  const = 0;
};
