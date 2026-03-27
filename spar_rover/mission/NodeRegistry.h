#pragma once
#include "../../shared/contracts/BTNode.h"
#include <memory>
#include <string>
#include <unordered_map>
#include <functional>
#include <stdexcept>

// Registry that maps node names to factory functions.
// Both hand-coded and ONNX-learned nodes are registered here after passing admission.
class NodeRegistry {
public:
    using Factory = std::function<std::unique_ptr<BTNode>()>;

    void register_node(std::string name, Factory factory) {
        factories_[std::move(name)] = std::move(factory);
    }

    std::unique_ptr<BTNode> create(const std::string& name) const {
        auto it = factories_.find(name);
        if (it == factories_.end())
            throw std::runtime_error("NodeRegistry: unknown node '" + name + "'");
        return it->second();
    }

    bool has(const std::string& name) const {
        return factories_.count(name) > 0;
    }

private:
    std::unordered_map<std::string, Factory> factories_;
};
