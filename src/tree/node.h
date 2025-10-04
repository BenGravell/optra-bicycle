#pragma once

#include <optional>

#include "core/space.h"
#include "core/trajectory.h"

struct Node {
    // State of the node.
    const StateVector state;

    // Pointer to parent node.
    const std::shared_ptr<Node> parent;

    // Trajectory leading from parent->state to this->state.
    // Satisfies endpoint conditions:
    // 1. parent->state == traj.stateAt(0) 
    // 2.   this->state == traj.stateTerminal()
    const std::optional<Trajectory<TRAJ_LENGTH_STEER>> traj;

    // Cost of traj.
    const double cost;

    // Cost to come to this node from the root of the tree.
    const double cost_to_come;

    // Whether this node came from warm-start or not.
    const bool is_warm{false};
};
