#pragma once

#include <Eigen/Dense>
#include <ostream>

// [x, y, yaw, speed]
static constexpr uint64_t num_states = 4;

// [acceleration, curvature]
static constexpr uint64_t num_actions = 2;

using StateVector = Eigen::Vector<double, num_states>;
using ActionVector = Eigen::Vector<double, num_actions>;

using StateStateMatrix = Eigen::Matrix<double, num_states, num_states>;
using StateActionMatrix = Eigen::Matrix<double, num_states, num_actions>;
using ActionActionMatrix = Eigen::Matrix<double, num_actions, num_actions>;
using ActionStateMatrix = Eigen::Matrix<double, num_actions, num_states>;