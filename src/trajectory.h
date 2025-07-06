#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <vector>
#include "space.h"

// Length of trajectory for steering function, i.e. for one node.
static constexpr uint64_t traj_length_steer = 10;

// Length of trajectory for trajectory optimization, i.e. the entire trajectory.
static constexpr uint64_t traj_length_opt = 100;

static_assert((traj_length_opt % traj_length_steer) == 0, "traj_length_opt must be a multiple of traj_length_steer");

template <int N>
using StateSequence = Eigen::Matrix<double, num_states, N + 1>;

template <int N>
using ActionSequence = Eigen::Matrix<double, num_actions, N>;

template <int N>
struct Trajectory {
    static constexpr int length = N;

    StateSequence<N> state_sequence;
    ActionSequence<N> action_sequence;

    // Getter for the state at a specific stage index.
    StateVector stateAt(const size_t stage_idx) const {
        return state_sequence.col(stage_idx);
    }

    // Getter for the action at a specific stage index.
    ActionVector actionAt(const size_t stage_idx) const {
        return action_sequence.col(stage_idx);
    }

    // Getter for the terminal state.
    StateVector stateTerminal() const {
        return state_sequence.col(state_sequence.cols() - 1);
    }

    // Setter for the state at a specific stage index.
    void setStateAt(const size_t stage_idx, const StateVector& state) {
        state_sequence.col(stage_idx) = state;
    }

    // Setter for the action at a specific stage index.
    void setActionAt(const size_t stage_idx, const ActionVector& action) {
        action_sequence.col(stage_idx) = action;
    }
};
