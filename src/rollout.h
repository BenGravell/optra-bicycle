#pragma once

#include <Eigen/Dense>
#include <vector>

#include "dynamics.h"
#include "policy.h"
#include "space.h"
#include "trajectory.h"

inline void rolloutOpenLoop(const ActionSequence& action_sequence, const StateVector& initial_state, const Dynamics& dynamics, Trajectory& traj) {
    // Initialize the first state in the state sequence.
    traj.setStateAt(0, initial_state);

    // Simulate dynamics forward using open-loop action sequence.
    for (size_t stage_idx = 0; stage_idx < traj_length; ++stage_idx) {
        traj.setActionAt(stage_idx, action_sequence.col(stage_idx));
        traj.setStateAt(stage_idx + 1, dynamics.forward(traj.stateAt(stage_idx), traj.actionAt(stage_idx)));
    }
}

inline void rolloutClosedLoop(const Policy& policy, const Trajectory& traj_ref, const Dynamics& dynamics, Trajectory& traj) {
    // Copy initial state from reference trajectory to trajectory.
    traj.setStateAt(0, traj_ref.stateAt(0));

    // Simulate dynamics forward using closed-loop feedback policy.
    for (size_t stage_idx = 0; stage_idx < traj_length; ++stage_idx) {
        // Get states.
        const StateVector& state_ref = traj_ref.stateAt(stage_idx);
        const StateVector& state = traj.stateAt(stage_idx);

        // Compute state deviation.
        const StateVector state_dev = state - state_ref;

        // Compute action as feedforward + feedback control.
        const ActionVector action = traj_ref.actionAt(stage_idx) + policy.act(state_dev, stage_idx);
        traj.setActionAt(stage_idx, action);

        // Simulate forward one step.
        traj.setStateAt(stage_idx + 1, dynamics.forward(state, action));
    }
}
