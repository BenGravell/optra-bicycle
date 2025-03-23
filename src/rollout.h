#pragma once

#include <Eigen/Dense>
#include <vector>

#include "constants.h"
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

inline void rolloutOpenLoopConstrained(const ActionSequence& action_sequence, const StateVector& initial_state, const Dynamics& dynamics, Trajectory& traj) {
    // Initialize the first state in the state sequence.
    traj.setStateAt(0, initial_state);

    // Simulate dynamics forward using open-loop action sequence.
    for (size_t stage_idx = 0; stage_idx < traj_length; ++stage_idx) {
        // Extract
        const StateVector& state = traj.stateAt(stage_idx);
        ActionVector action = action_sequence.col(stage_idx);

        // Project
        const double v_sq = state[3] * state[3];
        double lon_accel = action[0];
        double lat_accel = action[1] * v_sq;
        double curvature = action[1];
        const double dyn_max_curvature = std::min(max_curvature, max_lat_accel / (v_sq + 1e-12));
        lon_accel = std::clamp(lon_accel, -max_lon_accel, max_lon_accel);
        curvature = std::clamp(curvature, -dyn_max_curvature, dyn_max_curvature);
        action << lon_accel, curvature;

        // Advance
        traj.setActionAt(stage_idx, action);
        traj.setStateAt(stage_idx + 1, dynamics.forward(state, action));
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
