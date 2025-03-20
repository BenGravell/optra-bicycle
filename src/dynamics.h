#pragma once

#include <space.h>

#include <Eigen/Dense>
#include <utility>


struct Jacobian {
    const StateStateMatrix A;
    const StateActionMatrix B;
};

struct Dynamics {
    const double delta_time;

    StateVector forward(const StateVector& state, const ActionVector& action) const {
        // Extract states and actions.
        const double yaw = state[2];
        const double speed = state[3];
        const double accel = action[0];
        const double curvature = action[1];
        // Assemble output.
        return state + delta_time * StateVector{
                                        speed * std::cos(yaw),
                                        speed * std::sin(yaw),
                                        speed * curvature,
                                        accel};
    }

    Jacobian jacobian(const StateVector& state, const ActionVector& action) const {
        // Extract states and actions.
        const double yaw = state[2];
        const double speed = state[3];
        const double accel = action[0];
        const double curvature = action[1];

        // Compute intermediate quantities.
        const double dt_sin_yaw = delta_time * std::sin(yaw);
        const double dt_cos_yaw = delta_time * std::cos(yaw);

        // Assemble output.
        StateStateMatrix A;
        A << 1, 0, -speed * dt_sin_yaw, dt_cos_yaw,
            0, 1, speed * dt_cos_yaw, dt_sin_yaw,
            0, 0, 1, 0,
            0, 0, 0, 1;

        StateActionMatrix B;
        B << 0, 0,
            0, 0,
            0, speed * delta_time,
            delta_time, 0;

        return {A, B};
    }
};
