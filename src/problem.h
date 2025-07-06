#pragma once

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <vector>

#include "constants.h"
#include "dynamics.h"
#include "loss.h"
#include "trajectory.h"

// Struct for a trajectory optimization problem
struct Problem {
    Dynamics dynamics;
    Loss loss;
    StateVector initial_state;
    double total_time;
};

// Make a problem.
inline Problem makeProblem(const StateVector initial_state, const StateVector terminal_state_target, const double total_time, const int traj_length) {
    const double inverse_traj_length = 1.0 / traj_length;
    const double delta_time = inverse_traj_length * total_time;

    // Create the dynamics model.
    const Dynamics dynamics{delta_time};

    // Define the loss function.

    // Soft terms
    static constexpr double soft_scale = 0.1;
    static constexpr double accel_lon_scale = 5.0 * soft_scale;
    static constexpr double accel_lat_scale = 5.0 * soft_scale;
    static constexpr double curvature_scale = 1.0 * soft_scale;
    static constexpr double accel_lon_tol = 0.5;
    static constexpr double accel_lat_tol = 0.5;
    static constexpr double curvature_tol = 0.05;
    const SoftParams soft_params{accel_lon_scale, accel_lat_scale, curvature_scale, accel_lon_tol, accel_lat_tol, curvature_tol};

    // Vehicle limits
    static constexpr double speed_max = 40.0;      // ~90 mph
    static constexpr double speed_min = -10.0;     // ~22 mph

    const VehicleLimits vehicle_limits{speed_max,
                                       speed_min,
                                       accel_lon_max,
                                       accel_lat_max,
                                       curvature_max};

    static constexpr double speed_lim_scale = 0.01;
    static constexpr double accel_lon_max_scale = 0.01;
    static constexpr double accel_lat_max_scale = 0.01;
    static constexpr double curvature_max_scale = 0.01;

    static constexpr double speed_free_pos = 0.99 * speed_max;
    static constexpr double speed_free_neg = 0.99 * speed_min;
    static constexpr double accel_lon_free = 0.99 * accel_lon_max;
    static constexpr double accel_lat_free = 0.99 * accel_lat_max;
    static constexpr double curvature_free = 0.99 * curvature_max;

    const VehicleLimitsParams vehicle_limits_params{speed_lim_scale,
                                                    speed_free_pos,
                                                    speed_free_neg,
                                                    accel_lon_max_scale,
                                                    accel_lon_free,
                                                    accel_lat_max_scale,
                                                    accel_lat_free,
                                                    curvature_max_scale,
                                                    curvature_free};

    // Terminal state
    static constexpr double terminal_xy_scale = 1.0;        // 1 m per m
    static constexpr double terminal_xy_tol = 0.01;         // 1 cm
    static constexpr double terminal_yaw_scale = 5.0 / PI;  // 5 m per 180 deg
    static constexpr double terminal_yaw_tol = 0.02;        // ~1 degree
    static constexpr double terminal_speed_scale = 0.5;     // 1 m per 0.5 m/s
    static constexpr double terminal_speed_tol = 0.01;      // 1 cm/s

    const TerminalStateParams terminal_state_params{terminal_xy_scale,
                                                    terminal_xy_tol,
                                                    terminal_yaw_scale,
                                                    terminal_yaw_tol,
                                                    terminal_speed_scale,
                                                    terminal_speed_tol};

    // Instantiate the loss.
    const Loss loss{soft_params, vehicle_limits, vehicle_limits_params, terminal_state_params, terminal_state_target, inverse_traj_length};

    // Return the constructed optimal control problem.
    return Problem{dynamics, loss, initial_state, total_time};
}
