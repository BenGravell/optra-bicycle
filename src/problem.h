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
    const double accel_lon_scale = 0.005;
    const double accel_lat_scale = 0.005;
    const double curvature_scale = 0.001;
    const SoftParams soft_params{accel_lon_scale, accel_lat_scale, curvature_scale};

    // Vehicle limits
    const double speed_max = 40.0;      // ~90 mph
    const double speed_min = -10.0;     // ~22 mph
    const double accel_lon_max = 3.0;   // 0.3g
    const double accel_lat_max = 6.0;   // 0.6g
    const double curvature_max = 0.25;  // ~35 degrees steering angle @ 2.73 meter body length
    const VehicleLimits vehicle_limits{speed_max,
                                       speed_min,
                                       accel_lon_max,
                                       accel_lat_max,
                                       curvature_max};

    const double speed_lim_scale = 0.01;
    const double accel_lon_max_scale = 0.01;
    const double accel_lat_max_scale = 0.01;
    const double curvature_max_scale = 0.01;

    const double speed_free_pos = 0.99 * speed_max;
    const double speed_free_neg = 0.99 * speed_min;
    const double accel_lon_free = 0.99 * accel_lon_max;
    const double accel_lat_free = 0.99 * accel_lat_max;
    const double curvature_free = 0.99 * curvature_max;

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
    const double terminal_xy_scale = 1.0;        // 1 m per m
    const double terminal_xy_tol = 0.01;         // 1 cm
    const double terminal_yaw_scale = 5.0 / PI;  // 5 m per 180 deg
    const double terminal_yaw_tol = 0.02;        // ~1 degree
    const double terminal_speed_scale = 0.5;     // 1 m per 0.5 m/s
    const double terminal_speed_tol = 0.01;      // 1 cm/s

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
