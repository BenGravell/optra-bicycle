#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "util.h"
#include "solver.h"
#include "space.h"
#include "trajectory.h"

struct BoundaryCondition {
    const double x0;
    const double v0;
    const double xf;
    const double vf;
};

struct BoundaryConditionsXY {
    const BoundaryCondition x;
    const BoundaryCondition y;
};

struct CubicCoeffs {
    const double a3;
    const double a2;
    const double a1;
    const double a0;
};

// Cubic polynomial coefficients from boundary conditions.
inline CubicCoeffs bc2coeffs(const BoundaryCondition& bc, const double T) {
    const double T2 = T * T;
    const double T3 = T2 * T;

    const double dx = bc.xf - bc.x0;
    const double vsum = bc.v0 + bc.vf;

    const double a0 = bc.x0;
    const double a1 = bc.v0;
    const double a2 = 3.0 * dx / T2 - (bc.v0 + vsum) / T;
    const double a3 = (T * vsum - 2.0 * dx) / T3;

    return {a3, a2, a1, a0};
}

// Convert full state (x, y, yaw, v) to boundary conditions in x and y.
inline BoundaryConditionsXY states2bcs(const StateVector& start, const StateVector& goal) {
    const double x0 = start[0];
    const double y0 = start[1];
    const double yaw_0 = start[2];
    const double v0 = start[3];

    const double xf = goal[0];
    const double yf = goal[1];
    const double yaw_f = goal[2];
    const double vf = goal[3];

    const double vx0 = v0 * std::cos(yaw_0);
    const double vy0 = v0 * std::sin(yaw_0);

    const double vxf = vf * std::cos(yaw_f);
    const double vyf = vf * std::sin(yaw_f);

    const BoundaryCondition x_bc{x0, vx0, xf, vxf};
    const BoundaryCondition y_bc{y0, vy0, yf, vyf};

    return {x_bc, y_bc};
}

template <int N>
inline ActionSequence<N> steer_cubic(const StateVector& start, const StateVector& goal, const double t_total) {
    const BoundaryConditionsXY bcs = states2bcs(start, goal);

    const CubicCoeffs x_coeffs = bc2coeffs(bcs.x, t_total);
    const CubicCoeffs y_coeffs = bc2coeffs(bcs.y, t_total);

    const double dt = t_total / N;

    ActionSequence<N> action_sequence;

    for (int i = 0; i < N; ++i) {
        // Sample time at midpoint of the current segment.
        const double t1 = (i + 0.5) * dt;
        const double t2 = square(t1);

        const double dxdt = 3.0 * x_coeffs.a3 * t2 + 2.0 * x_coeffs.a2 * t1 + x_coeffs.a1;
        const double dydt = 3.0 * y_coeffs.a3 * t2 + 2.0 * y_coeffs.a2 * t1 + y_coeffs.a1;

        const double d2xdt2 = 6.0 * x_coeffs.a3 * t1 + 2.0 * x_coeffs.a2;
        const double d2ydt2 = 6.0 * y_coeffs.a3 * t1 + 2.0 * y_coeffs.a2;

        // Clamp speed below by a small positive value to prevent division by zero 
        // and avoid creating huge accelerations & curvatures.
        static constexpr double v_min = 0.01;
        const double v = std::max(std::sqrt(dxdt * dxdt + dydt * dydt), v_min);
        const double v3 = cube(v);

        // Equations from differential flatness for kinematic bicycle.
        const double accel = (dxdt * d2xdt2 + dydt * d2ydt2) / v;
        const double curvature = (dxdt * d2ydt2 - dydt * d2xdt2) / v3;
        const ActionVector action{accel, curvature};
        action_sequence.col(i) = action;
    }

    return action_sequence;
}
