#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "solver.h"
#include "space.h"
#include "trajectory.h"


namespace steer_cubic {


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

struct QuinticCoeffs {
    const double a5;
    const double a4;
    const double a3;
    const double a2;
    const double a1;
    const double a0;
};

// Generate a linearly spaced vector of length n from start to end (inclusive).
inline std::vector<double> linspace(const double start, const double end, const int n) {
    if (n < 2) {
        return {start};
    }
    std::vector<double> result(n);
    const double step = (end - start) / (n - 1);
    for (int i = 0; i < n; i++) {
        result[i] = start + i * step;
    }
    return result;
}

// Cubic polynomial coefficients from boundary conditions (x0, v0, xf, vf).
inline CubicCoeffs bc2coeffs_cubic(const BoundaryCondition& bc, const double t_end) {
    const double a0 = bc.x0;
    const double a1 = bc.v0;
    const double a2 = 3.0 * (bc.xf - bc.x0) / (t_end * t_end) - (2.0 * bc.v0 + bc.vf) / t_end;
    const double a3 = (t_end * (bc.v0 + bc.vf) - 2.0 * (bc.xf - bc.x0)) / (t_end * t_end * t_end);
    return {a3, a2, a1, a0};
}


// Quintic polynomial coefficients from boundary conditions (x0, v0, xf, vf).
inline QuinticCoeffs bc2coeffs_quintic(const BoundaryCondition& bc, const double t_end) {
    const double T = t_end;
    const double x0 = bc.x0, v0 = bc.v0, xf = bc.xf, vf = bc.vf;
    const double D = xf - x0 - v0 * T;
    const double dV = vf - v0;
    
    const double a0 = x0;
    const double a1 = v0;
    const double a2 = 0.0;
    const double a5 = (6.0 * D - 3.0 * dV * T) / (T * T * T * T * T);
    const double a3 = dV / (T * T) + (5.0 / 3.0) * a5 * (T * T);
    const double a4 = - dV / (2.0 * T * T * T) - (5.0 / 2.0) * a5 * T;
    
    return {a5, a4, a3, a2, a1, a0};
}



// Convert full state (x, y, yaw, v) to boundary conditions in x and y.
inline BoundaryConditionsXY states2bcs(const StateVector& start, const StateVector& goal) {
    // start = (x0, y0, yaw0, v0), goal = (xf, yf, yawf, vf)
    const double x0 = start[0];
    const double y0 = start[1];
    const double yaw_0 = start[2];
    const double v0 = start[3];

    const double xf = goal[0];
    const double yf = goal[1];
    const double yaw_f = goal[2];
    const double vf = goal[3];

    // Convert speed + yaw to velocity components
    const double vx0 = v0 * std::cos(yaw_0);
    const double vy0 = v0 * std::sin(yaw_0);

    const double vxf = vf * std::cos(yaw_f);
    const double vyf = vf * std::sin(yaw_f);

    // Boundary conditions for x, y
    const BoundaryCondition x_bc{x0, vx0, xf, vxf};
    const BoundaryCondition y_bc{y0, vy0, yf, vyf};

    return {x_bc, y_bc};
}

// Generate (x, y) samples for a cubic polynomial in [0, 1].
inline std::pair<std::vector<double>, std::vector<double>> steer_xy_cubic(const StateVector& start, const StateVector& goal, const double t_est, const int num_pts) {
    const BoundaryConditionsXY bcs = states2bcs(start, goal);

    const CubicCoeffs x_coeffs = bc2coeffs_cubic(bcs.x, t_est);
    const CubicCoeffs y_coeffs = bc2coeffs_cubic(bcs.y, t_est);

    const std::vector<double> u = linspace(0.0, t_est, num_pts);

    std::vector<double> x(num_pts);
    std::vector<double> y(num_pts);
    for (int i = 0; i < num_pts; i++) {
        const double u1 = u[i];
        const double u2 = u1 * u1;
        const double u3 = u2 * u1;
        x[i] = x_coeffs.a3 * u3 + x_coeffs.a2 * u2 + x_coeffs.a1 * u1 + x_coeffs.a0;
        y[i] = y_coeffs.a3 * u3 + y_coeffs.a2 * u2 + y_coeffs.a1 * u1 + y_coeffs.a0;
    }
    return {x, y};
}

// Generate (x, y) samples for a quintic polynomial in [0, 1].
inline std::pair<std::vector<double>, std::vector<double>> steer_xy_quintic(const StateVector& start, const StateVector& goal, const double t_est, const int num_pts) {
    const BoundaryConditionsXY bcs = states2bcs(start, goal);
    
    const QuinticCoeffs x_coeffs = bc2coeffs_quintic(bcs.x, t_est);
    const QuinticCoeffs y_coeffs = bc2coeffs_quintic(bcs.y, t_est);

    const std::vector<double> u = linspace(0.0, t_est, num_pts);

    std::vector<double> x(num_pts);
    std::vector<double> y(num_pts);
    for (int i = 0; i < num_pts; i++) {
        const double u1 = u[i];
        const double u2 = u1 * u1;
        const double u3 = u2 * u1;
        const double u4 = u2 * u2;
        const double u5 = u3 * u2;
        x[i] = x_coeffs.a5 * u5 + x_coeffs.a4 * u4 + x_coeffs.a3 * u3 + x_coeffs.a2 * u2 + x_coeffs.a1 * u1 + x_coeffs.a0;
        y[i] = y_coeffs.a5 * u5 + y_coeffs.a4 * u4 + y_coeffs.a3 * u3 + y_coeffs.a2 * u2 + y_coeffs.a1 * u1 + y_coeffs.a0;
    }
    return {x, y};
}



/**
 * @brief Convert a sequence of (x, y) positions to:
 *        a: constant acceleration at each segment,
 *        k: curvature at each segment,
 *        t: time stamps along the path.
 */
inline std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> xy2akt(const std::vector<double>& x, const std::vector<double>& y, const double v0, const double vf, const double max_lon_accel, const double max_lat_accel) {
    const int n = static_cast<int>(x.size());

    if (n < 2) {
        throw std::runtime_error("xy2akt requires at least two points.");
    }

    // 1) ds = segment lengths
    std::vector<double> dx(n - 1);
    std::vector<double> dy(n - 1);
    std::vector<double> ds(n - 1);
    for (int i = 0; i < n - 1; i++) {
        dx[i] = x[i + 1] - x[i];
        dy[i] = y[i + 1] - y[i];
        ds[i] = std::hypot(dx[i], dy[i]);
    }

    // 2) s = cumulative path length
    std::vector<double> s(n);
    s[0] = 0.0;
    for (int i = 1; i < n; i++) {
        s[i] = s[i - 1] + ds[i - 1];
    }
    const double s_tot = s[n - 1];
    if (s_tot < 1e-12) {
        // Degenerate path case: no distance
        // Return zeros or handle as needed. We'll just do trivial output.
        std::vector<double> a_out(n - 1, 0.0), k_out(n - 1, 0.0), t(n, 0.0);
        return {a_out, k_out, t};
    }

    // 3) Constant acceleration
    double a = (vf * vf - v0 * v0) / (2.0 * s_tot);
    // Clamp to control bounds
    a = std::clamp(a, -max_lon_accel, max_lon_accel);

    // 4) Time stamps t[i]
    //    t[i] = ( sqrt(v0^2 + 2*a*s[i]) - v0 ) / a
    //    (handle a = 0 as a special case)
    std::vector<double> t(n, 0.0);
    if (std::fabs(a) < 1e-12) {
        // If acceleration is ~0, then velocity is nearly constant
        // v ~ v0. We'll just assume constant v0 for the entire path.
        for (int i = 1; i < n; i++) {
            t[i] = t[i - 1] + ds[i - 1] / v0;
        }
    } else {
        for (int i = 0; i < n; i++) {
            // use constant acceleration formula
            // TODO verify this works for positive and negative a, v0, vf
            const double v_at_ti = std::max(v0 * v0 + 2.0 * a * s[i], 0.0);
            t[i] = std::max((std::sqrt(v_at_ti) - v0) / a, 0.0);
        }
    }

    // 5) a_out: constant a for each segment
    //    We'll store it at segment-level => length n-1
    std::vector<double> a_out(n - 1, a);

    // 6) velocity vx, vy at each segment
    //    dt[i] = t[i+1] - t[i]
    std::vector<double> dt(n - 1);
    for (int i = 0; i < n - 1; i++) {
        dt[i] = t[i + 1] - t[i];
        if (std::fabs(dt[i]) < 1e-12) {
            dt[i] = 1e-12;  // guard to avoid division by 0
        }
    }
    std::vector<double> vx(n - 1);
    std::vector<double> vy(n - 1);
    for (int i = 0; i < n - 1; i++) {
        vx[i] = dx[i] / dt[i];
        vy[i] = dy[i] / dt[i];
    }

    // 7) approximate dvx, dvy
    //    first diff, then pad at both ends, then average adjacent
    //    length of vx, vy is n-1 => diff is n-2
    std::vector<double> dvx(n - 1);
    std::vector<double> dvy(n - 1);

    // compute raw diffs
    std::vector<double> diff_vx(n - 2);
    std::vector<double> diff_vy(n - 2);
    for (int i = 0; i < n - 2; i++) {
        diff_vx[i] = vx[i + 1] - vx[i];
        diff_vy[i] = vy[i + 1] - vy[i];
    }

    // stack: [diff_vx[0], diff_vx..., diff_vx[n-3]]
    // after hstack -> length = (n-2) + 2 => n
    // then average adjacent => final length n-1
    std::vector<double> tmp_vx(n);
    std::vector<double> tmp_vy(n);
    // pad front/back
    tmp_vx[0] = diff_vx[0];
    tmp_vy[0] = diff_vy[0];
    for (int i = 0; i < n - 2; i++) {
        tmp_vx[i + 1] = diff_vx[i];
        tmp_vy[i + 1] = diff_vy[i];
    }
    tmp_vx[n - 1] = diff_vx[n - 3];
    tmp_vy[n - 1] = diff_vy[n - 3];

    // now average adjacent elements
    // dvx[i] = (tmp_vx[i] + tmp_vx[i+1]) / 2
    for (int i = 0; i < n - 1; i++) {
        dvx[i] = 0.5 * (tmp_vx[i] + tmp_vx[i + 1]);
        dvy[i] = 0.5 * (tmp_vy[i] + tmp_vy[i + 1]);
    }

    // 8) ax = dvx / dt, ay = dvy / dt
    std::vector<double> ax(n - 1);
    std::vector<double> ay(n - 1);
    for (int i = 0; i < n - 1; i++) {
        ax[i] = dvx[i] / dt[i];
        ay[i] = dvy[i] / dt[i];
    }

    // 9) curvature k = (vx * ay - vy * ax) / ( (vx^2 + vy^2)^(3/2) )
    //    store per segment => length n-1
    std::vector<double> k_out(n - 1, 0.0);
    for (int i = 0; i < n - 1; i++) {
        const double v_sq = vx[i] * vx[i] + vy[i] * vy[i];
        const double denom = std::pow(v_sq, 1.5);
        // Use zero curvature @ near-zero speed to avoid division by zero
        if (std::fabs(v_sq) < 1e-12) {
            k_out[i] = 0.0;
        } else {
            k_out[i] = (vx[i] * ay[i] - vy[i] * ax[i]) / denom;
        }
    }

    return {a_out, k_out, t};
}



// 1D Linear interpolation: given sorted xData, yData, and a new point x, return interpolated y.
inline double interp1(const std::vector<double>& xData,
                      const std::vector<double>& yData,
                      const double x) {
    // If out of bounds, clamp to boundary
    if (x <= xData.front()) {
        return yData.front();
    }
    if (x >= xData.back()) {
        return yData.back();
    }
    // Otherwise, find interval
    // (simple approach: std::lower_bound for the first element >= x)
    auto it = std::lower_bound(xData.begin(), xData.end(), x);
    int idx = static_cast<int>(it - xData.begin());
    if (idx == 0) {
        return yData.front();
    }
    // linear interpolation in [idx-1, idx]
    const double x1 = xData[idx - 1];
    const double x2 = xData[idx];
    const double y1 = yData[idx - 1];
    const double y2 = yData[idx];
    const double t = (x - x1) / (x2 - x1);
    return y1 + t * (y2 - y1);
}

// Resample a(t), k(t) at new time stamps t_new (uniform in [0, t.back()]).
inline ActionSequence resample(const std::vector<double>& a, const std::vector<double>& k, const std::vector<double>& t) {
    // t.size() = n, a.size() = n-1, k.size() = n-1
    const int n = static_cast<int>(t.size());
    // midpoints ta[i] = (t[i+1] + t[i]) / 2, i=0..n-2 => length n-1
    std::vector<double> ta(n - 1);
    for (int i = 0; i < n - 1; i++) {
        ta[i] = 0.5 * (t[i + 1] + t[i]);
    }

    // new t array
    const std::vector<double> t_new = linspace(t.front(), t.back(), traj_length + 1);

    // midpoints of t_new
    std::vector<double> ta_new(traj_length);
    for (int i = 0; i < traj_length; i++) {
        ta_new[i] = 0.5 * (t_new[i + 1] + t_new[i]);
    }

    // interpolate a, k onto ta_new
    ActionSequence action_sequence;
    for (int i = 0; i < traj_length; i++) {
        action_sequence(0, i) = interp1(ta, a, ta_new[i]);
        action_sequence(1, i) = interp1(ta, k, ta_new[i]);
    }

    return action_sequence;
}

inline std::tuple<ActionSequence, double> steer_cubic(const StateVector& start, const StateVector& goal, const double t_est, const double max_lon_accel, const double max_lat_accel) {
    // Generate (x, y) with 2*traj_length + 1 points so that we have a high-resolution path. We'll downsample later.
    const int num_pts_xy = (2 * traj_length) + 1;
    const auto [x, y] = steer_xy_cubic(start, goal, t_est, num_pts_xy);
    
    // Extract v0 and vf.
    const double v0 = start[3];
    const double vf = goal[3];

    // Convert to acceleration (a), curvature (k), time (t)
    const auto [a, k, t] = xy2akt(x, y, v0, vf, max_lon_accel, max_lat_accel);

    // Resample a, k, t to uniform timesteps
    const ActionSequence action_sequence = resample(a, k, t);

    // Total time is the last timestamp
    const double total_time = t.back();

    return std::make_tuple(action_sequence, total_time);
}


inline std::tuple<ActionSequence, double> steer_quintic(const StateVector& start, const StateVector& goal, const double t_est, const double max_lon_accel, const double max_lat_accel) {
    // Generate (x, y) with 2*traj_length + 1 points so that we have a high-resolution path. We'll downsample later.
    const int num_pts_xy = (2 * traj_length) + 1;
    const auto [x, y] = steer_xy_quintic(start, goal, t_est, num_pts_xy);
    
    // Extract v0 and vf.
    const double v0 = start[3];
    const double vf = goal[3];

    // Convert to acceleration (a), curvature (k), time (t)
    const auto [a, k, t] = xy2akt(x, y, v0, vf, max_lon_accel, max_lat_accel);

    // Resample a, k, t to uniform timesteps
    const ActionSequence action_sequence = resample(a, k, t);

    // Total time is the last timestamp
    const double total_time = t.back();

    return std::make_tuple(action_sequence, total_time);
}

} // namespace steer_cubic
