#pragma once

#include <vector>

template <typename T>
double lerp(const T a, const T b, const double t) {
    return (1.0 - t) * a + t * b;
}

// 1D Linear interpolation: given sorted xs, fs, and a new point x, return interpolated f.
inline double interp(const std::vector<double>& xs, const std::vector<double>& fs, const double x) {
    // Clamp to boundary.
    if (x <= xs.front()) {
        return fs.front();
    }
    if (x >= xs.back()) {
        return fs.back();
    }

    // Find the first element >= x.
    const auto it = std::lower_bound(xs.begin(), xs.end(), x);
    const int idx = static_cast<int>(it - xs.begin());
    if (idx == 0) {
        return fs.front();
    }

    // Linear interpolation in [idx-1, idx].
    const double x1 = xs[idx - 1];
    const double x2 = xs[idx];
    const double f1 = fs[idx - 1];
    const double f2 = fs[idx];
    const double t = (x - x1) / (x2 - x1);
    return lerp(f1, f2, t);
}
