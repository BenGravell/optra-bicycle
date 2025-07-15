#pragma once

#include <vector>

template <typename T>
T lerp(const T a, const T b, const double t) {
    return (1.0 - t) * a + t * b;
}

// 1D Linear interpolation: given sorted xs, fs, and a new point x, return interpolated f.
template <typename T>
inline T interp(const std::vector<double>& xs, const std::vector<T>& fs, const double x) {
    // Clamp to boundary.
    if (x <= xs.front()) {
        return fs.front();
    }
    if (x >= xs.back()) {
        return fs.back();
    }

    // Find the indices i1 and i2 for the interval on xs containing x.
    const auto it = std::lower_bound(xs.begin(), xs.end(), x);
    const int i2 = static_cast<int>(std::distance(xs.begin(), it));
    const int i1 = i2 - 1;

    // Linear interpolation in [idx-1, idx].
    const double x1 = xs[i1];
    const double x2 = xs[i2];
    const T f1 = fs[i1];
    const T f2 = fs[i2];
    const double t = (x - x1) / (x2 - x1);
    return lerp(f1, f2, t);
}
