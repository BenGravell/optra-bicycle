#pragma once

// #include <numbers>

static const double LOG2 = std::log(2.0);

// NOTE: raylib defines its own version of these
// static constexpr double PI = std::numbers::pi;
// static constexpr double RAD2DEG = 180.0 / std::numbers::pi;
// static constexpr double DEG2RAD = std::numbers::pi / 180.0;

// TODO move to a config struct
static constexpr double accel_lon_max = 3.0;   // 0.3g
static constexpr double accel_lat_max = 6.0;   // 0.6g
static constexpr double curvature_max = 0.25;  // ~35 degrees steering angle @ 2.73 meter body length