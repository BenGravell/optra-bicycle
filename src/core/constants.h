#pragma once

// #include <numbers>

static const double LOG2 = std::log(2.0);

// NOTE: raylib defines its own version of these
// static constexpr double PI = std::numbers::pi;
// static constexpr double RAD2DEG = 180.0 / std::numbers::pi;
// static constexpr double DEG2RAD = std::numbers::pi / 180.0;

// TODO move to a config struct - VehicleLimits ?
static constexpr double V_MAX = 10.0;
static constexpr double V_MIN = -V_MAX;

// static constexpr double V_MAX = 40.0;   // ~90 mph
// static constexpr double V_MIN = -10.0;  // ~22 mph

static constexpr double ACCEL_LON_MAX = 3.0;   // 0.3g
static constexpr double ACCEL_LAT_MAX = 6.0;   // 0.6g
static constexpr double CURVATURE_MAX = 0.25;  // ~35 degrees steering angle @ 2.73 meter body length