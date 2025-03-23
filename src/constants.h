#pragma once

// #include <numbers>


static const double LOG2 = std::log(2.0);

// static constexpr double PI = std::numbers::pi;
// static constexpr double RAD2DEG = 180.0 / std::numbers::pi;
// static constexpr double DEG2RAD = std::numbers::pi / 180.0;


// TODO move to a config struct
static constexpr double max_lon_accel = 3.0;
static constexpr double max_lat_accel = 6.0;
static constexpr double max_curvature = 0.25;