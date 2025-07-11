#pragma once

#include "core/constants.h"
#include "core/space.h"

static constexpr double x_max = 20.0;
static constexpr double x_min = 0.0;

static constexpr double y_max = 2.0;
static constexpr double y_min = -y_max;
// static constexpr double y_min = 0;

static constexpr double yaw_max = 0.5 * PI;
static constexpr double yaw_min = -yaw_max;

inline StateVector clampToSearchSpace(const StateVector& state) {
    const double x = std::clamp(state(0), x_min, x_max);
    const double y = std::clamp(state(1), y_min, y_max);
    const double yaw = std::clamp(state(2), yaw_min, yaw_max);
    const double v = std::clamp(state(3), v_min, v_max);
    return {x, y, yaw, v};
}