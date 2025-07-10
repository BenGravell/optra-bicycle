#pragma once

#include <raylib.h>

#include "core/position.h"
#include "core/space.h"

// Screen-space constants
static constexpr int gutter_ss_x = 200;
static constexpr int gutter_ss_y = 400;

static constexpr int scale_ss = 80;
static constexpr int origin_ss_x = gutter_ss_x;
static constexpr int origin_ss_y = gutter_ss_y + 2 * scale_ss;
const Vector2 origin_ss = {origin_ss_x, origin_ss_y};

inline Vector2 state2screen(const Vector2 state) {
    return {origin_ss.x + scale_ss * state.x, origin_ss.y + scale_ss * state.y};
}

inline Vector2 state2screen(const Position state) {
    const Vector2 vec{static_cast<float>(state.x), static_cast<float>(state.y)};
    return state2screen(vec);
}

inline Vector2 state2screen(const StateVector state) {
    const Vector2 vec{static_cast<float>(state[0]), static_cast<float>(state[1])};
    return state2screen(vec);
}

inline StateVector screen2state(const Vector2 point) {
    return {(point.x - origin_ss.x) / scale_ss, (point.y - origin_ss.y) / scale_ss, 0.0, 0.0};
}
