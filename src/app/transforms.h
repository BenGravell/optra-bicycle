#pragma once

#include <raylib.h>

#include "core/position.h"
#include "core/space.h"

// Screen-space constants
static constexpr int GUTTER_SS_X = 200;
static constexpr int GUTTER_SS_Y = 400;

static constexpr int SCALE_SS = 80;
static constexpr int ORIGIN_SS_X = GUTTER_SS_X;
static constexpr int ORIGIN_SS_Y = GUTTER_SS_Y + 2 * SCALE_SS;
const Vector2 ORIGIN_SS = {ORIGIN_SS_X, ORIGIN_SS_Y};

// Screen dimensions, px.
static constexpr int SCREEN_WIDTH = 2 * GUTTER_SS_X + 20 * SCALE_SS;
static constexpr int SCREEN_HEIGHT = 2 * GUTTER_SS_Y + (2 + 2) * SCALE_SS;

inline Vector2 state2screen(const Vector2 state) {
    return {ORIGIN_SS.x + SCALE_SS * state.x, ORIGIN_SS.y + SCALE_SS * state.y};
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
    return {(point.x - ORIGIN_SS.x) / SCALE_SS, (point.y - ORIGIN_SS.y) / SCALE_SS, 0.0, 0.0};
}
