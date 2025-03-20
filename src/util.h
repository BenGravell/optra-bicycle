#pragma once

#include "constants.h"

template <typename T>
T square(T x) {
    return x * x;
}

template <typename T>
T cube(T x) {
    return x * x * x;
}

template <typename T>
T quart(T x) {
    return square(square(x));
}

template <typename T>
T lerp(const T a, const T b, const T t) {
    return (1 - t) * a + t * b;
}

template <typename T>
T rad2deg(const T x) {
    return RAD2DEG * x;
}

template <typename T>
T deg2rad(const T x) {
    return DEG2RAD * x;
}
