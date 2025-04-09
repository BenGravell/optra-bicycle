#pragma once

#include "constants.h"

template <typename T>
T square(const T x) {
    return x * x;
}

template <typename T>
T cube(const T x) {
    return x * x * x;
}

template <typename T>
T quart(const T x) {
    return square(square(x));
}

template <typename T>
T rad2deg(const T x) {
    return RAD2DEG * x;
}

template <typename T>
T deg2rad(const T x) {
    return DEG2RAD * x;
}
