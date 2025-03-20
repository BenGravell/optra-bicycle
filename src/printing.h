#pragma once

#include <iostream>
#include <iomanip>
#include <string>

#include "space.h"
#include "util.h"

// Round to thousandths and convert to string.
template <typename T>
std::string val2str(const T x) {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(3)
           << std::setw(8) << std::setfill('0')
           << x;
    return stream.str();
}

std::string state2str(const StateVector state) {
    return val2str(state[0]) + "_" + val2str(state[1]) + "_" + val2str(rad2deg(state[2])) + "_" + val2str(state[3]);
}