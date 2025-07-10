#pragma once

#include <raylib.h>

#include "cmaps/crest_r.h"
#include "cmaps/flare_r.h"
#include "cmaps/inferno.h"
#include "cmaps/magma.h"
#include "cmaps/mako.h"
#include "cmaps/rocket.h"
#include "cmaps/turbo.h"
#include "cmaps/viridis.h"

// x must be in [0, 1]
inline Color colormapFromArray(const float x, const std::array<std::array<uint8_t, 3>, 256>& colormap) {
    // Scale to [0, 255] and round to nearest integer
    int idx = static_cast<int>(x * 255.0f + 0.5f);
    idx = std::clamp(idx, 0, 255);

    // Extract color from colormap @ idx.
    const auto& rgb = colormap[idx];

    // Convert to raylib Color format, with full opacity.
    return Color{rgb[0], rgb[1], rgb[2], 255};
}

inline Color colormap(const float x) {
    return colormapFromArray(x, turbo_colormap);
}

inline Color warmColormap(const float x) {
    return colormapFromArray(x, flare_r_colormap);
}

inline Color coolColormap(const float x) {
    return colormapFromArray(x, crest_r_colormap);
}


// Helper macro to convert hex to Color (expects 0xRRGGBBAA)
#define HEX2COLOR(hex)                             \
    (Color) { (unsigned char)((hex >> 24) & 0xFF), \
              (unsigned char)((hex >> 16) & 0xFF), \
              (unsigned char)((hex >> 8) & 0xFF),  \
              (unsigned char)(hex & 0xFF) }

// Define colors using hex format 0xRRGGBBAA

// Monokai
static constexpr Color MONOKAI_RED = HEX2COLOR(0xF92672FF);
static constexpr Color MONOKAI_ORANGE = HEX2COLOR(0xFD971FFF);
static constexpr Color MONOKAI_YELLOW = HEX2COLOR(0xE6DB74FF);
static constexpr Color MONOKAI_GREEN = HEX2COLOR(0xA6E22EFF);
static constexpr Color MONOKAI_BLUE = HEX2COLOR(0x66D9EFFF);
static constexpr Color MONOKAI_PURPLE = HEX2COLOR(0xAE81FFFF);

// Atlantis
static constexpr Color ATLANTIS_BLUE_DARK = HEX2COLOR(0x123370FF);
static constexpr Color ATLANTIS_BLUE_MEDIUM = HEX2COLOR(0x026BACFF);
static constexpr Color ATLANTIS_BLUE_BRIGHT = HEX2COLOR(0x2AB3E7FF);
static constexpr Color ATLANTIS_TEAL = HEX2COLOR(0x0D8EA1FF);
static constexpr Color ATLANTIS_GOLD = HEX2COLOR(0xF6B337FF);

static constexpr Color DARK_GRAY = {20, 20, 20, 255};
static constexpr Color MEDIUM_GRAY = {60, 60, 60, 255};

static constexpr Color COLOR_BACKGROUND = BLACK;
const Color COLOR_SEARCH_SPACE = DARK_GRAY;

static constexpr Color COLOR_OBSTACLE = MEDIUM_GRAY;

// static constexpr Color COLOR_START = ATLANTIS_GOLD;
// static constexpr Color COLOR_GOAL = ATLANTIS_GOLD;

const Color COLOR_TREE = Fade(WHITE, 0.1f);
static constexpr Color COLOR_WARM_START = ATLANTIS_TEAL;

static constexpr Color COLOR_TRAJ_POST_OPT = WHITE;
static constexpr Color COLOR_TRAJ_PRE_OPT = GRAY;