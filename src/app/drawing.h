#pragma once

#include <raylib.h>

#include "app/colors.h"
#include "app/transforms.h"
#include "core/dynamics.h"
#include "core/interp.h"
#include "core/problem.h"
#include "core/rollout.h"
#include "core/search_space.h"
#include "core/space.h"
#include "core/trajectory.h"
#include "tree/tree.h"

template <int N>
void drawTrajectory(const Trajectory<N>& traj, const float line_width, const Color color) {
    Vector2 ray_traj[N + 1];

    for (int i = 0; i <= N; i++) {
        const StateVector& state = traj.state_sequence.col(i);
        ray_traj[i] = state2screen(state);
    }

    for (int i = 0; i < N; i++) {
        DrawLineEx(ray_traj[i], ray_traj[i + 1], line_width, color);
    }
}

void drawPath(const Path& path, const float line_width, const float node_width) {
    for (const auto& node : path) {
        if (node == nullptr) {
            continue;
        }
        if (node->traj) {
            drawTrajectory(node->traj.value(), line_width, COLOR_TRAJ_PRE_OPT);
        }
        DrawCircleV(state2screen(node->state), 0.5 * node_width, COLOR_NODE_PRE_OPT);
    }
}

void drawTree(const Tree& tree, const bool warm) {
    // time_ix runs from 0 to NUM_STEER_SEGMENTS + 1, inclusive.
    // NOTE: NUM_STEER_SEGMENTS + 1 == tree.layers.size()
    // NOTE: start node is tree.layers[0].front()
    // NOTE:  goal node is tree.layers[NUM_STEER_SEGMENTS + 1].front()
    for (int time_ix = 0; time_ix <= NUM_STEER_SEGMENTS; ++time_ix) {
        const Nodes& nodes = tree.layers[time_ix];
        for (const auto& node : nodes) {
            if (node == nullptr) {
                continue;
            }
            if (node->parent == nullptr) {
                continue;
            }

            // TODO add UI button for coloring white or by time index.

            // Color by time index.
            const float line_width = 1.0;
            const float c = static_cast<float>(time_ix) / static_cast<float>(TIME_IX_MAX);
            const Color color = Fade(warm ? warmColormap(c) : coolColormap(c), 0.8f);
            // const Color color = warm ? warmColormap(c) : coolColormap(c);

            if (node->traj) {
                drawTrajectory(node->traj.value(), line_width, color);
            }
        }
    }
}

void DrawSquare(const Vector2 center, const float radius, const Color color) {
    DrawRectangle(center.x - radius, center.y - radius, 2 * radius, 2 * radius, color);
}

void DrawGoalTriangle(const Vector2 center, const float radius, const Color color) {
    Vector2 vertex_left = center;
    vertex_left.x = vertex_left.x - 0.5 * radius;
    vertex_left.y = vertex_left.y + 0.5 * radius;

    Vector2 vertex_right = center;
    vertex_right.x = vertex_right.x + 0.5 * radius;
    vertex_right.y = vertex_right.y + 0.5 * radius;

    Vector2 vertex_top = center;
    vertex_top.y = vertex_top.y - 0.5 * radius;

    DrawTriangle(vertex_top, vertex_left, vertex_right, color);
}

void DrawStar(const Vector2 center, const float radius, const Color color) {
    // Number of spikes. Must be >= 2.
    static constexpr int n = 5;
    static constexpr int n2 = 2 * n;
    static constexpr float angle_step = (2 * PI) / n2;

    const float outer_radius = radius;
    const float inner_radius = 0.4 * outer_radius;

    // Generate vertices.
    Vector2 vertices[n2];
    float angle = -0.5 * PI;
    for (int i = 0; i < n2; i++) {
        const float v_radius = ((i % 2) == 0) ? outer_radius : inner_radius;
        vertices[i].x = center.x + cosf(angle) * v_radius;
        vertices[i].y = center.y + sinf(angle) * v_radius;
        angle += angle_step;
    }

    // Draw filled triangles.
    for (int i = 0; i < n2; i++) {
        const int j = (i + 1) % n2;
        // Pass vertices in counter-clockwise order, required by DrawTriangle.
        DrawTriangle(center, vertices[j], vertices[i], color);
    }
}

void drawSeries(std::vector<double> vals, const double val_max, const double dt, const double total_time, const int plot_x, const int plot_y, const int plotWidth, const int plotHeight, const float line_width, const Color color) {
    for (int i = 0; (i + 1) < vals.size(); i++) {
        const float t0 = i * dt;
        const float t1 = (i + 1) * dt;
        const float val0 = vals[i];
        const float val1 = vals[i + 1];

        const float x0 = plot_x + plotWidth * (t0 / total_time);
        const float x1 = plot_x + plotWidth * (t1 / total_time);
        const float y0 = plot_y + plotHeight * (1.0f - (val0 / val_max));
        const float y1 = plot_y + plotHeight * (1.0f - (val1 / val_max));

        DrawLineEx((Vector2){x0, y0}, (Vector2){x1, y1}, line_width, color);
    }
}

struct VisibilitySettings {
    bool show_tree;
    bool show_pre_opt_traj;
    bool show_post_opt_traj;
};

struct TimePlotDataValues {
    std::vector<double> post_opt_traj;
    std::vector<double> pre_opt_traj;
};

static constexpr int PLOT_WIDTH = 300;
static constexpr int PLOT_HALF_HEIGHT = 50;
static constexpr int TIME_PLOT_MARGIN_X = 10;
static constexpr int TIME_PLOT_MARGIN_Y = 10;
static constexpr int TIME_PLOT_TITLE_FONT_SIZE = 20;
static constexpr int TIME_PLOT_TITLE_MARGIN_Y = TIME_PLOT_TITLE_FONT_SIZE + 10;

void drawTimePlot(const TimePlotDataValues& vals, const double val_max, const double dt, const double total_time, const VisibilitySettings& viz_settings, const int ix_plot, const std::string& name, const Font font) {
    const int plot_x = TIME_PLOT_MARGIN_X + ix_plot * (PLOT_WIDTH + TIME_PLOT_MARGIN_X);
    const int plot_y = SCREEN_HEIGHT - (2 * PLOT_HALF_HEIGHT) - TIME_PLOT_MARGIN_Y;

    // Draw border
    DrawRectangleLines(plot_x, plot_y, PLOT_WIDTH, PLOT_HALF_HEIGHT, COLOR_GRAY_160);
    DrawRectangleLines(plot_x, plot_y + PLOT_HALF_HEIGHT, PLOT_WIDTH, PLOT_HALF_HEIGHT, COLOR_GRAY_160);

    // Draw title
    const std::string title = name + " vs Time";
    DrawTextEx(font, title.c_str(), (Vector2){(float)plot_x, (float)plot_y - TIME_PLOT_TITLE_MARGIN_Y}, TIME_PLOT_TITLE_FONT_SIZE, 1, WHITE);

    // Post-opt traj
    if (viz_settings.show_post_opt_traj) {
        static constexpr float line_width = 2.0f;
        static constexpr Color color = COLOR_TRAJ_POST_OPT;
        drawSeries(vals.post_opt_traj, val_max, dt, total_time, plot_x, plot_y, PLOT_WIDTH, PLOT_HALF_HEIGHT, line_width, color);
    }

    // Pre-opt traj
    if (viz_settings.show_pre_opt_traj) {
        static constexpr float line_width = 1.0f;
        static constexpr Color color = COLOR_TRAJ_PRE_OPT;
        drawSeries(vals.pre_opt_traj, val_max, dt, total_time, plot_x, plot_y, PLOT_WIDTH, PLOT_HALF_HEIGHT, line_width, color);
    }
}