#pragma once

#include <raylib.h>

#include "app/colors.h"
#include "app/transforms.h"
#include "core/dynamics.h"
#include "core/interp.h"
#include "core/problem.h"
#include "core/rollout.h"
#include "core/space.h"
#include "core/trajectory.h"
#include "rrt/rrt.h"

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

void drawPath(const Path& path, const float line_width, const float node_width, const Color color) {
    for (const auto& node : path) {
        drawTrajectory(node->traj, line_width, color);
        DrawCircleV(state2screen(node->state), 0.5 * node_width, color);
    }
}

void drawTree(const Tree& tree, const bool warm) {
    for (const auto& node : tree.nodes) {
        // TODO add UI button for coloring white or by time index.

        // // Widen & special color for warm start node.
        // const float line_width = (node->is_warm) ? 2 : 1;
        // const auto color = (node->is_warm) ? COLOR_WARM_START : COLOR_TREE;

        // Color by time index.
        const float line_width = 1.0;
        const float c = static_cast<float>(node->time_ix) / static_cast<float>(time_ix_max);
        const Color color = Fade(warm ? warmColormap(c) : coolColormap(c), 0.2f);

        // Skip drawing traj for nodes with null parent (e.g. root node),
        // which have garbage node->traj.
        if (node->parent != nullptr) {
            drawTrajectory(node->traj, line_width, color);
        }

        // This incurs a huge drawing performance hit
        // DrawCircleV(state2screen(node->state), 2 * line_width, color);
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