#include <raylib.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>

#include "app/colors.h"
#include "app/drawing.h"
#include "app/transforms.h"
#include "core/dynamics.h"
#include "core/interp.h"
#include "core/problem.h"
#include "core/rollout.h"
#include "core/space.h"
#include "core/search_space.h"
#include "core/util.h"
#include "ilqr/solver.h"
#include "ilqr/solver_settings.h"
#include "planner/planner.h"
#include "rrt/rrt.h"

struct VisibilitySettings {
    bool show_tree;
    bool show_pre_opt_traj;
    bool show_post_opt_traj;
};

static constexpr int GAME_FPS = 30;

int main() {
    // Initialization
    SetConfigFlags(FLAG_VSYNC_HINT);
    const int screenWidth = 2 * gutter_ss_x + 20 * scale_ss;
    const int screenHeight = 2 * gutter_ss_y + (2 + 2) * scale_ss;
    InitWindow(screenWidth, screenHeight, "Motion Planner");

    // Clock times
    int tree_exp_clock_time = -1;
    int traj_opt_clock_time = -1;
    int draw_elm_clock_time = -1;
    int game_upd_clock_time = -1;
    const double tree_exp_clock_momentum = 0.95;
    const double traj_opt_clock_momentum = 0.95;
    const double draw_elm_clock_momentum = 0.95;
    const double game_upd_clock_momentum = 0.95;

    int draw_elm_clock_time_next = 0;

    // Load a monospaced font
    Font monoFont = LoadFont("fonts/IBMPlexMono-Bold.ttf");

    static const int buttonWidth = 300;
    static const int buttonHeight = 50;
    static const int buttonMargin = 10;
    static const int buttonX1 = screenWidth - 2 * (buttonWidth + buttonMargin);
    static const int buttonX2 = screenWidth - 1 * (buttonWidth + buttonMargin);

    Rectangle pauseButton = {buttonX1, buttonMargin + 0 * (buttonHeight + buttonMargin), buttonWidth, buttonHeight};
    Rectangle advanceButton = {buttonX1, buttonMargin + 1 * (buttonHeight + buttonMargin), buttonWidth, buttonHeight};
    Rectangle useWarmStartButton = {buttonX1, buttonMargin + 2 * (buttonHeight + buttonMargin), buttonWidth, buttonHeight};
    Rectangle useExplorationTreeButton = {buttonX1, buttonMargin + 3 * (buttonHeight + buttonMargin), buttonWidth, buttonHeight};
    Rectangle showTreeButton = {buttonX2, buttonMargin + 0 * (buttonHeight + buttonMargin), buttonWidth, buttonHeight};
    Rectangle showPreOptTrajButton = {buttonX2, buttonMargin + 1 * (buttonHeight + buttonMargin), buttonWidth, buttonHeight};
    Rectangle showPostOptTrajButton = {buttonX2, buttonMargin + 2 * (buttonHeight + buttonMargin), buttonWidth, buttonHeight};

    Rectangle searchSpaceRec = {origin_ss.x, origin_ss.y - 2 * scale_ss, 20 * scale_ss, 4 * scale_ss};

    bool paused = false;  // Game pause state
    bool useWarmStart = true;
    bool useExplorationTree = true;

    bool showTree = true;
    bool showPreOptTraj = true;
    bool showPostOptTraj = true;

    // Define a fixed start point and an initial goal point in state space
    const StateVector start{1.0, 0.0, 0.0, 0.0};
    const StateVector goal{19.0, 0.0, 0.0, 0.0};

    // Convert to screen space
    Vector2 startPoint = state2screen(start);
    Vector2 goalPoint = state2screen(goal);

    // Initial plan
    MultiPlannerOutputs planner_outputs;
    const MultiPlannerSettings planner_settings = {useWarmStart, useExplorationTree};
    planner_outputs = MultiPlanner::plan(planner_settings, start, goal, std::nullopt);

    SetTargetFPS(GAME_FPS);

    float lastTime = GetTime();

    while (!WindowShouldClose()) {
        // Calculate delta time
        const float currentTime = GetTime();
        const float deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        const Vector2 mousePoint = GetMousePosition();

        // check button hitboxes
        const bool mouseInPauseButton = CheckCollisionPointRec(mousePoint, pauseButton);
        const bool mouseInAdvanceButton = CheckCollisionPointRec(mousePoint, advanceButton);
        const bool mouseInUseWarmStartButton = CheckCollisionPointRec(mousePoint, useWarmStartButton);
        const bool mouseInUseExplorationTreeButton = CheckCollisionPointRec(mousePoint, useExplorationTreeButton);
        const bool mouseInShowTreeButton = CheckCollisionPointRec(mousePoint, showTreeButton);
        const bool mouseInShowPreOptTrajButton = CheckCollisionPointRec(mousePoint, showPreOptTrajButton);
        const bool mouseInShowPostOptTrajButton = CheckCollisionPointRec(mousePoint, showPostOptTrajButton);

        // check if mouse is in a button
        const bool mouseInButton = mouseInPauseButton || mouseInAdvanceButton || mouseInUseWarmStartButton || mouseInUseExplorationTreeButton || mouseInShowTreeButton || mouseInShowPreOptTrajButton || mouseInShowPostOptTrajButton;

        // update toggle states
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouseInPauseButton) {
            paused = !paused;
        }
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouseInUseWarmStartButton) {
            useWarmStart = !useWarmStart;
        }
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouseInUseExplorationTreeButton) {
            useExplorationTree = !useExplorationTree;
        }
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouseInShowTreeButton) {
            showTree = !showTree;
        }
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouseInShowPreOptTrajButton) {
            showPreOptTraj = !showPreOptTraj;
        }
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouseInShowPostOptTrajButton) {
            showPostOptTraj = !showPostOptTraj;
        }

        // check for explicit advance
        const bool explicitAdvance = IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouseInAdvanceButton;

        // update goal point from mouse
        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) && !mouseInButton) {
            goalPoint = mousePoint;
        }

        // update start point from mouse
        if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON) && !mouseInButton) {
            // Guard for start point inside obstacle
            if (!obstaclesCollidesWith(obstacles, screen2state(mousePoint))) {
                startPoint = mousePoint;
            }
        }

        // Convert from screen space to state space
        StateVector start = screen2state(startPoint);
        StateVector goal = screen2state(goalPoint);

        // Clamp to search space bounds
        start = clampToSearchSpace(start);
        goal = clampToSearchSpace(goal);

        // Convert back from state space to screen space
        startPoint = state2screen(start);
        goalPoint = state2screen(goal);

        // Update game state.
        const bool do_update_game = !paused || explicitAdvance;
        if (do_update_game) {
            const MultiPlannerSettings planner_settings = {useWarmStart, useExplorationTree};
            const auto warm = std::make_optional(planner_outputs.out.solution);
            planner_outputs = MultiPlanner::plan(planner_settings, start, goal, warm);
        }

        // Draw everything
        BeginDrawing();
        const float draw_elm_clock_start = GetTime();
        ClearBackground(COLOR_BACKGROUND);

        // Draw the search space
        DrawRectangleRec(searchSpaceRec, COLOR_SEARCH_SPACE);

        // Draw the obstacle
        for (const Obstacle& obstacle : obstacles) {
            const Vector2 obstacleCenterSS = state2screen(obstacle.center);
            const double obstacleRadiusSS = obstacle.radius * scale_ss;
            DrawCircleV(obstacleCenterSS, obstacleRadiusSS, COLOR_OBSTACLE);
        }

        // Draw planner outputs.
        const VisibilitySettings viz_settings{showTree, showPreOptTraj, showPostOptTraj};

        // Draw tree(s).
        if (viz_settings.show_tree) {
            if (useExplorationTree) {
                drawTree(planner_outputs.aux.tree, false);
            }
            if (useWarmStart) {
                drawTree(planner_outputs.pri.tree, true);
            }
        }

        // Draw pre-opt trajectory (RRT solution).
        if (viz_settings.show_pre_opt_traj) {
            static constexpr float line_width = 8;
            static constexpr float node_width = 16;
            static constexpr Color color = COLOR_TRAJ_PRE_OPT;
            drawPath(planner_outputs.out.path, line_width, node_width, color);
        }

        // Draw post-opt trajectory (iLQR solution).
        if (viz_settings.show_post_opt_traj) {
            static constexpr float line_width = 4;
            static constexpr Color color = COLOR_TRAJ_POST_OPT;
            drawTrajectory(planner_outputs.out.solution.traj, line_width, color);
        }

        // Draw start point and the goal point
        DrawSquare(startPoint, 10, WHITE);
        DrawSquare(startPoint, 6, BLACK);

        DrawStar(goalPoint, 16, WHITE);
        DrawStar(goalPoint, 8, BLACK);

        if (paused) {
            // Show pause overlay
            DrawText("Paused", (screenWidth / 2) - (MeasureText("Paused", 20) / 2), (gutter_ss_y / 2) - (20 / 2), 20, WHITE);
        }

        // Draw pause button
        DrawRectangleRec(pauseButton, GRAY);
        DrawText(paused ? "Resume" : "Pause", pauseButton.x + 10, pauseButton.y + 15, 20, RAYWHITE);

        // Draw advance button
        DrawRectangleRec(advanceButton, GRAY);
        DrawText("Advance", advanceButton.x + 10, advanceButton.y + 15, 20, RAYWHITE);

        // Draw use-warm-start button
        DrawRectangleRec(useWarmStartButton, GRAY);
        DrawText(useWarmStart ? "Disable warm-start tree" : "Enable warm-start tree", useWarmStartButton.x + 10, useWarmStartButton.y + 15, 20, RAYWHITE);

        // Draw use-exploration-tree button
        DrawRectangleRec(useExplorationTreeButton, GRAY);
        DrawText(useExplorationTree ? "Disable cold-start tree" : "Enable cold-start tree", useExplorationTreeButton.x + 10, useExplorationTreeButton.y + 15, 20, RAYWHITE);

        // Draw show-tree button
        DrawRectangleRec(showTreeButton, GRAY);
        DrawText(showTree ? "Hide tree" : "Show tree", showTreeButton.x + 10, showTreeButton.y + 15, 20, RAYWHITE);

        // Draw show-pre-opt-traj button
        DrawRectangleRec(showPreOptTrajButton, GRAY);
        DrawText(showPreOptTraj ? "Hide pre-opt traj" : "Show pre-opt traj", showPreOptTrajButton.x + 10, showPreOptTrajButton.y + 15, 20, RAYWHITE);

        // Draw show-post-opt-traj button
        DrawRectangleRec(showPostOptTrajButton, GRAY);
        DrawText(showPostOptTraj ? "Hide post-opt traj" : "Show post-opt traj", showPostOptTrajButton.x + 10, showPostOptTrajButton.y + 15, 20, RAYWHITE);

        // Draw the timer info
        if (tree_exp_clock_time < 0) {
            tree_exp_clock_time = planner_outputs.out.timing_info.tree_exp;
        }
        if (traj_opt_clock_time < 0) {
            traj_opt_clock_time = planner_outputs.out.timing_info.traj_opt;
        }
        if (draw_elm_clock_time < 0) {
            draw_elm_clock_time = draw_elm_clock_time_next;
        }
        if (game_upd_clock_time < 0) {
            game_upd_clock_time = static_cast<int>(1e6 * deltaTime);
        }
        tree_exp_clock_time = static_cast<int>(lerp(planner_outputs.out.timing_info.tree_exp, tree_exp_clock_time, paused ? 0.0 : tree_exp_clock_momentum));
        traj_opt_clock_time = static_cast<int>(lerp(planner_outputs.out.timing_info.traj_opt, traj_opt_clock_time, paused ? 0.0 : traj_opt_clock_momentum));
        draw_elm_clock_time = static_cast<int>(lerp(draw_elm_clock_time_next, draw_elm_clock_time, draw_elm_clock_momentum));
        game_upd_clock_time = static_cast<int>(lerp(static_cast<int>(1e6 * deltaTime), game_upd_clock_time, game_upd_clock_momentum));
        DrawTextEx(monoFont, TextFormat("Tree exp: %5.1f ms", 0.001 * static_cast<double>(tree_exp_clock_time)), (Vector2){10, 10 + 0 * 30}, 20, 1, WHITE);
        DrawTextEx(monoFont, TextFormat("Traj opt: %5.1f ms", 0.001 * static_cast<double>(traj_opt_clock_time)), (Vector2){10, 10 + 1 * 30}, 20, 1, WHITE);
        DrawTextEx(monoFont, TextFormat("Draw elm: %5.1f ms", 0.001 * static_cast<double>(draw_elm_clock_time)), (Vector2){10, 10 + 2 * 30}, 20, 1, LIGHTGRAY);
        DrawTextEx(monoFont, TextFormat("Game upd: %5.1f ms", 0.001 * static_cast<double>(game_upd_clock_time)), (Vector2){10, 10 + 3 * 30}, 20, 1, LIGHTGRAY);

        // Draw the planner stats
        const double v_avg = planner_outputs.out.solution.traj.state_sequence.row(3).cwiseAbs().mean();
        DrawTextEx(monoFont, TextFormat("       Traj total time %5.3f s", planner_outputs.out.solution.total_time), (Vector2){10, 150 + 0 * 30}, 20, 1, MONOKAI_YELLOW);
        DrawTextEx(monoFont, TextFormat("       Traj  avg speed %5.3f m/s", v_avg), (Vector2){10, 150 + 1 * 30}, 20, 1, MONOKAI_YELLOW);

        DrawTextEx(monoFont, TextFormat("Ratio rejected samples %5.0f%%", 100.0 * planner_outputs.out.tree.ratio_rejected_samples), (Vector2){10, 150 + 3 * 30}, 20, 1, MONOKAI_ORANGE);
        DrawTextEx(monoFont, TextFormat("       Traj  opt iters %5d", planner_outputs.out.solution.solve_record.iters), (Vector2){10, 150 + 4 * 30}, 20, 1, MONOKAI_ORANGE);
        DrawTextEx(monoFont, TextFormat("    Post-opt cost, sol %9.6f", planner_outputs.out.solution.cost), (Vector2){10, 150 + 5 * 30}, 20, 1, WHITE);
        DrawTextEx(monoFont, TextFormat("    Post-opt cost, pri %9.6f", planner_outputs.pri.solution.cost), (Vector2){10, 150 + 6 * 30}, 20, 1, MONOKAI_RED);
        DrawTextEx(monoFont, TextFormat("    Post-opt cost, aux %9.6f", planner_outputs.aux.solution.cost), (Vector2){10, 150 + 7 * 30}, 20, 1, MONOKAI_BLUE);

        // ---- Speed vs Time plot
        {
            const int plotWidth = 300;
            const int plotHeight = 100;
            const int plotX = 10 + 0 * (plotWidth + 10);
            const int plotY = screenHeight - (2 * plotHeight) - 50;

            // Draw border
            DrawRectangleLines(plotX, plotY, plotWidth, plotHeight, GRAY);
            DrawRectangleLines(plotX, plotY + plotHeight, plotWidth, plotHeight, GRAY);
            DrawTextEx(monoFont, "Speed vs Time", (Vector2){plotX, plotY - 20}, 18, 1, WHITE);

            const Trajectory<TRAJ_LENGTH_OPT>& traj = planner_outputs.out.solution.traj;
            const double total_time = planner_outputs.out.solution.total_time;
            const double dt = total_time / traj.length;

            // const double max_speed = traj.state_sequence.row(3).cwiseAbs().maxCoeff();
            const double max_speed = 5.0;

            // Plot the data

            // Post-opt traj
            if (showPostOptTraj) {
                for (int i = 0; i < traj.length; i++) {
                    float t0 = i * dt;
                    float t1 = (i + 1) * dt;
                    float v0 = traj.state_sequence(3, i);
                    float v1 = traj.state_sequence(3, i + 1);

                    float x0 = plotX + plotWidth * (t0 / total_time);
                    float x1 = plotX + plotWidth * (t1 / total_time);
                    float y0 = plotY + plotHeight * (1.0f - (v0 / max_speed));
                    float y1 = plotY + plotHeight * (1.0f - (v1 / max_speed));

                    DrawLineEx((Vector2){x0, y0}, (Vector2){x1, y1}, 2.0f, COLOR_TRAJ_POST_OPT);
                }
            }

            // Pre-opt traj
            if (showPreOptTraj) {
                for (int i = 0; i < planner_outputs.out.traj_pre_opt.length; i++) {
                    float t0 = i * dt;
                    float t1 = (i + 1) * dt;
                    float v0 = planner_outputs.out.traj_pre_opt.state_sequence(3, i);
                    float v1 = planner_outputs.out.traj_pre_opt.state_sequence(3, i + 1);

                    float x0 = plotX + plotWidth * (t0 / total_time);
                    float x1 = plotX + plotWidth * (t1 / total_time);
                    float y0 = plotY + plotHeight * (1.0f - (v0 / max_speed));
                    float y1 = plotY + plotHeight * (1.0f - (v1 / max_speed));

                    DrawLineEx((Vector2){x0, y0}, (Vector2){x1, y1}, 1.0f, COLOR_TRAJ_PRE_OPT);
                }
            }
        }

        // ---- Lon accel vs Time plot
        {
            const int plotWidth = 300;
            const int plotHeight = 100;
            const int plotX = 10 + 1 * (plotWidth + 10);
            const int plotY = screenHeight - (2 * plotHeight) - 50;

            // Draw border
            DrawRectangleLines(plotX, plotY, plotWidth, plotHeight, GRAY);
            DrawRectangleLines(plotX, plotY + plotHeight, plotWidth, plotHeight, GRAY);
            DrawTextEx(monoFont, "Lon Accel vs Time", (Vector2){plotX, plotY - 20}, 18, 1, WHITE);

            const Trajectory<TRAJ_LENGTH_OPT>& traj = planner_outputs.out.solution.traj;
            const double total_time = planner_outputs.out.solution.total_time;
            const double dt = total_time / traj.length;

            // Plot the data

            // Post-opt traj
            if (showPostOptTraj) {
                for (int i = 0; i < traj.length; i++) {
                    float t0 = i * dt;
                    float t1 = (i + 1) * dt;
                    float a0 = traj.action_sequence(0, i);
                    float a1 = traj.action_sequence(0, i + 1);

                    float x0 = plotX + plotWidth * (t0 / total_time);
                    float x1 = plotX + plotWidth * (t1 / total_time);
                    float y0 = plotY + plotHeight * (1.0f - (a0 / accel_lon_max));
                    float y1 = plotY + plotHeight * (1.0f - (a1 / accel_lon_max));

                    DrawLineEx((Vector2){x0, y0}, (Vector2){x1, y1}, 2.0f, COLOR_TRAJ_POST_OPT);
                }
            }
        }

        // ---- Lat accel vs Time plot
        {
            const int plotWidth = 300;
            const int plotHeight = 100;
            const int plotX = 10 + 2 * (plotWidth + 10);
            const int plotY = screenHeight - (2 * plotHeight) - 50;

            // Draw border
            DrawRectangleLines(plotX, plotY, plotWidth, plotHeight, GRAY);
            DrawRectangleLines(plotX, plotY + plotHeight, plotWidth, plotHeight, GRAY);
            DrawTextEx(monoFont, "Lat Accel vs Time", (Vector2){plotX, plotY - 20}, 18, 1, WHITE);

            const Trajectory<TRAJ_LENGTH_OPT>& traj = planner_outputs.out.solution.traj;
            const double total_time = planner_outputs.out.solution.total_time;
            const double dt = total_time / traj.length;

            // Plot the data

            // Post-opt traj
            if (showPostOptTraj) {
                for (int i = 0; i < traj.length; i++) {
                    float t0 = i * dt;
                    float t1 = (i + 1) * dt;
                    float v0 = planner_outputs.out.traj_pre_opt.state_sequence(3, i);
                    float v1 = planner_outputs.out.traj_pre_opt.state_sequence(3, i + 1);
                    float k0 = traj.action_sequence(1, i);
                    float k1 = traj.action_sequence(1, i + 1);
                    float a0 = k0 * square(v0);
                    float a1 = k1 * square(v1);

                    float x0 = plotX + plotWidth * (t0 / total_time);
                    float x1 = plotX + plotWidth * (t1 / total_time);
                    float y0 = plotY + plotHeight * (1.0f - (a0 / accel_lat_max));
                    float y1 = plotY + plotHeight * (1.0f - (a1 / accel_lat_max));

                    DrawLineEx((Vector2){x0, y0}, (Vector2){x1, y1}, 2.0f, COLOR_TRAJ_POST_OPT);
                }
            }
        }

        // ---- Curvature vs Time plot
        {
            const int plotWidth = 300;
            const int plotHeight = 100;
            const int plotX = 10 + 3 * (plotWidth + 10);
            const int plotY = screenHeight - (2 * plotHeight) - 50;

            // Draw border
            DrawRectangleLines(plotX, plotY, plotWidth, plotHeight, GRAY);
            DrawRectangleLines(plotX, plotY + plotHeight, plotWidth, plotHeight, GRAY);
            DrawTextEx(monoFont, "Curvature vs Time", (Vector2){plotX, plotY - 20}, 18, 1, WHITE);

            const Trajectory<TRAJ_LENGTH_OPT>& traj = planner_outputs.out.solution.traj;
            const double total_time = planner_outputs.out.solution.total_time;
            const double dt = total_time / traj.length;

            // Plot the data

            // Post-opt traj
            if (showPostOptTraj) {
                for (int i = 0; i < traj.length; i++) {
                    float t0 = i * dt;
                    float t1 = (i + 1) * dt;
                    float k0 = traj.action_sequence(1, i);
                    float k1 = traj.action_sequence(1, i + 1);

                    float x0 = plotX + plotWidth * (t0 / total_time);
                    float x1 = plotX + plotWidth * (t1 / total_time);
                    float y0 = plotY + plotHeight * (1.0f - (k0 / curvature_max));
                    float y1 = plotY + plotHeight * (1.0f - (k1 / curvature_max));

                    DrawLineEx((Vector2){x0, y0}, (Vector2){x1, y1}, 2.0f, COLOR_TRAJ_POST_OPT);
                }
            }
        }

        const float draw_elm_clock_stop = GetTime();
        draw_elm_clock_time_next = static_cast<int>(std::ceil(1e6 * (draw_elm_clock_stop - draw_elm_clock_start)));
        EndDrawing();
    }

    // Teardown
    UnloadFont(monoFont);
    CloseWindow();
    return 0;
}


// // UNIT TEST FOR steerCubic
// int main() {
//     const StateVector start{0.0, 0.0, 0.0, 0.0};
//     const StateVector goal{-1.0, 0.0, 0.0, 0.0};
//     const auto action_seq = steerCubic<TRAJ_LENGTH_STEER>(start, goal, steer_time);
//     for (int i = 0; i < TRAJ_LENGTH_STEER; ++i) {
//         std::cout << action_seq.col(i)(0) << std::endl;
//     }
//     return 0;
// }
