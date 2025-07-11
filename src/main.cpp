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
#include "core/search_space.h"
#include "core/space.h"
#include "core/util.h"
#include "ilqr/solver.h"
#include "ilqr/solver_settings.h"
#include "planner/planner.h"
#include "rrt/rrt.h"

static constexpr int GAME_FPS = 30;

template <int N>
std::vector<double> extractSpeed(const Trajectory<N>& traj) {
    std::vector<double> vals;
    for (const double val : traj.state_sequence.row(3)) {
        vals.push_back(val);
    }
    return vals;
}

template <int N>
std::vector<double> extractLonAccel(const Trajectory<N>& traj) {
    std::vector<double> vals;
    for (const double val : traj.action_sequence.row(0)) {
        vals.push_back(val);
    }
    return vals;
}

template <int N>
std::vector<double> extractLatAccel(const Trajectory<N>& traj) {
    std::vector<double> vals;
    for (int i = 0; i < traj.length; ++i) {
        const double v = traj.stateAt(i)(3);
        const double k = traj.actionAt(i)(1);
        const double a = k * square(v);
        vals.push_back(a);
    }
    return vals;
}

template <int N>
std::vector<double> extractCurvature(const Trajectory<N>& traj) {
    std::vector<double> vals;
    for (const double val : traj.action_sequence.row(1)) {
        vals.push_back(val);
    }
    return vals;
}

int main() {
    // Initialization
    SetConfigFlags(FLAG_VSYNC_HINT);

    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Motion Planner");

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
    Font mono_font = LoadFont("fonts/IBMPlexMono-Bold.ttf");

    static const int button_width = 300;
    static const int button_height = 50;
    static const int button_margin = 10;
    static const int button_x1 = SCREEN_WIDTH - 3 * (button_width + button_margin);
    static const int button_x2 = SCREEN_WIDTH - 2 * (button_width + button_margin);
    static const int button_x3 = SCREEN_WIDTH - 1 * (button_width + button_margin);

    // Column 1
    Rectangle pause_button = {button_x1, button_margin + 1 * (button_height + button_margin), button_width, button_height};
    Rectangle advance_button = {button_x1, button_margin + 2 * (button_height + button_margin), button_width, button_height};

    // Column 2
    Rectangle use_action_jitter_button = {button_x2, button_margin + 0 * (button_height + button_margin), button_width, button_height};
    Rectangle use_warm_start_button = {button_x2, button_margin + 1 * (button_height + button_margin), button_width, button_height};
    Rectangle use_exploration_tree_button = {button_x2, button_margin + 2 * (button_height + button_margin), button_width, button_height};

    // Column 3
    Rectangle show_tree_button = {button_x3, button_margin + 0 * (button_height + button_margin), button_width, button_height};
    Rectangle show_pre_opt_traj_button = {button_x3, button_margin + 1 * (button_height + button_margin), button_width, button_height};
    Rectangle show_post_opt_traj_button = {button_x3, button_margin + 2 * (button_height + button_margin), button_width, button_height};

    Rectangle search_space_rec = {ORIGIN_SS.x, ORIGIN_SS.y + (float)(Y_MIN * SCALE_SS), X_SIZE * SCALE_SS, Y_SIZE * SCALE_SS};

    // toggle-able states
    bool paused = false;
    bool use_warm_start = true;
    bool use_exploration_tree = true;
    bool use_action_jitter = true;

    bool show_tree = true;
    bool show_pre_opt_traj = true;
    bool show_post_opt_traj = true;

    // Define a fixed start point and an initial goal point in state space
    const StateVector start{1.0, 0.0, 0.0, 0.0};
    const StateVector goal{19.0, 0.0, 0.0, 0.0};

    // Convert to screen space
    Vector2 start_point = state2screen(start);
    Vector2 goal_point = state2screen(goal);

    // Initial plan
    MultiPlannerOutputs planner_outputs;
    const MultiPlannerSettings planner_settings = {use_warm_start, use_exploration_tree, use_action_jitter};
    planner_outputs = MultiPlanner::plan(planner_settings, start, goal, std::nullopt);

    SetTargetFPS(GAME_FPS);

    float last_time = GetTime();

    while (!WindowShouldClose()) {
        // Calculate delta time
        const float current_time = GetTime();
        const float delta_time = current_time - last_time;
        last_time = current_time;

        const Vector2 mouse_point = GetMousePosition();

        // check button hitboxes
        const bool mouse_in_pause_button = CheckCollisionPointRec(mouse_point, pause_button);
        const bool mouse_in_advance_button = CheckCollisionPointRec(mouse_point, advance_button);
        const bool mouse_in_use_warm_start_button = CheckCollisionPointRec(mouse_point, use_warm_start_button);
        const bool mouse_in_use_exploration_tree_button = CheckCollisionPointRec(mouse_point, use_exploration_tree_button);
        const bool mouse_in_use_action_jitter_button = CheckCollisionPointRec(mouse_point, use_action_jitter_button);

        const bool mouse_in_show_tree_button = CheckCollisionPointRec(mouse_point, show_tree_button);
        const bool mouse_in_show_pre_opt_traj_button = CheckCollisionPointRec(mouse_point, show_pre_opt_traj_button);
        const bool mouse_in_show_post_opt_traj_button = CheckCollisionPointRec(mouse_point, show_post_opt_traj_button);

        // check if mouse is in any button
        const bool mouse_in_button = mouse_in_pause_button || mouse_in_advance_button || mouse_in_use_warm_start_button || mouse_in_use_action_jitter_button || mouse_in_use_exploration_tree_button || mouse_in_show_tree_button || mouse_in_show_pre_opt_traj_button || mouse_in_show_post_opt_traj_button;

        // update toggle states
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouse_in_pause_button) {
            paused = !paused;
        }
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouse_in_use_warm_start_button) {
            use_warm_start = !use_warm_start;
        }
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouse_in_use_exploration_tree_button) {
            use_exploration_tree = !use_exploration_tree;
        }
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouse_in_use_action_jitter_button) {
            use_action_jitter = !use_action_jitter;
        }

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouse_in_show_tree_button) {
            show_tree = !show_tree;
        }
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouse_in_show_pre_opt_traj_button) {
            show_pre_opt_traj = !show_pre_opt_traj;
        }
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouse_in_show_post_opt_traj_button) {
            show_post_opt_traj = !show_post_opt_traj;
        }

        // check for explicit advance
        const bool explicit_advance = IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouse_in_advance_button;

        // update goal point from mouse
        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) && !mouse_in_button) {
            goal_point = mouse_point;
        }

        // update start point from mouse
        if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON) && !mouse_in_button) {
            // Guard for start point inside obstacle
            if (!obstaclesCollidesWith(obstacles, screen2state(mouse_point))) {
                start_point = mouse_point;
            }
        }

        // Convert from screen space to state space
        StateVector start = screen2state(start_point);
        StateVector goal = screen2state(goal_point);

        // Clamp to search space bounds
        start = clampToSearchSpace(start);
        goal = clampToSearchSpace(goal);

        // Convert back from state space to screen space
        start_point = state2screen(start);
        goal_point = state2screen(goal);

        // Update game state.
        const bool do_update_game = !paused || explicit_advance;
        if (do_update_game) {
            const MultiPlannerSettings planner_settings = {use_warm_start, use_exploration_tree, use_action_jitter};
            const auto warm = std::make_optional(planner_outputs.out.solution);
            planner_outputs = MultiPlanner::plan(planner_settings, start, goal, warm);
        }

        // Draw everything
        BeginDrawing();
        const float draw_elm_clock_start = GetTime();
        ClearBackground(COLOR_BACKGROUND);

        // Draw the search space
        DrawRectangleRec(search_space_rec, COLOR_SEARCH_SPACE);

        // Draw the obstacle
        for (const Obstacle& obstacle : obstacles) {
            const Vector2 obstacle_center_ss = state2screen(obstacle.center);
            const double obstacle_radius_ss = obstacle.radius * SCALE_SS;
            DrawCircleV(obstacle_center_ss, obstacle_radius_ss, COLOR_OBSTACLE);
        }

        // Draw planner outputs.
        const VisibilitySettings viz_settings{show_tree, show_pre_opt_traj, show_post_opt_traj};

        // Draw tree(s).
        if (viz_settings.show_tree) {
            if (use_exploration_tree) {
                drawTree(planner_outputs.aux.tree, false);
            }
            if (use_warm_start) {
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
        DrawSquare(start_point, 10, WHITE);
        DrawSquare(start_point, 6, BLACK);

        DrawStar(goal_point, 16, WHITE);
        DrawStar(goal_point, 8, BLACK);

        if (paused) {
            // Show pause overlay
            DrawText("Paused", (SCREEN_WIDTH / 2) - (MeasureText("Paused", 20) / 2), (GUTTER_SS_Y / 2) - (20 / 2), 20, WHITE);
        }

        // Draw pause button
        DrawRectangleRec(pause_button, GRAY);
        DrawText(paused ? "Resume" : "Pause", pause_button.x + 10, pause_button.y + 15, 20, RAYWHITE);

        // Draw advance button
        DrawRectangleRec(advance_button, GRAY);
        DrawText("Advance", advance_button.x + 10, advance_button.y + 15, 20, RAYWHITE);

        // Draw use-warm-start button
        DrawRectangleRec(use_warm_start_button, GRAY);
        DrawText(use_warm_start ? "Disable warm-start tree" : "Enable warm-start tree", use_warm_start_button.x + 10, use_warm_start_button.y + 15, 20, RAYWHITE);

        // Draw use-exploration-tree button
        DrawRectangleRec(use_exploration_tree_button, GRAY);
        DrawText(use_exploration_tree ? "Disable cold-start tree" : "Enable cold-start tree", use_exploration_tree_button.x + 10, use_exploration_tree_button.y + 15, 20, RAYWHITE);

        // Draw use-action-jitter button
        DrawRectangleRec(use_action_jitter_button, GRAY);
        DrawText(use_action_jitter ? "Disable action jitter" : "Enable action jitter", use_action_jitter_button.x + 10, use_action_jitter_button.y + 15, 20, RAYWHITE);

        // Draw show-tree button
        DrawRectangleRec(show_tree_button, GRAY);
        DrawText(show_tree ? "Hide tree" : "Show tree", show_tree_button.x + 10, show_tree_button.y + 15, 20, RAYWHITE);

        // Draw show-pre-opt-traj button
        DrawRectangleRec(show_pre_opt_traj_button, GRAY);
        DrawText(show_pre_opt_traj ? "Hide pre-opt traj" : "Show pre-opt traj", show_pre_opt_traj_button.x + 10, show_pre_opt_traj_button.y + 15, 20, RAYWHITE);

        // Draw show-post-opt-traj button
        DrawRectangleRec(show_post_opt_traj_button, GRAY);
        DrawText(show_post_opt_traj ? "Hide post-opt traj" : "Show post-opt traj", show_post_opt_traj_button.x + 10, show_post_opt_traj_button.y + 15, 20, RAYWHITE);


        // ---- Text stats
        static constexpr int STATS_MARGIN = 10;
        static constexpr int STATS_FONT_SIZE = 20;
        static constexpr int STATS_ROW_HEIGHT = STATS_FONT_SIZE + STATS_MARGIN;
        static constexpr int STATS_WIDTH = 300;


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
            game_upd_clock_time = static_cast<int>(1e6 * delta_time);
        }
        tree_exp_clock_time = static_cast<int>(lerp(planner_outputs.out.timing_info.tree_exp, tree_exp_clock_time, paused ? 0.0 : tree_exp_clock_momentum));
        traj_opt_clock_time = static_cast<int>(lerp(planner_outputs.out.timing_info.traj_opt, traj_opt_clock_time, paused ? 0.0 : traj_opt_clock_momentum));
        draw_elm_clock_time = static_cast<int>(lerp(draw_elm_clock_time_next, draw_elm_clock_time, draw_elm_clock_momentum));
        game_upd_clock_time = static_cast<int>(lerp(static_cast<int>(1e6 * delta_time), game_upd_clock_time, game_upd_clock_momentum));

        // Column 1
        DrawTextEx(mono_font, TextFormat("Tree exp: %5.1f ms", 0.001 * static_cast<double>(tree_exp_clock_time)), (Vector2){STATS_MARGIN, STATS_MARGIN + 0 * STATS_ROW_HEIGHT}, STATS_FONT_SIZE, 1, WHITE);
        DrawTextEx(mono_font, TextFormat("Traj opt: %5.1f ms", 0.001 * static_cast<double>(traj_opt_clock_time)), (Vector2){STATS_MARGIN, STATS_MARGIN + 1 * STATS_ROW_HEIGHT}, STATS_FONT_SIZE, 1, WHITE);
        DrawTextEx(mono_font, TextFormat("Draw elm: %5.1f ms", 0.001 * static_cast<double>(draw_elm_clock_time)), (Vector2){STATS_MARGIN, STATS_MARGIN + 2 * STATS_ROW_HEIGHT}, STATS_FONT_SIZE, 1, LIGHTGRAY);
        DrawTextEx(mono_font, TextFormat("Game upd: %5.1f ms", 0.001 * static_cast<double>(game_upd_clock_time)), (Vector2){STATS_MARGIN, STATS_MARGIN + 3 * STATS_ROW_HEIGHT}, STATS_FONT_SIZE, 1, LIGHTGRAY);

        // Draw the planner stats
        const double v_avg = planner_outputs.out.solution.traj.state_sequence.row(3).cwiseAbs().mean();

        // Column 2
        DrawTextEx(mono_font, TextFormat("       Traj total time %5.3f s", planner_outputs.out.solution.total_time), (Vector2){STATS_MARGIN + STATS_WIDTH, STATS_MARGIN + 0 * 30}, STATS_FONT_SIZE, 1, MONOKAI_YELLOW);
        DrawTextEx(mono_font, TextFormat("       Traj  avg speed %5.3f m/s", v_avg), (Vector2){STATS_MARGIN + STATS_WIDTH, STATS_MARGIN + 1 * STATS_ROW_HEIGHT}, STATS_FONT_SIZE, 1, MONOKAI_YELLOW);
        DrawTextEx(mono_font, TextFormat("Ratio rejected samples %5.0f%%", 100.0 * planner_outputs.out.tree.ratio_rejected_samples), (Vector2){STATS_MARGIN + STATS_WIDTH, STATS_MARGIN + 2 * STATS_ROW_HEIGHT}, STATS_FONT_SIZE, 1, MONOKAI_ORANGE);
        DrawTextEx(mono_font, TextFormat("       Traj  opt iters %5d", planner_outputs.out.solution.solve_record.iters), (Vector2){STATS_MARGIN + STATS_WIDTH, STATS_MARGIN + 3 * STATS_ROW_HEIGHT}, 20, 1, MONOKAI_ORANGE);
        DrawTextEx(mono_font, TextFormat("    Post-opt cost, sol %9.6f", planner_outputs.out.solution.cost), (Vector2){STATS_MARGIN + STATS_WIDTH, STATS_MARGIN + 4 * STATS_ROW_HEIGHT}, STATS_FONT_SIZE, 1, WHITE);
        DrawTextEx(mono_font, TextFormat("    Post-opt cost, pri %9.6f", planner_outputs.pri.solution.cost), (Vector2){STATS_MARGIN + STATS_WIDTH, STATS_MARGIN + 5 * STATS_ROW_HEIGHT}, STATS_FONT_SIZE, 1, MONOKAI_RED);
        DrawTextEx(mono_font, TextFormat("    Post-opt cost, aux %9.6f", planner_outputs.aux.solution.cost), (Vector2){STATS_MARGIN + STATS_WIDTH, STATS_MARGIN + 6 * STATS_ROW_HEIGHT}, STATS_FONT_SIZE, 1, MONOKAI_BLUE);

        // Time plots.
        {
            // Common data.
            const Trajectory<TRAJ_LENGTH_OPT>& traj_pre_opt = planner_outputs.out.traj_pre_opt;
            const Trajectory<TRAJ_LENGTH_OPT>& traj_post_opt = planner_outputs.out.solution.traj;
            const double total_time = planner_outputs.out.solution.total_time;
            const double dt = total_time / traj_post_opt.length;

            // Speed data
            const TimePlotDataValues speed_time_plot_data_vals = {extractSpeed(traj_post_opt), extractSpeed(traj_pre_opt)};

            // Lon accel data
            const TimePlotDataValues lon_accel_time_plot_data_vals = {extractLonAccel(traj_post_opt), extractLonAccel(traj_pre_opt)};

            // Lat accel data
            const TimePlotDataValues lat_accel_time_plot_data_vals = {extractLatAccel(traj_post_opt), extractLatAccel(traj_pre_opt)};

            // Curvature data
            const TimePlotDataValues curvature_time_plot_data_vals = {extractCurvature(traj_post_opt), extractCurvature(traj_pre_opt)};

            drawTimePlot(speed_time_plot_data_vals, V_MAX, dt, total_time, viz_settings, 0, "Speed", mono_font);
            drawTimePlot(lon_accel_time_plot_data_vals, ACCEL_LON_MAX, dt, total_time, viz_settings, 1, "Lon Accel", mono_font);
            drawTimePlot(lat_accel_time_plot_data_vals, ACCEL_LAT_MAX, dt, total_time, viz_settings, 2, "Lat Accel", mono_font);
            drawTimePlot(curvature_time_plot_data_vals, CURVATURE_MAX, dt, total_time, viz_settings, 3, "Curvature", mono_font);
        }

        const float draw_elm_clock_stop = GetTime();
        draw_elm_clock_time_next = static_cast<int>(std::ceil(1e6 * (draw_elm_clock_stop - draw_elm_clock_start)));
        EndDrawing();
    }

    // Teardown
    UnloadFont(mono_font);
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
