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

#include "dynamics.h"
#include "interp.h"
#include "printing.h"
#include "problem.h"
#include "rollout.h"
#include "rrt.h"
#include "solver.h"
#include "solver_settings.h"
#include "space.h"
#include "util.h"

struct PlannerOutputs {
    Tree tree;
    std::vector<std::shared_ptr<Node>> path;
    Solution<traj_length_opt> solution;
    Trajectory<traj_length_opt> traj_pre_opt;
    int tree_exp_clock_time;  // ms
    int path_ext_clock_time;  // ms
    int traj_opt_clock_time;  // ms
};

PlannerOutputs plan(const StateVector& start, const StateVector& goal, const std::optional<Solution<traj_length_opt>>& warm = std::nullopt) {
    // ---- RRT
    const float tree_exp_clock_start = GetTime();

    static constexpr int num_nodes = 300;
    Tree tree;
    tree.grow(start, goal, num_nodes, warm);

    const float tree_exp_clock_stop = GetTime();
    const int tree_exp_clock_time = static_cast<int>(std::ceil(1e6 * (tree_exp_clock_stop - tree_exp_clock_start)));

    // ---- Path extraction
    const float path_ext_clock_start = GetTime();
    const std::shared_ptr<Node> node_best = tree.getCheapestSolutionPrecise(goal);

    // Reconstruct the path by traversing parent pointers.
    std::vector<std::shared_ptr<Node>> path;
    std::shared_ptr<Node> node = node_best;
    while (node->parent != nullptr) {
        path.push_back(node);
        node = node->parent;
    }
    // NOTE: DO NOT push back the root, which has garbage trajectory
    std::reverse(path.begin(), path.end());

    double total_time = 0.0;
    std::vector<double> ts;
    std::vector<double> accels;
    std::vector<double> curvatures;
    {
        double t = 0.0;
        for (const std::shared_ptr<Node>& node : path) {
            total_time += node->total_time;
            const double dt = node->total_time / node->traj.length;
            for (int i = 0; i < node->traj.length; ++i) {
                ts.push_back(t);

                const ActionVector& action = node->traj.actionAt(i);
                accels.push_back(action(0));
                curvatures.push_back(action(1));

                t += dt;
            }
        }
    }

    const double dt = total_time / traj_length_opt;

    ActionSequence<traj_length_opt> action_sequence;
    {
        for (int i = 0; i < traj_length_opt; ++i) {
            const double t = i * dt;
            const double accel = interp(ts, accels, t);
            const double curvature = interp(ts, curvatures, t);
            const ActionVector action{accel, curvature};
            action_sequence.col(i) = action;
        }
    }

    const float path_ext_clock_stop = GetTime();
    const int path_ext_clock_time = static_cast<int>(std::ceil(1e6 * (path_ext_clock_stop - path_ext_clock_start)));

    // ---- Trajectory optimization
    const float traj_opt_clock_start = GetTime();

    // Define the optimal control problem.
    // TODO add feature in iLQR to optimize the total_time concurrently with the trajectory.
    // Then the issue of getting a precise estimate of total_time becomes much less severe.
    const Problem problem = makeProblem(start, goal, total_time, traj_length_opt);

    // // DEBUG: run traj opt to the best node instead of goal node.
    // // This removes the mismatch in duration which can cause issues. But not ideal either since we're not necessarily steering to the actual goal.
    // const Problem problem = makeProblem(start, node_best->state, total_time, traj_length_opt);

    // Get the trajectory pre-optimization for diagnostics later
    Trajectory<traj_length_opt> traj_pre_opt;
    rolloutOpenLoop(action_sequence, start, problem.dynamics, traj_pre_opt);

    // Solver settings.
    const SolverSettings settings = SolverSettings();
    settings.validate();

    // Instantiate the solver.
    Solver solver = Solver(std::make_shared<Problem>(problem), std::make_shared<SolverSettings>(settings));

    // Solve the optimal control problem.
    Solution<traj_length_opt> solution = solver.solve(action_sequence);

    // Replace the cost for consistency with the RRT.
    solution.cost = softLoss(solution.traj);
    const float traj_opt_clock_stop = GetTime();
    const int traj_opt_clock_time = static_cast<int>(std::ceil(1e6 * (traj_opt_clock_stop - traj_opt_clock_start)));

    return {tree, path, solution, traj_pre_opt, tree_exp_clock_time, path_ext_clock_time, traj_opt_clock_time};
}

int main() {
    static constexpr int origin_ss = 200;
    static constexpr int scale_ss = 80;
    const Vector2 origin_offset = {origin_ss, 2 * origin_ss};

    // Initialization
    const int screenWidth = 2 * origin_ss + 20 * scale_ss;
    const int screenHeight = 4 * origin_ss;
    InitWindow(screenWidth, screenHeight, "Motion Planner");

    // Clock times
    int tree_exp_clock_time = -1;
    int path_ext_clock_time = -1;
    int traj_opt_clock_time = -1;
    int gamp_upd_clock_time = 0;
    const double tree_exp_clock_momentum = 0.95;
    const double path_ext_clock_momentum = 0.95;
    const double traj_opt_clock_momentum = 0.95;

    // Load a monospaced font
    // Font monoFont = LoadFont("fonts/SpaceMono-Bold.ttf");
    Font monoFont = LoadFont("fonts/IBMPlexMono-Bold.ttf");

    Rectangle pauseButton = {screenWidth - 130, 10, 120, 50};
    Rectangle advanceButton = {screenWidth - 130, 70, 120, 50};
    bool paused = false;  // Game pause state

    // Define a fixed start point and an initial goal point
    Vector2 startPoint = origin_offset;
    Vector2 goalPoint = {origin_offset.x + 20 * scale_ss, origin_offset.y + 0 * scale_ss};

    const StateVector start{0.0, 0.0, 0.0, 0.0};
    const StateVector goal{(goalPoint.x - origin_offset.x) / scale_ss, (goalPoint.y - origin_offset.y) / scale_ss, 0.0, 0.0};

    // Initial plan
    PlannerOutputs planner_outputs = plan(start, goal);

    static constexpr int gameFPS = 120;
    static constexpr int planFPS = 50;

    SetTargetFPS(gameFPS);

    float timeAccumulator = 0.0f;
    const float updateInterval = 1.0f / planFPS;
    float lastTime = GetTime();

    while (!WindowShouldClose()) {
        // Calculate delta time
        float currentTime = GetTime();
        float deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        timeAccumulator += deltaTime;
        const bool enough_time_elapsed = timeAccumulator >= updateInterval;

        const Vector2 mousePoint = GetMousePosition();

        const bool mouseInPauseButton = CheckCollisionPointRec(mousePoint, pauseButton);
        const bool mouseInAdvanceButton = CheckCollisionPointRec(mousePoint, advanceButton);
        const bool mouseInButton = mouseInPauseButton || mouseInAdvanceButton;

        bool pauseStateToggled = false;
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouseInPauseButton) {
            pauseStateToggled = true;
            paused = !paused;  // Toggle pause state
        }

        const bool explicitAdvance = IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && mouseInAdvanceButton;

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) && !mouseInButton) {
            goalPoint = mousePoint;
        }

        // Update game state
        const bool do_update_game = (!paused && enough_time_elapsed) || explicitAdvance;
        if (do_update_game) {
            timeAccumulator = 0.0f;

            const StateVector start{0.0, 0.0, 0.0, 0.0};
            const StateVector goal{(goalPoint.x - origin_offset.x) / scale_ss, (goalPoint.y - origin_offset.y) / scale_ss, 0.0, 0.0};

            planner_outputs = plan(start, goal, planner_outputs.solution);
        }

        // Draw everything
        BeginDrawing();
        ClearBackground((Color){30, 30, 30, 255});  // Dark grey background

        // Draw the obstacle
        const Vector2 obstacleCenterSS = {origin_offset.x + 10 * scale_ss, origin_offset.y + 0 * scale_ss};
        const double obstacleRadiusSS = 1.0 * scale_ss;
        DrawCircleV(obstacleCenterSS, obstacleRadiusSS, RED);

        // Draw the tree
        {
            for (const auto& node : planner_outputs.tree.nodes) {
                // Skip the root node, which has garbage trajectory data.
                if (node->parent != nullptr) {
                    Vector2 node_ray_traj[node->traj.length + 1];

                    for (int i = 0; i <= node->traj.length; i++) {
                        const StateVector& state = node->traj.state_sequence.col(i);
                        node_ray_traj[i].x = origin_offset.x + scale_ss * state(0);
                        node_ray_traj[i].y = origin_offset.y + scale_ss * state(1);
                    }

                    // Special handling for warm start node
                    const auto color = (node->is_warm) ? LIME : Fade(WHITE, 0.4f);
                    const int lw = (node->is_warm) ? 3 : 1;

                    for (int i = 0; i < node->traj.length; i++) {
                        DrawLineEx(node_ray_traj[i], node_ray_traj[i + 1], lw, color);
                    }
                }

                Vector2 node_point;
                node_point.x = origin_offset.x + scale_ss * node->state(0);
                node_point.y = origin_offset.y + scale_ss * node->state(1);
                const auto color = (node->is_warm) ? LIME : Fade(WHITE, 0.6f);
                const int node_circle_size = (node->is_warm) ? 7 : 3;
                DrawCircleV(node_point, node_circle_size, color);
            }
        }

        // Draw the iLQR optimized trajectory
        {
            Vector2 ray_traj[planner_outputs.solution.traj.length + 1];

            for (int i = 0; i <= planner_outputs.solution.traj.length; i++) {
                const StateVector& state = planner_outputs.solution.traj.state_sequence.col(i);
                ray_traj[i].x = origin_offset.x + scale_ss * state(0);
                ray_traj[i].y = origin_offset.y + scale_ss * state(1);
            }

            for (int i = 0; i < planner_outputs.solution.traj.length; i++) {
                DrawLineEx(ray_traj[i], ray_traj[i + 1], 10, SKYBLUE);
            }
        }

        // Draw the RRT solution path
        // NOTE: drawing after iLQR trajectory so it is on top
        {
            for (const auto& node : planner_outputs.path) {
                Vector2 node_ray_traj[node->traj.length + 1];

                for (int i = 0; i <= node->traj.length; i++) {
                    const StateVector& state = node->traj.state_sequence.col(i);
                    node_ray_traj[i].x = origin_offset.x + scale_ss * state(0);
                    node_ray_traj[i].y = origin_offset.y + scale_ss * state(1);
                }

                for (int i = 0; i < node->traj.length; i++) {
                    DrawLineEx(node_ray_traj[i], node_ray_traj[i + 1], 5, PURPLE);
                }

                Vector2 node_point;
                node_point.x = origin_offset.x + scale_ss * node->state(0);
                node_point.y = origin_offset.y + scale_ss * node->state(1);

                DrawCircleV(node_point, 10, PURPLE);
            }
        }

        // Draw the start point and the goal point
        DrawCircleV(startPoint, 20, WHITE);
        DrawCircleV(goalPoint, 20, BLUE);

        if (paused) {
            // Show pause overlay
            DrawText("Paused", screenWidth / 2 - MeasureText("Paused", 20) / 2, screenHeight / 2 - 10, 20, BLACK);
        }

        // Draw the pause button
        DrawRectangleRec(pauseButton, GRAY);
        DrawText(paused ? "Resume" : "Pause", pauseButton.x + 10, pauseButton.y + 15, 20, RAYWHITE);

        // Draw the advance button
        DrawRectangleRec(advanceButton, GRAY);
        DrawText("Advance", advanceButton.x + 10, advanceButton.y + 15, 20, RAYWHITE);

        // Draw the timer info
        if (tree_exp_clock_time < 0) {
            tree_exp_clock_time = planner_outputs.tree_exp_clock_time;
        }
        if (path_ext_clock_time < 0) {
            path_ext_clock_time = planner_outputs.path_ext_clock_time;
        }
        if (traj_opt_clock_time < 0) {
            traj_opt_clock_time = planner_outputs.traj_opt_clock_time;
        }

        tree_exp_clock_time = static_cast<int>(lerp(planner_outputs.tree_exp_clock_time, tree_exp_clock_time, tree_exp_clock_momentum));
        path_ext_clock_time = static_cast<int>(lerp(planner_outputs.path_ext_clock_time, path_ext_clock_time, path_ext_clock_momentum));
        traj_opt_clock_time = static_cast<int>(lerp(planner_outputs.traj_opt_clock_time, traj_opt_clock_time, traj_opt_clock_momentum));
        gamp_upd_clock_time = static_cast<int>(1e6 / GetFPS());
        DrawTextEx(monoFont, TextFormat("Tree exp: %6d us", tree_exp_clock_time), (Vector2){10, 10}, 20, 1, WHITE);
        DrawTextEx(monoFont, TextFormat("Path ext: %6d us", path_ext_clock_time), (Vector2){10, 40}, 20, 1, WHITE);
        DrawTextEx(monoFont, TextFormat("Traj opt: %6d us", traj_opt_clock_time), (Vector2){10, 70}, 20, 1, WHITE);
        DrawTextEx(monoFont, TextFormat("Game upd: %6d us", gamp_upd_clock_time), (Vector2){10, 100}, 20, 1, LIGHTGRAY);

        // Draw the planner stats
        const double v_avg = planner_outputs.solution.traj.state_sequence.row(3).cwiseAbs().mean();
        const double soln_time_cost = timeCost(planner_outputs.solution);
        DrawTextEx(monoFont, TextFormat("Traj total time %4.2f s", planner_outputs.solution.total_time), (Vector2){10, 150}, 20, 1, YELLOW);
        DrawTextEx(monoFont, TextFormat("Traj  avg speed %4.2f m/s", v_avg), (Vector2){10, 180}, 20, 1, YELLOW);
        DrawTextEx(monoFont, TextFormat("Traj  time cost %4.2f", soln_time_cost), (Vector2){10, 210}, 20, 1, YELLOW);
        DrawTextEx(monoFont, TextFormat("Ratio rejected samples %3.0f%%", 100.0 * planner_outputs.tree.ratio_rejected_samples), (Vector2){10, 260}, 20, 1, ORANGE);

        // ---- Speed vs Time plot
        {
            const int plotX = 10;
            const int plotY = screenHeight - 150;
            const int plotWidth = 300;
            const int plotHeight = 100;

            // Draw border
            DrawRectangleLines(plotX, plotY, plotWidth, plotHeight, GRAY);
            DrawTextEx(monoFont, "Speed vs Time", (Vector2){plotX, plotY - 20}, 18, 1, WHITE);

            const Trajectory<traj_length_opt>& traj = planner_outputs.solution.traj;
            const double total_time = planner_outputs.solution.total_time;
            const double dt = total_time / traj.length;

            const double max_speed = traj.state_sequence.row(3).cwiseAbs().maxCoeff();

            // Plot the data
            for (int i = 0; i < traj.length; i++) {
                float t0 = i * dt;
                float t1 = (i + 1) * dt;
                float v0 = traj.state_sequence(3, i);
                float v1 = traj.state_sequence(3, i + 1);

                float x0 = plotX + plotWidth * (t0 / total_time);
                float x1 = plotX + plotWidth * (t1 / total_time);
                float y0 = plotY + plotHeight * (1.0f - (v0 / max_speed));
                float y1 = plotY + plotHeight * (1.0f - (v1 / max_speed));

                DrawLineEx((Vector2){x0, y0}, (Vector2){x1, y1}, 2.0f, SKYBLUE);
            }

            // Pre-opt traj
            for (int i = 0; i < planner_outputs.traj_pre_opt.length; i++) {
                float t0 = i * dt;
                float t1 = (i + 1) * dt;
                float v0 = planner_outputs.traj_pre_opt.state_sequence(3, i);
                float v1 = planner_outputs.traj_pre_opt.state_sequence(3, i + 1);

                float x0 = plotX + plotWidth * (t0 / total_time);
                float x1 = plotX + plotWidth * (t1 / total_time);
                float y0 = plotY + plotHeight * (1.0f - (v0 / max_speed));
                float y1 = plotY + plotHeight * (1.0f - (v1 / max_speed));

                DrawLineEx((Vector2){x0, y0}, (Vector2){x1, y1}, 1.0f, PURPLE);
            }
        }

        EndDrawing();
    }

    // Teardown
    UnloadFont(monoFont);
    CloseWindow();
    return 0;
}
