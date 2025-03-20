#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <optional>
#include <unordered_map>



#include "raylib.h"



#include "dynamics.h"
#include "problem.h"
#include "rollout.h"
#include "solver.h"
#include "solver_settings.h"
#include "space.h"
#include "util.h"
#include "printing.h"


#include "rrt.h"



void dump(const Trajectory& traj, const StateVector terminal_state_target) {
    // Output file name with terminal state values
    // std::string terminal_state_str = state2str(terminal_state_target);
    std::string terminal_state_str = "traj";
    std::string filename = "data/state/" + terminal_state_str + ".csv";
    // Open a file in write mode
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Write data to CSV
    for (int i = 0; i < traj_length + 1; ++i) {
        const StateVector& state = traj.stateAt(i);
        ofs << state(0) << ","
            << state(1) << ","
            << state(2) << ","
            << state(3) << "\n";
    }

    ofs.close();
    std::cout << "Wrote to " << filename << std::endl;

    std::string filename2 = "data/action/" + terminal_state_str + ".csv";
    std::ofstream ofs2(filename2);
    if (!ofs2.is_open()) {
        std::cerr << "Error opening file: " << filename2 << std::endl;
        return;
    }
    for (int i = 0; i < traj_length; ++i) {
        const ActionVector& action = traj.actionAt(i);
        ofs2 << action(0) << ","
             << action(1) << "\n";
    }

    ofs2.close();
    std::cout << "Wrote to " << filename2 << std::endl;
}

// int routine(const StateVector initial_state, const StateVector terminal_state_target) {
//     // Start timer
//     const auto start = std::chrono::high_resolution_clock::now();

//     // Define the optimal control problem.
//     const Problem problem = makeProblem(initial_state, terminal_state_target, total_time);

//     // Initialize the action sequence.
//     // TODO Use a more informative initial guess.
//     ActionSequence action_sequence = ActionSequence::Zero();

//     // Solver settings.
//     const SolverSettings settings = SolverSettings();
//     settings.validate();

//     // Instantiate the solver.
//     Solver solver = Solver(std::make_shared<Problem>(problem), std::make_shared<SolverSettings>(settings));

//     // Solve the optimal control problem.
//     const auto solve_start = std::chrono::high_resolution_clock::now();
//     const Solution solution = solver.solve(action_sequence);
//     // const Solution solution = solver.solveFixedIters(action_sequence);
//     const auto solve_end = std::chrono::high_resolution_clock::now();

//     const Trajectory& traj = solution.traj;
//     const double cost = solution.cost;
//     const SolveStatus solve_status = solution.solve_status;
//     const SolveRecord solve_record = solution.solve_record;

//     // Stop timer
//     const auto end = std::chrono::high_resolution_clock::now();
//     const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
//     const auto solve_duration = std::chrono::duration_cast<std::chrono::microseconds>(solve_end - solve_start).count();

//     std::cout << "Steering from " << state2str(initial_state) << " to " << state2str(terminal_state_target) << std::endl;
//     std::cout << "iters     = " << solve_record.iters << std::endl;
//     std::cout << "rollouts  = " << solve_record.rollouts << std::endl;

//     // problem.loss.soft_params
//     // SoftParams soft_params;
//     // VehicleLimits vehicle_limits;
//     // VehicleLimitsParams vehicle_limits_params;
//     // TerminalStateParams terminal_state_params;
//     const StateVector delta = traj.stateTerminal() - terminal_state_target;
//     const double dx = delta[0];
//     const double dy = delta[1];
//     const double dyaw = delta[2];
//     const double dv = delta[3];

//     // TODO use the TerminalStateParams. current numbers are set as 150% of the thresholds.
//     const double factor = 1.5;
//     const bool dxHit = std::abs(dx) < (factor * 0.01);
//     const bool dyHit = std::abs(dy) < (factor * 0.01);
//     const bool dyawHit = std::abs(dyaw) < (factor * 0.02);
//     const bool dvHit = std::abs(dv) < (factor * 0.01);
//     const bool terminalStateHit = dxHit && dyHit && dyawHit && dvHit;

//     // Check the solution status.
//     // Only dump results if optimization succeeded and terminal state was hit.
//     if (solve_status != SolveStatus::kConverged) {
//         std::cout << " Optimization failed with status: " << toString(solve_status) << std::endl;
//     } else if (!terminalStateHit) {
//         std::cout << "    Failed to hit terminal state." << std::endl;
//     } else {
//         std::cout << "Optimization succeeded with cost: " << cost << std::endl;
//         // Dump the trajectory out to file.
//         dump(traj, terminal_state_target);
//     }

//     const int it_per_ms = (1000 * solve_record.iters) / duration;
//     const int solve_it_per_ms = (1000 * solve_record.iters) / solve_duration;

//     std::cout << "Steered in: " << duration << " microseconds (" << it_per_ms << " it/ms)" << std::endl;
//     std::cout << " Solved in: " << solve_duration << " microseconds (" << solve_it_per_ms << " it/ms)" << std::endl;
//     std::cout << std::endl;

//     return it_per_ms;
// }

// int main(int argc, char* argv[]) {
//     // // DEBUG - single
//     // const double v0 = 10.0;
//     // const double x = 10.0;
//     // const double y = 1.0;
//     // const double yaw = 0.0;
//     // const double v = 10.0;
//     // const StateVector initial_state(0.0, 0.0, 0.0, v0);
//     // const StateVector terminal_state_target(x, y, yaw, v);
//     // routine(initial_state, terminal_state_target);

//     int total_it_per_ms = 0;
//     int count = 0;

//     // DEBUG - simple grid
//     const double v0 = 10.0;
//     const double yaw = 0.0;
//     const double v = v0;

//     // spatial resolution, meter
//     const double res = 0.1;
//     const int nx = round(2.0 / res);
//     const int ny = round(2.0 / res);
//     const int nx2 = nx / 2;

//     for (int i = -nx2; i <= nx2; ++i) {
//         for (int j = 0; j <= ny; ++j) {
//             const double x = 10.0 + res * i;
//             const double y = res * j;
//             const StateVector initial_state(0.0, 0.0, 0.0, v0);
//             const StateVector terminal_state_target(x, y, yaw, v);
//             const int it_per_ms = routine(initial_state, terminal_state_target);
//             total_it_per_ms += it_per_ms;
//             count += 1;
//         }
//     }

//     const int avg_it_per_ms = total_it_per_ms / count;
//     std::cout << "--------" << std::endl;
//     std::cout << "Average solve rate = " << avg_it_per_ms << " it/ms" << std::endl;

//     // const double dx_min = -2.0;
//     // const double dx_max = 2.0;
//     // const double dy_min = 0.0;
//     // const double dy_max = 3.0;
//     // const double yaw_min = deg2rad(-30.0);
//     // const double yaw_max = deg2rad(30.0);
//     // const double dv_min = -3.0;
//     // const double dv_max = 3.0;

//     // // const double d_dx = 1.0;
//     // // const double d_dy = 1.0;
//     // // const double d_yaw = deg2rad(10.0);
//     // // const double d_dv = 1.0;

//     // const double d_dx = 2.0;
//     // const double d_dy = 0.5;
//     // const double d_yaw = deg2rad(15.0);
//     // const double d_dv = 0.5;

//     // const double dx_min = -2.0;
//     // const double dx_max = 2.0;
//     // const double dy_min = 0.0;
//     // const double dy_max = 3.0;
//     // const double yaw_min = deg2rad(-5.0);
//     // const double yaw_max = deg2rad(5.0);
//     // const double dv_min = -2.0;
//     // const double dv_max = 2.0;

//     // const double d_dx = 0.5;
//     // const double d_dy = 0.5;
//     // const double d_yaw = deg2rad(2.5);
//     // const double d_dv = 0.5;

//     // const int nx = std::round((dx_max - dx_min) / d_dx);
//     // const int ny = std::round((dy_max - dy_min) / d_dy);
//     // const int nyaw = std::round((yaw_max - yaw_min) / d_yaw);
//     // const int nv = std::round((dv_max - dv_min) / d_dv);

//     // const double v0 = 10.0;

//     // // Iterate over grid.
//     // for (int i = 0; i <= nx; ++i) {
//     //     const double tx = static_cast<float>(i) / static_cast<float>(nx);
//     //     const double dx = lerp(dx_min, dx_max, tx);
//     //     for (int j = 0; j <= ny; ++j) {
//     //         const double ty = static_cast<float>(j) / static_cast<float>(ny);
//     //         const double dy = lerp(dy_min, dy_max, ty);
//     //         for (int k = 0; k <= nyaw; ++k) {
//     //             const double tyaw = static_cast<float>(k) / static_cast<float>(nyaw);
//     //             const double yaw = lerp(yaw_min, yaw_max, tyaw);
//     //             for (int l = 0; l <= nv; ++l) {
//     //                 const double tv = static_cast<float>(l) / static_cast<float>(nv);
//     //                 const double dv = lerp(dv_min, dv_max, tv);

//     //                 const double v = v0 + dv;
//     //                 const double v_avg = 0.5 * (v0 + v);

//     //                 const double x_nom = total_time * v_avg;
//     //                 const double y_nom = 0.0;
//     //                 const double x = x_nom + dx;
//     //                 const double y = y_nom + dy;

//     //                 const StateVector initial_state(0.0, 0.0, 0.0, v0);
//     //                 const StateVector terminal_state_target(x, y, yaw, v);
//     //                 routine(initial_state, terminal_state_target);
//     //             }
//     //         }
//     //     }
//     // }
// }









// int main(int argc, char* argv[]) {
//     const StateVector start{0.0, 0.0, 0.0, 0.0};
//     const StateVector goal{20.0, 0.0, 0.0, 0.0};


//     // ---- RRT

//     const int num_nodes = 200;
//     Tree tree;
//     tree.grow(start, goal, num_nodes);
    
//     const std::shared_ptr<Node> node_nearest_goal = tree.getNearest(goal);

//     // Reconstruct the path by traversing parent pointers.
//     std::vector<std::shared_ptr<Node>> path;
//     std::shared_ptr<Node> node = node_nearest_goal;
//     while (node->parent != nullptr) {
//         path.push_back(node);
//         node = node->parent;
//     }
//     path.push_back(node);  // root
//     std::reverse(path.begin(), path.end());

//     // Print the path.
//     std::cout << "Path:" << std::endl;
//     for (const std::shared_ptr<Node>& node : path) {
//         std::cout << state2str(node->state) << std::endl;
//     }

//     // dump the tree to data file, including trajectory & IDs
//     tree.dumpToFile("tree.json");


//     std::vector<double> ts;
//     std::vector<double> accels;
//     std::vector<double> curvatures;
//     {
//         double t = 0.0;
//         for (const std::shared_ptr<Node>& node : path) {
//             const double dt = node->total_time / traj_length;
//             for (int i = 0; i < traj_length; ++i) {
//                 const ActionVector& action = node->traj.actionAt(i);
//                 ts.push_back(t);
//                 accels.push_back(action(0));
//                 curvatures.push_back(action(1));
//                 t += dt;
//             }
//         }
//     }

//     double total_time = 0.0;
//     for (const std::shared_ptr<Node>& node : path) {
//         total_time += node->total_time;
//     }
//     const double dt = total_time / traj_length;
    
//     ActionSequence action_sequence;
//     {
//         for (int i = 0; i < traj_length; ++i) {
//             const double t = i * dt;
//             const double accel = steer_cubic::interp1(ts, accels, t);
//             const double curvature = steer_cubic::interp1(ts, curvatures, t);
//             const ActionVector action{accel, curvature};
//             action_sequence.col(i) = action;
//         }
//     }



//     // ---- Trajectory optimization

//     // Start timer
//     const auto clock_start = std::chrono::high_resolution_clock::now();

//     // Define the optimal control problem.
//     const Problem problem = makeProblem(start, goal, total_time);



//     // Initialize the action sequence.

//     // TODO Use a more informative initial guess using the RRT solution.
//     // Use the total_time from the RRT solution path and interpolate actions onto uniform time grid over trajectory_length steps

//     // ActionSequence action_sequence = ActionSequence::Zero();
//     // ActionSequence action_sequence = 0.1 * ActionSequence::Random();



//     // Solver settings.
//     const SolverSettings settings = SolverSettings();
//     settings.validate();

//     // Instantiate the solver.
//     Solver solver = Solver(std::make_shared<Problem>(problem), std::make_shared<SolverSettings>(settings));

//     // Solve the optimal control problem.
//     const auto solve_start = std::chrono::high_resolution_clock::now();
//     const Solution solution = solver.solve(action_sequence);
//     // const Solution solution = solver.solveFixedIters(action_sequence);
//     const auto solve_end = std::chrono::high_resolution_clock::now();

//     const Trajectory& traj = solution.traj;
//     const double cost = solution.cost;
//     const SolveStatus solve_status = solution.solve_status;
//     const SolveRecord solve_record = solution.solve_record;

//     // Stop timer
//     const auto clock_end = std::chrono::high_resolution_clock::now();
//     const auto clock_duration = std::chrono::duration_cast<std::chrono::milliseconds>(clock_end - clock_start).count();
//     const auto solve_duration = std::chrono::duration_cast<std::chrono::milliseconds>(solve_end - solve_start).count();

//     std::cout << "Steering from " << state2str(start) << " to " << state2str(goal) << std::endl;
//     std::cout << "iters     = " << solve_record.iters << std::endl;
//     std::cout << "rollouts  = " << solve_record.rollouts << std::endl;

//     // problem.loss.soft_params
//     // SoftParams soft_params;
//     // VehicleLimits vehicle_limits;
//     // VehicleLimitsParams vehicle_limits_params;
//     // TerminalStateParams terminal_state_params;
//     const StateVector delta = traj.stateTerminal() - goal;
//     const double dx = delta[0];
//     const double dy = delta[1];
//     const double dyaw = delta[2];
//     const double dv = delta[3];

//     // TODO use the TerminalStateParams. current numbers are set as 150% of the thresholds.
//     const double factor = 1.5;
//     const bool dxHit = std::abs(dx) < (factor * 0.01);
//     const bool dyHit = std::abs(dy) < (factor * 0.01);
//     const bool dyawHit = std::abs(dyaw) < (factor * 0.02);
//     const bool dvHit = std::abs(dv) < (factor * 0.01);
//     const bool terminalStateHit = dxHit && dyHit && dyawHit && dvHit;

//     // Check the solution status.
//     // Only dump results if optimization succeeded and terminal state was hit.
//     if (solve_status != SolveStatus::kConverged) {
//         std::cout << " Optimization failed with status: " << toString(solve_status) << std::endl;
//     } else if (!terminalStateHit) {
//         std::cout << "    Failed to hit terminal state." << std::endl;
//     } else {
//         std::cout << "Optimization succeeded with cost: " << cost << std::endl;
//     }

//     // Dump the trajectory out to file.
//     dump(traj, goal);

//     const int it_per_ms = (1000 * solve_record.iters) / clock_duration;
//     const int solve_it_per_ms = (1000 * solve_record.iters) / solve_duration;

//     std::cout << "Steered in: " << clock_duration << " milliseconds" << std::endl;
//     std::cout << " Solved in: " << solve_duration << " milliseconds" << std::endl;
//     std::cout << std::endl;


// }



struct PlannerOutputs {
    Tree tree;
    std::vector<std::shared_ptr<Node>> path;
    Solution solution;
};


PlannerOutputs plan(const StateVector& start, const StateVector& goal, const std::optional<Solution>& warm = std::nullopt) {
    // ---- RRT

    static constexpr int num_nodes = 300;
    Tree tree;
    // tree.grow(start, goal, num_nodes, warm);
    tree.grow(start, goal, num_nodes);
    
    const std::shared_ptr<Node> node_nearest_goal = tree.getNearestPrecise(goal);

    // Reconstruct the path by traversing parent pointers.
    std::vector<std::shared_ptr<Node>> path;
    std::shared_ptr<Node> node = node_nearest_goal;
    while (node->parent != nullptr) {
        path.push_back(node);
        node = node->parent;
    }
    path.push_back(node);  // root
    std::reverse(path.begin(), path.end());
    
    std::vector<double> ts;
    std::vector<double> accels;
    std::vector<double> curvatures;
    {
        double t = 0.0;
        for (const std::shared_ptr<Node>& node : path) {
            const double dt = node->total_time / traj_length;
            for (int i = 0; i < traj_length; ++i) {
                const ActionVector& action = node->traj.actionAt(i);
                ts.push_back(t);
                accels.push_back(action(0));
                curvatures.push_back(action(1));
                t += dt;
            }
        }
    }

    double total_time = 0.0;
    for (const std::shared_ptr<Node>& node : path) {
        total_time += node->total_time;
    }
    const double dt = total_time / traj_length;
    
    ActionSequence action_sequence;
    {
        for (int i = 0; i < traj_length; ++i) {
            const double t = i * dt;
            const double accel = steer_cubic::interp1(ts, accels, t);
            const double curvature = steer_cubic::interp1(ts, curvatures, t);
            const ActionVector action{accel, curvature};
            action_sequence.col(i) = action;
        }
    }


    // ---- Trajectory optimization

    // Start timer
    const auto clock_start = std::chrono::high_resolution_clock::now();

    // Define the optimal control problem.
    const Problem problem = makeProblem(start, goal, total_time);

    // Solver settings.
    const SolverSettings settings = SolverSettings();
    settings.validate();

    // Instantiate the solver.
    Solver solver = Solver(std::make_shared<Problem>(problem), std::make_shared<SolverSettings>(settings));

    // Solve the optimal control problem.
    const Solution solution = solver.solve(action_sequence);

    return {tree, path, solution};
}




int main() {
    static constexpr double origin_ss = 200;
    static constexpr double scale_ss = 80;

    // Initialization
    const int screenWidth = 2 * origin_ss + 20 * scale_ss;
    const int screenHeight = 2 * origin_ss;
    InitWindow(screenWidth, screenHeight, "Motion Planner");


    Rectangle pauseButton = { screenWidth - 110, 10, 100, 50 };
    bool paused = false;  // Game pause state

    // Define a fixed start point and an initial goal point
    Vector2 startPoint = {origin_ss, origin_ss};
    Vector2 goalPoint = {origin_ss + 20.0 * scale_ss, origin_ss + 0.0 * scale_ss};

    const StateVector start{0.0, 0.0, 0.0, 0.0};
    const StateVector goal{(goalPoint.x - origin_ss) / scale_ss, (goalPoint.y - origin_ss) / scale_ss, 0.0, 0.0};

    // Initial plan
    PlannerOutputs planner_outputs = plan(start, goal);


    static constexpr int gameFPS = 100;
    static constexpr int planFPS = 10;

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

        // Check if the left mouse button is clicked within the pause button's bounds
        bool pauseStateToggled = false;
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(mousePoint, pauseButton))
        {
            pauseStateToggled = true;
            paused = !paused;  // Toggle pause state
        }

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) && !CheckCollisionPointRec(mousePoint, pauseButton)) {
            goalPoint = mousePoint;
        }

        // Update game state
        const bool do_update_game = !paused && enough_time_elapsed;
        if (do_update_game) {
            timeAccumulator = 0.0f;
            
            const StateVector start{0.0, 0.0, 0.0, 0.0};
            const StateVector goal{(goalPoint.x - origin_ss) / scale_ss, (goalPoint.y - origin_ss) / scale_ss, 0.0, 0.0};

            // planner_outputs = plan(start, goal, planner_outputs.solution);
            planner_outputs = plan(start, goal);
        }

        // Draw everything
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Draw the obstacle
        Vector2 obstacleCenterSS = {origin_ss + 10.0 * scale_ss, origin_ss + 0.0 * scale_ss};
        double obstacleRadiusSS = 1.0 * scale_ss;
        DrawCircleV(obstacleCenterSS, obstacleRadiusSS, RED);

        // Draw the tree
        {
            for (const auto& node : planner_outputs.tree.nodes) {
                Vector2 node_ray_traj[traj_length + 1];

                for (int i = 0; i <= traj_length; i++) {
                    const StateVector& state = node->traj.state_sequence.col(i);
                    node_ray_traj[i].x = origin_ss + scale_ss * state(0);
                    node_ray_traj[i].y = origin_ss + scale_ss * state(1);
                }

                for (int i = 0; i < traj_length; i++) {
                    DrawLineV(node_ray_traj[i], node_ray_traj[i + 1], Fade(BLACK, 0.3f));
                }
                
                Vector2 node_point; 
                node_point.x = origin_ss + scale_ss * node->state(0);
                node_point.y = origin_ss + scale_ss * node->state(1);
                DrawCircleV(node_point, 5, Fade(BLACK, 0.4f));
            }
        }
        
        
        // Draw the RRT solution path
        {
            for (const auto& node : planner_outputs.path) {
                Vector2 node_ray_traj[traj_length + 1];

                for (int i = 0; i <= traj_length; i++) {
                    const StateVector& state = node->traj.state_sequence.col(i);
                    node_ray_traj[i].x = origin_ss + scale_ss * state(0);
                    node_ray_traj[i].y = origin_ss + scale_ss * state(1);
                }

                for (int i = 0; i < traj_length; i++) {
                    DrawLineEx(node_ray_traj[i], node_ray_traj[i + 1], 5, ORANGE);
                }
            }
        }
        
        // Draw the iLQR optimized trajectory
        {        
            Vector2 ray_traj[traj_length + 1];

            for (int i = 0; i <= traj_length; i++) {
                const StateVector& state = planner_outputs.solution.traj.state_sequence.col(i);
                ray_traj[i].x = origin_ss + scale_ss * state(0);
                ray_traj[i].y = origin_ss + scale_ss * state(1);
            }

            for (int i = 0; i < traj_length; i++) {
                DrawLineEx(ray_traj[i], ray_traj[i + 1], 5, BLUE);
            }
        }


        // Draw the start point (blue) and the goal point (green)
        DrawCircleV(startPoint, 10, BLACK);
        DrawCircleV(goalPoint, 10, BLUE);
        
        if (paused)
        {
            // Show pause overlay
            DrawText("Paused", screenWidth/2 - MeasureText("Paused", 20)/2, screenHeight/2 - 10, 20, BLACK);
        }

        // Draw the pause button
        DrawRectangleRec(pauseButton, GRAY);
        DrawText(paused ? "Resume" : "Pause", pauseButton.x + 10, pauseButton.y + 15, 20, RAYWHITE);


        EndDrawing();
    }

    // De-Initialization
    CloseWindow();
    return 0;
}
