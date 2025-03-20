#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <indicators/cursor_control.hpp>
#include <indicators/progress_bar.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <random>
#include <sstream>
#include <string>
#include <optional>
#include <vector>
#include <algorithm>


#include "constants.h"
#include "printing.h"
#include "solver.h"
#include "space.h"
#include "steer_cubic.h"
#include "trajectory.h"

// Define a global random number generator
std::random_device rd;
std::mt19937 gen(rd());  // Mersenne Twister engine

static int steer_hifi_count = 0;
static int steer_lofi_count = 0;
static int sample_count = 0;
static int get_nearest_heuristic_count = 0;

// Static global file counter
static int file_count = 0;

static constexpr double max_lon_accel = 3.0;
static constexpr double max_lat_accel = 6.0;
static constexpr double max_curvature = 0.25;


// DEBUG
static int total_solve_time = 0.0;




double angularDifference(double angle1, double angle2) {
    // Compute the difference and shift by PI
    double diff = std::fmod(angle2 - angle1 + PI, 2 * PI);
    // Adjust if fmod returns a negative result
    if (diff < 0)
        diff += 2 * PI;
    // Shift back by subtracting PI to get a range of [-PI, PI]
    return diff - PI;
}

inline double lowerBoundTimeBetween(const StateVector& start, const StateVector& goal) {
    const double distance = (start.head(2) - goal.head(2)).norm();
    const double velocity = (start[3] + goal[3]) / 2;
    const double speed = std::max(std::abs(velocity), 1e-3);
    return distance / speed;
}

inline double estimateTimeBetween(const StateVector& start, const StateVector& goal) {
    const double distance = (start.head(2) - goal.head(2)).norm();
    const double velocity = (start[3] + goal[3]) / 2;
    const double speed = std::max(std::abs(velocity), 1e-3);
    // Heuristic
    const double angle_diff = std::abs(angularDifference(start[2], goal[2])) / PI;
    const double anle_diff_factor = angle_diff * angle_diff;
    return (1.0 + anle_diff_factor) * (distance / speed);
}

struct Position {
    const double x;
    const double y;
};

inline double distanceSquaredPositionState(const Position& position, const StateVector& state) {
    return (state.head(2) - Eigen::Vector2d(position.x, position.y)).squaredNorm();
}

inline double distancePositionState(const Position& position, const StateVector& state) {
    return (state.head(2) - Eigen::Vector2d(position.x, position.y)).norm();
}

inline double heuristicCost(const StateVector& source, const StateVector& target) {
    // const double distance = (source.head(2) - target.head(2)).norm();
    // // HACK clamping to something just smaller than PI to keep cost finite / avoid division by zero
    // // NOTE this also assumes no angle wrap around issues... use angular difference in the future
    // const double d_yaw = std::clamp(source[2] - target[2], -3.14, 3.14);
    // const double yaw_cost = 10 * ((1 / std::cos(0.5 * d_yaw)) - 1);
    // const double velocity_cost = 0.5 * std::abs(source[3] - target[3]);
    // const double cost = distance + yaw_cost + velocity_cost;
    // return cost;

    // Just use estimateTimeBetween
    return estimateTimeBetween(source, target);
}

inline double timeCost(const Solution& solution) {
    // TODO pull cost weight out to global for configurable constant
    static constexpr double cost_weight = 0.1 / traj_length;
    return solution.total_time * (1.0 + cost_weight * solution.cost);
}

inline Solution steerHifi(const StateVector& start, const StateVector& goal, const double total_time, const ActionSequence& action_sequence) {
    steer_hifi_count++;

    // Define the optimal control problem.
    const Problem problem = makeProblem(start, goal, total_time);

    // Solver settings.
    const SolverSettings settings = SolverSettings();
    // settings.validate();

    // Instantiate the solver.
    Solver solver = Solver(std::make_shared<Problem>(problem), std::make_shared<SolverSettings>(settings));

    // Solve the optimal control problem.
    return solver.solve(action_sequence);

    // Solve with a fixed number of iterations. solveFixedIters() is faster on a per-iteration basis due to fewer conditionals.
    // Empirically, I've seen a distribution with
    // p000 =  1
    // p005 =  3
    // p025 =  6
    // p050 =  8
    // p075 = 10
    // p090 = 12
    // p095 = 16
    // p100 = 40
    // For now this does not seem to help much since there is so much variation in the # iters needed, and running too few iterations causes lots of failed steerHifi solutions.
    // return solver.solveFixedIters(action_sequence, 20);
}

// inline std::tuple<Solution, double> steer(const StateVector& start, const StateVector& goal) {
//     const auto [action_sequence, total_time] = steer_cubic(start, goal);

//     // // TODO use cubic action_sequence
//     // const ActionSequence action_sequence = ActionSequence::Zero();
//     // const double total_time = estimateTimeBetween(start, goal);

//     Solution solution = steerHifi(start, goal, total_time, action_sequence);

//     // TODO refactor
//     // Soft loss
//     double cost = 0.0;
//     for (int i = 0; i < traj_length; ++i) {
//         const StateVector& state = solution.traj.stateAt(i);
//         const ActionVector& action = solution.traj.actionAt(i);
//         const double lon_accel = action[0];
//         const double lat_accel = action[1] * state[3] * state[3];
//         cost += lon_accel * lon_accel + lat_accel * lat_accel;
//     }
//     solution.cost = cost;

//     return std::make_tuple(solution, total_time);
// }

inline double softLoss(const Trajectory& traj) {
    double cost = 0.0;
    for (int i = 0; i < traj_length; ++i) {
        const StateVector& state = traj.stateAt(i);
        const ActionVector& action = traj.actionAt(i);
        const double lon_accel = action[0];
        const double lat_accel = action[1] * state[3] * state[3];
        cost += lon_accel * lon_accel + lat_accel * lat_accel;
    }
    return cost;
}

inline Solution steerLofi(const StateVector& start, const StateVector& goal, const double t_est) {
    steer_lofi_count++;

    const auto [action_sequence, total_time] = steer_cubic::steer_cubic(start, goal, t_est, max_lon_accel, max_lat_accel);
    // const auto [action_sequence, total_time] = steer_cubic::steer_quintic(start, goal, t_est, max_lon_accel, max_lat_accel);

    const double delta_time = total_time / traj_length;

    const Dynamics dynamics{delta_time};

    Trajectory traj;
    rolloutOpenLoop(action_sequence, start, dynamics, traj);
    Policy policy;
    // const double cost = problem.loss.totalValue(traj);
    const double cost = softLoss(traj);

    const SolveStatus solve_status = SolveStatus::kConverged;
    const SolveRecord solve_record;
    const Solution solution{traj, policy, cost, total_time, solve_status, solve_record};

    return solution;
}

struct Node {
    const StateVector state;
    const std::shared_ptr<Node> parent;
    const Trajectory traj;
    const double cost;
    const double total_time;
    const int ix;
};

inline double urand() {
    // TODO this is inefficient, creating a new distribution on every call
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
}

inline double urand(const double x_min, const double x_max) {
    // TODO this is inefficient, creating a new distribution on every call
    std::uniform_real_distribution<double> dist(x_min, x_max);
    return dist(gen);
}

inline StateVector sample(const StateVector& goal) {
    sample_count++;

    const bool near_goal = urand() > 0.90;
    // constexpr bool near_goal = false;

    const double xmin = near_goal ? goal(0) - 0.5 : 0.0;
    const double xmax = near_goal ? goal(0) + 0.5 : 20.0;
    const double ymin = near_goal ? goal(1) - 0.5 : -2.0;
    const double ymax = near_goal ? goal(1) + 0.5 : 2.0;
    const double yaw_min = near_goal ? goal(2) - 0.05 * PI : -0.25 * PI;
    const double yaw_max = near_goal ? goal(2) + 0.05 * PI : 0.25 * PI;
    const double vmin = near_goal ? goal(3) - 0.2 : 0.0;
    const double vmax = near_goal ? goal(3) + 0.2 : 5.0;

    const double x = urand(xmin, xmax);
    const double y = urand(ymin, ymax);
    const double yaw = urand(yaw_min, yaw_max);
    const double v = urand(vmin, vmax);
    return StateVector(x, y, yaw, v);
}

struct Obstacle {
    const Position center;
    const double radius;

    bool collidesWith(const StateVector& state) const {
        return distanceSquaredPositionState(center, state) < (radius * radius);
    }

    // TODO use continuous collision checking
    bool collidesWith(const Trajectory& traj) const {
        for (int stage_idx = 0; stage_idx <= traj_length; ++stage_idx) {
            if (collidesWith(traj.stateAt(stage_idx))) {
                return true;
            }
        }
        return false;
    }
};

inline bool checkStateBounds(const StateVector& state) {
    const bool x_ok = (0.0 <= state[0]) && (state[0] <= 20.0);
    const bool y_ok = (-2.0 <= state[1]) && (state[1] <= 2.0);
    const bool yaw_ok = std::abs(state[2]) <= 0.2 * PI;
    const bool v_ok = (0.0 <= state[3]) && (state[3] <= 10.0);
    return x_ok && y_ok && yaw_ok && v_ok;
}

inline bool checkActionBounds(const Solution& solution) {
    // TODO should be taken care of in steering function itself and represented in Solution
    for (int i = 0; i < traj_length; ++i) {
        const StateVector& state = solution.traj.stateAt(i);
        const ActionVector& action = solution.traj.actionAt(i);
        const double lon_accel = action[0];
        const double lat_accel = action[1] * state[3] * state[3];
        const double curvature = action[1];
        if ((std::abs(lon_accel) > max_lon_accel) || (std::abs(lat_accel) > max_lat_accel) || (std::abs(curvature) > max_curvature)) {
            return false;
        }
    }
    return true;
}

inline bool checkSteerSuccess(const Solution& solution, const StateVector& target) {
    const bool converged = solution.solve_status == SolveStatus::kConverged;

    const StateVector delta = solution.traj.stateTerminal() - target;
    const double dx = delta[0];
    const double dy = delta[1];
    const double dyaw = delta[2];
    const double dv = delta[3];

    // TODO use the TerminalStateParams. current numbers are set as 150% of the thresholds.
    constexpr double factor = 1.5;
    const bool dx_hit = std::abs(dx) < (factor * 0.01);
    const bool dy_hit = std::abs(dy) < (factor * 0.01);
    const bool dyaw_hit = std::abs(dyaw) < (factor * 0.02);
    const bool dv_hit = std::abs(dv) < (factor * 0.01);

    const bool target_hit = dx_hit && dy_hit && dyaw_hit && dv_hit;

    // // TODO heuristic
    // const bool cost_too_high = solution.cost > 1000.0;
    return converged && target_hit;
    // return converged && target_hit && !cost_too_high;
}

struct Tree {
    std::vector<std::shared_ptr<Node>> nodes;

    const std::shared_ptr<Node> getNearestHeuristic(const StateVector& target) const {
        get_nearest_heuristic_count++;

        // Make sure the tree is not empty.
        if (nodes.empty()) {
            std::cerr << "Called getNearestHeuristic on empty tree." << std::endl;
            std::abort();
        }

        double min_cost = std::numeric_limits<double>::max();
        std::shared_ptr<Node> nearest_node = nullptr;
        for (std::shared_ptr<Node> node : nodes) {
            const double cost = heuristicCost(node->state, target);
            if (cost < min_cost) {
                min_cost = cost;
                nearest_node = node;
            }
        }
        return nearest_node;
    }

    // const std::vector<std::shared_ptr<Node>> getNearHeuristic(const StateVector& target, const int k) const {
    //     // Create a vector of pairs: each pair contains the heuristic cost and the corresponding node.
    //     std::vector<std::pair<double, std::shared_ptr<Node>>> cost_node_pairs;
    //     for (const auto& node : nodes) {
    //         const double cost = heuristicCost(node->state, target);
    //         cost_node_pairs.emplace_back(cost, node);
    //     }
        
    //     // Sort the vector by the heuristic cost (lower cost means closer to the target).
    //     std::sort(cost_node_pairs.begin(), cost_node_pairs.end(),
    //               [](const auto& a, const auto& b) {
    //                   return a.first < b.first;
    //               });
        
    //     // Prepare the result vector and take the first k nodes (or all if there are fewer than k)
    //     std::vector<std::shared_ptr<Node>> near_nodes;
    //     const int n = std::min(k, static_cast<int>(cost_node_pairs.size()));
    //     for (int i = 0; i < n; ++i) {
    //         near_nodes.push_back(cost_node_pairs[i].second);
    //     }
        
    //     return near_nodes;
    // }
    
    
    const std::vector<std::shared_ptr<Node>> getNearHeuristic(const StateVector& target, const int k, const int m) const {
        // Create a vector of pairs: each pair holds the heuristic cost and the corresponding node.
        std::vector<std::pair<double, std::shared_ptr<Node>>> cost_node_pairs;
        for (const auto& node : nodes) {
            double cost = heuristicCost(node->state, target);  // assuming Node has a member 'state'
            cost_node_pairs.emplace_back(cost, node);
        }
        
        // Sort the pairs by their heuristic cost (lower cost indicates closer proximity).
        std::sort(cost_node_pairs.begin(), cost_node_pairs.end(),
                  [](const auto& a, const auto& b) {
                      return a.first < b.first;
                  });
        
        std::vector<std::shared_ptr<Node>> result;
        
        // Select the k nearest nodes (or all if fewer than k are available).
        int num_nearest = std::min(k, static_cast<int>(cost_node_pairs.size()));
        for (int i = 0; i < num_nearest; ++i) {
            result.push_back(cost_node_pairs[i].second);
        }
        
        // If additional m nodes are requested, select them randomly from the remaining nodes.
        if (m > 0 && cost_node_pairs.size() > static_cast<size_t>(num_nearest)) {
            std::vector<std::shared_ptr<Node>> remaining;
            // Collect all nodes that were not in the nearest k.
            for (size_t i = num_nearest; i < cost_node_pairs.size(); ++i) {
                remaining.push_back(cost_node_pairs[i].second);
            }
            // Shuffle the remaining nodes using a random engine.
            std::random_device rd;
            std::mt19937 gen(rd());
            std::shuffle(remaining.begin(), remaining.end(), gen);
            
            // Choose up to m nodes from the shuffled remaining nodes.
            int num_random = std::min(m, static_cast<int>(remaining.size()));
            for (int i = 0; i < num_random; ++i) {
                result.push_back(remaining[i]);
            }
        }
        
        return result;
    }
    

    const std::shared_ptr<Node> getNearestPrecise(const StateVector& target) const {
        // Make sure the tree is not empty.
        if (nodes.empty()) {
            std::cerr << "Called getNearest on empty tree." << std::endl;
            std::abort();
        }

        double min_cost = std::numeric_limits<double>::max();
        std::shared_ptr<Node> nearest_node = nullptr;

        for (std::shared_ptr<Node> node : nodes) {
            // Skip if too far.
            // NOTE: Having an accurate and tight estimate for short-circuiting makes a huge difference in performance,
            // since it lets us eliminate unnecessary calls to steerLofi(),
            // which is a moderately expensive subroutine otherwise called many times.
            const double cost_lower_bound = lowerBoundTimeBetween(node->state, target);
            if (cost_lower_bound > min_cost) {
                continue;
            }

            // Lofi steering
            const double t_est = estimateTimeBetween(node->state, target);
            const Solution solution = steerLofi(node->state, target, t_est);

            const double time_cost = timeCost(solution);
            const bool acceptable = time_cost < min_cost;
            // const bool acceptable = checkSteerSuccess(solution, target) && (time_cost < min_cost);
            if (!acceptable) {
                continue;
            }
            min_cost = time_cost;
            nearest_node = node;
        }

        if (nearest_node == nullptr) {
            nearest_node = getNearestHeuristic(target);
        }

        return nearest_node;
    }


    const std::shared_ptr<Node> getNearest(const StateVector& target) const {
        // Make sure the tree is not empty.
        if (nodes.empty()) {
            std::cerr << "Called getNearest on empty tree." << std::endl;
            std::abort();
        }

        double min_cost = std::numeric_limits<double>::max();
        std::shared_ptr<Node> nearest_node = nullptr;

        static constexpr int num_nodes_heuristic_neighbors = 10;
        static constexpr int num_nodes_random_neighbors = 10;            
        const std::vector<std::shared_ptr<Node>> nodes_selected = getNearHeuristic(target, num_nodes_heuristic_neighbors, num_nodes_random_neighbors);
        for (std::shared_ptr<Node> node : nodes_selected) {
            // Skip if too far.
            // NOTE: Having an accurate and tight estimate for short-circuiting makes a huge difference in performance,
            // since it lets us eliminate unnecessary calls to steerLofi(),
            // which is a moderately expensive subroutine otherwise called many times.
            const double cost_lower_bound = lowerBoundTimeBetween(node->state, target);
            if (cost_lower_bound > min_cost) {
                continue;
            }

            // Lofi steering
            const double t_est = estimateTimeBetween(node->state, target);
            const Solution solution = steerLofi(node->state, target, t_est);



            // // TRAIN
            // // Create a JSON object
            // nlohmann::json j;

            // // Populate the JSON
            // j["start"] = { node->state(0), node->state(1), node->state(2), node->state(3) };
            // j["goal"] = { target(0), target(1), target(2), target(3) };
            // j["total_time"] = total_time;

            
            // // Build the filename from fileCounter (8 chars, zero-padded)
            // std::ostringstream oss;
            // oss << std::setw(8) << std::setfill('0') << file_count++;
            // std::string filename = "data/" + oss.str() + ".json";

            // // Write JSON to the file
            // std::ofstream outFile(filename);
            // outFile << j.dump(4) << std::endl;
            // outFile.close();            



            const double time_cost = timeCost(solution);
            const bool acceptable = time_cost < min_cost;
            if (!acceptable) {
                continue;
            }
            min_cost = time_cost;
            nearest_node = node;
        }

        if (nearest_node == nullptr) {
            nearest_node = getNearestHeuristic(target);
        }

        return nearest_node;
    }

    void addNode(const std::shared_ptr<Node>& node) {
        nodes.push_back(node);
    }

    void grow(const StateVector& start, const StateVector& goal, const int num_nodes, const std::optional<Solution>& warm = std::nullopt) {
        const Position center{10.0, 0.0};
        const double radius = 1.0;
        const Obstacle obstacle{center, radius};

        // Create root node and add to the tree.
        const Trajectory root_traj;
        const Node root_node = Node(start, nullptr, root_traj, 0.0, 0.0, 0);
        const std::shared_ptr<Node> root_node_ptr = std::make_shared<Node>(root_node);
        addNode(root_node_ptr);


        int next_node_ix = 1;
        
        // Add warm-start node.
        if (warm) {
            const Node warm_node = Node(warm->traj.stateTerminal(), root_node_ptr, warm->traj, warm->cost, warm->total_time, next_node_ix);
            addNode(std::make_shared<Node>(warm_node));
            next_node_ix++;
        }

        // indicators::show_console_cursor(false);
        // indicators::ProgressBar bar{
        //     indicators::option::BarWidth{50},
        //     indicators::option::Start{"["},
        //     indicators::option::Fill{"="},
        //     indicators::option::Lead{">"},
        //     indicators::option::Remainder{" "},
        //     indicators::option::End{"]"},
        //     indicators::option::PostfixText{"Growing tree"},
        //     indicators::option::ShowPercentage{true},
        //     indicators::option::ShowElapsedTime{true},
        //     indicators::option::ShowRemainingTime{true}};

        for (int node_ix = 1; node_ix <= num_nodes; ++node_ix) {
            // const auto clock_start = std::chrono::high_resolution_clock::now();

            bool node_added = false;
            while (!node_added) {
                // Sample a new state.
                StateVector state = sample(goal);

                // Terminate iteration early if sampled state is in collision.
                if (obstacle.collidesWith(state)) {
                    continue;
                }

                // Set the parent as the nearest neighbor to the state.
                std::shared_ptr<Node> parent = getNearest(state);
                // // // DEBUG - this is currently not a good proxy metric, so can result in bad trees
                // std::shared_ptr<Node> parent = getNearestHeuristic(state);

                // Estimate time.
                const double t_est = estimateTimeBetween(parent->state, state);
                // Skip if too far.
                // NOTE this is somewhat optional, but empirically seems to help reduce wasting effort on hard-to-connect states.
                // It also helps keep the fixed number of steps in Trajectory responsible for a small time-delta, which helps limit time discretization inaccuracies.
                if (t_est > 2.0) {
                    continue;
                }

                // Steer from parent to child.
                Solution solution = steerLofi(parent->state, state, t_est);

                // Project actions onto box bounds, and reproject state sample as the terminal state in the trajectory
                // TODO put this in a function
                for (int i = 0; i < traj_length; ++i) {
                    const StateVector& state = solution.traj.stateAt(i);
                    const ActionVector& action = solution.traj.actionAt(i);
                    const double v_sq = state[3] * state[3];
                    double lon_accel = action[0];
                    double lat_accel = action[1] * v_sq;
                    double curvature = action[1];
                    const double dyn_max_curvature = std::min(max_curvature, max_lat_accel / (v_sq + 1e-12));
                    lon_accel = std::clamp(lon_accel, -max_lon_accel, max_lon_accel);
                    curvature = std::clamp(curvature, -dyn_max_curvature, dyn_max_curvature);
                    const ActionVector new_action{lon_accel, curvature};
                    solution.traj.setActionAt(i, new_action);
                }
                const double delta_time = solution.total_time / traj_length;
                const Dynamics dynamics{delta_time};
                Trajectory traj;
                rolloutOpenLoop(solution.traj.action_sequence, parent->state, dynamics, traj);
                solution.traj = traj;
                solution.cost = softLoss(solution.traj);
                double time_cost = timeCost(solution);

                // NOTE: strictly speaking, we could still be violating the box bounds here, as they are state-dependent and we changed the trajectory.
                // But it is unlikely to happen in practice, and it is not worth re-checking over and over again.
                const StateVector& state_after_action_projection = traj.stateTerminal();

                // // If action projection was too aggressive, skip
                // if (heuristicCost(state_after_action_projection, state) > 1.0) {
                //     continue;
                // }

                // Update state
                state = state_after_action_projection;

                // Terminate iteration early if sampled state is in collision.
                // Have to check again after projection.
                if (obstacle.collidesWith(state)) {
                    continue;
                }

                // TODO this is ~optional. should be paired with checks on the entire trajectory to be in the free space too.
                // NOTE: this can lead to 10% longer solve times!!!
                // if action projection caused state to exit free space, continue
                if (!checkStateBounds(state)) {
                    continue;
                }

                // Terminate iteration early if trajectory is in collision.
                // NOTE: this may not be necessary (or even helpful) if hifi steering is used, especially if hifi steering has obstacle avoidance capability
                if (obstacle.collidesWith(solution.traj)) {
                    continue;
                }



                // // final hifi steer
                // solution = steerHifi(parent->state, state, total_time, solution.traj.action_sequence);

                // solution.cost = softLoss(solution.traj);
                // time_cost = timeCost(solution);

                // // Terminate iteration early if steering failed.
                // if (!checkSteerSuccess(solution, state)) {
                //     continue;
                // }

                // // Terminate iteration early if trajectory is in collision.
                // // Have to check again after hifi steer
                // if (obstacle.collidesWith(solution.traj)) {
                //     continue;
                // }




                // Create node from sampled state and add to the tree.
                const Node node{state, parent, solution.traj, time_cost, solution.total_time, next_node_ix};
                addNode(std::make_shared<Node>(node));
                next_node_ix++;
                node_added = true;
                // // DEBUG
                // std::cout << "iLQR iters = " << solution.solve_record.iters << std::endl;
            }

            // // DEBUG
            // const auto clock_stop = std::chrono::high_resolution_clock::now();
            // const auto clock_duration = std::chrono::duration_cast<std::chrono::microseconds>(clock_stop - clock_start).count();
            // total_solve_time += clock_duration;

            // // Update progress bar.
            // const double progress = (static_cast<double>(node_ix) / static_cast<double>(num_nodes)) * 100.0;
            // bar.set_progress(static_cast<size_t>(progress));
            // std::stringstream ss;
            // ss << "Added node " << node_ix << " at " << state2str(nodes.back()->state) << " with cost=" << val2str(nodes.back()->cost) << " [" << sample_count << " sample(), " << steer_hifi_count << " steerHifi(), " << steer_lofi_count << " steerLofi()" << "]";
            // bar.set_option(indicators::option::PostfixText{ss.str()});
        }

        // bar.mark_as_completed();
        // std::cout << "Total solve time = " << total_solve_time / 1000 << " milliseconds" << std::endl;
    }

    // Dump the tree to a JSON file.
    void dumpToFile(const std::string& filename) const {
        nlohmann::json j;
        j["nodes"] = nlohmann::json::array();

        for (const auto& node : nodes) {
            nlohmann::json node_json;
            node_json["ix"] = node->ix;

            // Dump the state vector as an array.
            node_json["state"] = {
                node->state(0),
                node->state(1),
                node->state(2),
                node->state(3)};

            // Dump parent's index (or -1 if there is no parent).
            node_json["parent_ix"] = node->parent ? node->parent->ix : -1;
            node_json["total_time"] = node->total_time;
            node_json["cost"] = node->cost;

            // Dump the trajectory's state sequence.
            nlohmann::json state_seq = nlohmann::json::array();
            for (int col = 0; col < node->traj.state_sequence.cols(); ++col) {
                nlohmann::json state;
                for (int row = 0; row < node->traj.state_sequence.rows(); ++row) {
                    state.push_back(node->traj.state_sequence(row, col));
                }
                state_seq.push_back(state);
            }
            node_json["state_sequence"] = state_seq;

            // Dump the trajectory's action sequence.
            nlohmann::json action_seq = nlohmann::json::array();
            for (int col = 0; col < node->traj.action_sequence.cols(); ++col) {
                nlohmann::json action;
                for (int row = 0; row < node->traj.action_sequence.rows(); ++row) {
                    action.push_back(node->traj.action_sequence(row, col));
                }
                action_seq.push_back(action);
            }
            node_json["action_sequence"] = action_seq;

            j["nodes"].push_back(node_json);
        }

        // Write the JSON data to a file with pretty printing.
        std::ofstream file(filename);
        if (file.is_open()) {
            file << std::setw(4) << j;
            file.close();
        }
    }
};
