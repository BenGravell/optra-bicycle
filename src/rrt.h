#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
// #include <indicators/cursor_control.hpp>
// #include <indicators/progress_bar.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "constants.h"
#include "printing.h"
#include "solver.h"
#include "space.h"
#include "steer.h"
#include "trajectory.h"

// Define a global random number generator
std::random_device rd;
std::mt19937 gen(rd());  // Mersenne Twister engine

static constexpr double x_max = 20.0;
static constexpr double x_min = 0.0;

static constexpr double y_max = 2.0;
static constexpr double y_min = -y_max;

static constexpr double yaw_max = 0.5 * PI;
static constexpr double yaw_min = -yaw_max;

static constexpr double v_max = 5.0;
static constexpr double v_min = 0.0;

// Number of neighbor nodes to consider in the getNearest function
static constexpr int num_nodes_heuristic_neighbors = 4;
static constexpr int num_nodes_random_neighbors = 4;

// Max estimated trajectory duration allowed between samples
static constexpr double max_t_est_sample = 0.5 * traj_length_steer;  // seconds
// Factor used to iteratively pull samples closer to nearest parent
static constexpr double attract_factor = 0.6;  // should be in (0, 1), closer to 1 means less aggressive moves (precise search, more iterations)

// TODO move somewhere more generic
double angularDifference(double angle1, double angle2) {
    // Compute the difference and shift by PI
    double diff = std::fmod(angle2 - angle1 + PI, 2 * PI);
    // Adjust if fmod returns a negative result
    if (diff < 0) {
        diff += 2 * PI;
    }
    // Shift back by subtracting PI to get a range of [-PI, PI]
    return diff - PI;
}

inline double estimateTimeBetween(const StateVector& start, const StateVector& goal) {
    const double distance = (start.head(2) - goal.head(2)).norm();
    // Heuristic
    const double angle_diff = std::abs(angularDifference(start[2], goal[2])) / PI;  // in [0, 1]
    const double anle_diff_factor = angle_diff * angle_diff;

    const double d = (1.0 + anle_diff_factor) * distance;

    // Speed
    const double v1 = start[3];
    const double v2 = goal[3];
    const double v_mid = std::abs(0.5 * (v1 + v2));

    // Use a lesser amount of acceleration than the max.
    const double a_factor = 0.3;
    const double a_nom = a_factor * max_lon_accel;
    const double ad = a_nom * d;
    // Constant acceleration to the midpoint.
    const double vm1 = std::sqrt(v1 * v1 + ad);
    const double vm2 = std::sqrt(v2 * v2 + ad);
    const double vm = std::abs(0.5 * (vm1 + vm2));

    const double v_nom = 0.5 * (v_mid + vm);
    const double speed = std::max(v_nom, 0.01);

    return d / speed;
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

template <int N>
inline double timeCost(const Solution<N>& solution) {
    // NOTE: solution.cost is the sum of all stage costs.
    // So here we divide by traj.length to get the stage-averaged cost before multiplying with the duration.
    // TODO pull cost weight out to global for configurable constant
    static constexpr double cost_weight = 0.3 / solution.traj.length;
    return solution.total_time * (1.0 + cost_weight * solution.cost);
}

template <int N>
inline double softLoss(const Trajectory<N>& traj) {
    double cost = 0.0;
    for (int i = 0; i < traj.length; ++i) {
        const StateVector& state = traj.stateAt(i);
        const ActionVector& action = traj.actionAt(i);
        const double lon_accel = action[0];
        const double lat_accel = action[1] * state[3] * state[3];
        cost += std::sqrt(lon_accel * lon_accel + lat_accel * lat_accel);
    }
    return cost;
}

inline Solution<traj_length_steer> steerLofi(const StateVector& start, const StateVector& goal, const double total_time, const bool project) {
    const ActionSequence<traj_length_steer> action_sequence = steer_cubic<traj_length_steer>(start, goal, total_time);

    const double dt = total_time / traj_length_steer;

    const Dynamics dynamics{dt};

    Trajectory<traj_length_steer> traj;
    if (project) {
        rolloutOpenLoopConstrained(action_sequence, start, dynamics, traj);
    } else {
        rolloutOpenLoop(action_sequence, start, dynamics, traj);
    }

    // Calculate cost
    const double cost = softLoss<traj_length_steer>(traj);

    // Dummies
    const Policy<traj_length_steer> policy;
    const SolveStatus solve_status = SolveStatus::kConverged;
    const SolveRecord solve_record;

    return {traj, policy, cost, total_time, solve_status, solve_record};
}

struct Node {
    const StateVector state;
    const std::shared_ptr<Node> parent;
    const Trajectory<traj_length_steer> traj;
    const double cost;
    const double cost_to_come;
    const double total_time;
    const int ix;
    const bool is_warm{false};
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

inline std::pair<StateVector, int> sample(const StateVector& goal, const std::optional<Solution<traj_length_opt>>& warm = std::nullopt) {
    const double selector = urand();
    int sample_type = 0;  // nominal
    if ((selector > 0.8) && warm) {
        sample_type = 1;  // warm
    }
    if (selector > 0.9) {
        sample_type = 2;  // goal
    }

    double x_min_s{0.0};
    double x_max_s{0.0};
    double y_min_s{0.0};
    double y_max_s{0.0};
    double yaw_min_s{0.0};
    double yaw_max_s{0.0};
    double v_min_s{0.0};
    double v_max_s{0.0};

    // TODO use an enum and a switch statement
    if (sample_type == 0) {
        x_min_s = x_min;
        x_max_s = x_max;
        y_min_s = y_min;
        y_max_s = y_max;
        yaw_min_s = yaw_min;
        yaw_max_s = yaw_max;
        v_min_s = v_min;
        v_max_s = v_max;
    } else if (sample_type == 1) {
        // Choose a random state from the warm-start solution
        const int stage_ix = std::lround(urand(0, traj_length_opt));
        const StateVector& state = warm->traj.stateAt(stage_ix);

        x_min_s = state(0) - 0.2;
        x_max_s = state(0) + 0.2;
        y_min_s = state(1) - 0.2;
        y_max_s = state(1) + 0.2;
        yaw_min_s = state(2) - 0.05 * PI;
        yaw_max_s = state(2) + 0.05 * PI;
        v_min_s = state(3) - 0.2;
        v_max_s = state(3) + 0.2;
    } else if (sample_type == 2) {
        x_min_s = goal(0) - 0.2;
        x_max_s = goal(0) + 0.2;
        y_min_s = goal(1) - 0.2;
        y_max_s = goal(1) + 0.2;
        yaw_min_s = goal(2) - 0.05 * PI;
        yaw_max_s = goal(2) + 0.05 * PI;
        v_min_s = goal(3) - 0.2;
        v_max_s = goal(3) + 0.2;
    }

    x_min_s = std::clamp(x_min_s, x_min, x_max);
    x_max_s = std::clamp(x_max_s, x_min, x_max);
    y_min_s = std::clamp(y_min_s, y_min, y_max);
    y_max_s = std::clamp(y_max_s, y_min, y_max);
    yaw_min_s = std::clamp(yaw_min_s, yaw_min, yaw_max);
    yaw_max_s = std::clamp(yaw_max_s, yaw_min, yaw_max);
    v_min_s = std::clamp(v_min_s, v_min, v_max);
    v_max_s = std::clamp(v_max_s, v_min, v_max);

    const double x = urand(x_min_s, x_max_s);
    const double y = urand(y_min_s, y_max_s);
    const double yaw = urand(yaw_min_s, yaw_max_s);
    const double v = urand(v_min_s, v_max_s);

    return std::make_pair(StateVector(x, y, yaw, v), sample_type);
}

struct Obstacle {
    const Position center;
    const double radius;

    bool collidesWith(const StateVector& state) const {
        return distanceSquaredPositionState(center, state) < (radius * radius);
    }

    template <int N>
    bool collidesWith(const Trajectory<N>& traj) const {
        for (int stage_ix = N; stage_ix >= 0; --stage_ix) {
            if (collidesWith(traj.stateAt(stage_ix))) {
                return true;
            }
        }
        return false;
    }
};

const Position obstacle_center{10.0, 0.0};
const double obstacle_radius = 1.0;
const Obstacle obstacle{obstacle_center, obstacle_radius};

inline bool outsideEnvironment(const StateVector& state) {
    const bool x_ok = (x_min <= state[0]) && (state[0] <= x_max);
    const bool y_ok = (y_min <= state[1]) && (state[1] <= y_max);
    const bool yaw_ok = (yaw_min <= state[2]) && (state[2] <= yaw_max);
    const bool v_ok = (v_min <= state[3]) && (state[3] <= v_max);
    return !(x_ok && y_ok && yaw_ok && v_ok);
}

template <int N>
inline bool outsideEnvironment(const Trajectory<N>& traj) {
    // Iterate in reverse since, heuristically, states at the end of the trajectory are more likely to fail the check and terminate the loop early.
    for (int stage_ix = N; stage_ix >= 0; --stage_ix) {
        if (outsideEnvironment(traj.stateAt(stage_ix))) {
            return true;
        }
    }
    return false;
}

inline bool checkTargetHit(const StateVector& state, const StateVector& target, const double tol_factor) {
    const StateVector delta = state - target;
    const double dx = delta[0];
    const double dy = delta[1];
    const double dyaw = delta[2];
    const double dv = delta[3];

    // TODO use the TerminalStateParams. current numbers are set as a factor of the thresholds.
    const bool dx_hit = std::abs(dx) < (tol_factor * 0.01);
    const bool dy_hit = std::abs(dy) < (tol_factor * 0.01);
    const bool dyaw_hit = std::abs(dyaw) < (tol_factor * 0.02);
    const bool dv_hit = std::abs(dv) < (tol_factor * 0.01);

    return dx_hit && dy_hit && dyaw_hit && dv_hit;
}

struct Tree {
    std::vector<std::shared_ptr<Node>> nodes;
    double ratio_rejected_samples{0.0};

    const std::shared_ptr<Node> getNearestHeuristic(const StateVector& target) const {
        double min_cost = std::numeric_limits<double>::max();
        std::shared_ptr<Node> nearest_node = nullptr;
        for (std::shared_ptr<Node> node : nodes) {
            const double cost = estimateTimeBetween(node->state, target);
            const bool acceptable = cost < min_cost;
            if (acceptable) {
                min_cost = cost;
                nearest_node = node;
            }
        }
        return nearest_node;
    }

    const std::vector<std::shared_ptr<Node>> getNearHeuristic(const StateVector& target, const int k, const int m) const {
        // Create a vector of pairs: each pair holds the heuristic cost and the corresponding node.
        std::vector<std::pair<double, std::shared_ptr<Node>>> cost_node_pairs;
        for (const auto& node : nodes) {
            const double cost = estimateTimeBetween(node->state, target);
            cost_node_pairs.emplace_back(cost, node);
        }

        // Sort the pairs by their heuristic cost (lower cost indicates closer proximity).
        std::sort(cost_node_pairs.begin(), cost_node_pairs.end(),
                  [](const auto& a, const auto& b) {
                      return a.first < b.first;
                  });

        std::vector<std::shared_ptr<Node>> result;

        // Select the k nearest nodes (or all if fewer than k are available).
        const int num_nearest = std::min(k, static_cast<int>(cost_node_pairs.size()));
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
            const int num_random = std::min(m, static_cast<int>(remaining.size()));
            for (int i = 0; i < num_random; ++i) {
                result.push_back(remaining[i]);
            }
        }

        return result;
    }

    const std::shared_ptr<Node> getCheapestSolutionPrecise(const StateVector& target) const {
        double min_cost = std::numeric_limits<double>::max();
        std::shared_ptr<Node> best_node = nullptr;
        for (std::shared_ptr<Node> node : nodes) {
            static constexpr double tol_factor = 5.0;
            const bool target_hit = checkTargetHit(node->state, target, tol_factor);
            const double cost = node->cost_to_come;
            const bool cost_improved = cost < min_cost;
            const bool acceptable = cost_improved && target_hit;
            if (acceptable) {
                min_cost = cost;
                best_node = node;
            }
        }

        return (best_node != nullptr) ? best_node : getNearestHeuristic(target);
    }

    const std::shared_ptr<Node> getNearest(const StateVector& target) const {
        double min_cost = std::numeric_limits<double>::max();
        std::shared_ptr<Node> nearest_node = nullptr;

        const std::vector<std::shared_ptr<Node>> nodes_selected = getNearHeuristic(target, num_nodes_heuristic_neighbors, num_nodes_random_neighbors);
        for (std::shared_ptr<Node> node : nodes_selected) {
            // Skip if too far.
            // NOTE: Having an accurate and tight estimate for short-circuiting makes a huge difference in performance,
            // since it lets us eliminate unnecessary calls to steerLofi(),
            // which is a moderately expensive subroutine otherwise called many times.
            const double t_est = estimateTimeBetween(node->state, target);
            if (t_est > min_cost) {
                continue;
            }

            // Lofi steering
            const bool project = true;
            const auto solution = steerLofi(node->state, target, t_est, project);
            const double soln_cost = timeCost(solution);
            static constexpr double tol_factor = 50.0;
            const bool target_hit = checkTargetHit(solution.traj.stateTerminal(), target, tol_factor);
            const bool cost_improved = soln_cost < min_cost;
            const bool acceptable = cost_improved && target_hit;
            if (acceptable) {
                min_cost = soln_cost;
                nearest_node = node;
            }
        }

        return (nearest_node != nullptr) ? nearest_node : getNearestHeuristic(target);
    }

    void addNode(const std::shared_ptr<Node>& node) {
        nodes.push_back(node);
    }

    void grow(const StateVector& start, const StateVector& goal, const int num_nodes, const std::optional<Solution<traj_length_opt>>& warm = std::nullopt) {
        int next_node_ix = 0;
        int num_total_samples = 0;
        int num_rejected_samples = 0;

        // Create root node and add to the tree.
        const Trajectory<traj_length_steer> root_traj;
        const Node root_node = Node(start, nullptr, root_traj, 0.0, 0.0, 0.0, next_node_ix);
        // const std::shared_ptr<Node> root_node_ptr = std::make_shared<Node>(root_node);
        addNode(std::make_shared<Node>(root_node));
        next_node_ix++;

        // Add warm-start.
        if (warm) {
            // Break solution up into several smaller nodes
            static constexpr int num_nodes_warm = static_cast<int>(std::ceil(static_cast<double>(traj_length_opt) / static_cast<double>(traj_length_steer)));
            std::shared_ptr<Node> sub_parent = std::make_shared<Node>(root_node);
            for (int i = 0; i < num_nodes_warm; ++i) {
                // Infer the indices into the whole solution for the current sub part.
                const int ix_offset = i * traj_length_steer;

                // Form the sub-solution
                Trajectory<traj_length_steer> sub_traj;
                Policy<traj_length_steer> sub_policy;
                for (int stage_ix = 0; stage_ix <= traj_length_steer; ++stage_ix) {
                    const int ix_in_warm_traj = ix_offset + stage_ix;
                    sub_traj.setStateAt(stage_ix, warm->traj.stateAt(ix_in_warm_traj));
                    if (stage_ix < traj_length_steer) {
                        sub_traj.setActionAt(stage_ix, warm->traj.actionAt(ix_in_warm_traj));
                        sub_policy.setFeedbackGainAt(stage_ix, warm->policy.feedbackGainAt(ix_in_warm_traj));
                        sub_policy.setFeedfrwdGainAt(stage_ix, warm->policy.feedfrwdGainAt(ix_in_warm_traj));
                    }
                }
                sub_policy.feedfrwd_gain_scale = warm->policy.feedfrwd_gain_scale;
                const double sub_cost = softLoss(sub_traj);
                const double sub_total_time = warm->total_time / num_nodes_warm;
                const SolveStatus sub_solve_status = SolveStatus::kConverged;
                const SolveRecord sub_solve_record = warm->solve_record;

                // Populate the sub-solution
                Solution<traj_length_steer> sub_soln{sub_traj, sub_policy, sub_cost, sub_total_time, sub_solve_status, sub_solve_record};

                const double sub_time_cost = timeCost(sub_soln);

                const bool is_warm = true;
                const Node sub_node = Node(sub_soln.traj.stateTerminal(), sub_parent, sub_soln.traj, sub_time_cost, sub_time_cost + sub_parent->cost_to_come, sub_total_time, next_node_ix, is_warm);
                const std::shared_ptr<Node> sub_node_ptr = std::make_shared<Node>(sub_node);
                addNode(sub_node_ptr);
                sub_parent = sub_node_ptr;
                next_node_ix++;
            }
        }

        for (int node_count = 1; node_count <= num_nodes; ++node_count) {
            int num_rejected_samples_this_iter = -1;
            bool node_added = false;
            while (!node_added) {
                num_rejected_samples_this_iter++;
                num_total_samples++;

                // Sample a new state.
                auto [state, mode] = sample(goal, warm);

                // Set the parent as the nearest neighbor to the state.
                // std::shared_ptr<Node> parent = getNearest(state);
                // DEBUG
                // std::shared_ptr<Node> parent = (urand() > 0.5) ? getCheapestSolutionPrecise(state) : getNearest(state);
                std::shared_ptr<Node> parent = (mode == 2) ? getCheapestSolutionPrecise(state) : getNearest(state);

                // Attempt to pull back the sampled state inside the free space
                static constexpr int num_max_pullback_for_sample_collision_attempts = 10;
                for (int i = 0; i < num_max_pullback_for_sample_collision_attempts; ++i) {
                    if (obstacle.collidesWith(state)) {
                        // Move the sampled state closer. Do not change the sampled speed.
                        state.head<3>() = parent->state.head<3>() + attract_factor * (state.head<3>() - parent->state.head<3>());
                    } else {
                        break;
                    }
                }

                // Terminate iteration early if sampled state is in collision.
                if (obstacle.collidesWith(state)) {
                    continue;
                }

                // Estimate time.
                double t_est = estimateTimeBetween(parent->state, state);

                // Move sample closer to parent if too far.
                // NOTE this is somewhat optional, but empirically seems to help reduce wasting effort on hard-to-connect states.
                // It also helps keep the fixed number of steps in Trajectory responsible for a small time-delta, which helps limit time discretization inaccuracies.
                while (t_est > max_t_est_sample) {
                    // Move the sampled state closer. Do not change the sampled speed.
                    state.head<3>() = parent->state.head<3>() + attract_factor * (state.head<3>() - parent->state.head<3>());
                    t_est = estimateTimeBetween(parent->state, state);
                }

                // Steer from parent to child.
                // Choose a random duration around the original estimate
                const double t_steer = std::pow(1.5, urand(-1.0, 1.0)) * t_est;
                const bool project = true;
                const auto solution = steerLofi(parent->state, state, t_steer, project);

                // Calculate time cost
                const double time_cost = timeCost(solution);

                // Reset state sample as the terminal state in the trajectory.
                state = solution.traj.stateTerminal();

                // Terminate iteration early if trajectory is in collision.
                if (obstacle.collidesWith(solution.traj)) {
                    continue;
                }

                // Terminate iteration early if action projection caused any trajectory state to go outside the environment.
                // NOTE: this can lead to 10% longer solve times!!!
                if (outsideEnvironment(solution.traj)) {
                    continue;
                }

                // Create node from sampled state and add to the tree.
                const Node node{state, parent, solution.traj, time_cost, time_cost + parent->cost_to_come, solution.total_time, next_node_ix};
                addNode(std::make_shared<Node>(node));
                next_node_ix++;
                node_added = true;
            }

            num_rejected_samples += num_rejected_samples_this_iter;
        }
        ratio_rejected_samples = static_cast<double>(num_rejected_samples) / static_cast<double>(num_total_samples);

        // ---- Steer to goal
        const std::shared_ptr<Node> node_best = getNearest(goal);
        static constexpr double tol_factor = 2.0;
        const bool goal_hit = checkTargetHit(node_best->state, goal, tol_factor);
        if (!goal_hit) {
            // Estimate time.
            const double t_est = estimateTimeBetween(node_best->state, goal);

            // Steer from parent to child.
            const bool project = true;
            const auto solution = steerLofi(node_best->state, goal, t_est, project);
            const double time_cost = timeCost(solution);
            const StateVector& state = solution.traj.stateTerminal();

            // Add node if trajectory is OK.
            if (!(obstacle.collidesWith(solution.traj) || outsideEnvironment(solution.traj))) {
                const Node node{state, node_best, solution.traj, time_cost, time_cost + node_best->cost_to_come, solution.total_time, next_node_ix};
                addNode(std::make_shared<Node>(node));
                next_node_ix++;
            }
        }
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
