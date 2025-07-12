#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "core/constants.h"
#include "core/obstacle.h"
#include "core/search_space.h"
#include "core/space.h"
#include "core/trajectory.h"
#include "core/util.h"
#include "ilqr/solver.h"
#include "rrt/steer.h"

// Define a global random number generator
std::random_device rd;
std::mt19937 gen(42);  // Mersenne Twister engine w/ fixed seed
// std::mt19937 gen(rd());  // Mersenne Twister engine w/ random seed

// Time of a single step, seconds
static constexpr double DT = 0.1;

// Time of a steering function trajectory, seconds
static constexpr double STEER_TIME = DT * TRAJ_LENGTH_STEER;

// Depth of the tree.
// Integer division is OK because TRAJ_LENGTH_OPT is an integer multiple of TRAJ_LENGTH_STEER.
static constexpr int TREE_DEPTH = TRAJ_LENGTH_OPT / TRAJ_LENGTH_STEER;

// Time index runs from 0 to TREE_DEPTH - 1 so that there are TREE_DEPTH such time indices.
static constexpr int TIME_IX_MAX = TREE_DEPTH - 1;

// Leave one time ix for goal node.
static constexpr int TIME_IX_MAX_FOR_SAMPLING = TIME_IX_MAX - 1;

// Special function to efficiently compute the end state after using zero action for t seconds.
// TODO move this to rollout or dynamics
StateVector rolloutZeroAction(const StateVector& start, const double t) {
    const double x = start(0);
    const double y = start(1);
    const double yaw = start(2);
    const double v = start(3);

    const double vx = v * std::cos(yaw);
    const double vy = v * std::sin(yaw);
    const double dx = vx * t;
    const double dy = vy * t;

    return {x + dx, y + dy, yaw, v};
}

inline double distance(const StateVector& start, const StateVector& goal) {
    return (goal.head(2) - start.head(2)).norm();
}

// TODO include contribution from speed diff
// TODO calibrate this heuristic using data from steering function and many start-goal pairs
inline double distanceHeuristic(const StateVector& start, const StateVector& goal) {
    // Heuristic: Scale distance by an additional factor due to yaw difference.
    const double yaw1 = start(2);
    const double yaw2 = goal(2);

    // Compute the raw normalized yaw difference in [0, 1].
    const double yaw_diff = std::abs(angularDifference(yaw1, yaw2)) / PI;

    // Compute a factor based on the yaw diff. Apply square() to max small yaw differences << 1 have less impact.
    const double yaw_diff_factor = square(yaw_diff);

    // Compute scaled distance.
    return (1.0 + yaw_diff_factor) * distance(start, goal);
}

// Distance from [zero-action-point of start] to [goal]
// This is a good proxy for softLoss since acceleration is proportional to
// distance-from-zap(start)-to-goal under simplifying kinematic assumptions e.g. @ high speed.
inline double zapDistanceHeuristic(const StateVector& start, const StateVector& goal) {
    return distanceHeuristic(rolloutZeroAction(start, STEER_TIME), goal);
}

// Time-averaged acceleration magnitude.
template <int N>
inline double softLoss(const Trajectory<N>& traj) {
    double cost = 0.0;
    for (int i = 0; i < traj.length; ++i) {
        const StateVector& state = traj.stateAt(i);
        const ActionVector& action = traj.actionAt(i);
        const double lon_accel = action(0);
        const double lat_accel = action(1) * square(state(3));
        const double mag_accel = shypot(lon_accel, lat_accel);
        cost += mag_accel;
    }
    return cost / N;
}

struct SteerOutputs {
    Trajectory<TRAJ_LENGTH_STEER> traj;
    double cost;
};

inline SteerOutputs steer(const StateVector& start, const StateVector& goal, const bool constrain) {
    // Cubic polynomial steering.
    const ActionSequence<TRAJ_LENGTH_STEER> action_sequence = steerCubic<TRAJ_LENGTH_STEER>(start, goal, STEER_TIME);

    // Rollout.
    const Dynamics dynamics{DT};
    Trajectory<TRAJ_LENGTH_STEER> traj;
    if (constrain) {
        rolloutOpenLoopConstrained(action_sequence, start, dynamics, traj);
    } else {
        rolloutOpenLoop(action_sequence, start, dynamics, traj);
    }

    // Calculate cost.
    const double cost = softLoss(traj);

    return {traj, cost};
}

struct Node {
    const StateVector state;
    const std::shared_ptr<Node> parent;
    const Trajectory<TRAJ_LENGTH_STEER> traj;
    const double cost;
    const double cost_to_come;
    const double total_time;
    const int time_ix;
    const int ix;
    const bool is_warm{false};
};

using Path = std::vector<std::shared_ptr<Node>>;

inline double urand() {
    // TODO this is inefficient, creating a new distribution on every call (?)
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
}

inline double urand(const double x_min, const double x_max) {
    // TODO this is inefficient, creating a new distribution on every call (?)
    std::uniform_real_distribution<double> dist(x_min, x_max);
    return dist(gen);
}

inline StateVector sample() {
    const double x = urand(X_MIN, X_MAX);
    const double y = urand(Y_MIN, Y_MAX);
    const double yaw = urand(YAW_MIN, YAW_MAX);
    const double v = urand(V_MIN, V_MAX);
    return {x, y, yaw, v};
}

inline StateVector sampleNear(const StateVector& state, const double perturb_factor = 1.0) {
    const double d = perturb_factor * 0.5;
    const double d_yaw = perturb_factor * 0.2 * 0.5 * PI;
    const double dv = perturb_factor * 1.0;

    double x_min_s = state(0) - d;
    double x_max_s = state(0) + d;
    double y_min_s = state(1) - d;
    double y_max_s = state(1) + d;
    double yaw_min_s = state(2) - d_yaw;
    double yaw_max_s = state(2) + d_yaw;
    double v_min_s = state(3) - dv;
    double v_max_s = state(3) + dv;

    x_min_s = std::clamp(x_min_s, X_MIN, X_MAX);
    x_max_s = std::clamp(x_max_s, X_MIN, X_MAX);
    y_min_s = std::clamp(y_min_s, Y_MIN, Y_MAX);
    y_max_s = std::clamp(y_max_s, Y_MIN, Y_MAX);
    yaw_min_s = std::clamp(yaw_min_s, YAW_MIN, YAW_MAX);
    yaw_max_s = std::clamp(yaw_max_s, YAW_MIN, YAW_MAX);
    v_min_s = std::clamp(v_min_s, V_MIN, V_MAX);
    v_max_s = std::clamp(v_max_s, V_MIN, V_MAX);

    const double x = urand(x_min_s, x_max_s);
    const double y = urand(y_min_s, y_max_s);
    const double yaw = urand(yaw_min_s, yaw_max_s);
    const double v = urand(v_min_s, v_max_s);
    return {x, y, yaw, v};
}

inline StateVector sampleNearWarm(const Solution<TRAJ_LENGTH_OPT>& warm, const int time_ix) {
    // Choose a random state from the warm-start solution.

    // Selector for pure or sub-trajectory random index.
    const double selector = urand();
    static constexpr double sub_traj_proba = 0.2;
    const bool use_sub_traj{selector < sub_traj_proba};

    int ix_lwr;
    int ix_upr;
    double perturb_factor;
    if (use_sub_traj) {
        // Within sub-trajectory corresponding to given time_ix
        ix_lwr = time_ix * TRAJ_LENGTH_STEER;
        ix_upr = ix_lwr + TRAJ_LENGTH_STEER;
        perturb_factor = 0.4;
    } else {
        // Pure random
        ix_lwr = 0;
        ix_upr = TRAJ_LENGTH_OPT;
        perturb_factor = 1.0;
    }

    const int stage_ix = std::lround(urand(ix_lwr, ix_upr));

    const StateVector& state = warm.traj.stateAt(stage_ix);
    return sampleNear(state, perturb_factor);
}

inline StateVector sample(const StateVector& goal, const std::optional<Solution<TRAJ_LENGTH_OPT>>& warm, const int time_ix) {
    // Always sample around the warm-start trajectory if available.
    if (warm) {
        return sampleNearWarm(warm.value(), time_ix);
    }

    // Sample near the goal sometimes.
    static constexpr double goal_sampling_proba = 0.05;
    const double selector = urand();
    const bool sample_near_goal{selector < goal_sampling_proba};
    const double perturb_factor = 1.0;
    if (sample_near_goal) {
        return sampleNear(goal, perturb_factor);
    }

    return sample();
}

inline bool outsideEnvironment(const StateVector& state) {
    const bool x_ok = (X_MIN <= state(0)) && (state(0) <= X_MAX);
    const bool y_ok = (Y_MIN <= state(1)) && (state(1) <= Y_MAX);
    const bool yaw_ok = (YAW_MIN <= state(2)) && (state(2) <= YAW_MAX);
    const bool v_ok = (V_MIN <= state(3)) && (state(3) <= V_MAX);
    return !(x_ok && y_ok && yaw_ok && v_ok);
}

template <int N>
inline bool outsideEnvironment(const Trajectory<N>& traj) {
    // Iterate in reverse since, heuristically, states at the end of the trajectory
    // are more likely to fail the check and terminate the loop early.
    for (int stage_ix = N; stage_ix >= 0; --stage_ix) {
        if (outsideEnvironment(traj.stateAt(stage_ix))) {
            return true;
        }
    }
    return false;
}

inline bool checkTargetHit(const StateVector& state, const StateVector& target, const double tol_factor = 1.0) {
    const StateVector delta = state - target;
    const double dx = delta(0);
    const double dy = delta(1);
    const double dyaw = delta(2);
    const double dv = delta(3);

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

    const std::shared_ptr<Node> getNearest(const StateVector& target, const int target_time_ix) const {
        double min_cost = std::numeric_limits<double>::max();
        std::shared_ptr<Node> nearest_node = nullptr;
        for (std::shared_ptr<Node> node : nodes) {
            // Skip if time index is not compatible.
            if ((node->time_ix + 1) != target_time_ix) {
                continue;
            }

            const double cost = zapDistanceHeuristic(node->state, target);

            if (cost < min_cost) {
                min_cost = cost;
                nearest_node = node;
            }
        }

        return nearest_node;
    }

    // NOTE: this is fast but destroys the rapid epxloration of RRT, tends to cluster around start!
    const std::shared_ptr<Node> getNearestLimited(const StateVector& target, const int target_time_ix, const int max_num_neighbors) const {
        // Reservoir sampling.
        std::vector<std::shared_ptr<Node>> reservoir;
        reservoir.reserve(max_num_neighbors);
        int n = 0;
        for (const auto& node : nodes) {
            // Skip if time index is not compatible.
            if ((node->time_ix + 1) != target_time_ix) {
                continue;
            }

            // Update the reservoir.
            ++n;
            if (reservoir.size() < max_num_neighbors) {
                reservoir.push_back(node);
            } else {
                std::uniform_int_distribution<int> dist(0, n - 1);
                const int j = dist(gen);
                if (j < max_num_neighbors) {
                    reservoir[j] = node;
                }
            }
        }

        // Nearest node in the reservoir.
        double min_cost = std::numeric_limits<double>::max();
        std::shared_ptr<Node> nearest_node = nullptr;
        for (const auto& node : reservoir) {
            const double cost = zapDistanceHeuristic(node->state, target);
            if (cost < min_cost) {
                min_cost = cost;
                nearest_node = node;
            }
        }

        return nearest_node;
    }

    // Get the node which is nearest to the target in terms of achieving the lowest cost to come to the target via the node.
    const std::shared_ptr<Node> getNearestCostToCome(const StateVector& target, const int target_time_ix) const {
        double min_cost_to_come = std::numeric_limits<double>::max();
        std::shared_ptr<Node> nearest_node = nullptr;
        for (std::shared_ptr<Node> node : nodes) {
            // Skip if time index is not compatible.
            if ((node->time_ix + 1) != target_time_ix) {
                continue;
            }

            // Steer from node to target.
            const bool constrain = false;
            const auto steer_outputs = steer(node->state, target, constrain);

            // Check if cost-to-come is improved relative to current best.
            const double cost_to_come = node->cost_to_come + steer_outputs.cost;
            const bool cost_improved = cost_to_come < min_cost_to_come;

            if (cost_improved) {
                min_cost_to_come = cost_to_come;
                nearest_node = node;
            }
        }

        if (nearest_node == nullptr) {
            return getNearest(target, target_time_ix);
        }

        return nearest_node;
    }

    // Get the node which represents the cheapest solution,
    // i.e. minimum cost-to-come that gets near enough to the target w/o additional steering.
    const std::shared_ptr<Node> getCheapestSolutionPrecise(const StateVector& target, const int target_time_ix) const {
        double min_cost = std::numeric_limits<double>::max();
        std::shared_ptr<Node> best_node = nullptr;
        for (std::shared_ptr<Node> node : nodes) {
            // Skip if time index is not compatible.
            if ((node->time_ix + 1) != target_time_ix) {
                continue;
            }

            // Check if target was hit.
            static constexpr double tol_factor = 20.0;
            const bool target_hit = checkTargetHit(node->state, target, tol_factor);

            // Check if cost is improved compared with current best.
            const double cost = node->cost_to_come;
            const bool cost_improved = cost < min_cost;

            const bool found_new_best = cost_improved && target_hit;
            if (found_new_best) {
                min_cost = cost;
                best_node = node;
            }
        }

        if (best_node == nullptr) {
            return getNearest(target, target_time_ix);
        }

        return best_node;
    }

    void addNode(const std::shared_ptr<Node>& node) {
        nodes.push_back(node);
    }

    void grow(const StateVector& start, const StateVector& goal, const int num_nodes, const std::optional<Solution<TRAJ_LENGTH_OPT>>& warm = std::nullopt) {
        int next_node_ix = 0;
        int num_total_samples = 0;
        int num_rejected_samples = 0;

        // Create root node and add to the tree.
        const Trajectory<TRAJ_LENGTH_STEER> root_traj;
        const int time_ix_root_node = -1;
        const Node root_node = Node(start, nullptr, root_traj, 0.0, 0.0, 0.0, time_ix_root_node, next_node_ix);
        const std::shared_ptr<Node> root_node_ptr = std::make_shared<Node>(root_node);
        addNode(root_node_ptr);
        next_node_ix++;

        // Add warm-start nodes.
        // TODO make this a method
        if (warm) {
            // Same time for all sub-nodes.
            const double sub_total_time = warm->total_time / TREE_DEPTH;

            // Break solution up into several smaller sub-nodes.
            std::shared_ptr<Node> sub_parent = root_node_ptr;
            for (int time_ix = 0; time_ix < TREE_DEPTH; ++time_ix) {
                // Infer the indices into the whole solution for the current sub-node.
                const int ix_offset = time_ix * TRAJ_LENGTH_STEER;

                // Form the sub-node.
                Trajectory<TRAJ_LENGTH_STEER> sub_traj;
                for (int stage_ix = 0; stage_ix <= TRAJ_LENGTH_STEER; ++stage_ix) {
                    const int ix_in_warm_traj = ix_offset + stage_ix;
                    sub_traj.setStateAt(stage_ix, warm->traj.stateAt(ix_in_warm_traj));
                    if (stage_ix < TRAJ_LENGTH_STEER) {
                        sub_traj.setActionAt(stage_ix, warm->traj.actionAt(ix_in_warm_traj));
                    }
                }
                const double sub_cost = softLoss(sub_traj);
                static constexpr bool is_warm = true;
                const Node sub_node{sub_traj.stateTerminal(), sub_parent, sub_traj, sub_cost, sub_cost + sub_parent->cost_to_come, sub_total_time, time_ix, next_node_ix, is_warm};
                const std::shared_ptr<Node> sub_node_ptr = std::make_shared<Node>(sub_node);
                addNode(sub_node_ptr);
                next_node_ix++;
                sub_parent = sub_node_ptr;
            }
        }

        // Grow the tree by adding samples.
        // TODO make this a method
        int last_time_ix = -1;
        for (int node_count = 1; node_count <= num_nodes; ++node_count) {
            // Add a single node.

            int num_rejected_samples_this_iter = -1;
            bool node_added = false;

            // Time index of sample.
            // Round-robin.
            int time_ix = (last_time_ix + 1) % (TIME_IX_MAX_FOR_SAMPLING + 1);
            int num_rejected_samples_this_round = -1;

            while (!node_added) {
                num_rejected_samples_this_iter++;
                num_total_samples++;
                num_rejected_samples_this_round++;

                // Safety valve in case it is too difficult to add a node
                static constexpr int max_num_rejected_samples_per_iter = 100;
                if (num_rejected_samples_this_iter >= max_num_rejected_samples_per_iter) {
                    break;
                }

                // Safety valve in case the current time_ix is too difficult to find a good connection from.
                static constexpr int max_num_rejected_samples_per_round = 10;
                if (num_rejected_samples_this_round >= max_num_rejected_samples_per_round) {
                    // Reset the round-robin.
                    last_time_ix = TIME_IX_MAX_FOR_SAMPLING;
                    time_ix = 0;
                    num_rejected_samples_this_round = 0;
                }

                // Sample a new state.
                StateVector state = sample(goal, warm, time_ix);

                // Set the parent.
                std::shared_ptr<Node> parent = getNearest(state, time_ix);

                // // Use getNearestLimited to keep the search for a parent quick, even for large trees.
                // static constexpr int max_num_neighbors = 64;
                // std::shared_ptr<Node> parent = getNearestLimited(state, time_ix, max_num_neighbors);

                // Could not find a parent.
                if (parent == nullptr) {
                    continue;
                }

                // Sampled state is in collision.
                bool in_collision = false;
                for (const auto& obstacle : obstacles) {
                    in_collision = in_collision || obstacle.collidesWith(state);
                }
                if (in_collision) {
                    continue;
                }

                // Steer from parent to child.
                // NOTE: Using projection onto the action constraints is critical to sampling efficiency.
                // Using rejection sampling to honor action constraints leads to very few feasible samples and long runtimes.
                // NOTE: Using projection tends to produce bang-bang trajectories.
                // This might not be good on its own, but using traj opt post-processing mitigates any ill-effects.
                const bool constrain = true;
                const auto steer_outputs = steer(parent->state, state, constrain);
                const auto& traj = steer_outputs.traj;
                const double cost = steer_outputs.cost;

                // Reset state sample as the terminal state in the trajectory.
                state = traj.stateTerminal();

                // Trajectory is in collision.
                for (const auto& obstacle : obstacles) {
                    in_collision = in_collision || obstacle.collidesWith(traj);
                }
                if (in_collision) {
                    continue;
                }

                // Something, e.g. action constraint projection, caused a trajectory state to go outside the environment.
                if (outsideEnvironment(traj)) {
                    continue;
                }

                // Create node from sampled state and add to the tree.
                const Node node{state, parent, traj, cost, cost + parent->cost_to_come, STEER_TIME, time_ix, next_node_ix};
                addNode(std::make_shared<Node>(node));
                next_node_ix++;
                node_added = true;
            }

            // Update round-robin for time index.
            last_time_ix = time_ix;

            num_rejected_samples += num_rejected_samples_this_iter;
        }
        ratio_rejected_samples = static_cast<double>(num_rejected_samples) / static_cast<double>(num_total_samples);

        // ---- Add node to steer to goal.
        // TODO make this a method
        {
            const std::shared_ptr<Node> node_nearest_goal = getNearestCostToCome(goal, TIME_IX_MAX);
            if (node_nearest_goal != nullptr) {
                // Steer from parent to goal.
                const bool constrain = true;
                const auto steer_outputs = steer(node_nearest_goal->state, goal, constrain);
                const auto& traj = steer_outputs.traj;
                const double cost = steer_outputs.cost;
                const StateVector& state = traj.stateTerminal();

                // Add node regardless of whether trajectory satisfies any constraints.
                // This ensures the tree has at least one node with time_ix = TIME_IX_MAX.
                const Node node{state, node_nearest_goal, traj, cost, cost + node_nearest_goal->cost_to_come, STEER_TIME, TIME_IX_MAX, next_node_ix};
                addNode(std::make_shared<Node>(node));
                next_node_ix++;
            }
        }
    }

    Path extractPathToGoal(const StateVector& goal) const {
        int time_ix_for_path_extraction = TIME_IX_MAX + 1;
        std::shared_ptr<Node> node_best = nullptr;

        // In the best case, we will get the goal node on the first iteration of this loop.
        // In the worst case, we will get the root node and the loop terminates in finite iterations.
        while (node_best == nullptr) {
            node_best = getCheapestSolutionPrecise(goal, time_ix_for_path_extraction);
            time_ix_for_path_extraction--;
        }

        // Reconstruct the path by traversing parent pointers.
        Path path;
        std::shared_ptr<Node> node = node_best;
        while (node->parent != nullptr) {
            path.push_back(node);
            node = node->parent;
        }
        // NOTE: DO NOT push back the root, which has garbage for parent info e.g. trajectory.

        // Reverse the path so it starts at the root.
        std::reverse(path.begin(), path.end());

        return path;
    }
};
