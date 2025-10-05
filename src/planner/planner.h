#pragma once

#include "core/interp.h"
#include "core/problem.h"
#include "core/rollout.h"
#include "core/space.h"
#include "core/util.h"
#include "ilqr/solver.h"
#include "ilqr/solver_settings.h"
#include "tree/tree.h"

// Tree settings
static constexpr int NUM_NODE_ATTEMPTS_COLD = 4000;
static constexpr int NUM_NODE_ATTEMPTS_WARM = 1000;

struct TimingInfo {
    int tree_exp;  // ms
    int traj_opt;  // ms
};

struct PlannerOutputs {
    Tree tree;
    Path path;
    Solution<TRAJ_LENGTH_OPT> solution;
    Trajectory<TRAJ_LENGTH_OPT> traj_pre_opt;
    TimingInfo timing_info;
};

struct Planner {
    static std::tuple<Tree, int> expandTree(const StateVector& start, const StateVector& goal, const int num_node_attempts, const std::optional<Solution<TRAJ_LENGTH_OPT>>& warm) {
        const float clock_start = GetTime();

        Tree tree;
        tree.grow(start, goal, num_node_attempts, warm ? std::optional(warm->traj) : std::nullopt);

        const float clock_stop = GetTime();
        const int clock_time = static_cast<int>(std::ceil(1e6 * (clock_stop - clock_start)));

        return {tree, clock_time};
    }

    static ActionSequence<TRAJ_LENGTH_OPT> convertPathToActionSequence(const Path& path) {
        // Concatenate actions from all nodes of path.
        ActionSequence<TRAJ_LENGTH_OPT> action_sequence;
        int j = 0;
        for (const NodePtr& node : path) {
            if (!node->traj) {
                continue;
            }
            for (int i = 0; i < node->traj->length; ++i) {
                action_sequence.col(j) = node->traj->actionAt(i);
                j++;
            }
        }
        return action_sequence;
    }

    static std::tuple<Solution<TRAJ_LENGTH_OPT>, Trajectory<TRAJ_LENGTH_OPT>, int> optimizeTrajectory(const StateVector& start, const StateVector& goal, const ActionSequence<TRAJ_LENGTH_OPT>& action_sequence) {
        const float clock_start = GetTime();

        // Define the optimal control problem.
        const Problem problem = makeProblem(start, goal, TRAJ_DURATION_OPT);

        // Get the pre-optimization trajectory for diagnostics later.
        Trajectory<TRAJ_LENGTH_OPT> traj_pre_opt;
        rolloutOpenLoop(action_sequence, start, traj_pre_opt);

        // Solver settings.
        const SolverSettings settings = SolverSettings();
        settings.validate();

        // Instantiate the solver.
        Solver solver = Solver(std::make_shared<Problem>(problem), std::make_shared<SolverSettings>(settings));

        // Solve the optimal control problem.
        Solution<TRAJ_LENGTH_OPT> solution = solver.solve(action_sequence);

        // Assign the cost using arbitrary loss.
        solution.cost = softLoss(solution.traj);

        const float clock_stop = GetTime();
        const int clock_time = static_cast<int>(std::ceil(1e6 * (clock_stop - clock_start)));

        return {solution, traj_pre_opt, clock_time};
    }

    template <int N>
    static void addJitter(ActionSequence<N>& action_sequence, double sigma_a = 0.01, double sigma_k = 0.001) {
        // Create RNG generator.
        static thread_local std::default_random_engine generator(std::random_device{}());

        // Set the distributions.
        std::normal_distribution<double> distribution_a(0.0, sigma_a);
        std::normal_distribution<double> distribution_k(0.0, sigma_k);

        // Generate noise vectors
        Eigen::Matrix<double, 1, N> noise_a;
        Eigen::Matrix<double, 1, N> noise_k;

        for (int i = 0; i < N; ++i) {
            noise_a(0, i) = distribution_a(generator);
            noise_k(0, i) = distribution_k(generator);
        }

        // Add noise in a single vectorized step
        action_sequence.row(0) += noise_a;
        action_sequence.row(1) += noise_k;
    }

    static PlannerOutputs plan(const StateVector& start, const StateVector& goal, const int num_node_attempts, const std::optional<Solution<TRAJ_LENGTH_OPT>>& warm, const bool use_action_jitter) {
        const auto [tree, tree_exp_clock_time] = expandTree(start, goal, num_node_attempts, warm);
        const Path path = tree.extractPathToGoal();
        auto action_sequence = convertPathToActionSequence(path);

        if (use_action_jitter) {
            // Add jitter on actions just before traj opt to try and jiggle out of bad local minima
            addJitter(action_sequence);
        }

        const auto [solution, traj_pre_opt, traj_opt_clock_time] = optimizeTrajectory(start, goal, action_sequence);

        return {tree, path, solution, traj_pre_opt, {tree_exp_clock_time, traj_opt_clock_time}};
    }

    static PlannerOutputs planTrajOptOnly(const StateVector& start, const StateVector& goal, const std::optional<Solution<TRAJ_LENGTH_OPT>>& warm, const bool use_action_jitter) {
        const Tree tree;
        static constexpr int tree_exp_clock_time = 0;

        ActionSequence<TRAJ_LENGTH_OPT> action_sequence = warm ? warm->traj.action_sequence : ActionSequence<TRAJ_LENGTH_OPT>::Zero();

        if (use_action_jitter) {
            // Add jitter on actions just before traj opt to try and jiggle out of bad local minima
            addJitter(action_sequence);
        }

        const Path path;

        const auto [solution, traj_pre_opt, traj_opt_clock_time] = optimizeTrajectory(start, goal, action_sequence);

        return {tree, path, solution, traj_pre_opt, {tree_exp_clock_time, traj_opt_clock_time}};
    }
};

struct MultiPlannerOutputs {
    PlannerOutputs out;
    PlannerOutputs pri;
    PlannerOutputs aux;
    PlannerOutputs def;
};

struct MultiPlannerSettings {
    bool use_warm_start;
    bool use_exploration_tree;
    bool use_action_jitter;
};

struct MultiPlanner {
    static MultiPlannerOutputs
    plan(MultiPlannerSettings settings, const StateVector& start, const StateVector& goal, const std::optional<Solution<TRAJ_LENGTH_OPT>>& warm) {
        MultiPlannerOutputs planner_outputs;

        if (settings.use_warm_start) {
            // Run the primary planner, including warm-start.
            planner_outputs.pri = Planner::plan(start, goal, NUM_NODE_ATTEMPTS_WARM, warm, settings.use_action_jitter);
        }

        if (settings.use_exploration_tree) {
            // Run the secondary planner, without warm-starting.
            // Experimental idea. Works well in practice to avoid getting stuck in local minima induced by warm-starting.
            // This probably outweighs the cost of running the planner twice,
            // especially if the secondary planner could run in a separate thread concurrently.
            planner_outputs.aux = Planner::plan(start, goal, NUM_NODE_ATTEMPTS_COLD, std::nullopt, settings.use_action_jitter);
        }

        if (!settings.use_warm_start && !settings.use_exploration_tree) {
            // Run the traj-opt-only planner.
            planner_outputs.def = Planner::planTrajOptOnly(start, goal, warm, settings.use_action_jitter);
        }

        // Set the ultimate output planner_outputs.out.
        if (settings.use_warm_start && !settings.use_exploration_tree) {
            planner_outputs.out = planner_outputs.pri;
        } else if (!settings.use_warm_start && settings.use_exploration_tree) {
            planner_outputs.out = planner_outputs.aux;
        } else if (settings.use_warm_start && settings.use_exploration_tree) {
            // ---- Combine the pri and aux planner outputs.

            // Check if primary and auxiliary planner solutions are valid.
            const bool pri_soln_valid = checkTargetHit(planner_outputs.pri.solution.traj.stateTerminal(), goal);
            const bool aux_soln_valid = checkTargetHit(planner_outputs.aux.solution.traj.stateTerminal(), goal);

            // Decide which solution to use.
            bool use_aux_soln = false;
            if (pri_soln_valid && aux_soln_valid) {
                // If both solutions are valid, use the lower-cost one.
                const bool aux_cost_is_better = planner_outputs.aux.solution.cost < planner_outputs.pri.solution.cost;
                use_aux_soln = aux_cost_is_better;
            } else if (!pri_soln_valid && aux_soln_valid) {
                // If aux solution is valid but not the primary solution, use the aux one.
                use_aux_soln = true;
            } else if (pri_soln_valid && !aux_soln_valid) {
                // If pri solution is valid but not the aux solution, do not use the aux one.
                use_aux_soln = false;
            } else {
                // If neither solution is valid, use the one that hits closer to the goal.
                const double d_pri = distanceHeuristic(planner_outputs.pri.solution.traj.stateTerminal(), goal);
                const double d_aux = distanceHeuristic(planner_outputs.aux.solution.traj.stateTerminal(), goal);
                const bool aux_hits_closer_to_goal = d_aux < d_pri;
                use_aux_soln = aux_hits_closer_to_goal;
            }

            // Replace outputs.
            // TODO replacing/adding each field is error prone, put in a function or handle more elegantly.
            if (use_aux_soln) {
                planner_outputs.out.tree = planner_outputs.aux.tree;
                planner_outputs.out.path = planner_outputs.aux.path;
                planner_outputs.out.solution = planner_outputs.aux.solution;
                planner_outputs.out.traj_pre_opt = planner_outputs.aux.traj_pre_opt;
            } else {
                planner_outputs.out.tree = planner_outputs.pri.tree;
                planner_outputs.out.path = planner_outputs.pri.path;
                planner_outputs.out.solution = planner_outputs.pri.solution;
                planner_outputs.out.traj_pre_opt = planner_outputs.pri.traj_pre_opt;
            }
            // Add outputs.
            planner_outputs.out.timing_info.tree_exp = planner_outputs.pri.timing_info.tree_exp + planner_outputs.aux.timing_info.tree_exp;
            planner_outputs.out.timing_info.traj_opt = planner_outputs.pri.timing_info.traj_opt + planner_outputs.aux.timing_info.traj_opt;
        } else {
            planner_outputs.out = planner_outputs.def;
        }

        return planner_outputs;
    }
};
