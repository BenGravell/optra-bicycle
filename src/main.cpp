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

#include "cmaps/viridis.h"
#include "cmaps/magma.h"
#include "cmaps/inferno.h"
#include "cmaps/turbo.h"
#include "cmaps/mako.h"
#include "cmaps/rocket.h"
#include "cmaps/flare_r.h"
#include "cmaps/crest_r.h"


float Slider(Rectangle bounds, float value, float minValue, float maxValue, const char* label) {
    // Draw the slider background
    DrawRectangleRec(bounds, GRAY);

    // Compute normalized value
    float normalized = (value - minValue) / (maxValue - minValue);
    float knobX = bounds.x + normalized * bounds.width;

    Rectangle knob = {knobX - 5, bounds.y - 5, 10, bounds.height + 10};
    bool isHovered = CheckCollisionPointRec(GetMousePosition(), knob);
    bool isActive = isHovered && IsMouseButtonDown(MOUSE_LEFT_BUTTON);

    if (isActive) {
        float mouseX = GetMouseX();
        mouseX = std::clamp(mouseX, bounds.x, bounds.x + bounds.width);
        normalized = (mouseX - bounds.x) / bounds.width;
        value = minValue + normalized * (maxValue - minValue);
    }

    // Draw the knob
    DrawRectangleRec(knob, isHovered ? DARKGRAY : LIGHTGRAY);
    DrawText(label, bounds.x, bounds.y - 20, 10, BLACK);
    char valText[32];
    snprintf(valText, sizeof(valText), "%.2f", value);
    DrawText(valText, bounds.x + bounds.width + 10, bounds.y, 10, BLACK);

    return value;
}

// x must be in [0, 1]
Color Colormap(const float x) {
    // Scale to [0, 255] and round to nearest integer
    int idx = static_cast<int>(x * 255.0f + 0.5f);
    idx = std::clamp(idx, 0, 255);

    const auto& rgb = turbo_colormap[idx];

    return Color{rgb[0], rgb[1], rgb[2], 255};
}

// x must be in [0, 1]
Color WarmColormap(const float x) {
    // Scale to [0, 255] and round to nearest integer
    int idx = static_cast<int>(x * 255.0f + 0.5f);
    idx = std::clamp(idx, 0, 255);

    // const auto& rgb = magma_colormap[idx];
    // const auto& rgb = inferno_colormap[idx];
    // const auto& rgb = rocket_colormap[idx];
    const auto& rgb = flare_r_colormap[idx];

    return Color{rgb[0], rgb[1], rgb[2], 255};
}

// x must be in [0, 1]
Color CoolColormap(const float x) {
    // Scale to [0, 255] and round to nearest integer
    int idx = static_cast<int>(x * 255.0f + 0.5f);
    idx = std::clamp(idx, 0, 255);

    // const auto& rgb = viridis_colormap[idx];
    // const auto& rgb = mako_colormap[idx];
    const auto& rgb = crest_r_colormap[idx];

    return Color{rgb[0], rgb[1], rgb[2], 255};
}


// Helper macro to convert hex to Color (expects 0xRRGGBBAA)
#define HEX2COLOR(hex)                             \
    (Color) { (unsigned char)((hex >> 24) & 0xFF), \
              (unsigned char)((hex >> 16) & 0xFF), \
              (unsigned char)((hex >> 8) & 0xFF),  \
              (unsigned char)(hex & 0xFF) }

// Define colors using hex format 0xRRGGBBAA

// Monokai
static constexpr Color MONOKAI_RED = HEX2COLOR(0xF92672FF);
static constexpr Color MONOKAI_ORANGE = HEX2COLOR(0xFD971FFF);
static constexpr Color MONOKAI_YELLOW = HEX2COLOR(0xE6DB74FF);
static constexpr Color MONOKAI_GREEN = HEX2COLOR(0xA6E22EFF);
static constexpr Color MONOKAI_BLUE = HEX2COLOR(0x66D9EFFF);
static constexpr Color MONOKAI_PURPLE = HEX2COLOR(0xAE81FFFF);

// Atlantis
static constexpr Color ATLANTIS_BLUE_DARK = HEX2COLOR(0x123370FF);
static constexpr Color ATLANTIS_BLUE_MEDIUM = HEX2COLOR(0x026BACFF);
static constexpr Color ATLANTIS_BLUE_BRIGHT = HEX2COLOR(0x2AB3E7FF);
static constexpr Color ATLANTIS_TEAL = HEX2COLOR(0x0D8EA1FF);
static constexpr Color ATLANTIS_GOLD = HEX2COLOR(0xF6B337FF);

static constexpr Color DARK_GRAY = {20, 20, 20, 255};
static constexpr Color MEDIUM_GRAY = {60, 60, 60, 255};

static constexpr Color COLOR_BACKGROUND = BLACK;
const Color COLOR_SEARCH_SPACE = DARK_GRAY;

static constexpr Color COLOR_OBSTACLE = MEDIUM_GRAY;

// static constexpr Color COLOR_START = ATLANTIS_GOLD;
// static constexpr Color COLOR_GOAL = ATLANTIS_GOLD;

const Color COLOR_TREE = Fade(WHITE, 0.1f);
static constexpr Color COLOR_WARM_START = ATLANTIS_TEAL;

static constexpr Color COLOR_TRAJ_POST_OPT = WHITE;
static constexpr Color COLOR_TRAJ_PRE_OPT = GRAY;

// Screen-space constants
static constexpr int gutter_ss_x = 200;
static constexpr int gutter_ss_y = 400;

static constexpr int scale_ss = 80;
static constexpr int origin_ss_x = gutter_ss_x;
static constexpr int origin_ss_y = gutter_ss_y + 2 * scale_ss;
const Vector2 origin_ss = {origin_ss_x, origin_ss_y};

// RRT settings
static constexpr int num_nodes = 1000;

inline Vector2 state2screen(const Vector2 state) {
    return {origin_ss.x + scale_ss * state.x, origin_ss.y + scale_ss * state.y};
}

inline Vector2 state2screen(const Position state) {
    const Vector2 vec{static_cast<float>(state.x), static_cast<float>(state.y)};
    return state2screen(vec);
}

inline Vector2 state2screen(const StateVector state) {
    const Vector2 vec{static_cast<float>(state[0]), static_cast<float>(state[1])};
    return state2screen(vec);
}

inline StateVector screen2state(const Vector2 point) {
    return {(point.x - origin_ss.x) / scale_ss, (point.y - origin_ss.y) / scale_ss, 0.0, 0.0};
}

using Path = std::vector<std::shared_ptr<Node>>;

struct PlannerOutputs {
    Tree tree;
    Path path;
    Solution<traj_length_opt> solution;
    Trajectory<traj_length_opt> traj_pre_opt;
    int tree_exp_clock_time;  // ms
    int traj_opt_clock_time;  // ms
};


PlannerOutputs planZeroInit(const StateVector& start, const StateVector& goal) {
    // ---- RRT
    Tree tree;
    const int tree_exp_clock_time = 0;

    // ---- Path extraction
    Path path;

    // ---- Trajectory optimization
    ActionSequence<traj_length_opt> action_sequence;
    {
        for (int i = 0; i < traj_length_opt; ++i) {
            const double accel = 0.0;
            const double curvature = 0.0;
            const ActionVector action{accel, curvature};
            action_sequence.col(i) = action;
        }
    }

    const float traj_opt_clock_start = GetTime();

    // Define the optimal control problem.
    const double total_time = dt * traj_length_opt;
    const Problem problem = makeProblem(start, goal, total_time, traj_length_opt);

    // Get the pre-optimization trajectory for diagnostics later
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

    return {tree, path, solution, traj_pre_opt, tree_exp_clock_time, traj_opt_clock_time};
}

PlannerOutputs planWarm(const StateVector& start, const StateVector& goal, const Trajectory<traj_length_opt>& warm) {
    // ---- RRT
    Tree tree;
    const int tree_exp_clock_time = 0;

    // ---- Path extraction
    Path path;

    // ---- Trajectory optimization
    ActionSequence<traj_length_opt> action_sequence;
    {
        for (int i = 0; i < traj_length_opt; ++i) {
            action_sequence.col(i) = warm.actionAt(i);
        }
    }

    const float traj_opt_clock_start = GetTime();

    // Define the optimal control problem.
    const double total_time = dt * traj_length_opt;
    const Problem problem = makeProblem(start, goal, total_time, traj_length_opt);

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

    return {tree, path, solution, warm, tree_exp_clock_time, traj_opt_clock_time};
}

PlannerOutputs plan(const StateVector& start, const StateVector& goal, const std::optional<Solution<traj_length_opt>>& warm = std::nullopt) {
    // ---- RRT
    const float tree_exp_clock_start = GetTime();

    Tree tree;
    // static constexpr int num_nodes_cool = 0.8 * num_nodes;
    // static constexpr int num_nodes_warm = 0.2 * num_nodes;
    // const int num_nodes_this_tree = warm ? num_nodes_warm : num_nodes_cool;
    
    const int num_nodes_this_tree = num_nodes;

    tree.grow(start, goal, num_nodes_this_tree, warm);

    const float tree_exp_clock_stop = GetTime();
    const int tree_exp_clock_time = static_cast<int>(std::ceil(1e6 * (tree_exp_clock_stop - tree_exp_clock_start)));

    // ---- Path extraction
    int time_ix_for_path_extraction = time_ix_max + 1;
    std::shared_ptr<Node> node_best = nullptr;

    // In the best case, we will get the goal node on the first iteration of this loop.
    // In the worst case, we will get the root node and the loop terminates in finite iterations.
    while (node_best == nullptr) {
        node_best = tree.getCheapestSolutionPrecise(goal, time_ix_for_path_extraction);
        time_ix_for_path_extraction--;
    }

    // Reconstruct the path by traversing parent pointers.
    Path path;
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

    // ---- Trajectory optimization
    const float traj_opt_clock_start = GetTime();

    // Define the optimal control problem.
    const Problem problem = makeProblem(start, goal, total_time, traj_length_opt);

    // Get the pre-optimization trajectory for diagnostics later
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

    return {tree, path, solution, traj_pre_opt, tree_exp_clock_time, traj_opt_clock_time};
}

struct VisibilitySettings {
    bool show_tree;
    bool show_pre_opt_traj;
    bool show_post_opt_traj;
};

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
        const Color color = Fade(warm ? WarmColormap(c) : CoolColormap(c), 0.2f);

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

StateVector clampToSearchSpace(const StateVector& state) {
    const double x = std::clamp(state(0), x_min, x_max);
    const double y = std::clamp(state(1), y_min, y_max);
    const double yaw = std::clamp(state(2), yaw_min, yaw_max);
    const double v = std::clamp(state(3), v_min, v_max);
    return {x, y, yaw, v};
}

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
    // Font monoFont = LoadFont("fonts/SpaceMono-Bold.ttf");
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
    PlannerOutputs planner_outputs = plan(start, goal);
    PlannerOutputs planner_outputs_pri = plan(start, goal);
    PlannerOutputs planner_outputs_aux = plan(start, goal);
    PlannerOutputs planner_outputs_def = planZeroInit(start, goal);

    static constexpr int gameFPS = 30;
    SetTargetFPS(gameFPS);

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
            // Run a single planner
            // const auto warm = useWarmStart ? std::make_optional(planner_outputs.solution) : std::nullopt;
            // planner_outputs = plan(start, goal, warm);

            if (useWarmStart) {
                // Run the primary planner, including warm-start.
                const auto warm = std::make_optional(planner_outputs.solution);
                planner_outputs_pri = plan(start, goal, warm);
            }

            if (useExplorationTree) {
                // Run the secondary planner, without warm-starting.
                // Experimental idea. Works well in practice to avoid getting stuck in local minima induced by warm-starting.
                // This probably outweighs the cost of running the planner twice,
                // especially if the secondary planner could run in a separate thread concurrently.
                planner_outputs_aux = plan(start, goal);
            }

            if (!useWarmStart && !useExplorationTree) {
                // Run the backup planner, traj opt only w/ zero action initialization.
                // planner_outputs_def = planZeroInit(start, goal);
                planner_outputs_def = planWarm(start, goal, planner_outputs.solution.traj);
            }

            if (useWarmStart && !useExplorationTree) {
                planner_outputs = planner_outputs_pri;
            } else if (!useWarmStart && useExplorationTree) {
                planner_outputs = planner_outputs_aux;
            } else if (useWarmStart && useExplorationTree) {
                // ---- Combine the planner outputs.

                // Pick the best solution from the planners.
                const bool pri_soln_valid = checkTargetHit(planner_outputs_pri.solution.traj.stateTerminal(), goal);
                const bool aux_soln_valid = checkTargetHit(planner_outputs_aux.solution.traj.stateTerminal(), goal);
                bool use_aux_soln = false;
                if (pri_soln_valid && aux_soln_valid) {
                    // If both solutions are valid, use the lower-cost one.
                    const bool aux_cost_is_better = planner_outputs_aux.solution.cost < planner_outputs_pri.solution.cost;
                    use_aux_soln = aux_cost_is_better;
                } else if (!pri_soln_valid && aux_soln_valid) {
                    // If aux solution is valid but not the primary solution, use the aux one.
                    use_aux_soln = true;
                } else if (pri_soln_valid && !aux_soln_valid) {
                    // If pri solution is valid but not the aux solution, do not use the aux one.
                    use_aux_soln = false;
                } else {
                    // If neither solution is valid, use the one that hits closer to the goal.
                    const double d_pri = distanceHeuristic(planner_outputs_pri.solution.traj.stateTerminal(), goal);
                    const double d_aux = distanceHeuristic(planner_outputs_aux.solution.traj.stateTerminal(), goal);
                    const bool aux_hits_closer_to_goal = d_aux < d_pri;
                    use_aux_soln = aux_hits_closer_to_goal;
                }

                // TODO replacing/adding each field is error prone, put in a function or handle more elegantly.
                // Replace outputs.
                if (use_aux_soln) {
                    planner_outputs.tree = planner_outputs_aux.tree;
                    planner_outputs.path = planner_outputs_aux.path;
                    planner_outputs.solution = planner_outputs_aux.solution;
                    planner_outputs.traj_pre_opt = planner_outputs_aux.traj_pre_opt;
                } else {
                    planner_outputs.tree = planner_outputs_pri.tree;
                    planner_outputs.path = planner_outputs_pri.path;
                    planner_outputs.solution = planner_outputs_pri.solution;
                    planner_outputs.traj_pre_opt = planner_outputs_pri.traj_pre_opt;
                }
                // Add outputs.
                planner_outputs.tree_exp_clock_time = planner_outputs_pri.tree_exp_clock_time + planner_outputs_aux.tree_exp_clock_time;
                planner_outputs.traj_opt_clock_time = planner_outputs_pri.traj_opt_clock_time + planner_outputs_aux.traj_opt_clock_time;
            } else {
                planner_outputs = planner_outputs_def;
            }
        }

        // Draw everything
        BeginDrawing();
        const float draw_elm_clock_start = GetTime();
        ClearBackground(COLOR_BACKGROUND);

        // Draw the search space
        DrawRectangleRec(searchSpaceRec, COLOR_SEARCH_SPACE);

        // Draw the obstacle
        for (const auto& obstacle : obstacles) {
            const Vector2 obstacleCenterSS = state2screen(obstacle.center);
            const double obstacleRadiusSS = obstacle.radius * scale_ss;
            DrawCircleV(obstacleCenterSS, obstacleRadiusSS, COLOR_OBSTACLE);
        }

        // Draw planner outputs.
        const VisibilitySettings viz_settings{showTree, showPreOptTraj, showPostOptTraj};
        // Draw tree(s).
        if (viz_settings.show_tree) {
            // drawTree(planner_outputs.tree);

            if (useExplorationTree) {
                drawTree(planner_outputs_aux.tree, false);
            }
            if (useWarmStart) {
                drawTree(planner_outputs_pri.tree, true);
            }
        }
        // Draw pre-opt trajectory (RRT solution).
        if (viz_settings.show_pre_opt_traj) {
            static constexpr float line_width = 8;
            static constexpr float node_width = 16;
            static constexpr Color color = COLOR_TRAJ_PRE_OPT;
            drawPath(planner_outputs.path, line_width, node_width, color);
        }
        // Draw post-opt trajectory (iLQR solution).
        if (viz_settings.show_post_opt_traj) {
            static constexpr float line_width = 4;
            static constexpr Color color = COLOR_TRAJ_POST_OPT;
            drawTrajectory(planner_outputs.solution.traj, line_width, color);
        }

        // Draw start point and the goal point
        DrawSquare(startPoint, 10, WHITE);
        DrawSquare(startPoint, 6, BLACK);

        DrawStar(goalPoint, 16, WHITE);
        DrawStar(goalPoint, 8, BLACK);

        // DrawGoalTriangle(goalPoint, 20, WHITE);
        // DrawGoalTriangle(goalPoint, 12, BLACK);

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
            tree_exp_clock_time = planner_outputs.tree_exp_clock_time;
        }
        if (traj_opt_clock_time < 0) {
            traj_opt_clock_time = planner_outputs.traj_opt_clock_time;
        }
        if (draw_elm_clock_time < 0) {
            draw_elm_clock_time = draw_elm_clock_time_next;
        }
        if (game_upd_clock_time < 0) {
            game_upd_clock_time = static_cast<int>(1e6 * deltaTime);
        }
        tree_exp_clock_time = static_cast<int>(lerp(planner_outputs.tree_exp_clock_time, tree_exp_clock_time, paused ? 0.0 : tree_exp_clock_momentum));
        traj_opt_clock_time = static_cast<int>(lerp(planner_outputs.traj_opt_clock_time, traj_opt_clock_time, paused ? 0.0 : traj_opt_clock_momentum));
        draw_elm_clock_time = static_cast<int>(lerp(draw_elm_clock_time_next, draw_elm_clock_time, draw_elm_clock_momentum));
        game_upd_clock_time = static_cast<int>(lerp(static_cast<int>(1e6 * deltaTime), game_upd_clock_time, game_upd_clock_momentum));
        DrawTextEx(monoFont, TextFormat("Tree exp: %5.1f ms", 0.001 * static_cast<double>(tree_exp_clock_time)), (Vector2){10, 10 + 0 * 30}, 20, 1, WHITE);
        DrawTextEx(monoFont, TextFormat("Traj opt: %5.1f ms", 0.001 * static_cast<double>(traj_opt_clock_time)), (Vector2){10, 10 + 1 * 30}, 20, 1, WHITE);
        DrawTextEx(monoFont, TextFormat("Draw elm: %5.1f ms", 0.001 * static_cast<double>(draw_elm_clock_time)), (Vector2){10, 10 + 2 * 30}, 20, 1, LIGHTGRAY);
        DrawTextEx(monoFont, TextFormat("Game upd: %5.1f ms", 0.001 * static_cast<double>(game_upd_clock_time)), (Vector2){10, 10 + 3 * 30}, 20, 1, LIGHTGRAY);

        // Draw the planner stats
        const double v_avg = planner_outputs.solution.traj.state_sequence.row(3).cwiseAbs().mean();
        DrawTextEx(monoFont, TextFormat("       Traj total time %5.3f s", planner_outputs.solution.total_time), (Vector2){10, 150 + 0 * 30}, 20, 1, MONOKAI_YELLOW);
        DrawTextEx(monoFont, TextFormat("       Traj  avg speed %5.3f m/s", v_avg), (Vector2){10, 150 + 1 * 30}, 20, 1, MONOKAI_YELLOW);

        DrawTextEx(monoFont, TextFormat("Ratio rejected samples %5.0f%%", 100.0 * planner_outputs.tree.ratio_rejected_samples), (Vector2){10, 150 + 3 * 30}, 20, 1, MONOKAI_ORANGE);
        DrawTextEx(monoFont, TextFormat("       Traj  opt iters %5d", planner_outputs.solution.solve_record.iters), (Vector2){10, 150 + 4 * 30}, 20, 1, MONOKAI_ORANGE);
        DrawTextEx(monoFont, TextFormat("    Post-opt cost, sol %9.6f", planner_outputs.solution.cost), (Vector2){10, 150 + 5 * 30}, 20, 1, WHITE);
        DrawTextEx(monoFont, TextFormat("    Post-opt cost, pri %9.6f", planner_outputs_pri.solution.cost), (Vector2){10, 150 + 6 * 30}, 20, 1, MONOKAI_RED);
        DrawTextEx(monoFont, TextFormat("    Post-opt cost, aux %9.6f", planner_outputs_aux.solution.cost), (Vector2){10, 150 + 7 * 30}, 20, 1, MONOKAI_BLUE);

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

            const Trajectory<traj_length_opt>& traj = planner_outputs.solution.traj;
            const double total_time = planner_outputs.solution.total_time;
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
                for (int i = 0; i < planner_outputs.traj_pre_opt.length; i++) {
                    float t0 = i * dt;
                    float t1 = (i + 1) * dt;
                    float v0 = planner_outputs.traj_pre_opt.state_sequence(3, i);
                    float v1 = planner_outputs.traj_pre_opt.state_sequence(3, i + 1);

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

            const Trajectory<traj_length_opt>& traj = planner_outputs.solution.traj;
            const double total_time = planner_outputs.solution.total_time;
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

            const Trajectory<traj_length_opt>& traj = planner_outputs.solution.traj;
            const double total_time = planner_outputs.solution.total_time;
            const double dt = total_time / traj.length;
            
            // Plot the data

            // Post-opt traj
            if (showPostOptTraj) {
                for (int i = 0; i < traj.length; i++) {
                    float t0 = i * dt;
                    float t1 = (i + 1) * dt;
                    float v0 = planner_outputs.traj_pre_opt.state_sequence(3, i);
                    float v1 = planner_outputs.traj_pre_opt.state_sequence(3, i + 1);
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

            const Trajectory<traj_length_opt>& traj = planner_outputs.solution.traj;
            const double total_time = planner_outputs.solution.total_time;
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
//     const auto action_seq = steerCubic<traj_length_steer>(start, goal, steer_time);
//     for (int i = 0; i < traj_length_steer; ++i) {
//         std::cout << action_seq.col(i)(0) << std::endl;
//     }
//     return 0;
// }

// // SLIDER
// // TODO use this for filtering the time index range shown in the tree
// // TODO make the slider keep holding onto the mouse while click & drag engaged

// int main(void) {
//     InitWindow(800, 600, "Raylib Slider Widget");
//     SetTargetFPS(60);

//     float sliderValue = 0.5f;
//     Rectangle sliderBounds = { 100, 100, 300, 10 };

//     while (!WindowShouldClose()) {
//         BeginDrawing();
//         ClearBackground(RAYWHITE);

//         sliderValue = Slider(sliderBounds, sliderValue, 0.0f, 1.0f, "Volume");

//         EndDrawing();
//     }

//     CloseWindow();
//     return 0;
// }