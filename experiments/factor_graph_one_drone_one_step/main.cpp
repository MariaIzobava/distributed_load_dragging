/* ----------------------------------------------------------------------------
 * A C++ example for GTSAM that implements the factor graph for a robot
 * controlling a load with a cable.
 *
 * This is a trajectory optimization problem. We formulate it as a fixed-lag
 * smoother, where all states and controls over a time horizon are estimated
 * simultaneously.
 *
 * To compile, you will need GTSAM installed. See: https://gtsam.org/
 *
 * Then, use the provided CMakeLists.txt:
 * $ mkdir build && cd build
 * $ cmake ..
 * $ make
 * $ ./robot_load_trajectory
 * -------------------------------------------------------------------------- */

// Include necessary GTSAM headers
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

// My Factor Graph classes
#include "factor_graph_lib/cable_factors.hpp"
#include "factor_graph_lib/control_factors.hpp"
#include "factor_graph_lib/dynamics_factors.hpp"

using namespace std;
using namespace gtsam;

using symbol_t = gtsam::Symbol;


int main(int argc, char** argv) {
    // --- 2. Create the Factor Graph ---
    NonlinearFactorGraph graph;

    // --- Define problem parameters ---
    const int num_time_steps = 20; //20;
    const double dt = 0.005;
    const double robot_mass = 0.025; // kg
    const double load_mass = 0.001; // 0.0001;   // kg
    const double gravity = -9.81;
    const double mu = 0.1;
    const double cable_length = 1.0; // meters
    const double cable_stiffness = 500.0; // N/m
    const double cable_damping = 20.0;

    // --- Define weight Models ---
    // Weights for the penalty factors. Adjust these to control the "softness" of the constraints.
    // Higher weights mean stronger enforcement.
    double weight_tension_lower_bound = 1000000.0; // High weight to strongly enforce T >= 0
    double weight_cable_stretch = 1000000.0;     // Very high weight to strongly prevent cable from over-stretching
    double weight_tension_slack = 50.0;       // Moderate weight to discourage tension when the cable should be slack

    // These represent the uncertainty of each factor (1/covariance)
    auto dynamics_cost = noiseModel::Diagonal::Sigmas(
        (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
    auto goal_cost = noiseModel::Diagonal::Sigmas(
        (Vector(4) << 0.00005, 0.00005, 1000.1, 1000.1).finished());
    auto init_cost = noiseModel::Diagonal::Sigmas(
        (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
    auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
    auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-3);


    // --- Add Factors to the Graph ---
    Vector4 initial_load_state(0.0, 0.0, 0, 0);
    Vector4 initial_robot_state(-1, 0, 0, 0);
    graph.add(PriorFactor<Vector4>(symbol_t('x', 0), initial_robot_state, init_cost));
    graph.add(PriorFactor<Vector4>(symbol_t('l', 0), initial_load_state, init_cost));

    for (int k = 0; k < num_time_steps; ++k) {
        graph.add(RobotDynamicsFactor(
            symbol_t('x', k), 
            symbol_t('l', k), 
            symbol_t('u', k),
            symbol_t('t', k),
            symbol_t('x', k+1),
            dt,
            robot_mass,
            dynamics_cost
            ));

        graph.add(LoadDynamicsFactor(
            symbol_t('l', k),
            symbol_t('x', k),
            symbol_t('t', k),
            symbol_t('l', k+1),
            dt,
            load_mass,
            mu,
            gravity,
            dynamics_cost
            ));

        // Factor 1: Enforce T_k >= 0
        graph.add(TensionLowerBoundFactor(symbol_t('t', k), weight_tension_lower_bound, tension_cost));
        
        // Factor 2: Penalize if ||p_r - p_l|| > cable_length
        //graph.add(CableStretchPenaltyFactor(symbol_t('x', k), symbol_t('l', k), cable_length, weight_cable_stretch, tension_cost));
        
        // Factor 3: Penalize if T_k > 0 AND ||p_r - p_l|| < cable_length (i.e., tension in slack cable)
        graph.add(TensionSlackPenaltyFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length, weight_tension_slack, tension_cost));

        // Add a soft cost on control input to keep it constrained (prevents wild solutions)
        graph.add(MagnitudeUpperBoundFactor(symbol_t('u', k), 3, control_cost));
        graph.add(MagnitudeLowerBoundFactor(symbol_t('u', k), 0.003, control_cost));
    }

    // Add a goal cost on the final load state
    Vector4 final_load_goal(-0.01, 0.001, -0.05, 0.01); // Goal: move to (10, 5)
    graph.add(PriorFactor<Vector4>(symbol_t('l', num_time_steps), final_load_goal, goal_cost));


    cout << "Factor Graph built." << endl;
    graph.print("Factor Graph:\n");


    // --- 3. Create Initial Estimate ---
    // The optimizer needs an initial guess for all variables.
    Values initial_values;
    Vector2 init_u(0.0, 0.0);
    Vector1 init_t(0.0);
    for (int k = 0; k <= num_time_steps; ++k) {
        // A simple initial guess: stay at the start position.
        initial_values.insert(symbol_t('x', k), initial_robot_state);
        initial_values.insert(symbol_t('l', k), initial_load_state);
        // Initial guess for controls is zero.
        if (k < num_time_steps) {
            initial_values.insert(symbol_t('u', k), init_u);
            initial_values.insert(symbol_t('t', k), init_t);
        }
    }

    cout << "\nInitial values created." << endl;
    initial_values.print("Initial Values:\n");

    // // --- 4. Optimize ---
    LevenbergMarquardtParams params;
    params.setMaxIterations(1000);
    params.setVerbosity("TERMINATION"); // Print info at the end
    LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);

    cout << "\nOptimizing..." << endl;
    Values result = optimizer.optimize();
    cout << "Optimization complete." << endl;
    cout << "Initial Error: " << graph.error(initial_values) << endl;
    cout << "Final Error: " << graph.error(result) << endl;
    cout << "Opt error: " << optimizer.error() << endl;


    // --- 5. Print Results ---
    cout << "\nOptimized Trajectory:" << endl;
    for (int k = 0; k <= num_time_steps; ++k) {
        Vector4 robot_state = result.at<Vector4>(symbol_t('x', k));
        Vector4 load_state = result.at<Vector4>(symbol_t('l', k));
        cout << "--- Time Step " << k << " ---" << endl;
        cout << "  Robot Pos: (" << robot_state[0] << ", " << robot_state[1] << ", " << robot_state[2] << ", " << robot_state[3] << ")" << endl;
        cout << "  Load  Pos: (" << load_state[0]  << ", " << load_state[1] << ", " << load_state[2] << ", " << load_state[3] << ")" << endl;
        if (k < num_time_steps) {
            Vector2 control = result.at<Vector2>(symbol_t('u', k));
            cout << "  Control:   (" << control[0] << ", " << control[1] << ")" << endl;
            Vector1 tension = result.at<Vector1>(symbol_t('t', k));
            cout << "  Tension:   (" << tension[0] << ")" << endl;
        }
    }

    return 0;
}
