#ifndef FACTOR_EXECUTOR_TWO_ROBOTS_WITH_ORI_HPP
#define FACTOR_EXECUTOR_TWO_ROBOTS_WITH_ORI_HPP

#include <bits/stdc++.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/Ordering.h>


#include <Eigen/Dense>

#include "cable_factors.hpp"
#include "dynamics_factors_with_angle.hpp"
#include "control_factors.hpp"
#include "factor_executor.hpp"

using namespace std;
using namespace gtsam;
using symbol_t = gtsam::Symbol;


class FactorExecutorTwoRobotsWithOrientation: public FactorExecutor {

    Vector6 initial_load_state_;
    Vector4 initial_robot1_state_;
    Vector4 initial_robot2_state_;
    Vector6 final_load_goal_;
    double robot_height1_, robot_height2_;
    string debug_mode_;

public:

    FactorExecutorTwoRobotsWithOrientation(
        string debug_mode, 
        const Vector6& initial_load_state, 
        const Vector4& initial_robot1_state, 
        const Vector4& initial_robot2_state, 
        const Vector6& final_load_goal, 
        double robot_height1, 
        double robot_height2,
        const map<string, double>& tune_d,
        const map<string, bool>& tune_b
        ):
    FactorExecutor(tune_d, tune_b),
    debug_mode_(debug_mode), 
    initial_load_state_(initial_load_state), 
    initial_robot1_state_(initial_robot1_state),
    initial_robot2_state_(initial_robot2_state), 
    final_load_goal_(final_load_goal), 
    robot_height1_(robot_height1),
    robot_height2_(robot_height2) {}

    std::vector<double> run(map<string, double>& factor_errors, double& pos_error) const override {

        NonlinearFactorGraph graph;

        // --- Define problem parameters ---
        const int num_time_steps = 20;
        const double dt = 0.005;
        const double robot_mass = 0.025; // kg
        const double load_mass = 0.015;   // kg
        const double inertia = 0.000399; //399
        const double gravity = 9.81;
        const double mu = 0.3;
        const double mu2 = 0.3;
        const double cable_length1 = sqrt(1.03 * 1.03 - (robot_height1_ - 0.2) * (robot_height1_ - 0.2));
        const double cable_length2 = sqrt(1.03 * 1.03 - (robot_height2_ - 0.2) * (robot_height2_ - 0.2));

        // VALUES TO TUNE
        // =============================
        // =============================
        const int atp2 = 1;  // attachement point of the 2nd drone. Values are 1 or 2 depending on which face of the cube it's attached to
        const double u_upper_bound = getd("u_upper_bound", 0.5); 
        const double u_lower_bound = getd("u_lower_bound", 0.003);

        double weight_tension_lower_bound = getd("weight_tension_lower_bound", 1000000.0);
        double weight_cable_stretch = getd("weight_cable_stretch", 100.0) ;
        double weight_tension_slack = getd("weight_tension_slack", 50.0);
        double weight_tether_tension = getd("weight_tether_tension", 0.266292); //0.0008096

        double cable_stretch_penalty_offset = getd("cable_stretch_penalty_offset", 0.0);
        double tension_slack_penalty_offset = getd("tension_slack_penalty_offset", 0.2); 
        double tether_tension_offset = getd("tether_tension_offset", 0.3); //0.38672

        bool have_uk_prior = getb("have_uk_prior", true);
        
        bool have_tension_lower_bound_factor = getb("have_tension_lower_bound_factor", false);
        bool have_cable_stretch_factor = getb("have_cable_stretch_factor", false);
        bool have_tension_slack_penalty_factor = getb("have_tension_slack_penalty_factor", false); 
        bool have_tether_tension_factor = getb("have_tether_tension_factor", true);

        bool have_trajectory_reference_factor = getb("have_trajectory_reference_factor", true);

        double goal_cost_v = getd("goal_cost", 0.00005); 

        // STATE PRIORS
        auto init_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
        auto init_cost_with_angle = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.000005, 0.000005, 0.000005, 0.000005, 0.000005, 0.000005).finished());
        auto goal_cost_with_angle = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.000005, 0.000005, 0.00005, 1000.1, 1000.1, 1000.1).finished());

        // DYNAMICS
        auto dynamics_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
        auto dynamics_cost_with_angle = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.001, 0.001, 0.01, 0.001, 0.001, 0.01).finished());

        // CONTROL
        auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-2);
        auto control_interim_cost = noiseModel::Diagonal::Sigmas(
            (Vector(2) << 10.0, 10.0).finished());

        // TENSION
        auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
        // =============================
        // =============================


        graph.add(PriorFactor<Vector4>(symbol_t('x', 0), initial_robot1_state_, init_cost));
        graph.add(PriorFactor<Vector4>(symbol_t('X', 0), initial_robot2_state_, init_cost));
        graph.add(PriorFactor<Vector6>(symbol_t('l', 0), initial_load_state_, init_cost_with_angle));
        graph.add(PriorFactor<Vector6>(symbol_t('l', num_time_steps), final_load_goal_, goal_cost_with_angle));

        double xdiff = (final_load_goal_(0) - initial_load_state_(0)) / num_time_steps;
        double ydiff = (final_load_goal_(1) - initial_load_state_(1)) / num_time_steps;
        double adiff = (final_load_goal_(2) - initial_load_state_(2)) / num_time_steps;
        Vector6 diff(xdiff, ydiff, adiff, 0, 0, 0);

        if (debug_mode_ == "one_of" || debug_mode_ == "sim") {
        CableVectorHelper h1(initial_robot1_state_, initial_load_state_);
        CableVectorHelper h2(initial_robot2_state_, initial_load_state_, 2);
        double dist1 = sqrt((initial_robot1_state_[0] - h1.ap_x) * (initial_robot1_state_[0] - h1.ap_x)  + (initial_robot1_state_[1] - h1.ap_y) * (initial_robot1_state_[1] - h1.ap_y) ) ;
        double dist2 = sqrt((initial_robot2_state_[0] - h2.ap_x) * (initial_robot2_state_[0] - h2.ap_x)  + (initial_robot2_state_[1] - h2.ap_y) * (initial_robot2_state_[1] -  h2.ap_y) );
        cout << "Cable lengths: " << cable_length1 << ' ' << cable_length2 << std::endl;
        cout << "  Actual dist: " << dist1 << ' '<< dist2 << endl;
        }

        for (int k = 0; k < num_time_steps; ++k) {
            graph.add(RobotDynamicsWithLoadOrientationFactor(
                symbol_t('x', k), 
                symbol_t('l', k),
                symbol_t('u', k),
                symbol_t('t', k),
                symbol_t('x', k+1),
                dt,
                robot_mass,
                true,
                dynamics_cost
                ));

            graph.add(RobotDynamicsWithLoadOrientationFactor(
                symbol_t('X', k), 
                symbol_t('l', k),
                symbol_t('U', k),
                symbol_t('T', k),
                symbol_t('X', k+1),
                dt,
                robot_mass,
                true,
                dynamics_cost,
                atp2
                ));

            //graph.add(RobotsDistanceFactor(symbol_t('x', k), symbol_t('X', k), 0.2, tension_cost));

            graph.add(LoadDynamicsTwoRobotsWithLoadOrientationFactor(
                symbol_t('l', k),
                symbol_t('x', k),
                symbol_t('t', k),
                symbol_t('X', k),
                symbol_t('T', k),
                symbol_t('l', k+1),
                dt,
                load_mass,
                mu,
                mu2,
                gravity,
                inertia,
                true,
                atp2,
                dynamics_cost_with_angle
                ));

            // Factor 1: Enforce T_k >= 0
            if (have_tension_lower_bound_factor) {
            graph.add(TensionLowerBoundFactor(symbol_t('t', k), weight_tension_lower_bound, tension_cost));
            graph.add(TensionLowerBoundFactor(symbol_t('T', k), weight_tension_lower_bound, tension_cost));
            }
            
            // Factor 2: Penalize if ||p_r - p_l|| > cable_length
            if (have_cable_stretch_factor) {
            graph.add(CableStretchPenaltyWithOrientationFactor(symbol_t('x', k), symbol_t('l', k), cable_length1, weight_cable_stretch, tension_cost));
            graph.add(CableStretchPenaltyWithOrientationFactor(symbol_t('X', k), symbol_t('l', k), cable_length2, weight_cable_stretch, tension_cost, atp2));
            }

            // Factor 3: Penalize if T_k > 0 AND ||p_r - p_l|| < cable_length (i.e., tension in slack cable)
            if (have_tension_slack_penalty_factor) {
            graph.add(TensionSlackPenaltyWithLoadOrientationFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length1, weight_tension_slack, tension_cost));
            graph.add(TensionSlackPenaltyWithLoadOrientationFactor(symbol_t('T', k), symbol_t('X', k), symbol_t('l', k), cable_length2, weight_tension_slack, tension_cost, atp2));
            }

            if (have_tether_tension_factor) {
            graph.add(TethertensionFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length1 - tether_tension_offset, weight_tether_tension, tension_cost));
            graph.add(TethertensionFactor(symbol_t('T', k), symbol_t('X', k), symbol_t('l', k), cable_length2 - tether_tension_offset, weight_tether_tension, tension_cost, atp2));
            }

            graph.add(MagnitudeUpperBoundFactor(symbol_t('u', k), u_upper_bound, control_cost));
            graph.add(MagnitudeLowerBoundFactor(symbol_t('u', k), u_lower_bound, control_cost));
            graph.add(MagnitudeUpperBoundFactor(symbol_t('U', k), u_upper_bound, control_cost));
            graph.add(MagnitudeLowerBoundFactor(symbol_t('U', k), u_lower_bound, control_cost));

            if (k > 0 && have_trajectory_reference_factor) {
                Vector6 mid_state(6);
                mid_state << initial_load_state_ + k * diff;
                graph.add(PriorFactor<Vector6>(symbol_t('l', k), mid_state, goal_cost_with_angle));
            }

            if (have_uk_prior) {
            graph.add(PriorFactor<Vector2>(symbol_t('u', k), Vector2::Zero(), control_interim_cost));
            graph.add(PriorFactor<Vector2>(symbol_t('U', k), Vector2::Zero(), control_interim_cost));
            }
        }

        // --- 3. Create Initial Estimate ---
        Values initial_values;
        Vector2 init_u(0.0, 0.0);
        Vector1 init_t(0.0);
        for (int k = 0; k <= num_time_steps; ++k) {
            initial_values.insert(symbol_t('x', k), initial_robot1_state_);
            initial_values.insert(symbol_t('X', k), initial_robot2_state_);
            initial_values.insert(symbol_t('l', k), initial_load_state_);
            if (k < num_time_steps) {
                initial_values.insert(symbol_t('u', k), init_u);
                initial_values.insert(symbol_t('t', k), init_t);
                initial_values.insert(symbol_t('U', k), init_u);
                initial_values.insert(symbol_t('T', k), init_t);
            }
        }

        Values result = runOptimizer(debug_mode_, graph, initial_values, factor_errors, dt, mu, load_mass, gravity, mu2, inertia, 2, true);

        Vector6 last_state = result.at<Vector6>(symbol_t('l', num_time_steps));
        double a1 = sqrt((final_load_goal_[0] - last_state[0]) * (final_load_goal_[0] - last_state[0]) + (final_load_goal_[1] - last_state[1]) * (final_load_goal_[1] - last_state[1]));
        double a2 = sqrt((final_load_goal_[0] - initial_load_state_[0]) * (final_load_goal_[0] - initial_load_state_[0]) + (final_load_goal_[1] - initial_load_state_[1]) * (final_load_goal_[1] - initial_load_state_[1]));
        pos_error = a1 / a2;

        Vector4 next_state1 = result.at<Vector4>(symbol_t('x', 1));
        Vector4 next_state2 = result.at<Vector4>(symbol_t('X', 1));
        Vector2 next_ctrl1 = result.at<Vector2>(symbol_t('u', 0));
        Vector2 next_ctrl2 = result.at<Vector2>(symbol_t('U', 0));
        Vector1 tension1 = result.at<Vector1>(symbol_t('t', 0));
        Vector1 tension2 = result.at<Vector1>(symbol_t('T', 0));

        if (debug_mode_ == "one_of" || debug_mode_ == "sim") {
        cout << "  Cur tension: " << tension1[0] << ' ' << tension2[0] << endl;
        cout << "Next controls: " << next_ctrl1[0] << ' ' << next_ctrl1[1] << ' ' << next_ctrl2[0] << ' ' << next_ctrl2[1] << endl;
        }
        return {next_state1[2], next_state1[3], next_state2[2], next_state2[3], tension1[0], tension2[0], next_ctrl1[0], next_ctrl1[1], next_ctrl2[0], next_ctrl2[1] };
    }
};

//         tether_tension_offset 0.28672
        // weight_tether_tension 0.4096

// tether_tension_offset 0.0001575
// weight_tether_tension 0.65536
// have_trajectory_reference_factor 1
// have_uk_prior 0

//  Position mean: 0.00142579
//  Load error mean: 13658.2


//  tether_tension_offset 0.0001575
// weight_tether_tension 20.9715
// have_trajectory_reference_factor 1
// have_uk_prior 1

//  Position mean: 4.25172e-06
//  Load error mean: 153151


//  tether_tension_offset 7e-05
// weight_tether_tension 10.4858
// have_trajectory_reference_factor 1
// have_uk_prior 1

//  Position mean: 3.5451e-06
//  Load error mean: 136141

// =======================
// tether_tension_offset 0.000105
// weight_tether_tension 10.4858
// have_trajectory_reference_factor 1
// have_uk_prior 1

//  Position mean: 3.55206e-06
//  Load error mean: 131986


//  tether_tension_offset 0.000105
// weight_tether_tension 2.62144
// have_trajectory_reference_factor 1
// have_uk_prior 1

//  Position mean: 2.59747e-06
//  Load error mean: 18575.3

// =======================
// tether_tension_offset 0.0001575
// weight_tether_tension 2.62144
// have_trajectory_reference_factor 1
// have_uk_prior 1

//  Position mean: 2.61502e-06
//  Load error mean: 18699.5

// =======================
// tether_tension_offset 7e-05
// weight_tether_tension 2.62144
// have_trajectory_reference_factor 1
// have_uk_prior 1

//  Position mean: 2.62003e-06
//  Load error mean: 18742.4

// =======================
// tether_tension_offset 0.00023625
// weight_tether_tension 2.62144
// have_trajectory_reference_factor 1
// have_uk_prior 1

//  Position mean: 2.64288e-06
//  Load error mean: 18795.3


//  tether_tension_offset 0.0001575
// weight_tether_tension 0.00512
// have_trajectory_reference_factor 0
// have_uk_prior 1

//  Position mean: 1.24812e-06
//  Load error mean: 11145.6

// =======================
// tether_tension_offset 7e-05
// weight_tether_tension 0.32768
// have_trajectory_reference_factor 1
// have_uk_prior 1

//  Position mean: 1.2484e-06
//  Load error mean: 16669.6

// =======================
// tether_tension_offset 7e-05
// weight_tether_tension 0.04096
// have_trajectory_reference_factor 0
// have_uk_prior 0

//  Position mean: 1.24841e-06
//  Load error mean: 14197

// =======================
// tether_tension_offset 0.00023625
// weight_tether_tension 0.00016
// have_trajectory_reference_factor 0
// have_uk_prior 1

//  Position mean: 1.24879e-06
//  Load error mean: 10060.6

// =======================
// tether_tension_offset 0.000354375
// weight_tether_tension 0.00512
// have_trajectory_reference_factor 0
// have_uk_prior 1

//  Position mean: 1.24898e-06
//  Load error mean: 11093.6

// =======================
// tether_tension_offset 0.0001575
// weight_tether_tension 0.00016
// have_trajectory_reference_factor 0
// have_uk_prior 1

//  Position mean: 1.249e-06
//  Load error mean: 10103.6

//  ========================== 
// tether_tension_offset 0.000354375  7e1-5     0.000797344   0.000531562
// weight_tether_tension 0.00032      0.00064   0.00128       0.00128
// have_trajectory_reference_factor 1 1         1             1
// have_uk_prior 1                    1         1             1

//  Position mean: 1.249e-06
//  Load error mean: 10103.6


#endif // FACTOR_EXECUTOR_TWO_ROBOTS_WITH_ORI_HPP