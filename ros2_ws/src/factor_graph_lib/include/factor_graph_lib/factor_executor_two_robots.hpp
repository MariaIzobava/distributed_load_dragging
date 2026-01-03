#ifndef FACTOR_EXECUTOR_TWO_ROBOTS_HPP
#define FACTOR_EXECUTOR_TWO_ROBOTS_HPP

#include <bits/stdc++.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/Ordering.h>
#include <Eigen/Dense>

#include "cable_factors.hpp"
#include "dynamics_factors.hpp"
#include "control_factors.hpp"
#include "factor_executor.hpp"

using namespace std;
using namespace gtsam;
using symbol_t = gtsam::Symbol;


class FactorExecutorTwoRobots: public FactorExecutor {

    Vector4 initial_load_state_;
    Vector4 initial_robot1_state_;
    Vector4 initial_robot2_state_;
    Vector4 final_load_goal_;
    double robot_height1_, robot_height2_;
    std::vector<double> desired_robot_heights_;
    Vector2 last_u1_;
    Vector2 last_u2_;
    string debug_mode_;

public:

    FactorExecutorTwoRobots(
        string debug_mode, 
        const Vector4& initial_load_state, 
        const Vector4& initial_robot1_state, 
        const Vector4& initial_robot2_state, 
        const Vector4& final_load_goal, 
        double robot_height1, 
        double robot_height2,
        const std::vector<double>& desired_robot_heights,
        const Vector2& last_u1,
        const Vector2& last_u2,
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
    robot_height2_(robot_height2),
    desired_robot_heights_(desired_robot_heights),
    last_u1_(last_u1),
    last_u2_(last_u2) {}

    FactorExecutorResult run(map<string, double>& factor_errors, double& pos_error) const override {

        NonlinearFactorGraph graph;

        // --- Define problem parameters ---
        const int num_time_steps = 20;
        const double dt = 0.005;
        const double robot_mass = 0.025; // kg
        const double load_mass = 0.015;   // kg
        const double gravity = 9.81; 
        const double mu = 0.3;
        const double half_size = 0.2;
        const double real_cable_length = 1.03;

        
        const double cable_length1 = half_size + sqrt(real_cable_length * real_cable_length - (robot_height1_ - half_size) * (robot_height1_ - half_size));
        const double cable_length2 = half_size + sqrt(real_cable_length * real_cable_length - (robot_height2_ - half_size) * (robot_height2_ - half_size));

        // VALUES TO TUNE
        // =============================
        // =============================
        const double u_upper_bound = getd("u_upper_bound", 0.5); 
        const double u_lower_bound = getd("u_lower_bound", 0.003);

        double weight_tension_lower_bound = getd("weight_tension_lower_bound", 1000000.0);
        double weight_cable_stretch = getd("weight_cable_stretch", 100.0) ;
        double weight_tension_slack = getd("weight_tension_slack", 50.0);
        double weight_tether_tension = getd("weight_tether_tension", 0.266292);// 12.5);

        double cable_stretch_penalty_offset = getd("cable_stretch_penalty_offset", 0.0);
        double tension_slack_penalty_offset = getd("tension_slack_penalty_offset", 0.02); 
        double tether_tension_offset = getd("tether_tension_offset", 0.3 );  //0.021875
        
        bool have_tension_lower_bound_factor = getb("have_tension_lower_bound_factor", false);
        bool have_cable_stretch_factor = getb("have_cable_stretch_factor", false);
        bool have_tension_slack_penalty_factor = getb("have_tension_slack_penalty_factor", false); 
        bool have_tether_tension_factor = getb("have_tether_tension_factor", true);

        bool have_trajectory_reference_factor = getb("have_trajectory_reference_factor", true);

        double goal_cost_v = getd("goal_cost", 0.00005); 

        // STATE PRIORS
        auto init_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
        auto goal_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << goal_cost_v, goal_cost_v, 1000.1, 1000.1).finished());

        // DYNAMICS
        auto dynamics_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
        
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
        graph.add(PriorFactor<Vector4>(symbol_t('l', 0), initial_load_state_, init_cost));
        graph.add(PriorFactor<Vector4>(symbol_t('l', num_time_steps), final_load_goal_, goal_cost));

        double xdiff = (final_load_goal_(0) - initial_load_state_(0)) / num_time_steps;
        double ydiff = (final_load_goal_(1) - initial_load_state_(1)) / num_time_steps;
        Vector4 diff(xdiff, ydiff, 0, 0);

        if (debug_mode_ == "one_of" || debug_mode_ == "sim") {
        cout << "Cable lengths: " << cable_length1 << ' ' << cable_length2 << std::endl;
        cout << "  Actual dist: " << sqrt((initial_robot1_state_[0] - initial_load_state_[0]) * (initial_robot1_state_[0] - initial_load_state_[0])  + (initial_robot1_state_[1] - initial_load_state_[1]) * (initial_robot1_state_[1] - initial_load_state_[1]) ) << ' ' << sqrt((initial_robot2_state_[0] - initial_load_state_[0]) * (initial_robot2_state_[0] - initial_load_state_[0])  + (initial_robot2_state_[1] - initial_load_state_[1]) * (initial_robot2_state_[1] - initial_load_state_[1]) ) << endl;
        }

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

            graph.add(RobotDynamicsFactor(
                symbol_t('X', k), 
                symbol_t('l', k),
                symbol_t('U', k),
                symbol_t('T', k),
                symbol_t('X', k+1),
                dt,
                robot_mass,
                dynamics_cost
                ));

            graph.add(RobotsDistanceFactor(symbol_t('x', k), symbol_t('X', k), 0.2, tension_cost));

            graph.add(LoadDynamicsTwoRobotsFactor(
                symbol_t('l', k),
                symbol_t('x', k),
                symbol_t('t', k),
                symbol_t('X', k),
                symbol_t('T', k),
                symbol_t('l', k+1),
                dt,
                load_mass,
                mu,
                gravity,
                dynamics_cost
                ));

            // Factor 1: Enforce T_k >= 0
            if (have_tension_lower_bound_factor) {
            graph.add(TensionLowerBoundFactor(symbol_t('t', k), weight_tension_lower_bound, tension_cost));
            graph.add(TensionLowerBoundFactor(symbol_t('T', k), weight_tension_lower_bound, tension_cost));
            }

            // Factor 2: Penalize if ||p_r - p_l|| > cable_length
            if (have_cable_stretch_factor) {
            graph.add(CableStretchPenaltyFactor(symbol_t('x', k), symbol_t('l', k), cable_length1 + cable_stretch_penalty_offset, weight_cable_stretch, tension_cost));
            graph.add(CableStretchPenaltyFactor(symbol_t('X', k), symbol_t('l', k), cable_length2 + cable_stretch_penalty_offset, weight_cable_stretch, tension_cost));
            }

            // Factor 3: Penalize if T_k > 0 AND ||p_r - p_l|| < cable_length (i.e., tension in slack cable)
            if (have_tension_slack_penalty_factor) {
                //cable_length1 - tension_slack_penalty_offset
            graph.add(TensionSlackPenaltyFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length1 - tension_slack_penalty_offset, weight_tension_slack, tension_cost));
            graph.add(TensionSlackPenaltyFactor(symbol_t('T', k), symbol_t('X', k), symbol_t('l', k), cable_length2 - tension_slack_penalty_offset, weight_tension_slack, tension_cost));
            }

            if (have_tether_tension_factor) {
            graph.add(TethertensionFactorNoOri(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length1 - tether_tension_offset, weight_tether_tension, tension_cost));
            graph.add(TethertensionFactorNoOri(symbol_t('T', k), symbol_t('X', k), symbol_t('l', k), cable_length2 - tether_tension_offset, weight_tether_tension, tension_cost));
            }

            graph.add(MagnitudeUpperBoundFactor(symbol_t('u', k), u_upper_bound, control_cost));
            graph.add(MagnitudeLowerBoundFactor(symbol_t('u', k), u_lower_bound, control_cost));
            graph.add(MagnitudeUpperBoundFactor(symbol_t('U', k), u_upper_bound, control_cost));
            graph.add(MagnitudeLowerBoundFactor(symbol_t('U', k), u_lower_bound, control_cost));

            if (k > 0 && have_trajectory_reference_factor) {
                Vector4 mid_state(4);
                mid_state << initial_load_state_ + k * diff;
                graph.add(PriorFactor<Vector4>(symbol_t('l', k), mid_state, goal_cost));
            }

            //Vector2 init_u(0.0, 0.0);
            graph.add(PriorFactor<Vector2>(symbol_t('u', k), Vector2::Zero(), control_interim_cost));
            graph.add(PriorFactor<Vector2>(symbol_t('U', k), Vector2::Zero(), control_interim_cost));
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

        Values result = runOptimizer(debug_mode_, graph, initial_values, factor_errors, dt, mu, load_mass, gravity, 0.0, 0.0, 2, false);

        Vector4 last_state = result.at<Vector4>(symbol_t('l', num_time_steps));
        double a1 = sqrt((final_load_goal_[0] - last_state[0]) * (final_load_goal_[0] - last_state[0]) + (final_load_goal_[1] - last_state[1]) * (final_load_goal_[1] - last_state[1]));
        double a2 = sqrt((final_load_goal_[0] - initial_load_state_[0]) * (final_load_goal_[0] - initial_load_state_[0]) + (final_load_goal_[1] - initial_load_state_[1]) * (final_load_goal_[1] - initial_load_state_[1]));
        pos_error = graph.error(result);

        Vector4 next_state1 = result.at<Vector4>(symbol_t('x', 1));
        Vector4 next_state2 = result.at<Vector4>(symbol_t('X', 1));
        Vector2 next_ctrl1 = result.at<Vector2>(symbol_t('u', 0));
        Vector2 next_ctrl2 = result.at<Vector2>(symbol_t('U', 0));
        Vector1 cur_t1 = result.at<Vector1>(symbol_t('t', 0));
        Vector1 cur_t2 = result.at<Vector1>(symbol_t('T', 0));

        FactorExecutorResult exec_result = {
            {
                .drone_vel = {next_state1[2], next_state1[3], desired_robot_heights_[0] - robot_height1_},
                .controls = {next_ctrl1(0), next_ctrl1(1), 0.0},
                .tension = cur_t1(0),
            },
            {
                .drone_vel = {next_state2[2], next_state2[3], desired_robot_heights_[1] - robot_height2_},
                .controls = {next_ctrl2(0), next_ctrl2(1), 0.0},
                .tension = cur_t2(0),
            },
        };

        if (debug_mode_ == "one_of" || debug_mode_ == "sim") {
            cout << "  Cur tension: " << cur_t1[0] << ' ' << cur_t2[0] << endl;
            cout << "Next controls: " << next_ctrl1[0] << ' ' << next_ctrl1[1] << ' ' << next_ctrl2[0] << ' ' << next_ctrl2[1] << endl;
        }

        return exec_result;
    }
};

#endif // FACTOR_EXECUTOR_TWO_ROBOTS_HPP