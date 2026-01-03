#ifndef FACTOR_EXECUTOR_MULTI_ROBOT_WITH_ORI_HPP
#define FACTOR_EXECUTOR_MULTI_ROBOT_WITH_ORI_HPP

#include <bits/stdc++.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/Ordering.h>
#include <Eigen/Dense>

#include "cable_factors.hpp"
#include "control_factors.hpp"
#include "dynamics_factors_with_angle.hpp"
#include "dynamics_factors_multi_robots_with_angle.hpp"
#include "factor_executor.hpp"

using namespace std;
using namespace gtsam;
using symbol_t = gtsam::Symbol;


class FactorExecutorMultiRobotsWithOrientation: public FactorExecutor {

    Vector6 initial_load_state_;
    std::vector<Vector4> initial_robot_states_;
    Vector6 final_load_goal_;
    std::vector<double> robot_heights_, desired_robot_heights_;
    string debug_mode_;
    int robot_num_;

public:

    FactorExecutorMultiRobotsWithOrientation(
        string debug_mode, 
        int robot_num,
        const Vector6& initial_load_state, 
        const std::vector<Vector4>& initial_robot_states, 
        const Vector6& final_load_goal, 
        const std::vector<double>& robot_heights,
        const std::vector<double>& desired_robot_heights,
        const map<string, double>& tune_d,
        const map<string, bool>& tune_b
        ):
    FactorExecutor(tune_d, tune_b),
    debug_mode_(debug_mode), 
    robot_num_(robot_num),
    initial_load_state_(initial_load_state), 
    initial_robot_states_(initial_robot_states), 
    final_load_goal_(final_load_goal), 
    robot_heights_(robot_heights),
    desired_robot_heights_(desired_robot_heights) {}

    FactorExecutorResult run(map<string, double>& factor_errors, double& pos_error) const override {

        NonlinearFactorGraph graph;

        // --- Define problem parameters ---
        const int num_time_steps = 20;
        const double dt = 0.005;
        const double robot_mass = 0.025; // kg
        const double load_mass = 0.015;   // kg
        const double inertia = 0.000399;
        const double gravity = 9.81;
        const double mu = 0.3;
        const double mu2 = 0.3;
        std::vector<double> cable_lengths;
        for (int i = 0; i < robot_num_; i++) {
            double h = robot_heights_[i];
            cable_lengths.push_back(sqrt(1.03 * 1.03 - (h - 0.2) * (h - 0.2)));
        }

        // VALUES TO TUNE
        // =============================
        // =============================
        double u_upper_bound = getd("u_upper_bound", 0.5); 
        // For detacheable scenario:
        // double u_upper_bound = getd("u_upper_bound", 0.45); 
        // if (robot_num_ == 3) {
        //     u_upper_bound = 0.55;
        // }
        const double u_lower_bound = getd("u_lower_bound", 0.003);

        double weight_tension_lower_bound = getd("weight_tension_lower_bound", 1000000.0);
        double weight_cable_stretch = getd("weight_cable_stretch", 100.0) ;
        double weight_tension_slack = getd("weight_tension_slack", 50.0);
        double weight_tether_tension = getd("weight_tether_tension", 0.296292); //0.296292 for 4 segments!

        double cable_stretch_penalty_offset = getd("cable_stretch_penalty_offset", 0.0);
        double tension_slack_penalty_offset = getd("tension_slack_penalty_offset", 0.2); 
        double tether_tension_offset = getd("tether_tension_offset", 0.3); //0.38672

        bool have_uk_prior = getb("have_uk_prior", true);
        
        bool have_tension_lower_bound_factor = getb("have_tension_lower_bound_factor", false);
        bool have_cable_stretch_factor = getb("have_cable_stretch_factor", false);
        bool have_tension_slack_penalty_factor = getb("have_tension_slack_penalty_factor", false); 
        bool have_tether_tension_factor = getb("have_tether_tension_factor", true);

        bool have_trajectory_reference_factor = getb("have_trajectory_reference_factor", true);

        double min_distance_between_robots = getd("min_distance_between_robots", 0.05);

        // double goal_cost_v = getd("goal_cost", 0.00005); 

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
        auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
        auto control_interim_cost = noiseModel::Diagonal::Sigmas(
            (Vector(2) << 10.0, 10.0).finished());

        // TENSION
        auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);

        // DISTANCE
        auto distance_cost = noiseModel::Isotropic::Sigma(1, 1e-2);
        // =============================
        // =============================


        for (int i = 0; i < robot_num_; i++) {
            graph.add(PriorFactor<Vector4>(symbol_t(x_[i], 0), initial_robot_states_[i], init_cost));
        }
        graph.add(PriorFactor<Vector6>(symbol_t('l', 0), initial_load_state_, init_cost_with_angle));
        graph.add(PriorFactor<Vector6>(symbol_t('l', num_time_steps), final_load_goal_, goal_cost_with_angle));

        double xdiff = (final_load_goal_(0) - initial_load_state_(0)) / num_time_steps;
        double ydiff = (final_load_goal_(1) - initial_load_state_(1)) / num_time_steps;
        double adiff = (final_load_goal_(2) - initial_load_state_(2)) / num_time_steps;
        Vector6 diff(xdiff, ydiff, adiff, 0, 0, 0);

        if (debug_mode_ == "one_of" || debug_mode_ == "sim") {
        for (int i = 0; i < robot_num_; i++) {
        const auto& state = initial_robot_states_[i];
        CableVectorHelper h(state, initial_load_state_);
        double dist = sqrt((state[0] - h.ap_x) * (state[0] - h.ap_x)  + (state[1] - h.ap_y) * (state[1] - h.ap_y));
        cout << "Cable length " << i + 1 << ": " << cable_lengths[i] << std::endl;
        cout << " Actual dist " << i + 1 << ": " << dist << endl;
        }
        }

        for (int k = 0; k < num_time_steps; ++k) {
            for (int i = 0; i < robot_num_; i++) {
                graph.add(RobotDynamicsWithLoadOrientationFactor(
                    symbol_t(x_[i], k), 
                    symbol_t('l', k),
                    symbol_t(u_[i], k),
                    symbol_t(t_[i], k),
                    symbol_t(x_[i], k+1),
                    dt,
                    robot_mass,
                    true,
                    dynamics_cost
                    ));

                for (int j = i + 1; j < robot_num_; j++) {
                    graph.add(RobotsDistanceFactor(symbol_t(x_[i], k), symbol_t(x_[j], k), min_distance_between_robots, distance_cost));
                }

                // Didn't use for both 3 and 4 drones with 2 seg
                //graph.add(RobotLoadWithOrientationFactor(symbol_t(x_[i], k), symbol_t('l', k), cable_lengths[i] - 0.1, 1.0, tension_cost));
            }

            if (robot_num_ == 4)  {
            // graph.add(LoadDynamicsMultiRobotsWithLoadOrientationFactor4(
            //     symbol_t('l', k),
            //     symbol_t(x_[0], k),
            //     symbol_t(t_[0], k),
            //     symbol_t(x_[1], k),
            //     symbol_t(t_[1], k),
            //     symbol_t(x_[2], k),
            //     symbol_t(t_[2], k),
            //     symbol_t(x_[3], k),
            //     symbol_t(t_[3], k),
            //     symbol_t('l', k+1),
            //     dt,
            //     load_mass,
            //     mu,
            //     mu2,
            //     gravity,
            //     inertia,
            //     dynamics_cost_with_angle
            //     ));

            graph.add(LoadDynamicsFourRobotsWithLoadOrientationFactor(
                symbol_t('l', k),
                symbol_t(x_[0], k),
                symbol_t(t_[0], k),
                symbol_t(x_[1], k),
                symbol_t(t_[1], k),
                symbol_t(x_[2], k),
                symbol_t(t_[2], k),
                symbol_t(x_[3], k),
                symbol_t(t_[3], k),
                symbol_t('l', k+1),
                dt,
                load_mass,
                mu,
                mu2,
                gravity,
                inertia,
                dynamics_cost_with_angle
                ));

            
            } else if (robot_num_ == 3) {
            // graph.add(LoadDynamicsMultiRobotsWithLoadOrientationFactor3(
            //     symbol_t('l', k),
            //     symbol_t(x_[0], k),
            //     symbol_t(t_[0], k),
            //     symbol_t(x_[1], k),
            //     symbol_t(t_[1], k),
            //     symbol_t(x_[2], k),
            //     symbol_t(t_[2], k),
            //     symbol_t('l', k+1),
            //     dt,
            //     load_mass,
            //     mu,
            //     mu2,
            //     gravity,
            //     inertia,
            //     dynamics_cost_with_angle
            //     ));

            

            graph.add(LoadDynamicsThreeRobotsWithLoadOrientationFactor(
                symbol_t('l', k),
                symbol_t(x_[0], k),
                symbol_t(t_[0], k),
                symbol_t(x_[1], k),
                symbol_t(t_[1], k),
                symbol_t(x_[2], k),
                symbol_t(t_[2], k),
                symbol_t('l', k+1),
                dt,
                load_mass,
                mu,
                mu2,
                gravity,
                inertia,
                dynamics_cost_with_angle
                ));
            } else if (robot_num_ == 2) {
            graph.add(LoadDynamicsTwoRobotsWithLoadOrientationFactor(
                symbol_t('l', k),
                symbol_t(x_[0], k),
                symbol_t(t_[0], k),
                symbol_t(x_[1], k),
                symbol_t(t_[1], k),
                symbol_t('l', k+1),
                dt,
                load_mass,
                mu,
                mu2,
                gravity,
                inertia,
                true,
                1,
                dynamics_cost_with_angle
                ));

            // graph.add(LoadDynamicsMultiRobotsWithLoadOrientationFactor2(
            //     symbol_t('l', k),
            //     symbol_t('x', k),
            //     symbol_t('t', k),
            //     symbol_t('X', k),
            //     symbol_t('T', k),
            //     symbol_t('l', k+1),
            //     dt,
            //     load_mass,
            //     mu,
            //     mu2,
            //     gravity,
            //     inertia,
            //     dynamics_cost_with_angle
            //     ));
            }

            // Factor 1: Enforce T_k >= 0
            if (have_tension_lower_bound_factor) {
            for (int i = 0; i < robot_num_; i++) {
            graph.add(TensionLowerBoundFactor(symbol_t(t_[i], k), weight_tension_lower_bound, tension_cost));
            }
            }
            
            // Factor 2: Penalize if ||p_r - p_l|| > cable_length
            if (have_cable_stretch_factor) {
            for (int i = 0; i < robot_num_; i++) {
            graph.add(CableStretchPenaltyWithOrientationFactor(symbol_t(x_[i], k), symbol_t('l', k), cable_lengths[i], weight_cable_stretch, tension_cost));
            }
            }

            // Factor 3: Penalize if T_k > 0 AND ||p_r - p_l|| < cable_length (i.e., tension in slack cable)
            if (have_tension_slack_penalty_factor) {
            for (int i = 0; i < robot_num_; i++) {
            graph.add(TensionSlackPenaltyWithLoadOrientationFactor(symbol_t(t_[i], k), symbol_t(x_[i], k), symbol_t('l', k), cable_lengths[i], weight_tension_slack, tension_cost));
            }
            }

            if (have_tether_tension_factor) {
            for (int i = 0; i < robot_num_; i++) {
            graph.add(TethertensionFactor(symbol_t(t_[i], k), symbol_t(x_[i], k), symbol_t('l', k), cable_lengths[i] - tether_tension_offset, weight_tether_tension, tension_cost));
            }
            }

            for (int i = 0; i < robot_num_; i++) {
            graph.add(MagnitudeUpperBoundFactor(symbol_t(u_[i], k), u_upper_bound, control_cost));
            graph.add(MagnitudeLowerBoundFactor(symbol_t(u_[i], k), u_lower_bound, control_cost));
            }

            if (k > 0 && have_trajectory_reference_factor) {
                Vector6 mid_state(6);
                mid_state << initial_load_state_ + k * diff;
                graph.add(PriorFactor<Vector6>(symbol_t('l', k), mid_state, goal_cost_with_angle));
            }

            if (have_uk_prior) {
            for (int i = 0; i < robot_num_; i++) {
            graph.add(PriorFactor<Vector2>(symbol_t(u_[i], k), Vector2::Zero(), control_interim_cost));
            }
            }
        }

        // --- 3. Create Initial Estimate ---
        Values initial_values;
        Vector2 init_u(0.0, 0.0);
        Vector1 init_t(0.0);
        for (int k = 0; k <= num_time_steps; ++k) {
            for (int i = 0; i < robot_num_; i++) {
            initial_values.insert(symbol_t(x_[i], k), initial_robot_states_[i]);
            }
            initial_values.insert(symbol_t('l', k), initial_load_state_);
            if (k < num_time_steps) {
                for (int i = 0; i < robot_num_; i++) {
                initial_values.insert(symbol_t(u_[i], k), init_u);
                initial_values.insert(symbol_t(t_[i], k), init_t);
                }
            }
        }
    
        Values result = runOptimizer(debug_mode_, graph, initial_values, factor_errors, dt, mu, load_mass, gravity, mu2, inertia, robot_num_, true);

        Vector6 last_state = result.at<Vector6>(symbol_t('l', num_time_steps));
        double a1 = sqrt((final_load_goal_[0] - last_state[0]) * (final_load_goal_[0] - last_state[0]) + (final_load_goal_[1] - last_state[1]) * (final_load_goal_[1] - last_state[1]));
        double a2 = sqrt((final_load_goal_[0] - initial_load_state_[0]) * (final_load_goal_[0] - initial_load_state_[0]) + (final_load_goal_[1] - initial_load_state_[1]) * (final_load_goal_[1] - initial_load_state_[1]));
        pos_error = graph.error(result);

        FactorExecutorResult exec_result = {};
        for (int i = 0; i < robot_num_; i++) {

            Vector4 next_state = result.at<Vector4>(symbol_t(x_[i], 1));
            Vector2 next_ctrl = result.at<Vector2>(symbol_t(u_[i], 0));
            Vector1 cur_t = result.at<Vector1>(symbol_t(t_[i], 0));

            exec_result.push_back(
                {
                    .drone_vel = {next_state[2], next_state[3], desired_robot_heights_[i] - robot_heights_[i]},
                    .controls = {next_ctrl(0), next_ctrl(1), 0.0},
                    .tension = cur_t(0),
                }
            );

            if (debug_mode_ == "one_of" || debug_mode_ == "sim") {
                cout << "  Cur tension " << i + 1 << ": " << cur_t[0] << endl;
                cout << "Next controls " << i + 1 << ": " << next_ctrl[0] << ' ' << next_ctrl[1] << endl;
            }
        }

        return exec_result;
    }
};

#endif // FACTOR_EXECUTOR_MULTI_ROBOT_WITH_ORI_HPP