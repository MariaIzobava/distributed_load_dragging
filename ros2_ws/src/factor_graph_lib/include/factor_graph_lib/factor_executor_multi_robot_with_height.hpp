#ifndef FACTOR_EXECUTOR_MULTI_ROBOT_WITH_HEIGHT_HPP
#define FACTOR_EXECUTOR_MULTI_ROBOT_WITH_HEIGHT_HPP

#include <bits/stdc++.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/Ordering.h>
#include <Eigen/Dense>

#include "cable_factors.hpp"
#include "control_factors.hpp"
#include "dynamics_factors_with_height.hpp"
#include "dynamics_factors_multi_robots_with_height.hpp"
#include "factor_executor.hpp"

using namespace std;
using namespace gtsam;
using symbol_t = gtsam::Symbol;


class FactorExecutorMultiRobotsWithHeight: public FactorExecutor {

    Vector4 initial_load_state_;
    std::vector<Vector6> initial_robot_states_;
    Vector4 final_load_goal_;
    string debug_mode_;
    int robot_num_;

    static inline const char x_[] = "xXzZ";
    static inline const char t_[] = "tTdD";
    static inline const char u_[] = "uUyY";

public:

    FactorExecutorMultiRobotsWithHeight(
        string debug_mode, 
        int robot_num,
        const Vector4& initial_load_state, 
        const std::vector<Vector6>& initial_robot_states, 
        const Vector4& final_load_goal, 
        const map<string, double>& tune_d,
        const map<string, bool>& tune_b
        ):
    FactorExecutor(tune_d, tune_b),
    debug_mode_(debug_mode), 
    robot_num_(robot_num),
    initial_load_state_(initial_load_state), 
    initial_robot_states_(initial_robot_states), 
    final_load_goal_(final_load_goal) {}

    FactorExecutorResult run(map<string, double>& factor_errors, double& pos_error) const override {

        NonlinearFactorGraph graph;

        // --- Define problem parameters ---
        const int num_time_steps = 20;
        const double dt = 0.005;
        const double robot_mass = 0.025; // kg
        const double load_mass = 0.005;   // kg
        const double gravity = 9.81;
        const double mu = 0.3;
        const double mu2 = 0.3;
        std::vector<double> cable_lengths;
        for (int i = 0; i < robot_num_; i++) {
            double h = initial_robot_states_[i][2];
            double d = sqrt(1.01 * 1.01 - (h - 0.2) * (h - 0.2));
            cable_lengths.push_back(sqrt((0.2 + d) * (0.2 + d) + (h - 0.2) * (h - 0.2)));
        }

        // VALUES TO TUNE
        // =============================
        // =============================
        const double u_upper_bound = getd("u_upper_bound", 0.7); 
        const double u_lower_bound = getd("u_lower_bound", 0.003);

        double weight_tension_lower_bound = getd("weight_tension_lower_bound", 1000000.0);
        double weight_cable_stretch = getd("weight_cable_stretch", 100.0) ;
        double weight_tension_slack = getd("weight_tension_slack", 50.0);
        double weight_tether_tension = getd("weight_tether_tension", 0.366292); //0.0008096

        double cable_stretch_penalty_offset = getd("cable_stretch_penalty_offset", 0.0);
        double tension_slack_penalty_offset = getd("tension_slack_penalty_offset", 0.2); 
        double tether_tension_offset = getd("tether_tension_offset", 0.25); //0.38672

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
        auto init_cost_with_height = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.000005, 0.000005, 0.000005, 0.000005, 0.000005, 0.000005).finished());
        auto goal_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.000005, 0.000005, 1000.1, 1000.1).finished());
        auto robot_height_cost = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 1000.001, 1000.001, 0.0001, 1000.001, 1000.001, 0.001).finished());

        // DYNAMICS
        auto dynamics_robot_cost = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001).finished());
        auto dynamics_load_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());

        // CONTROL
        auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
        auto control_interim_cost = noiseModel::Diagonal::Sigmas(
            (Vector(3) << 10.0, 10.0, 10.0).finished());

        // TENSION
        auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
        // =============================
        // =============================


        for (int i = 0; i < robot_num_; i++) {
            graph.add(PriorFactor<Vector6>(symbol_t(x_[i], 0), initial_robot_states_[i], init_cost_with_height));
        }
        graph.add(PriorFactor<Vector4>(symbol_t('l', 0), initial_load_state_, init_cost));
        graph.add(PriorFactor<Vector4>(symbol_t('l', num_time_steps), final_load_goal_, goal_cost));

        double xdiff = (final_load_goal_(0) - initial_load_state_(0)) / num_time_steps;
        double ydiff = (final_load_goal_(1) - initial_load_state_(1)) / num_time_steps;
        Vector4 diff(xdiff, ydiff, 0, 0);

        if (debug_mode_ == "one_of" || debug_mode_ == "sim") {
        for (int i = 0; i < robot_num_; i++) {
        const auto& state = initial_robot_states_[i];
        const auto& l = initial_load_state_;
        double dist = sqrt((state[0] - l[0]) * (state[0] - l[0])  + (state[1] - l[1]) * (state[1] - l[1]) + (state[2] - 0.2) * (state[2] - 0.2));
        cout << "Cable length " << i + 1 << ": " << cable_lengths[i] << std::endl;
        cout << " Actual dist " << i + 1 << ": " << dist << endl;
        }
        }

        for (int k = 0; k < num_time_steps; ++k) {
            for (int i = 0; i < robot_num_; i++) {
                graph.add(RobotDynamicsWithHeightFactor(
                    symbol_t(x_[i], k), 
                    symbol_t('l', k),
                    symbol_t(u_[i], k),
                    symbol_t(t_[i], k),
                    symbol_t(x_[i], k+1),
                    dt,
                    robot_mass,
                    dynamics_robot_cost
                    ));

                for (int j = i + 1; j < robot_num_; j++) {
                    graph.add(RobotsDistanceWithHeightFactor(symbol_t(x_[i], k), symbol_t(x_[j], k), 0.2, tension_cost));
                }

                graph.add(RobotsHeightLowerBoundFactor(symbol_t(x_[i], k+1), 0.45, control_cost));
                graph.add(RobotsHeightUpperBoundFactor(symbol_t(x_[i], k+1), 0.55, control_cost));

                Vector6 heightx = initial_robot_states_[i];
                heightx(2) = 0.5;
                heightx(5) = 0.5 - initial_robot_states_[i][2];
                graph.add(PriorFactor<Vector6>(symbol_t(x_[i], k+1), heightx, robot_height_cost));
            }

            if (robot_num_ == 1) {
            graph.add(LoadDynamicsWithHeightFactor(
                symbol_t('l', k),
                symbol_t('x', k),
                symbol_t('t', k),
                symbol_t('l', k+1),
                dt,
                load_mass,
                mu,
                gravity,
                dynamics_load_cost
                ));
            } else if (robot_num_ == 2) {
            graph.add(LoadDynamicsMultiRobotsWithHeightFactor2(
                symbol_t('l', k),
                symbol_t(x_[0], k),
                symbol_t(t_[0], k),
                symbol_t(x_[1], k),
                symbol_t(t_[1], k),
                symbol_t('l', k+1),
                dt,
                load_mass,
                mu,
                gravity,
                dynamics_load_cost
                ));
            } else if (robot_num_ == 3) {
            graph.add(LoadDynamicsMultiRobotsWithHeightFactor3(
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
                gravity,
                dynamics_load_cost
                ));
            }

            if (have_tether_tension_factor) {
            for (int i = 0; i < robot_num_; i++) {
            graph.add(TethertensionFactorNoOriWithHeight(symbol_t(t_[i], k), symbol_t(x_[i], k), symbol_t('l', k), cable_lengths[i] - tether_tension_offset , weight_tether_tension, tension_cost, "middle"));//1.21865
            }
            }

            if (k > 0 && have_uk_prior) {
                for (int i = 0; i < robot_num_; i++) {
                graph.add(PriorFactor<Vector3>(symbol_t(u_[i], k), Vector3::Zero(), control_interim_cost));
                }
            }

            for (int i = 0; i < robot_num_; i++) {
                graph.add(MagnitudeUpperBoundWithHeightFactor(symbol_t(u_[i], k), u_upper_bound, control_cost));
                graph.add(MagnitudeLowerBoundWithHeightFactor(symbol_t(u_[i], k), u_lower_bound, control_cost));
            }

            if (k > 0 && have_trajectory_reference_factor) {
                Vector4 mid_state(4);
                mid_state << initial_load_state_ + k * diff;
                graph.add(PriorFactor<Vector4>(symbol_t('l', k), mid_state, goal_cost));
            }
        }

        // --- 3. Create Initial Estimate ---
        Values initial_values;
        Vector3 init_u(0.0, 0.0, 0.0);
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
    
        Values result = runOptimizer(debug_mode_, graph, initial_values, factor_errors, dt, mu, load_mass, gravity, 0.0, 0.0, robot_num_, false);

        Vector4 last_state = result.at<Vector4>(symbol_t('l', num_time_steps));
        double a1 = sqrt((final_load_goal_[0] - last_state[0]) * (final_load_goal_[0] - last_state[0]) + (final_load_goal_[1] - last_state[1]) * (final_load_goal_[1] - last_state[1]));
        double a2 = sqrt((final_load_goal_[0] - initial_load_state_[0]) * (final_load_goal_[0] - initial_load_state_[0]) + (final_load_goal_[1] - initial_load_state_[1]) * (final_load_goal_[1] - initial_load_state_[1]));
        pos_error = graph.error(result);

        FactorExecutorResult exec_result = {};
        for (int i = 0; i < robot_num_; i++) {

            Vector6 next_state = result.at<Vector6>(symbol_t(x_[i], 1));
            Vector3 next_ctrl = result.at<Vector3>(symbol_t(u_[i], 0));
            Vector1 cur_t = result.at<Vector1>(symbol_t(t_[i], 0));

            exec_result.push_back(
                {
                    .drone_vel = {next_state[3], next_state[4], next_state[5]},
                    .controls = {next_ctrl(0), next_ctrl(1), next_ctrl(2)},
                    .tension = cur_t(0),
                }
            );

            if (debug_mode_ == "one_of" || debug_mode_ == "sim") {
                cout << "  Cur tension " << i + 1 << ": " << cur_t[0] << endl;
                cout << "Next controls " << i + 1 << ": " << next_ctrl[0] << ' ' << next_ctrl[1] << ' ' << next_ctrl[2] << endl;
            }
        }

        return exec_result;
    }
};

#endif // FACTOR_EXECUTOR_MULTI_ROBOT_WITH_HEIGHT_HPP