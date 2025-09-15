#ifndef FACTOR_EXECUTOR_MULTI_ROBOT_WITH_HEIGHT_AND_ORI_HPP
#define FACTOR_EXECUTOR_MULTI_ROBOT_WITH_HEIGHT_AND_ORI_HPP

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
#include "dynamics_factors_with_angle.hpp"
#include "dynamics_factors_multi_robots_with_height_and_ori.hpp"
#include "factor_executor.hpp"

using namespace std;
using namespace gtsam;
using symbol_t = gtsam::Symbol;


class FactorExecutorMultiRobotsWithHeightAndOri: public FactorExecutor {

    Vector6 initial_load_state_;
    std::vector<Vector6> initial_robot_states_;
    Vector6 final_load_goal_;
    string debug_mode_;
    int robot_num_;

    static inline const char x_[] = "xXzZ";
    static inline const char t_[] = "tTdD";
    static inline const char u_[] = "uUyY";

public:

    FactorExecutorMultiRobotsWithHeightAndOri(
        string debug_mode, 
        int robot_num,
        const Vector6& initial_load_state, 
        const std::vector<Vector6>& initial_robot_states, 
        const Vector6& final_load_goal, 
        const map<string, double>& tune_d,
        const map<string, bool>& tune_b
        ):
    FactorExecutor(tune_d, tune_b),
    debug_mode_(debug_mode), 
    robot_num_(robot_num),
    initial_load_state_(initial_load_state), 
    initial_robot_states_(initial_robot_states), 
    final_load_goal_(final_load_goal) {}

    std::vector<double> run(map<string, double>& factor_errors, double& pos_error) const override {

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
        const double cable_length = 1.03;

        // VALUES TO TUNE
        // =============================
        // =============================
        const double u_upper_bound = getd("u_upper_bound", 0.7); //0.55
        const double u_lower_bound = getd("u_lower_bound", 0.003); //0.245

        double weight_tension_lower_bound = getd("weight_tension_lower_bound", 1000000.0);
        double weight_cable_stretch = getd("weight_cable_stretch", 100.0) ;
        double weight_tension_slack = getd("weight_tension_slack", 50.0);
        double weight_tether_tension = getd("weight_tether_tension", 0.266292); //0.0008096  0.266292

        double cable_stretch_penalty_offset = getd("cable_stretch_penalty_offset", 0.0);
        double tension_slack_penalty_offset = getd("tension_slack_penalty_offset", 0.2); 
        double tether_tension_offset = getd("tether_tension_offset", 0.3); //0.38672  IT SHOULD BE SMALLER! Try the factor to encourage large distance between drone and load

        bool have_uk_prior = getb("have_uk_prior", true);
        
        bool have_tension_lower_bound_factor = getb("have_tension_lower_bound_factor", false);
        bool have_cable_stretch_factor = getb("have_cable_stretch_factor", false);
        bool have_tension_slack_penalty_factor = getb("have_tension_slack_penalty_factor", false); 
        bool have_tether_tension_factor = getb("have_tether_tension_factor", true);

        bool have_trajectory_reference_factor = getb("have_trajectory_reference_factor", true);

        double goal_cost_v = getd("goal_cost", 0.00005);
        double robot_height_v_cost = getd("robot_height_v_cost", 0.01); //0.001

//         robot_height_v_cost 0.001
// tether_tension_offset 0.0015
// u_upper_bound 0.4
// weight_tether_tension 0.0064



        // STATE PRIORS
        auto init_cost = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.000005, 0.000005, 0.000005, 0.000005, 0.000005, 0.000005).finished());
        auto goal_cost = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.000005, 0.000005, 0.000005, 1000.1, 1000.1, 1000.1).finished());
        auto robot_height_cost = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 1000.001, 1000.001, 0.0001, 1000.001, 1000.001, robot_height_v_cost).finished());

        // DYNAMICS
        auto dynamics_robot_cost = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001).finished());
        auto dynamics_load_cost = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.001, 0.001, 0.01, 0.001, 0.001, 0.01).finished());

        // CONTROL
        auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
        auto control_interim_cost = noiseModel::Diagonal::Sigmas(
            (Vector(3) << 10.0, 10.0, 10.0).finished());

        // TENSION
        auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);

        // DISTANCE
        auto distance_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
        // =============================
        // =============================


        for (int i = 0; i < robot_num_; i++) {
            graph.add(PriorFactor<Vector6>(symbol_t(x_[i], 0), initial_robot_states_[i], init_cost));
        }
        graph.add(PriorFactor<Vector6>(symbol_t('l', 0), initial_load_state_, init_cost));
        graph.add(PriorFactor<Vector6>(symbol_t('l', num_time_steps), final_load_goal_, goal_cost));

        double xdiff = (final_load_goal_(0) - initial_load_state_(0)) / num_time_steps;
        double ydiff = (final_load_goal_(1) - initial_load_state_(1)) / num_time_steps;
        double adiff = (final_load_goal_(2) - initial_load_state_(2)) / num_time_steps;
        Vector6 diff(xdiff, ydiff, adiff, 0, 0, 0);

        if (debug_mode_ == "one_of" || debug_mode_ == "sim") {
        cout << "Cable length: " << cable_length << std::endl;
        for (int i = 0; i < robot_num_; i++) {
        const auto& state = initial_robot_states_[i];
        const auto& l = initial_load_state_;
        CableVectorHelper h(state, l);
        double dist = sqrt((state[0] - h.ap_x) * (state[0] - h.ap_x)  + (state[1] - h.ap_y) * (state[1] - h.ap_y) + (state[2] - 0.2) * (state[2] - 0.2));
        cout << " Actual dist " << i + 1 << ": " << dist << endl;
        }
        }

        for (int k = 0; k < num_time_steps; ++k) {
            for (int i = 0; i < robot_num_; i++) {
                graph.add(RobotDynamicsWithHeightAndOriFactor(
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
                    graph.add(RobotsDistanceWithHeightFactor(symbol_t(x_[i], k), symbol_t(x_[j], k), 0.1, distance_cost));
                }

                graph.add(RobotLoadWithHeightAndOrientationFactor(symbol_t(x_[i], k), symbol_t('l', k), cable_length - 0.1, 1.0, tension_cost));


                double height = 0.5;//0.3 + 0.1 * i;

                graph.add(RobotsHeightLowerBoundFactor(symbol_t(x_[i], k+1), height - 0.05, control_cost));
                graph.add(RobotsHeightUpperBoundFactor(symbol_t(x_[i], k+1), height + 0.05, control_cost));

                Vector6 heightx = initial_robot_states_[i];
                heightx(2) = height;
                heightx(5) = height - initial_robot_states_[i][2];
                graph.add(PriorFactor<Vector6>(symbol_t(x_[i], k+1), heightx, robot_height_cost));
            }

            if (robot_num_ == 1) {
            graph.add(LoadDynamicsWithHeightAndOriFactor(
                symbol_t('l', k),
                symbol_t('x', k),
                symbol_t('t', k),
                symbol_t('l', k+1),
                dt,
                load_mass,
                mu,
                mu2,
                gravity,
                inertia,
                dynamics_load_cost
                ));
            } 
            else if (robot_num_ == 2) {
            graph.add(LoadDynamicsMultiRobotsWithHeightAndOriFactor2(
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
                dynamics_load_cost
                ));
            }
             else if (robot_num_ == 3) {
            graph.add(LoadDynamicsMultiRobotsWithHeightAndOriFactor3( 
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
                dynamics_load_cost
                ));
            }
            else if (robot_num_ == 4) {
            graph.add(LoadDynamicsMultiRobotsWithHeightAndOriFactor4( 
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
                dynamics_load_cost
                ));
            }

            if (have_tether_tension_factor) {
            for (int i = 0; i < robot_num_; i++) {
            graph.add(TethertensionFactorWithHeight(symbol_t(t_[i], k), symbol_t(x_[i], k), symbol_t('l', k), cable_length - tether_tension_offset , weight_tether_tension, tension_cost, "middle"));//1.21865
            }
            }

            if (k > 0 && have_uk_prior) {
                 Vector3 ui(0.0, 0.0, 0.0); //0.245
                for (int i = 0; i < robot_num_; i++) {
                graph.add(PriorFactor<Vector3>(symbol_t(u_[i], k), ui, control_interim_cost));
                }
            }

            for (int i = 0; i < robot_num_; i++) {
                graph.add(MagnitudeUpperBoundWithHeightFactor(symbol_t(u_[i], k), u_upper_bound, control_cost));
                graph.add(MagnitudeLowerBoundWithHeightFactor(symbol_t(u_[i], k), u_lower_bound, control_cost));
            }

            if (k > 0 && have_trajectory_reference_factor) {
                Vector6 mid_state(6);
                mid_state << initial_load_state_ + k * diff;
                graph.add(PriorFactor<Vector6>(symbol_t('l', k), mid_state, goal_cost));
            }
        }

        // --- 3. Create Initial Estimate ---
        Values initial_values;
        Vector3 init_u(0.0, 0.0, 0.0);  //0.245
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

        Vector6 last_state = result.at<Vector6>(symbol_t('l', num_time_steps));
        double a1 = sqrt((final_load_goal_[0] - last_state[0]) * (final_load_goal_[0] - last_state[0]) + (final_load_goal_[1] - last_state[1]) * (final_load_goal_[1] - last_state[1]));
        double a2 = sqrt((final_load_goal_[0] - initial_load_state_[0]) * (final_load_goal_[0] - initial_load_state_[0]) + (final_load_goal_[1] - initial_load_state_[1]) * (final_load_goal_[1] - initial_load_state_[1]));
        pos_error = graph.error(result);

        std::vector<double> next_vels;
        for (int i = 0; i < robot_num_; i++) {

            Vector6 next_state = result.at<Vector6>(symbol_t(x_[i], 1));
            next_vels.push_back(next_state[3]);
            next_vels.push_back(next_state[4]);
            next_vels.push_back(next_state[5]);

            Vector3 next_ctrl = result.at<Vector3>(symbol_t(u_[i], 1));
            Vector1 cur_t = result.at<Vector1>(symbol_t(t_[i], 0));
            if (debug_mode_ == "one_of" || debug_mode_ == "sim") {
                cout << "  Cur tension " << i + 1 << ": " << cur_t[0] << endl;
                cout << "Next controls " << i + 1 << ": " << next_ctrl[0] << ' ' << next_ctrl[1] << ' ' << next_ctrl[2] << endl;
            }
        }
        for (int i = 0; i < robot_num_; i++) {
            Vector1 cur_t = result.at<Vector1>(symbol_t(t_[i], 0));
            next_vels.push_back(cur_t[0]);
        }
        for (int i = 0; i < robot_num_; i++) {
            Vector3 next_ctrl = result.at<Vector3>(symbol_t(u_[i], 0));
            next_vels.push_back(next_ctrl[0]);
            next_vels.push_back(next_ctrl[1]);
            next_vels.push_back(next_ctrl[2]);
        }
        return next_vels;
    }
};

#endif // FACTOR_EXECUTOR_MULTI_ROBOT_WITH_HEIGHT_AND_ORI_HPP