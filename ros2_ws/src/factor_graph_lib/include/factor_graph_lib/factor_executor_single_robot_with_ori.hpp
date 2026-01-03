#ifndef FACTOR_EXECUTOR_SINGLE_ROBOT_WITH_ORI_HPP
#define FACTOR_EXECUTOR_SINGLE_ROBOT_WITH_ORI_HPP

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


class FactorExecutorSingleRobotWithOrientation: public FactorExecutor {

    Vector6 initial_load_state_;
    Vector4 initial_robot_state_;
    Vector6 final_load_goal_;
    double robot_height_, desired_robot_height_;
    string debug_mode_;

public:

    FactorExecutorSingleRobotWithOrientation(
        string debug_mode, 
        const Vector6& initial_load_state, 
        const Vector4& initial_robot_state, 
        const Vector6& final_load_goal, 
        double robot_height,
        double desired_robot_height,
        const map<string, double>& tune_d,
        const map<string, bool>& tune_b
        ):
    FactorExecutor(tune_d, tune_b),
    debug_mode_(debug_mode), 
    initial_load_state_(initial_load_state), 
    initial_robot_state_(initial_robot_state), 
    final_load_goal_(final_load_goal), 
    robot_height_(robot_height),
    desired_robot_height_(desired_robot_height) {}

    FactorExecutorResult run(map<string, double>& factor_errors, double& pos_error) const override {
    
        NonlinearFactorGraph graph;

        // --- Define problem parameters ---
        const int num_time_steps = 20;
        const double dt = 0.005;
        const double robot_mass = 0.025; // kg
        const double gravity = 9.81;
        const double load_mass = 0.005;   // kg
        const double inertia = 0.000133;
        const double mu = 0.3;
        const double mu2 = 0.3;
        const double cable_length = sqrt(1.03 * 1.03 - (robot_height_ - 0.2) * (robot_height_ - 0.2));

        // VALUES TO TUNE
        // =============================
        // =============================

        const double u_upper_bound = getd("u_upper_bound", 0.45); //0.55
        const double u_lower_bound = getd("u_lower_bound", 0.003);

        double weight_tension_lower_bound = getd("weight_tension_lower_bound", 1000000.0);
        double weight_cable_stretch = getd("weight_cable_stretch", 100.0);
        double weight_tension_slack = getd("weight_tension_slack", 50.0);
        double weight_tether_tension = getd("weight_tether_tension", 0.266292); //0.266292 for 4 segments
        
        double cable_stretch_offset = getd("cable_stretch_offset", 0.001);
        double cable_length_offset = getd("cable_length_offset", 0.3); //0.021875); //0.38672 0.0021875 good enough for 0.001 mass
         
        bool have_uk_prior = getb("have_uk_prior", true); 
        bool have_t0_prior = getb("have_t0_prior", false);
        bool have_tk_prior = getb("have_tk_prior", false);

        // Goal prior + tether_tension factor makes the full rank system
        bool have_goal_prior = getb("have_goal_prior", true);

        bool have_tension_lower_bound_factor = getb("have_tension_lower_bound_factor", false);
        bool have_cable_stretch_factor = getb("have_cable_stretch_factor", false);
        bool have_tension_slack_penalty_factor = getb("have_tension_slack_penalty_factor", false); 
        bool have_tether_tension_factor = getb("have_tether_tension_factor", true);

        bool have_trajectory_reference_factor = getb("have_trajectory_reference_factor", true);

        // STATE PRIORS TUNED VALUES
        double goal_cost_v = getd("goal_cost", 0.000005);
        double goal_ori_cost_v = getd("goal_cost", 0.00005);  // 0.00005 the best
        double control_interim_cost_v = getd("control_interim_cost", 10.0);
        double tension_cost_v = getd("tension_cost", 1e-3); 


        // STATE PRIORS
        auto init_cost = noiseModel::Diagonal::Sigmas(
        (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
        auto init_with_angle_cost = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.000005, 0.000005, 0.000005, 0.000005, 0.000005, 0.000005).finished());
        auto goal_with_angle_cost = noiseModel::Diagonal::Sigmas(
        (Vector(6) << goal_cost_v, goal_cost_v, goal_ori_cost_v, 1000.1, 1000.1, 1000.1).finished());
        auto interim_goal_with_angle_cost = noiseModel::Diagonal::Sigmas(
        (Vector(6) << goal_cost_v, goal_cost_v, goal_ori_cost_v, 1000.1, 1000.1, 1000.1).finished());

        // DYNAMICS
        auto dynamics_cost = noiseModel::Diagonal::Sigmas(
        (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
        auto dynamics_with_angle_cost = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.001, 0.001, 0.01, 0.001, 0.001, 0.01).finished());

        // CONTROL
        auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-2);
        auto control_interim_cost = noiseModel::Diagonal::Sigmas(
            (Vector(2) << control_interim_cost_v, control_interim_cost_v).finished());

        // TENSION
        auto tension_cost = noiseModel::Isotropic::Sigma(1, tension_cost_v);
        // =============================
        // =============================


        graph.add(PriorFactor<Vector4>(symbol_t('x', 0), initial_robot_state_, init_cost));
        graph.add(PriorFactor<Vector6>(symbol_t('l', 0), initial_load_state_, init_with_angle_cost));
        if (have_goal_prior) {
        graph.add(PriorFactor<Vector6>(symbol_t('l', num_time_steps), final_load_goal_, goal_with_angle_cost));
        }

        double xdiff = (final_load_goal_(0) - initial_load_state_(0)) / num_time_steps;
        double ydiff = (final_load_goal_(1) - initial_load_state_(1)) / num_time_steps;
        double adiff = (final_load_goal_(2) - initial_load_state_(2)) / num_time_steps;
        Vector6 diff(xdiff, ydiff, adiff, 0, 0, 0);

        if (debug_mode_ == "one_of" || debug_mode_ == "sim") {
        CableVectorHelper h(initial_robot_state_, initial_load_state_);
        cout << " Cable length: " << cable_length << std::endl;
        cout << "  Actual dist: " << sqrt((initial_robot_state_[0] - h.ap_x) * (initial_robot_state_[0] - h.ap_x)  + (initial_robot_state_[1] - h.ap_y) * (initial_robot_state_[1] - h.ap_y) ) << endl;
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

            graph.add(LoadDynamicsWithLoadOrientationFactor(
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
                true,
                dynamics_with_angle_cost
                ));

            // Factor 1: Enforce T_k >= 0
            if (have_tension_lower_bound_factor) {
            graph.add(TensionLowerBoundFactor(symbol_t('t', k), weight_tension_lower_bound, tension_cost));
            }
            
            // Factor 2: Penalize if ||p_r - p_l|| > cable_length
            if (have_cable_stretch_factor) {
            graph.add(CableStretchPenaltyWithOrientationFactor(symbol_t('x', k), symbol_t('l', k), cable_length, weight_cable_stretch, tension_cost));
            }

            // Factor 3: Penalize if T_k > 0 AND ||p_r - p_l|| < cable_length (i.e., tension in slack cable)
            if (have_tension_slack_penalty_factor) {
            graph.add(TensionSlackPenaltyWithLoadOrientationFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length, weight_tension_slack, tension_cost));
            }

            // Factor 4: Penalise if T_k != k * max(0, distance - cable_length)
            if (have_tether_tension_factor) {
            graph.add(TethertensionFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length - cable_length_offset, weight_tether_tension, tension_cost));
            }
            
            graph.add(MagnitudeUpperBoundFactor(symbol_t('u', k), u_upper_bound, control_cost));
            graph.add(MagnitudeLowerBoundFactor(symbol_t('u', k), u_lower_bound, control_cost));

            if (k > 0 && have_trajectory_reference_factor) {
                Vector6 mid_state(6);
                mid_state << initial_load_state_ + k * diff;
                graph.add(PriorFactor<Vector6>(symbol_t('l', k), mid_state, interim_goal_with_angle_cost));
            }
        
            if (have_uk_prior) {
            graph.add(PriorFactor<Vector2>(symbol_t('u', k), Vector2::Zero(), control_interim_cost));
            }
        }

        // --- 3. Create Initial Estimate ---
        Values initial_values;
        Vector2 init_u(0.0, 0.0);
        Vector1 init_t(0.0);
        for (int k = 0; k <= num_time_steps; ++k) {
            initial_values.insert(symbol_t('l', k), initial_load_state_);
            initial_values.insert(symbol_t('x', k), initial_robot_state_);
            if (k < num_time_steps) {
                initial_values.insert(symbol_t('u', k), init_u);
                initial_values.insert(symbol_t('t', k), init_t);
            }
        }

        Values result = runOptimizer(debug_mode_, graph, initial_values, factor_errors, dt, mu, load_mass, gravity, mu2, inertia, 1, true);

        Vector4 next_state = result.at<Vector4>(symbol_t('x', 1));
        Vector2 next_ctrl = result.at<Vector2>(symbol_t('u', 0));
        Vector1 cur_t = result.at<Vector1>(symbol_t('t', 0));

        if (debug_mode_ == "one_of" || debug_mode_ == "sim") {
            cout << " Cur tension: " << cur_t[0] << endl;
            cout << "Next control: " << next_ctrl[0] << ", " << next_ctrl[1] << endl;
        }
        pos_error = graph.error(result);

        FactorExecutorResult exec_result = {
            {
                .drone_vel = {next_state[2], next_state[3], desired_robot_height_ - robot_height_},
                .controls = {next_ctrl(0), next_ctrl(1), 0.0},
                .tension = cur_t(0),
            }
        };
        return exec_result;
    }
};


#endif // FACTOR_EXECUTOR_SINGLE_ROBOT_WITH_ORI_HPP