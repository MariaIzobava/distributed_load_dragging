#ifndef FACTOR_EXECUTOR_SINGLE_ROBOT_HPP
#define FACTOR_EXECUTOR_SINGLE_ROBOT_HPP

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


class FactorExecutorSingleRobot: public FactorExecutor {

    Vector4 initial_load_state_;
    Vector4 initial_robot_state_;
    Vector4 final_load_goal_;
    Vector2 last_u_;
    Vector1 last_tension_;
    double robot_height_;
    string debug_mode_;

public:

    FactorExecutorSingleRobot(
        string debug_mode, 
        const Vector4& initial_load_state, 
        const Vector4& initial_robot_state, 
        const Vector4& final_load_goal, 
        double robot_height, 
        const Vector2& last_u, 
        const Vector1& last_tension,
        const map<string, double>& tune_d,
        const map<string, bool>& tune_b
        ):
    FactorExecutor(tune_d, tune_b),
    debug_mode_(debug_mode), 
    initial_load_state_(initial_load_state), 
    initial_robot_state_(initial_robot_state), 
    final_load_goal_(final_load_goal), 
    robot_height_(robot_height),
    last_u_(last_u),
    last_tension_(last_tension) {}

    std::vector<double> run(map<string, double>& factor_errors, double& pos_error) const override {
    
        NonlinearFactorGraph graph;

        // Non changing values
        const int num_time_steps = 20;
        const double dt = 0.005;
        const double robot_mass = 0.025; // kg
        const double load_mass = 0.005;   // kg
        const double gravity = 9.81;
        const double mu = 0.3;
        const double cable_length = 0.20 + sqrt(1.03 * 1.03 - (robot_height_) * (robot_height_));

        // VALUES TO TUNE
        // =============================
        // =============================
        const double u_upper_bound = getd("u_upper_bound", 0.6);
        const double u_lower_bound = getd("u_lower_bound", 0.003);

        double weight_tension_lower_bound = getd("weight_tension_lower_bound", 1000000.0);
        double weight_cable_stretch = getd("weight_cable_stretch", 100.0);
        double weight_tension_slack = getd("weight_tension_slack", 50.0);
        double weight_tether_tension = getd("weight_tether_tension",  0.0008096); //12.5  0.0008096
        
        double cable_stretch_offset = getd("cable_stretch_offset", 0.001);
        double cable_length_offset = getd("cable_length_offset", 0.38672); //0.021875); //0.0021875 good enough for 0.001 mass 0.38672
         
        bool have_u0_prior = getb("have_u0_prior", true);
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
        double goal_cost_v = getd("goal_cost", 0.00005); 
        double control_interim_cost_v = getd("control_interim_cost", 10.0);
        double tension_cost_v = getd("tension_cost", 1e-5); 


        // STATE PRIORS
        auto init_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
        auto goal_interim_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.01,    0.01,    1000.1, 1000.1).finished());
        auto goal_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << goal_cost_v, goal_cost_v, 1000.1, 1000.1).finished());

        // DYNAMICS
        auto dynamics_robot_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
        auto dynamics_load_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());

        // CONTROL
        auto control_init_cost = noiseModel::Diagonal::Sigmas(
            (Vector(2) << 10.0, 10.0).finished());
        auto control_interim_cost = noiseModel::Diagonal::Sigmas(
            (Vector(2) << control_interim_cost_v, control_interim_cost_v).finished());
        auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-1);

        // TENSION
        auto tension_prior_cost = noiseModel::Isotropic::Sigma(1, 1e-2);
        auto tension_interim_cost = noiseModel::Isotropic::Sigma(1, 1e-2);
        auto tension_cost = noiseModel::Isotropic::Sigma(1, tension_cost_v);
        // =============================
        // =============================


        graph.add(PriorFactor<Vector4>(symbol_t('x', 0), initial_robot_state_, init_cost));
        graph.add(PriorFactor<Vector4>(symbol_t('l', 0), initial_load_state_, init_cost));

        if (have_goal_prior) {
        graph.add(PriorFactor<Vector4>(symbol_t('l', num_time_steps), final_load_goal_, goal_cost));
        }

        if (have_u0_prior) {
          Vector2 u0(0.0, 0.0);
          graph.add(PriorFactor<Vector2>(symbol_t('u', 0), u0, control_init_cost));
        }
        if (have_t0_prior) {
            graph.add(PriorFactor<Vector1>(symbol_t('t', 0), last_tension_, tension_prior_cost));
        }


        double xdiff = (final_load_goal_(0) - initial_load_state_(0)) / num_time_steps;
        double ydiff = (final_load_goal_(1) - initial_load_state_(1)) / num_time_steps;
        Vector4 diff(xdiff, ydiff, 0, 0);

        if (debug_mode_ == "sim" || debug_mode_ == "one_of") {
        cout << "  Cable length: " << cable_length << endl;
        cout << "   Actual dist: " << sqrt((initial_robot_state_[0] - initial_load_state_[0]) * (initial_robot_state_[0] - initial_load_state_[0])  + (initial_robot_state_[1] - initial_load_state_[1]) * (initial_robot_state_[1] - initial_load_state_[1]) ) << endl;
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
                dynamics_robot_cost
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
                dynamics_load_cost
                ));

            // Factor 1: Enforce T_k >= 0
            if (have_tension_lower_bound_factor) {
            graph.add(TensionLowerBoundFactor(symbol_t('t', k), weight_tension_lower_bound, tension_cost));
            }
            
            // Factor 2: Penalize if ||p_r - p_l|| > cable_length
            if (have_cable_stretch_factor) {
            graph.add(CableStretchPenaltyFactor(symbol_t('x', k), symbol_t('l', k), cable_length + cable_stretch_offset, weight_cable_stretch, tension_cost));
            }
            
            // Factor 3: Penalize if T_k > 0 AND ||p_r - p_l|| < cable_length (i.e., tension in slack cable)
            if (have_tension_slack_penalty_factor) {
            graph.add(TensionSlackPenaltyFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length - cable_length_offset, weight_tension_slack, tension_cost));
            }

            // Factor 4: Penalise if T_k != k * max(0, distance - cable_length)
            if (have_tether_tension_factor) {
            graph.add(TethertensionFactorNoOri(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length - cable_length_offset, weight_tether_tension, tension_cost));
            }
           
            if (k > 0 && have_uk_prior) {
            Vector2 u0(0.0, 0.0);
            graph.add(PriorFactor<Vector2>(symbol_t('u', k), u0, control_interim_cost));
            }

            if (k > 0 && have_tk_prior) {
            graph.add(PriorFactor<Vector1>(symbol_t('t', k), last_tension_, tension_interim_cost));
            }

            graph.add(MagnitudeUpperBoundFactor(symbol_t('u', k), u_upper_bound, control_cost));
            graph.add(MagnitudeLowerBoundFactor(symbol_t('u', k), u_lower_bound, control_cost));

            if (k > 0 && have_trajectory_reference_factor) {
                Vector4 mid_state(4);
                mid_state << initial_load_state_ + k * diff;
                graph.add(PriorFactor<Vector4>(symbol_t('l', k), mid_state, goal_interim_cost));
            }
        }

        // --- 3. Create Initial Estimate ---
        Values initial_values;
        Vector2 init_u(0.0, 0.0);
        Vector1 init_t(0.0);
        for (int k = 0; k <= num_time_steps; ++k) {
            initial_values.insert(symbol_t('x', k), initial_robot_state_);
            initial_values.insert(symbol_t('l', k), initial_load_state_);
            if (k < num_time_steps) {
                initial_values.insert(symbol_t('u', k), last_u_);
                initial_values.insert(symbol_t('t', k), init_t);
            }
        }

        Values result = runOptimizer(debug_mode_, graph, initial_values, factor_errors, dt, mu, load_mass, gravity, 0.0, 0.0);

        Vector4 last_state = result.at<Vector4>(symbol_t('l', num_time_steps));
        double a1 = sqrt((final_load_goal_[0] - last_state[0]) * (final_load_goal_[0] - last_state[0]) + (final_load_goal_[1] - last_state[1]) * (final_load_goal_[1] - last_state[1]));
        double a2 = sqrt((final_load_goal_[0] - initial_load_state_[0]) * (final_load_goal_[0] - initial_load_state_[0]) + (final_load_goal_[1] - initial_load_state_[1]) * (final_load_goal_[1] - initial_load_state_[1]));
        pos_error = a1 / a2;

        Vector4 next_state = result.at<Vector4>(symbol_t('x', 1));
        Vector2 next_ctrl = result.at<Vector2>(symbol_t('u', 1));
        Vector1 next_tension = result.at<Vector1>(symbol_t('t', 1));

        return {next_state[2], next_state[3], next_ctrl[0], next_ctrl[1], next_tension[0]};
    }
};


        //         cable_length_offset 0.007
        // cable_stretch_offset 0.001
        // control_interim_cost 0.256
        // goal_cost 0.0005
        // u_lower_bound 0.003
        // u_upper_bound 40.8
        // weight_cable_stretch 1000
        // weight_tension_lower_bound 100000
        // weight_tension_slack 1000
        // weight_tether_tension 64

        // cable_length_offset 0.000512
        // cable_stretch_offset 0.001
        // control_interim_cost 0.002
        // goal_cost 0.0005
        // u_lower_bound 0.003
        // u_upper_bound 40.8
        // weight_cable_stretch 1000
        // weight_tension_lower_bound 100000
        // weight_tension_slack 1000
        // weight_tether_tension 4096

        // cable_length_offset 1e-05
        // cable_stretch_offset 0.001
        // control_interim_cost 0.001
        // goal_cost 0.0005
        // u_lower_bound 0.003
        // u_upper_bound 0.8
        // weight_cable_stretch 1000
        // weight_tension_lower_bound 100000
        // weight_tension_slack 1000
        // weight_tether_tension 16

        // cable_length_offset 0.0001
        // cable_stretch_offset 0.001
        // control_interim_cost 0.001
        // goal_cost 0.0005
        // u_lower_bound 0.003
        // u_upper_bound 0.8
        // weight_cable_stretch 1000
        // weight_tension_lower_bound 100000
        // weight_tension_slack 1000
        // weight_tether_tension 16

        // cable_length_offset 7e-05
        // cable_stretch_offset 0.001
        // control_interim_cost 1
        // goal_cost 5e-05
        // u_lower_bound 0.003
        // u_upper_bound 0.8
        // weight_cable_stretch 1000
        // weight_tension_lower_bound 100000
        // weight_tension_slack 1000
        // weight_tether_tension 1.6


        // cable_length_offset 7e-05
        // cable_stretch_offset 0.0001
        // control_interim_cost 1
        // goal_cost 5e-05
        // u_lower_bound 0.003
        // u_upper_bound 0.8
        // weight_cable_stretch 1000
        // weight_tension_lower_bound 1e+06
        // weight_tension_slack 5000
        // weight_tether_tension 0.0016

        // cable_length_offset 0.021875
        // cable_stretch_offset 0.0001
        // control_interim_cost 1
        // goal_cost 5e-05
        // u_lower_bound 0.003
        // u_upper_bound 10.8
        // weight_cable_stretch 1000
        // weight_tension_lower_bound 1e+06
        // weight_tension_slack 5000
        // weight_tether_tension 12.5

        //  Position mean: 0.0179714
        //  Load error mean: 2391.26

#endif // FACTOR_EXECUTOR_SINGLE_ROBOT_HPP