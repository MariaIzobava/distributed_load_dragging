#ifndef FACTOR_EXECUTOR_HPP
#define FACTOR_EXECUTOR_HPP

#include <bits/stdc++.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/Ordering.h>


#include <Eigen/Dense>

#include "cable_factors.hpp"
#include "dynamics_factors.hpp"
#include "dynamics_factors_with_angle.hpp"
#include "control_factors.hpp"

using namespace gtsam;
using symbol_t = gtsam::Symbol;

class FactorExecutor{

protected:
    std::string printKeys(const KeyVector& keys) const {
        std::string s = "[";
        for (int i = 0; i < keys.size(); i++) {
            Symbol ss(keys[i]);
            s += ss.string();
            if (i < keys.size() - 1) {
                s += ", ";
            }
        }
        s += "]";
        return s;
    }

    std::string getFactorKeyString(const KeyVector& keys) const {
        std::string s = "[";
        for (int i = 0; i < keys.size(); i++) {
            Symbol ss(keys[i]);
            s += ss.string()[0];
            if (i < keys.size() - 1) {
                s += ", ";
            }
        }
        s += "]";
        return s;
    }

    void anlyzeKernelwithOrdering(
        const Eigen::MatrixXd& kernelMatrix,
        const gtsam::Ordering& ordering,
        bool with_angle = false
    ) const {
        size_t numVariables = kernelMatrix.rows();
        size_t numKernelVectors = kernelMatrix.cols();
        map<char, size_t> dim_map;
        dim_map['x'] = 4;
        dim_map['X'] = 4;
        dim_map['l'] = (with_angle) ? 6 : 4;
        dim_map['u'] = 2;
        dim_map['U'] = 2;
        dim_map['t'] = 1;
        dim_map['T'] = 1;

        for (int col = 0; col < numKernelVectors; col++) {
            cout << "----------------------------\n";
            cout << "Vector #" << col + 1 << endl;
            size_t row_index = 0;

            for (const auto& key_info : ordering) {
                auto key = Symbol(key_info).string();
                size_t dim = dim_map[key[0]];
                for (size_t d = 0; d < dim; d++) {
                    double value = kernelMatrix(row_index, col);
                    if (fabs(value) > 1e-9) {
                        cout << " Variable " << key << "[" << d+1 << "]: " << value << endl;
                    }
                    row_index++;
                }
            }

            if (row_index != numVariables) {
                cerr << "Number of ordering values " << row_index << " is not equal to the number of kernel rows " << numVariables << endl;
                return;
            }
            cout << endl;
        }
    }

    void analyzeJacobianRank(const NonlinearFactorGraph& graph, const Values& initial_values) const {
        Ordering ordering = graph.orderingCOLAMD();
        GaussianFactorGraph::shared_ptr gfg = graph.linearize(initial_values);

        // Get the Jacobian matrix (A)
        Eigen::MatrixXd A = gfg->jacobian(ordering).first;

        // Compute the rank and null space
        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A);
        int rank = lu_decomp.rank();
        int num_variables_dims = A.cols();

        std::cout << "\n--- Linear System Analysis ---" << std::endl;
        std::cout << "Jacobian Matrix (A) dimensions: " << A.rows() << " x " << A.cols() << std::endl;
        std::cout << "Rank of Jacobian matrix: " << rank << std::endl;
        std::cout << "Number of variable dimensions (columns in A): " << num_variables_dims << std::endl;

        if (rank < num_variables_dims) {
            std::cout << "\033[1;31mThe linear system is *rank-deficient*!\033[0m" << std::endl;
            int nullity = num_variables_dims - rank;
            std::cout << "This means there are " << nullity << " unconstrained degrees of freedom (nullity)." << std::endl;

            // Get the null space basis
            Eigen::MatrixXd null_space_basis = lu_decomp.kernel(); // 'kernel()' returns the null space basis
            anlyzeKernelwithOrdering(null_space_basis, ordering);

        } else {
            std::cout << "The linear system is full rank. If there is a problem it might be numerical or related to initial estimates." << std::endl;
        }
        //graph.saveGraph("factor_graph.dot", initial_values); 
        //cout << "To visualize, run: dot -Tpng factor_graph.dot -o graph.png" << endl; 
    }

    void printFactorErrors(const NonlinearFactorGraph& graph, const Values& result) const {
        map<string, double> factor_errors;
        for (int i = 0; i < graph.size(); i++) {
            if (graph[i]) {
                string key = getFactorKeyString(graph[i]->keys());
                if (factor_errors.count(key) == 0) {
                    factor_errors[key] = 0;
                }
                factor_errors[key] += graph[i]->error(result);
                cout << "Factor "<< " error: " << graph[i]->error(result)  << " " << printKeys(graph[i]->keys())  << endl;
            }
        }

        for (const auto& f : factor_errors) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << f.second;
            auto v_str = oss.str();
            int w = max(f.first.length(), v_str.length());
            cout << std::left << std::setw(w + 2) << f.first;
        }
        cout << endl;

        for (const auto& f : factor_errors) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << f.second;
            auto v_str = oss.str();
            int w = max(f.first.length(), v_str.length());
            cout << std::left << std::setw(w + 2) << v_str;
        }
        cout << endl;
    }

    void checkMarginals(const NonlinearFactorGraph& graph, const Values& result) const {
        try {
            Marginals marg(graph, result);
            //marg.print();
        } catch (exception& e) {
            cout << "\033[1;31mMarginals creation failed with: " << e.what() << "\033[0m" << endl;
        }
    }

    void printOptimizedTrajectory(const Values& result, double dt, double mu, double load_mass, double gravity, bool two_bots) const {
        int num_time_steps = 20;
        cout << "\nOptimized Trajectory:" << endl;
        Vector4 prev_load, prev_robot1, prev_robot2;
        Vector1 prev_tt1, prev_tt2(0);
        for (int k = 0; k <= num_time_steps; ++k) {
            Vector4 robot_state1 = result.at<Vector4>(symbol_t('x', k));
            Vector4 robot_state2(0, 0, 0, 0);
            if (two_bots) {
                robot_state2 = result.at<Vector4>(symbol_t('X', k));
            }
            Vector4 load_state = result.at<Vector4>(symbol_t('l', k));
            cout << "--- Time Step " << k << " ---" << endl;
            cout << "  Robot 1 Pos: (" << robot_state1[0] << ", " << robot_state1[1] << ", " << robot_state1[2] << ", " << robot_state1[3] << ")" << endl;
            if (two_bots) {
            cout << "  Robot 2 Pos: (" << robot_state2[0] << ", " << robot_state2[1] << ", " << robot_state2[2] << ", " << robot_state2[3] << ")" << endl;
            }
            cout << "  Load  Pos: (" << load_state[0]  << ", " << load_state[1] << ", " << load_state[2] << ", " << load_state[3] << ")" << endl;
            if (k > 0) {
                CableVectorHelper h1(prev_robot1, prev_load);
                CableVectorHelper h2(prev_robot2, prev_load);
                Vector2 vel_k = prev_load.tail<2>();
                double v_norm = vel_k.norm();
                Vector2 nv = Vector2::Zero();
                if (v_norm > 1e-6) {
                    nv << vel_k / v_norm;
                }
                double v1 = prev_load[2] + dt * (prev_tt1[0]* h1.e_norm(0) + prev_tt2[0]* h2.e_norm(0) - mu * load_mass * gravity * nv(0)) / load_mass;
                double v2 = prev_load[3] + dt * (prev_tt1[0]* h1.e_norm(1) + prev_tt2[0]* h2.e_norm(1) - mu * load_mass * gravity * nv(1)) / load_mass;
                cout << " Load  Pos2: (" << prev_load[0] + dt * prev_load[2]  << ", " << prev_load[1] + dt * prev_load[3] << ", " << v1 << ", " << v2 << ")" << endl;
                
                cout << "E 1: " << h1.e_norm(0) << ' ' << h1.e_norm(1) << " T: " << prev_tt1[0] << endl;
                if (two_bots) {
                cout << "E 2: " << h2.e_norm(0) << ' ' << h2.e_norm(1) << " T: " << prev_tt2[0] << endl;
                }
                
                cout << "normed vel: " << nv(0) << ' ' << nv(1) << endl;
                
                cout << "Pulling force 1: " << prev_tt1[0]* h1.e_norm(0)  / load_mass << ' '<< prev_tt1[0]* h1.e_norm(1) / load_mass << endl;
                if (two_bots) {
                cout << "Pulling force 2: " << prev_tt2[0]* h2.e_norm(0)  / load_mass << ' '<< prev_tt2[0]* h2.e_norm(1) / load_mass << endl;
                }
                
                cout << "Friction force: " << mu * gravity * nv(0) << ' ' << mu * gravity * nv(1) << endl;
            }
            prev_load = load_state;
            prev_robot1 = robot_state1;
            prev_robot2 = robot_state2;
            
            if (k < num_time_steps) {
                Vector2 control = result.at<Vector2>(symbol_t('u', k));
                cout << "  Control 1:   (" << control[0] << ", " << control[1] << ")" << endl;
                Vector1 tension = result.at<Vector1>(symbol_t('t', k));
                cout << "  Tension 1:   (" << tension[0] << ")" << endl;
                prev_tt1 = tension;

                if (two_bots) {
                    Vector2 control2 = result.at<Vector2>(symbol_t('U', k));
                    cout << "  Control 2:   (" << control2[0] << ", " << control2[1] << ")" << endl;
                    Vector1 tension2 = result.at<Vector1>(symbol_t('T', k));
                    cout << "  Tension 2:   (" << tension2[0] << ")" << endl;
                    prev_tt2 = tension2;
                }
            }

            cout << "  Dist 1: " << sqrt((robot_state1[0] - load_state[0]) * (robot_state1[0] - load_state[0])  + (robot_state1[1] - load_state[1]) * (robot_state1[1] - load_state[1]) ) << endl;
            cout << "  Dist 2: " << sqrt((robot_state2[0] - load_state[0]) * (robot_state2[0] - load_state[0])  + (robot_state2[1] - load_state[1]) * (robot_state2[1] - load_state[1]) ) << endl;
        }
    }

    void printOptimizedTrajectory(const Values& result, double dt, double mu, double load_mass, double gravity, double mu2, double inertia, bool two_bots) const {
        int num_time_steps = 20;
        cout << "\nOptimized Trajectory:" << endl;
        Vector6 prev_load;
        Vector4 prev_robot1, prev_robot2;
        Vector1 prev_tt1, prev_tt2(0);
        for (int k = 0; k <= num_time_steps; ++k) {
            Vector4 robot_state1 = result.at<Vector4>(symbol_t('x', k));
            Vector4 robot_state2(0, 0, 0, 0);
            if (two_bots) {
                robot_state2 = result.at<Vector4>(symbol_t('X', k));
            }
            Vector6 load_state = result.at<Vector6>(symbol_t('l', k));
            cout << "--- Time Step " << k << " ---" << endl;
            cout << "  Robot 1 Pos: (" << robot_state1[0] << ", " << robot_state1[1] << ", " << robot_state1[2] << ", " << robot_state1[3] << ")" << endl;
            if (two_bots) {
            cout << "  Robot 2 Pos: (" << robot_state2[0] << ", " << robot_state2[1] << ", " << robot_state2[2] << ", " << robot_state2[3] << ")" << endl;
            }
            cout << "  Load  Pos: (" << load_state[0]  << ", " << load_state[1] << ", " << load_state[2] << ", " << load_state[3] << ", " << load_state[4] << ", " << load_state[5] << ")" << endl;
            if (k > 0) {
                CableVectorHelper h1(prev_robot1, prev_load);
                CableVectorHelper h2(prev_robot2, prev_load, 2);
                Vector2 vel_k(2);
                vel_k << prev_load(3), prev_load(4);
                double v_norm = vel_k.norm();
                Vector2 nv = Vector2::Zero();
                if (v_norm > 1e-12) {
                    nv << vel_k / v_norm;
                }
                double p1 = prev_load[0] + dt * prev_load[3];
                double p2 = prev_load[1] + dt * prev_load[4];
                double p3 = prev_load[2] + dt * prev_load[5]; //atan2(sin(prev_load[2] + dt * prev_load[5]), cos(prev_load[2] + dt * prev_load[5]));
                double v1 = prev_load[3] + dt * (prev_tt1[0]* h1.e_norm(0) + prev_tt2[0]* h2.e_norm(0) - mu * load_mass * gravity * nv(0)) / load_mass;
                double v2 = prev_load[4] + dt * (prev_tt1[0]* h1.e_norm(1) + prev_tt2[0]* h2.e_norm(1) - mu * load_mass * gravity * nv(1)) / load_mass;

                double r1 = -0.2 * cos(prev_load(2));
                double r2 = -0.2 * sin(prev_load(2));

                double r21 = -0.2 * sin(prev_load(2));
                double r22 = 0.2 * cos(prev_load(2));

                double v3 = prev_load(5) + dt / inertia * (r1 * prev_tt1(0) * h1.e_norm(1) - r2 * prev_tt1(0) * h1.e_norm(0) + r21 * prev_tt2(0) * h2.e_norm(1) - r22 * prev_tt2(0) * h2.e_norm(0) - mu2 * load_mass * gravity * tanh(10000000.0 * prev_load(5))); 

                cout << " Load  Pos2: (" << p1 << ", " << p2 << ", " << p3 << ", " << v1 << ", " << v2 << ", " << v3 << ")" << endl;
                
                cout << "E 1: " << h1.e_norm(0) << ' ' << h1.e_norm(1) << " T: " << prev_tt1[0] << endl;
                if (two_bots) {
                cout << "E 2: " << h2.e_norm(0) << ' ' << h2.e_norm(1) << " T: " << prev_tt2[0] << endl;
                }
                
                cout << "normed vel: " << nv(0) << ' ' << nv(1) << endl;
                
                cout << "Pulling force 1: " << prev_tt1[0]* h1.e_norm(0)  / load_mass << ' '<< prev_tt1[0]* h1.e_norm(1) / load_mass << endl;
                if (two_bots) {
                cout << "Pulling force 2: " << prev_tt2[0]* h2.e_norm(0)  / load_mass << ' '<< prev_tt2[0]* h2.e_norm(1) / load_mass << endl;
                }
                
                cout << "Friction force: " << -mu * gravity * nv(0) << ' ' << -mu * gravity * nv(1) << endl;
                
                cout << "Torque 1: " << r1 * prev_tt1(0) * h1.e_norm(1) - r2 * prev_tt1(0) * h1.e_norm(0) << endl;
                if (two_bots) {
                cout << "Torque 2: " <<  r21 * prev_tt2(0) * h2.e_norm(1) - r22 * prev_tt2(0) * h2.e_norm(0) << endl; 
                }
                
                cout << "Friction torque: " << mu2 * load_mass * gravity * tanh(10000000.0 * prev_load(5)) << endl;
            }
            prev_load = load_state;
            prev_robot1 = robot_state1;
            prev_robot2 = robot_state2;
            
            if (k < num_time_steps) {
                Vector2 control = result.at<Vector2>(symbol_t('u', k));
                cout << "  Control:   (" << control[0] << ", " << control[1] << ")" << endl;
                Vector1 tension = result.at<Vector1>(symbol_t('t', k));
                cout << "  Tension:   (" << tension[0] << ")" << endl;
                prev_tt1 = tension;

                if (two_bots) {
                    Vector2 control2 = result.at<Vector2>(symbol_t('U', k));
                    cout << "  Control 2:   (" << control2[0] << ", " << control2[1] << ")" << endl;
                    Vector1 tension2 = result.at<Vector1>(symbol_t('T', k));
                    cout << "  Tension 2:   (" << tension2[0] << ")" << endl;
                    prev_tt2 = tension2;
                }
            }
            CableVectorHelper h1(robot_state1, load_state);
            CableVectorHelper h2(robot_state2, load_state, 2);
            cout << "  Dist 1: " << sqrt((robot_state1[0] - h1.ap_x) * (robot_state1[0] - h1.ap_x)  + (robot_state1[1] - h1.ap_y) * (robot_state1[1] - h1.ap_y) ) << endl;
            cout << "  Dist 2: " << sqrt((robot_state2[0] - h2.ap_x) * (robot_state2[0] - h2.ap_x)  + (robot_state2[1] - h2.ap_y) * (robot_state2[1] - h2.ap_y) ) << endl;
        }
    }

public:
    virtual std::vector<double> run() const = 0;
};


class FactorExecutorSingleRobot: public FactorExecutor {

    Vector4 initial_load_state_;
    Vector4 initial_robot_state_;
    Vector4 final_load_goal_;
    Vector2 last_u_;
    Vector1 last_tension_;
    double robot_height_;
    bool debug_mode_;

public:

    FactorExecutorSingleRobot(bool debug_mode, const Vector4& initial_load_state, const Vector4& initial_robot_state, const Vector4& final_load_goal, double robot_height, const Vector2& last_u, const Vector1& last_tension):
    debug_mode_(debug_mode), 
    initial_load_state_(initial_load_state), 
    initial_robot_state_(initial_robot_state), 
    final_load_goal_(final_load_goal), 
    robot_height_(robot_height),
    last_u_(last_u),
    last_tension_(last_tension) {}

    std::vector<double> run() const override {
    
        NonlinearFactorGraph graph;

        // Non changing values
        const int num_time_steps = 20;
        const double dt = 0.005;
        const double robot_mass = 0.025; // kg
        const double load_mass = 0.005;   // kg
        const double gravity = 9.81;
        const double mu = 0.3;

        // VALUES TO TUNE
        // =============================
        // =============================
        const double u_upper_bound = 0.8;
        const double u_lower_bound = 0.003;
        double weight_tension_lower_bound = 1000000.0;
        double weight_cable_stretch = 100.0;
        double weight_tension_slack = 50.0;
        double weight_tether_tension = 21.0;
        const double cable_length = 0.20 + sqrt(1.03 * 1.03 - (robot_height_ - 0.2) * (robot_height_ - 0.2));
        double cable_length_offset = 0.007;

        bool have_u0_prior = false;
        bool have_uk_prior = false;
        bool have_t0_prior = false;
        bool have_tension_slack_penalty_factor = true;
        bool have_tether_tension_factor = false;
        bool have_trajectory_reference_factor = false;

        // STATE PRIORS
        auto init_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
        auto goal_interim_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.01,    0.01,    1000.1, 1000.1).finished());
        auto goal_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.00005, 0.00005, 1000.1, 1000.1).finished());
        
        // DYNAMICS
        auto dynamics_robot_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
        auto dynamics_load_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());

        // CONTROL
        auto control_init_cost = noiseModel::Diagonal::Sigmas(
            (Vector(2) << 10, 10).finished());
        auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-1);
        auto control_interim_cost = noiseModel::Diagonal::Sigmas(
            (Vector(2) << 10, 10).finished());

        // TENSION
        auto tension_prior_cost = noiseModel::Isotropic::Sigma(1, 1e-2);
        auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
        // =============================
        // =============================


        // const double u_upper_bound = 1.4;
        // const double u_lower_bound = 0.0;
        // double weight_tension_lower_bound = 1000.0;
        // double weight_cable_stretch = 100.0;
        // double weight_tension_slack = 50.0;
        // const double cable_length = 0.20 + sqrt(1.03 * 1.03 - (robot_height_ - 0.2) * (robot_height_ - 0.2));

        // auto init_cost = noiseModel::Diagonal::Sigmas(
        //     (Vector(4) << 0.001, 0.001, 0.1, 0.1).finished());

        // auto dynamics_robot_cost = noiseModel::Diagonal::Sigmas(
        //     (Vector(4) << 0.0001, 0.0001, 0.0001, 0.0001).finished());
        // auto dynamics_load_cost = noiseModel::Diagonal::Sigmas(
        //     (Vector(4) << 0.0001, 0.0001, 0.0001, 0.0001).finished());
        // auto control_init_cost = noiseModel::Diagonal::Sigmas(
        //     (Vector(2) << 10, 10).finished());
        // auto control_interim_cost = noiseModel::Diagonal::Sigmas(
        //     (Vector(2) << 10, 10).finished());
        // auto smoothness_cost = noiseModel::Diagonal::Sigmas(
        //     (Vector(2) << 0.1, 0.1).finished());
        // auto tension_prior_cost = noiseModel::Isotropic::Sigma(1, 1e-2);
        // auto tension_cost = noiseModel::Isotropic::Sigma(1, 5e-4);
        // auto control_cost = noiseModel::Isotropic::Sigma(1, 1);

        // auto goal_cost = noiseModel::Diagonal::Sigmas(
        //     (Vector(4) << 0.00001, 0.00001, 1000.001, 1000.001).finished());
        // auto goal_interim_cost = noiseModel::Diagonal::Sigmas(
        //     (Vector(4) << 0.01, 0.01, 1000.001, 1000.001).finished());

        // --- Add Factors to the Graph ---

        // Add prior factors for the initial and final states
        graph.add(PriorFactor<Vector4>(symbol_t('x', 0), initial_robot_state_, init_cost));
        graph.add(PriorFactor<Vector4>(symbol_t('l', 0), initial_load_state_, init_cost));
        graph.add(PriorFactor<Vector4>(symbol_t('l', num_time_steps), final_load_goal_, goal_cost));

        if (have_u0_prior) {
          graph.add(PriorFactor<Vector2>(symbol_t('u', 0), last_u_, control_init_cost));
        }
        if (have_t0_prior) {
            graph.add(PriorFactor<Vector1>(symbol_t('t', 0), last_tension_, tension_prior_cost));
        }


        double xdiff = (final_load_goal_(0) - initial_load_state_(0)) / num_time_steps;
        double ydiff = (final_load_goal_(1) - initial_load_state_(1)) / num_time_steps;
        Vector4 diff(xdiff, ydiff, 0, 0);

        cout << "Cable length: " << cable_length << std::endl;

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
            graph.add(TensionLowerBoundFactor(symbol_t('t', k), weight_tension_lower_bound, tension_cost));
            
            // Factor 2: Penalize if ||p_r - p_l|| > cable_length
            graph.add(CableStretchPenaltyFactor(symbol_t('x', k), symbol_t('l', k), cable_length, weight_cable_stretch, tension_cost));
            
            // Factor 3: Penalize if T_k > 0 AND ||p_r - p_l|| < cable_length (i.e., tension in slack cable)
            if (have_tension_slack_penalty_factor) {
            graph.add(TensionSlackPenaltyFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length - cable_length_offset, weight_tension_slack, tension_cost));
            }

            // Factor 4: Penalise if T_k != k * max(0, distance - cable_length)
            if (have_tether_tension_factor) {
            graph.add(TethertensionFactorNoOri(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length - cable_length_offset, weight_tether_tension, tension_cost));
            }
           
            if (k > 0 && have_uk_prior) {
            graph.add(PriorFactor<Vector2>(symbol_t('u', k), last_u_, control_interim_cost));
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
                initial_values.insert(symbol_t('u', k), init_u);
                initial_values.insert(symbol_t('t', k), init_t);
            }
        }

        if (debug_mode_) {
            analyzeJacobianRank(graph, initial_values);
        }
        

        // --- 4. Optimize ---
        LevenbergMarquardtParams params;
        LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);

        cout << "\nOptimizing..." << endl;
        Values result = optimizer.optimize();
        cout << "Optimization complete." << endl;

        if (debug_mode_) {
            printFactorErrors(graph, result);
            checkMarginals(graph, result);
        }

        cout << "Initial Error: " << graph.error(initial_values) << endl;
        cout << "Final Error: " << graph.error(result) << endl;

        if (debug_mode_) {
            printOptimizedTrajectory(result, dt, mu, load_mass, gravity, false);
        }

        Vector4 next_state = result.at<Vector4>(symbol_t('x', 1));
        Vector2 next_ctrl = result.at<Vector2>(symbol_t('u', 1));
        Vector1 next_tension = result.at<Vector1>(symbol_t('t', 1));

        return {next_state[2], next_state[3], next_ctrl[0], next_ctrl[1], next_tension[0]};
    }
};


class FactorExecutorSingleRobotWithOrientation: public FactorExecutor {

    Vector6 initial_load_state_;
    Vector4 initial_robot_state_;
    Vector6 final_load_goal_;
    double robot_height_;
    bool debug_mode_;

public:

    FactorExecutorSingleRobotWithOrientation(bool debug_mode, const Vector6& initial_load_state, const Vector4& initial_robot_state, const Vector6& final_load_goal, double robot_height):
    debug_mode_(debug_mode), 
    initial_load_state_(initial_load_state), 
    initial_robot_state_(initial_robot_state), 
    final_load_goal_(final_load_goal), 
    robot_height_(robot_height) {}

    std::vector<double> run() const override {
    
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

        // VALUES TO TUNE
        // =============================
        // =============================
        const double u_upper_bound = 0.4;
        const double u_lower_bound = 0.003;
        double weight_tension_lower_bound = 1000000.0;
        double weight_cable_stretch = 100.0;
        double weight_tension_slack = 50.0;
        double weight_tether_tension = 21.0;
        const double cable_length = sqrt(1.03 * 1.03 - (robot_height_ - 0.2) * (robot_height_ - 0.2));
        
        bool have_tension_slack_penalty_factor = true;
        bool have_tether_tension_factor = false;
        bool have_trajectory_reference_factor = true;

        // STATE PRIORS
        auto init_cost = noiseModel::Diagonal::Sigmas(
        (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
        auto init_with_angle_cost = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.000005, 0.000005, 0.000005, 0.000005, 0.000005, 0.000005).finished());
        // 0.00005 the best
        auto goal_with_angle_cost = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.000005, 0.000005, 0.00005, 1000.1, 1000.1, 1000.1).finished());
        auto interim_goal_with_angle_cost = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.000005, 0.000005, 0.00005,    1000.1, 1000.1, 1000.1).finished());

        // DYNAMICS
        auto dynamics_cost = noiseModel::Diagonal::Sigmas(
        (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
        auto dynamics_with_angle_cost = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.001, 0.001, 0.01, 0.001, 0.001, 0.01).finished());

        // CONTROL
        auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-2);

        // TENSION
        auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
        // =============================
        // =============================


        graph.add(PriorFactor<Vector4>(symbol_t('x', 0), initial_robot_state_, init_cost));
        graph.add(PriorFactor<Vector6>(symbol_t('l', 0), initial_load_state_, init_with_angle_cost));
        graph.add(PriorFactor<Vector6>(symbol_t('l', num_time_steps), final_load_goal_, goal_with_angle_cost));

        double xdiff = (final_load_goal_(0) - initial_load_state_(0)) / num_time_steps;
        double ydiff = (final_load_goal_(1) - initial_load_state_(1)) / num_time_steps;
        double adiff = (final_load_goal_(2) - initial_load_state_(2));
        //adiff = atan2(sin(adiff), cos(adiff));
        adiff /= num_time_steps;
        Vector6 diff(xdiff, ydiff, adiff, 0, 0, 0);

        cout << "EXEC: Cable length: " << cable_length << std::endl;

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
            graph.add(TensionLowerBoundFactor(symbol_t('t', k), weight_tension_lower_bound, tension_cost));
            
            // Factor 2: Penalize if ||p_r - p_l|| > cable_length
            graph.add(CableStretchPenaltyWithOrientationFactor(symbol_t('x', k), symbol_t('l', k), cable_length, weight_cable_stretch, tension_cost));

            // Factor 3: Penalize if T_k > 0 AND ||p_r - p_l|| < cable_length (i.e., tension in slack cable)
            if (have_tension_slack_penalty_factor) {
            graph.add(TensionSlackPenaltyWithLoadOrientationFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length, weight_tension_slack, tension_cost));
            }

            // Factor 4: Penalise if T_k != k * max(0, distance - cable_length)
            if (have_tether_tension_factor) {
            graph.add(TethertensionFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length, weight_tether_tension, tension_cost));
            }
            
            graph.add(MagnitudeUpperBoundFactor(symbol_t('u', k), u_upper_bound, control_cost));
            graph.add(MagnitudeLowerBoundFactor(symbol_t('u', k), u_lower_bound, control_cost));

            if (k > 0 && have_trajectory_reference_factor) {
                Vector6 mid_state(6);
                mid_state << initial_load_state_ + k * diff;
                //mid_state(2) = atan2(sin(mid_state(2)), cos(mid_state(2))) ;
                graph.add(PriorFactor<Vector6>(symbol_t('l', k), mid_state, interim_goal_with_angle_cost));
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

        if (debug_mode_) {
            analyzeJacobianRank(graph, initial_values);
        }
        
        // // --- 4. Optimize ---
        LevenbergMarquardtParams params;
        params.setMaxIterations(1000);
        LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);

        cout << "\nOptimizing..." << endl;
        Values result = optimizer.optimize();
        cout << "Optimization complete." << endl;

        if (debug_mode_) {
            printFactorErrors(graph, result);
            checkMarginals(graph, result);
        }

        cout << "Initial Error: " << graph.error(initial_values) << endl;
        cout << "Final Error: " << graph.error(result) << endl;

        if (debug_mode_) {
            printOptimizedTrajectory(result, dt, mu, load_mass, gravity, mu2, inertia, false);
        }

        Vector4 next_state = result.at<Vector4>(symbol_t('x', 1));
        Vector2 next_ctrl = result.at<Vector2>(symbol_t('u', 1));

        cout << "Next control: " << next_ctrl[0] << ' ' << next_ctrl[1] << endl;
        return {next_state[2], next_state[3]};
    }
};


class FactorExecutorTwoRobots: public FactorExecutor {

    Vector4 initial_load_state_;
    Vector4 initial_robot1_state_;
    Vector4 initial_robot2_state_;
    Vector4 final_load_goal_;
    double robot_height1_, robot_height2_;
    bool debug_mode_;

public:

    FactorExecutorTwoRobots(bool debug_mode, const Vector4& initial_load_state, const Vector4& initial_robot1_state, const Vector4& initial_robot2_state, const Vector4& final_load_goal, double robot_height1, double robot_height2):
    debug_mode_(debug_mode), 
    initial_load_state_(initial_load_state), 
    initial_robot1_state_(initial_robot1_state),
    initial_robot2_state_(initial_robot2_state), 
    final_load_goal_(final_load_goal), 
    robot_height1_(robot_height1),
    robot_height2_(robot_height2) {}

    std::vector<double> run() const override {

        NonlinearFactorGraph graph;

        // --- Define problem parameters ---
        const int num_time_steps = 20;
        const double dt = 0.005;
        const double robot_mass = 0.025; // kg
        const double load_mass = 0.01;   // kg
        const double gravity = 9.81; 
        const double mu = 0.3;
        

        // VALUES TO TUNE
        // =============================
        // =============================
        const double u_upper_bound = 0.4; 
        const double u_lower_bound = 0.03;
        double weight_tension_lower_bound = 1000000.0;
        double weight_cable_stretch = 100.0;
        double weight_tension_slack = 50.0;
        const double cable_length1 = 1.02; //0.20 + sqrt(1.03 * 1.03 - (robot_height1_) * (robot_height1_));
        const double cable_length2 = 1.02; //0.20 + sqrt(1.03 * 1.03 - (robot_height2_) * (robot_height2_));
        int cable_stretch_penalty_offset = 0; //0.002;
        int cable_tension_penalty_offset = 0; //0.007;
        
        bool have_trajectory_reference_factor = true;

        // STATE PRIORS
        auto init_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
        auto goal_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.00005, 0.00005, 1000.1, 1000.1).finished());

        // DYNAMICS
        auto dynamics_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
        
        // CONTROL
        auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-2);

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

        cout << "Cable lengths: " << cable_length1 << ' ' << cable_length2 << std::endl;

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
            graph.add(TensionLowerBoundFactor(symbol_t('t', k), weight_tension_lower_bound, tension_cost));
            graph.add(TensionLowerBoundFactor(symbol_t('T', k), weight_tension_lower_bound, tension_cost));
            
            // Factor 2: Penalize if ||p_r - p_l|| > cable_length
            //graph.add(CableStretchPenaltyFactor(symbol_t('x', k), symbol_t('l', k), cable_length1 + cable_stretch_penalty_offset, weight_cable_stretch, tension_cost));
            //graph.add(CableStretchPenaltyFactor(symbol_t('X', k), symbol_t('l', k), cable_length2 + cable_stretch_penalty_offset, weight_cable_stretch, tension_cost));
            
            // Factor 3: Penalize if T_k > 0 AND ||p_r - p_l|| < cable_length (i.e., tension in slack cable)
            graph.add(TensionSlackPenaltyFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length1 - cable_tension_penalty_offset, weight_tension_slack, tension_cost));
            graph.add(TensionSlackPenaltyFactor(symbol_t('T', k), symbol_t('X', k), symbol_t('l', k), cable_length2 - cable_tension_penalty_offset, weight_tension_slack, tension_cost));

            // Add a soft cost on control input to keep it constrained (prevents wild and too slow solutions).
            graph.add(MagnitudeUpperBoundFactor(symbol_t('u', k), u_upper_bound, control_cost));
            graph.add(MagnitudeLowerBoundFactor(symbol_t('u', k), u_lower_bound, control_cost));
            graph.add(MagnitudeUpperBoundFactor(symbol_t('U', k), u_upper_bound, control_cost));
            graph.add(MagnitudeLowerBoundFactor(symbol_t('U', k), u_lower_bound, control_cost));

            if (k > 0 && have_trajectory_reference_factor) {
                Vector4 mid_state(4);
                mid_state << initial_load_state_ + k * diff;
                graph.add(PriorFactor<Vector4>(symbol_t('l', k), mid_state, goal_cost));
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

        if (debug_mode_) {
            analyzeJacobianRank(graph, initial_values);
        }

        // --- 4. Optimize ---
        LevenbergMarquardtParams params;
        LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);

        cout << "\nOptimizing..." << endl;
        Values result = optimizer.optimize();

        if (debug_mode_) {
            printFactorErrors(graph, result);
            checkMarginals(graph, result);
        }

        cout << "Optimization complete." << endl;
        cout << "Initial Error: " << graph.error(initial_values) << endl;
        cout << "Final Error: " << graph.error(result) << endl;

        if (debug_mode_) {
            printOptimizedTrajectory(result, dt, mu, load_mass, gravity, true);
            // cout << "\nOptimized Trajectory:" << endl;
            // Vector4 prev_load, prev_robot1, prev_robot2;
            // Vector2 prev_c1, prev_c2;
            // Vector1 prev_tt1, prev_tt2;
            // for (int k = 0; k <= num_time_steps; ++k) {
            //     Vector4 robot_state1 = result.at<Vector4>(symbol_t('x', k));
            //     Vector4 robot_state2 = result.at<Vector4>(symbol_t('X', k));
            //     Vector4 load_state = result.at<Vector4>(symbol_t('l', k));
            //     cout << "--- Time Step " << k << " ---" << endl;
            //     cout << "  Robot 1 Pos: (" << robot_state1[0] << ", " << robot_state1[1] << ", " << robot_state1[2] << ", " << robot_state1[3] << ")" << endl;
            //     cout << "  Robot 2 Pos: (" << robot_state2[0] << ", " << robot_state2[1] << ", " << robot_state2[2] << ", " << robot_state2[3] << ")" << endl;
            //     cout << "  Load  Pos: (" << load_state[0]  << ", " << load_state[1] << ", " << load_state[2] << ", " << load_state[3] << ")" << endl;
            //     if (k > 0) {
            //         CableVectorHelper h1(prev_robot1, prev_load);
            //         CableVectorHelper h2(prev_robot2, prev_load);
            //         Vector2 vel_k = prev_load.tail<2>();
            //         double v_norm = vel_k.norm();
            //         Vector2 nv = Vector2::Zero();
            //         if (v_norm > 1e-6) {
            //             nv << vel_k / v_norm;
            //         }
            //         double v1 = prev_load[2] + dt * (prev_tt1[0]* h1.e_norm(0) + prev_tt2[0]* h2.e_norm(0) - mu * load_mass * gravity * nv(0)) / load_mass;
            //         double v2 = prev_load[3] + dt * (prev_tt1[0]* h1.e_norm(1) + prev_tt2[0]* h2.e_norm(1) - mu * load_mass * gravity * nv(1)) / load_mass;
            //         cout << " Load  Pos2: (" << prev_load[0] + dt * prev_load[2]  << ", " << prev_load[1] + dt * prev_load[3] << ", " << v1 << ", " << v2 << ")" << endl;
            //         cout << "E 1: " << h1.e_norm(0) << ' ' << h1.e_norm(1) << " T: " << prev_tt1[0] << endl;
            //         cout << "E 2: " << h2.e_norm(0) << ' ' << h2.e_norm(1) << " T: " << prev_tt2[0] << endl;
            //         cout << "normed vel: " << nv(0) << ' ' << nv(1) << endl;
            //         cout << "Pulling force 1: " << prev_tt1[0]* h1.e_norm(0)  / load_mass << ' '<< prev_tt1[0]* h1.e_norm(1) / load_mass << endl;
            //         cout << "Pulling force 2: " << prev_tt2[0]* h2.e_norm(0)  / load_mass << ' '<< prev_tt2[0]* h2.e_norm(1) / load_mass << endl;
            //         cout << "Friction force: " << mu * gravity * nv(0) << ' ' << mu * gravity * nv(1) << endl;
            //     }
            //     prev_load = load_state;
            //     prev_robot1 = robot_state1;
            //     prev_robot2 = robot_state2;
                
            //     if (k < num_time_steps) {
            //         Vector2 control1 = result.at<Vector2>(symbol_t('u', k));
            //         cout << "  Control 1:   (" << control1[0] << ", " << control1[1] << ")" << endl;
            //         Vector1 tension1 = result.at<Vector1>(symbol_t('t', k));
            //         cout << "  Tension 1:   (" << tension1[0] << ")" << endl;
            //         prev_tt1 = tension1;
            //         prev_c1 = control1;

            //         Vector2 control = result.at<Vector2>(symbol_t('U', k));
            //         cout << "  Control 2:   (" << control[0] << ", " << control[1] << ")" << endl;
            //         Vector1 tension = result.at<Vector1>(symbol_t('T', k));
            //         cout << "  Tension 2:   (" << tension[0] << ")" << endl;
            //         prev_tt2 = tension;
            //         prev_c2 = control;
            //     }
            //     cout << "  Dist 1: " << sqrt((robot_state1[0] - load_state[0]) * (robot_state1[0] - load_state[0])  + (robot_state1[1] - load_state[1]) * (robot_state1[1] - load_state[1]) ) << endl;
            //     cout << "  Dist 2: " << sqrt((robot_state2[0] - load_state[0]) * (robot_state2[0] - load_state[0])  + (robot_state2[1] - load_state[1]) * (robot_state2[1] - load_state[1]) ) << endl;
            // }
        }

        Vector4 next_state1 = result.at<Vector4>(symbol_t('x', 1));
        Vector4 next_state2 = result.at<Vector4>(symbol_t('X', 1));
        Vector2 next_ctrl1 = result.at<Vector2>(symbol_t('u', 1));
        Vector2 next_ctrl2 = result.at<Vector2>(symbol_t('U', 1));

        cout << "Next controls: " << next_ctrl1[0] << ' ' << next_ctrl1[1] << ' ' << next_ctrl2[0] << ' ' << next_ctrl2[1] << endl;
        return {next_state1[2], next_state1[3], next_state2[2], next_state2[3]};
    }


};


class FactorExecutorTwoRobotsWithOrientation: public FactorExecutor {

    Vector6 initial_load_state_;
    Vector4 initial_robot1_state_;
    Vector4 initial_robot2_state_;
    Vector6 final_load_goal_;
    double robot_height1_, robot_height2_;
    bool debug_mode_;

public:

    FactorExecutorTwoRobotsWithOrientation(bool debug_mode, const Vector6& initial_load_state, const Vector4& initial_robot1_state, const Vector4& initial_robot2_state, const Vector6& final_load_goal, double robot_height1, double robot_height2):
    debug_mode_(debug_mode), 
    initial_load_state_(initial_load_state), 
    initial_robot1_state_(initial_robot1_state),
    initial_robot2_state_(initial_robot2_state), 
    final_load_goal_(final_load_goal), 
    robot_height1_(robot_height1),
    robot_height2_(robot_height2) {}

    std::vector<double> run() const override {

        NonlinearFactorGraph graph;

        // --- Define problem parameters ---
        const int num_time_steps = 20;
        const double dt = 0.005;
        const double robot_mass = 0.025; // kg
        const double load_mass = 0.01;   // kg
        const double inertia = 0.000266;
        const double gravity = 9.81;
        const double mu = 0.3;
        const double mu2 = 0.3;
        
        // VALUES TO TUNE
        // =============================
        // =============================
        const double u_upper_bound = 1.8;
        const double u_lower_bound = 0.07;
        double weight_tension_lower_bound = 1000.0;
        double weight_cable_stretch = 100.0;
        double weight_tension_slack = 50.0;
        double weight_tether_tension = 500.0;
        const double cable_length1 = sqrt(1.03 * 1.03 - (robot_height1_) * (robot_height1_));
        const double cable_length2 = sqrt(1.03 * 1.03 - (robot_height2_) * (robot_height2_));
        

        bool have_tension_slack_penalty_factor = true;
        bool have_tether_tension_factor = false;
        bool have_trajectory_reference_factor = false;

        // STATE PRIORS
        auto init_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.0000005, 0.0000005, 0.0000005, 0.0000005).finished());
        auto init_cost_with_angle = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.0000005, 0.0000005, 0.0000005, 0.0000005, 0.0000005, 0.0000005).finished());
        auto goal_cost_with_angle = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.000001, 0.000001, 0.00001, 1000.1, 1000.1, 1000.1).finished());

        // DYNAMICS
        auto dynamics_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.0001, 0.0001, 0.0001, 0.0001).finished());
        auto dynamics_cost_with_angle = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.0001, 0.0001, 0.0001, 1000.0001, 1000.0001, 1000.001).finished());

        // CONTROL
        auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-3);

        // TENSION
        auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
        auto tension2_cost = noiseModel::Isotropic::Sigma(1, 1e-4);
        // =============================
        // =============================


        graph.add(PriorFactor<Vector4>(symbol_t('x', 0), initial_robot1_state_, init_cost));
        graph.add(PriorFactor<Vector4>(symbol_t('X', 0), initial_robot2_state_, init_cost));
        graph.add(PriorFactor<Vector6>(symbol_t('l', 0), initial_load_state_, init_cost_with_angle));
        graph.add(PriorFactor<Vector6>(symbol_t('l', num_time_steps), final_load_goal_, goal_cost_with_angle));

        double xdiff = (final_load_goal_(0) - initial_load_state_(0)) / num_time_steps;
        double ydiff = (final_load_goal_(1) - initial_load_state_(1)) / num_time_steps;
        double adiff = (final_load_goal_(2) - initial_load_state_(2));
        adiff = atan2(sin(adiff), cos(adiff));
        adiff /= num_time_steps;
        Vector6 diff(xdiff, ydiff, adiff, 0, 0, 0);

        cout << "Cable lengths: " << cable_length1 << ' ' << cable_length2 << std::endl;

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
                2
                ));

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
                dynamics_cost_with_angle
                ));

            // Factor 1: Enforce T_k >= 0
            graph.add(TensionLowerBoundFactor(symbol_t('t', k), weight_tension_lower_bound, tension_cost));
            graph.add(TensionLowerBoundFactor(symbol_t('T', k), weight_tension_lower_bound, tension_cost));
            
            // Factor 2: Penalize if ||p_r - p_l|| > cable_length
            graph.add(CableStretchPenaltyWithOrientationFactor(symbol_t('x', k), symbol_t('l', k), cable_length1, weight_cable_stretch, tension_cost));
            graph.add(CableStretchPenaltyWithOrientationFactor(symbol_t('X', k), symbol_t('l', k), cable_length2, weight_cable_stretch, tension_cost, 2));
            
            // Factor 3: Penalize if T_k > 0 AND ||p_r - p_l|| < cable_length (i.e., tension in slack cable)
            if (have_tension_slack_penalty_factor) {
            graph.add(TensionSlackPenaltyWithLoadOrientationFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length1, weight_tension_slack, tension_cost));
            graph.add(TensionSlackPenaltyWithLoadOrientationFactor(symbol_t('T', k), symbol_t('X', k), symbol_t('l', k), cable_length2, weight_tension_slack, tension_cost, 2));
            }

            if (have_tether_tension_factor) {
            graph.add(TethertensionFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length1, weight_tether_tension, tension2_cost));
            graph.add(TethertensionFactor(symbol_t('T', k), symbol_t('X', k), symbol_t('l', k), cable_length2, weight_tether_tension, tension2_cost, 2));
            }

            // Add a soft cost on control input to keep it constrained (prevents wild and too slow solutions).
            graph.add(MagnitudeUpperBoundFactor(symbol_t('u', k), u_upper_bound, control_cost));
            graph.add(MagnitudeLowerBoundFactor(symbol_t('u', k), u_lower_bound, control_cost));
            graph.add(MagnitudeUpperBoundFactor(symbol_t('U', k), u_upper_bound, control_cost));
            graph.add(MagnitudeLowerBoundFactor(symbol_t('U', k), u_lower_bound, control_cost));

            if (k > 0 && have_trajectory_reference_factor) {
                Vector6 mid_state(6);
                mid_state << initial_load_state_ + k * diff;
                mid_state(2) = atan2(sin(mid_state(2)), cos(mid_state(2))) ;
                graph.add(PriorFactor<Vector6>(symbol_t('l', k), mid_state, goal_cost_with_angle));
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

        if (debug_mode_) {
            analyzeJacobianRank(graph, initial_values);
        }

        // --- 4. Optimize ---
        LevenbergMarquardtParams params;
        LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);

        cout << "\nOptimizing..." << endl;
        Values result = optimizer.optimize();

        if (debug_mode_) {
            printFactorErrors(graph, result);
            checkMarginals(graph, result);
        }

        cout << "Optimization complete." << endl;
        cout << "Initial Error: " << graph.error(initial_values) << endl;
        cout << "Final Error: " << graph.error(result) << endl;

        if (debug_mode_) {
            printOptimizedTrajectory(result, dt, mu, load_mass, gravity, mu2, inertia, true);
        }

        Vector4 next_state1 = result.at<Vector4>(symbol_t('x', 1));
        Vector4 next_state2 = result.at<Vector4>(symbol_t('X', 1));
        Vector2 next_ctrl1 = result.at<Vector2>(symbol_t('u', 1));
        Vector2 next_ctrl2 = result.at<Vector2>(symbol_t('U', 1));

        cout << "Next controls: " << next_ctrl1[0] << ' ' << next_ctrl1[1] << ' ' << next_ctrl2[0] << ' ' << next_ctrl2[1] << endl;
        return {next_state1[2], next_state1[3], next_state2[2], next_state2[3]};
    }
};
    

class FactorExecutorFactory {

public:

    static std::unique_ptr<FactorExecutor> create(
        bool debug_mode, 
        const Vector4& initial_load_state, 
        const Vector4& initial_robot_state, 
        const Vector4& final_load_goal, 
        double robot_height,
        const Vector2& last_u,
        const Vector1& last_tension) {
        return std::make_unique<FactorExecutorSingleRobot>(debug_mode, initial_load_state, initial_robot_state, final_load_goal, robot_height, last_u, last_tension);
    }

    static std::unique_ptr<FactorExecutor> create(
        bool debug_mode, 
        const Vector6& initial_load_state, 
        const Vector4& initial_robot_state, 
        const Vector6& final_load_goal, 
        double robot_height) {
        return std::make_unique<FactorExecutorSingleRobotWithOrientation>(debug_mode, initial_load_state, initial_robot_state, final_load_goal, robot_height);
    }

    static std::unique_ptr<FactorExecutor> create(
        bool debug_mode, 
        const Vector4& initial_load_state, 
        const Vector4& initial_robot1_state,
        const Vector4& initial_robot2_state, 
        const Vector4& final_load_goal, 
        double robot_height1,
        double robot_height2) {
        return std::make_unique<FactorExecutorTwoRobots>(debug_mode, initial_load_state, initial_robot1_state, initial_robot2_state, final_load_goal, robot_height1, robot_height2);
    }

    static std::unique_ptr<FactorExecutor> create(
        bool debug_mode, 
        const Vector6& initial_load_state, 
        const Vector4& initial_robot1_state,
        const Vector4& initial_robot2_state, 
        const Vector6& final_load_goal, 
        double robot_height1,
        double robot_height2) {
        return std::make_unique<FactorExecutorTwoRobotsWithOrientation>(debug_mode, initial_load_state, initial_robot1_state, initial_robot2_state, final_load_goal, robot_height1, robot_height2);
    }
};


#endif // FACTOR_EXECUTOR_HPP