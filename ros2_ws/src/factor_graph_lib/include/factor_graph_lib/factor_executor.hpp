#ifndef FACTOR_EXECUTOR_HPP
#define FACTOR_EXECUTOR_HPP

#include <bits/stdc++.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/Ordering.h>


#include <Eigen/Dense>

#include "common.hpp"

using namespace std;
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
        bool with_angle
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

    void analyzeJacobianRank(const NonlinearFactorGraph& graph, const Values& initial_values, bool with_angle = false) const {
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
            //anlyzeKernelwithOrdering(null_space_basis, ordering, with_angle);

        } else {
            std::cout << "The linear system is full rank. If there is a problem it might be numerical or related to initial estimates." << std::endl;
        }
        //graph.saveGraph("factor_graph.dot", initial_values); 
        //cout << "To visualize, run: dot -Tpng factor_graph.dot -o graph.png" << endl; 
    }

    map<string, double> getFactorErrors(const NonlinearFactorGraph& graph, const Values& result, bool print_err) const {
        map<string, double> factor_errors;
        for (int i = 0; i < graph.size(); i++) {
            if (graph[i]) {
                string key = getFactorKeyString(graph[i]->keys());
                if (factor_errors.count(key) == 0) {
                    factor_errors[key] = 0;
                }
                factor_errors[key] += graph[i]->error(result);
                if (print_err) {
                    cout << "Factor "<< " error: " << graph[i]->error(result)  << " " << printKeys(graph[i]->keys())  << endl;
                }
            }
        }

        if (print_err) {
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
        return factor_errors;
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
                if (v_norm > 1e-12) {
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
            if (two_bots){
            cout << "  Dist 2: " << sqrt((robot_state2[0] - load_state[0]) * (robot_state2[0] - load_state[0])  + (robot_state2[1] - load_state[1]) * (robot_state2[1] - load_state[1]) ) << endl;   
            }
        }
    }

    void printOptimizedTrajectory(const Values& result, double dt, double mu, double load_mass, double gravity, double mu2, double inertia, int num_bots) const {
        int num_time_steps = 20;
        double eps_ = 1000000.0;
        cout << "\nOptimized Trajectory:" << endl;
        Vector6 prev_load;
        std::vector<Vector4> prev_robot;
        std::vector<Vector1> prev_tt;
        prev_robot.resize(num_bots);
        prev_tt.resize(num_bots);
        for (int k = 0; k <= num_time_steps; ++k) {
            cout << "--- Time Step " << k << " ---" << endl;

            std::vector<Vector4> robot_state;
            for (int i = 0; i < num_bots; i++) {
                Vector4 robot_state1 = result.at<Vector4>(symbol_t(x_[i], k));
                robot_state.push_back(robot_state1);
                cout << "  Robot " << i+1 << " Pos: (" << robot_state1[0] << ", " << robot_state1[1] << ", " << robot_state1[2] << ", " << robot_state1[3] << ")" << endl;
            }

            Vector6 load_state = result.at<Vector6>(symbol_t('l', k));
            cout << "  Load  Pos: (" << load_state[0]  << ", " << load_state[1] << ", " << load_state[2] << ", " << load_state[3] << ", " << load_state[4] << ", " << load_state[5] << ")" << endl;
            
            if (k > 0) {
                std::vector<CableVectorHelper> h;
                double v1_tensionf = 0;
                double v2_tensionf = 0;
                double v3_tensionf = 0;

                double r1 = -0.2 * cos(prev_load(2));
                double r2 = -0.2 * sin(prev_load(2));

                double r21 = -0.2 * sin(prev_load(2));
                double r22 = 0.2 * cos(prev_load(2));

                for (int i = 0; i < num_bots; i++) {
                    CableVectorHelper hh(prev_robot[i], prev_load);
                    h.push_back(hh);
                    v1_tensionf += prev_tt[i][0]* hh.e_norm(0);
                    v2_tensionf += prev_tt[i][0]* hh.e_norm(1);
                    v3_tensionf += r1 * prev_tt[i](0) * hh.e_norm(1) - r2 * prev_tt[i](0) * hh.e_norm(0);
                }

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
                double v1 = prev_load[3] + dt * (v1_tensionf - mu * load_mass * gravity * nv(0)) / load_mass;
                double v2 = prev_load[4] + dt * (v2_tensionf - mu * load_mass * gravity * nv(1)) / load_mass;
                double v3 = prev_load(5) + dt / inertia * (v3_tensionf - mu2 * load_mass * gravity * tanh(eps_ * prev_load(5))); 

                cout << " Load  Pos2: (" << p1 << ", " << p2 << ", " << p3 << ", " << v1 << ", " << v2 << ", " << v3 << ")" << endl;
                cout << "normed vel: " << nv(0) << ' ' << nv(1) << endl;

                for (int i = 0; i < num_bots; i++) {
                    cout << "E " << i+1 << " : " << h[i].e_norm(0) << ' ' << h[i].e_norm(1) << " T: " << prev_tt[i][0] << endl;
                    cout << "Pulling force " << i+1 << " : " << prev_tt[i][0]* h[i].e_norm(0)  / load_mass << ' '<< prev_tt[i][0]* h[i].e_norm(1) / load_mass << endl;
                    cout << "Torque " << i+1 << " : " << r1 * prev_tt[i](0) * h[i].e_norm(1) - r2 * prev_tt[i](0) * h[i].e_norm(0) << endl;
                }
                
                cout << "Friction force: " << -mu * gravity * nv(0) << ' ' << -mu * gravity * nv(1) << endl;
                cout << "Friction torque: " << -mu2 * load_mass * gravity * tanh(eps_ * prev_load(5)) << endl;
            }
            prev_load = load_state;
            for (int i = 0; i < num_bots; i++) {
                prev_robot[i] = robot_state[i];
            }
            
            if (k < num_time_steps) {
                for (int i = 0; i < num_bots; i++) {
                    Vector2 control = result.at<Vector2>(symbol_t(u_[i], k));
                    cout << "  Control "<<  i+1 << " :   (" << control[0] << ", " << control[1] << ")" << endl;
                    Vector1 tension = result.at<Vector1>(symbol_t(t_[i], k));
                    cout << "  Tension "<< i+1 << " :   (" << tension[0] << ")" << endl;
                    prev_tt[i] = tension;
                }
            }
            for (int i = 0; i < num_bots; i++) {
                CableVectorHelper h1(robot_state[i], load_state);
                cout << "  Dist "<< i+1 << " : " << sqrt((robot_state[i][0] - h1.ap_x) * (robot_state[i][0] - h1.ap_x)  + (robot_state[i][1] - h1.ap_y) * (robot_state[i][1] - h1.ap_y) ) << endl;

            }
        }
    }

    void printOptimizedTrajectoryWithHeight(const Values& result, double dt, double mu, double load_mass, double gravity, bool two_bots) const {
        int num_time_steps = 20;
        cout << "\nOptimized Trajectory:" << endl;
        Vector4 prev_load;
        Vector6 prev_robot1, prev_robot2;
        Vector1 prev_tt1, prev_tt2(0);
        for (int k = 0; k <= num_time_steps; ++k) {
            Vector6 robot_state1 = result.at<Vector6>(symbol_t('x', k));
            Vector6 robot_state2(0, 0, 0, 0, 0, 0);
            if (two_bots) {
                robot_state2 = result.at<Vector6>(symbol_t('X', k));
            }
            Vector4 load_state = result.at<Vector4>(symbol_t('l', k));
            cout << "--- Time Step " << k << " ---" << endl;
            cout << "  Robot 1 Pos: (" << robot_state1[0] << ", " << robot_state1[1] << ", " << robot_state1[2] << ", " << robot_state1[3] << ", " << robot_state1[4] << ", " << robot_state1[5] << ")" << endl;
            if (two_bots) {
            cout << "  Robot 2 Pos: (" << robot_state2[0] << ", " << robot_state2[1] << ", " << robot_state2[2] << ", " << robot_state2[3] << ", " << robot_state1[4] << ", " << robot_state1[5] << ")" << endl;
            }
            cout << "  Load  Pos: (" << load_state[0]  << ", " << load_state[1] << ", " << load_state[2] << ", " << load_state[3] << ")" << endl;
            if (k > 0) {
                CableVectorHelper h1(prev_robot1, prev_load, "middle");
                CableVectorHelper h2(prev_robot2, prev_load, "middle");
                Vector2 vel_k = prev_load.tail<2>();
                double v_norm = vel_k.norm();
                Vector2 nv = Vector2::Zero();
                if (v_norm > 1e-12) {
                    nv << vel_k / v_norm;
                }
                double v1 = prev_load[2] + dt * (prev_tt1[0]* h1.e_norm(0) + prev_tt2[0]* h2.e_norm(0) - mu * (load_mass * gravity - prev_tt1[0] * h1.e3_norm(2)) * nv(0)) / load_mass;
                double v2 = prev_load[3] + dt * (prev_tt1[0]* h1.e_norm(1) + prev_tt2[0]* h2.e_norm(1) - mu * (load_mass * gravity - prev_tt2[0] * h2.e3_norm(2)) * nv(1)) / load_mass;
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
                Vector3 control = result.at<Vector3>(symbol_t('u', k));
                cout << "  Control 1:   (" << control[0] << ", " << control[1] << ", " << control[2] << ")" << endl;
                Vector1 tension = result.at<Vector1>(symbol_t('t', k));
                cout << "  Tension 1:   (" << tension[0] << ")" << endl;
                prev_tt1 = tension;

                if (two_bots) {
                    Vector3 control2 = result.at<Vector3>(symbol_t('U', k));
                    cout << "  Control 2:   (" << control2[0] << ", " << control2[1] << ", " << control[2] << ")" << endl;
                    Vector1 tension2 = result.at<Vector1>(symbol_t('T', k));
                    cout << "  Tension 2:   (" << tension2[0] << ")" << endl;
                    prev_tt2 = tension2;
                }
            }

            cout << "  Dist 1: " << sqrt((robot_state1[0] - load_state[0]) * (robot_state1[0] - load_state[0])  + (robot_state1[1] - load_state[1]) * (robot_state1[1] - load_state[1])  + (robot_state1[2] - 0.2) * (robot_state1[2] - 0.2) ) << endl;
            if (two_bots){
            cout << "  Dist 2: " << sqrt((robot_state2[0] - load_state[0]) * (robot_state2[0] - load_state[0])  + (robot_state2[1] - load_state[1]) * (robot_state2[1] - load_state[1]) ) << endl;   
            }
        }
    }

    Values runOptimizer(
        const string& debug_mode, 
        NonlinearFactorGraph& graph, 
        const Values& initial_values, 
        map<string, double>& factor_errors,
        double dt, double mu, double load_mass, double gravity, double mu2, double inertia, int num_robots, bool with_angle = false) const {

        if (debug_mode == "one_of") {
            analyzeJacobianRank(graph, initial_values);
        }

        // --- 4. Optimize ---
        LevenbergMarquardtParams params;
        if (debug_mode == "one_of") {
            params.setMaxIterations(1000);
            cout << "\nOptimizing..." << endl;
        }
        LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);

        Values result = optimizer.optimize();

        if (debug_mode == "one_of" || debug_mode == "auto") {
            factor_errors = getFactorErrors(graph, result, (debug_mode == "one_of"));
        }

        if (debug_mode == "one_of") {
            checkMarginals(graph, result);
        }

        if (debug_mode == "one_of" || debug_mode == "sim") {
            cout << "Initial Error: " << graph.error(initial_values) << endl;
            cout << "Final Error: " << graph.error(result) << endl;
        }

        if (debug_mode == "one_of") {
            if (with_angle) {
                printOptimizedTrajectory(result, dt, mu, load_mass, gravity, mu2, inertia, num_robots);
            } else {
                printOptimizedTrajectory(result, dt, mu, load_mass, gravity, (num_robots > 1));
            }
        }

        return result;
    }

    map<string, double> tune_d_;
    map<string, bool> tune_b_;

    double getd(string field, double def) const {
        if (!tune_d_.count(field)) {
            return def;
        }
        return tune_d_.at(field);
    }

    bool getb(string field, bool def) const {
        if (!tune_b_.count(field)) {
            return def;
        }
        return tune_b_.at(field);
    }

    static inline const char x_[] = "xXzZ";
    static inline const char t_[] = "tTdD";
    static inline const char u_[] = "uUyY";

public:
    virtual FactorExecutorResult run(map<string, double>& factor_errors, double& pos_error) const = 0;

    FactorExecutor() : tune_d_({}), tune_b_({}) {}

    FactorExecutor(
        const map<string, double>& tune_d,
        const map<string, bool>& tune_b) : tune_d_(tune_d), tune_b_(tune_b) {}
};


#endif // FACTOR_EXECUTOR_HPP