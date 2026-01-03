#ifndef FACTOR_EXECUTOR_FACTORY_HPP
#define FACTOR_EXECUTOR_FACTORY_HPP

#include <bits/stdc++.h>

#include "factor_executor.hpp"
#include "factor_executor_single_robot.hpp"
#include "factor_executor_single_robot_with_ori.hpp"
#include "factor_executor_single_robot_with_height.hpp"
#include "factor_executor_two_robots.hpp"
#include "factor_executor_two_robots_with_ori.hpp"
#include "factor_executor_multi_robot_with_ori.hpp"
#include "factor_executor_multi_robot_with_height.hpp"
#include "factor_executor_multi_robot_with_height_and_ori.hpp"

using namespace std;
using namespace gtsam;
using symbol_t = gtsam::Symbol;


class FactorExecutorFactory {

public:

    static std::unique_ptr<FactorExecutor> create(
        string debug_mode, 
        const Vector4& initial_load_state, 
        const Vector4& initial_robot_state, 
        const Vector4& final_load_goal, 
        double robot_height,
        const std::vector<double>& desired_robot_heights,
        const Vector2& last_u,
        const Vector1& last_tension,
        const map<string, double>& tune_d,
        const map<string, bool>& tune_b
    ) {
        return std::make_unique<FactorExecutorSingleRobot>(debug_mode, initial_load_state, initial_robot_state, final_load_goal, robot_height, desired_robot_heights, last_u, last_tension, tune_d, tune_b);
    }

    static std::unique_ptr<FactorExecutor> create(
        string debug_mode, 
        const Vector6& initial_load_state, 
        const Vector4& initial_robot_state, 
        const Vector6& final_load_goal, 
        double robot_height,
        double desired_robot_height,
        const map<string, double>& tune_d,
        const map<string, bool>& tune_b) {
        return std::make_unique<FactorExecutorSingleRobotWithOrientation>(debug_mode, initial_load_state, initial_robot_state, final_load_goal, robot_height, desired_robot_height, tune_d, tune_b);
    }

    static std::unique_ptr<FactorExecutor> create(
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
        const map<string, bool>& tune_b) {
        return std::make_unique<FactorExecutorTwoRobots>(debug_mode, initial_load_state, initial_robot1_state, initial_robot2_state, final_load_goal, robot_height1, robot_height2, desired_robot_heights, last_u1, last_u2, tune_d, tune_b);
    }

    static std::unique_ptr<FactorExecutor> create(
        string debug_mode, 
        const Vector6& initial_load_state, 
        const Vector4& initial_robot1_state,
        const Vector4& initial_robot2_state, 
        const Vector6& final_load_goal, 
        double robot_height1,
        double robot_height2,
        const std::vector<double>& desired_robot_heights,
        const map<string, double>& tune_d,
        const map<string, bool>& tune_b) {
        return std::make_unique<FactorExecutorTwoRobotsWithOrientation>(debug_mode, initial_load_state, initial_robot1_state, initial_robot2_state, final_load_goal, robot_height1, robot_height2, desired_robot_heights, tune_d, tune_b);
    }

    static std::unique_ptr<FactorExecutor> create(
        string debug_mode, 
        const Vector4& initial_load_state, 
        const Vector6& initial_robot_state, 
        const Vector4& final_load_goal, 
        double robot_height,
        const Vector3& last_u,
        const Vector1& last_tension,
        const map<string, double>& tune_d,
        const map<string, bool>& tune_b
    ) {
        return std::make_unique<FactorExecutorSingleRobotWithHeight>(debug_mode, initial_load_state, initial_robot_state, final_load_goal, robot_height, last_u, last_tension, tune_d, tune_b);
    }

    static std::unique_ptr<FactorExecutor> create(
        string debug_mode, 
        int robot_num,
        const Vector6& initial_load_state, 
        const std::vector<Vector4>& initial_robot_states, 
        const Vector6& final_load_goal, 
        const std::vector<double>& robot_heights,
        const std::vector<double>& desired_robot_heights,
        const map<string, double>& tune_d,
        const map<string, bool>& tune_b
    ) {
        return std::make_unique<FactorExecutorMultiRobotsWithOrientation>(debug_mode, robot_num, initial_load_state, initial_robot_states, final_load_goal, robot_heights, desired_robot_heights, tune_d, tune_b);
    }

    static std::unique_ptr<FactorExecutor> create(
        string debug_mode, 
        int robot_num,
        const Vector4& initial_load_state, 
        const std::vector<Vector6>& initial_robot_states, 
        const Vector4& final_load_goal, 
        const map<string, double>& tune_d,
        const map<string, bool>& tune_b
    ) {
        return std::make_unique<FactorExecutorMultiRobotsWithHeight>(debug_mode, robot_num, initial_load_state, initial_robot_states, final_load_goal, tune_d, tune_b);
    }

    static std::unique_ptr<FactorExecutor> create(
        string debug_mode, 
        int robot_num,
        const Vector6& initial_load_state, 
        const std::vector<Vector6>& initial_robot_states, 
        const Vector6& final_load_goal, 
        const map<string, double>& tune_d,
        const map<string, bool>& tune_b
    ) {
        return std::make_unique<FactorExecutorMultiRobotsWithHeightAndOri>(debug_mode, robot_num, initial_load_state, initial_robot_states, final_load_goal, tune_d, tune_b);
    }
    
};

#endif // FACTOR_EXECUTOR_FACTORY_HPP