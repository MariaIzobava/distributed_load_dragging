#include "rclcpp/rclcpp.hpp"

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <Eigen/Geometry>

#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// My classes
#include "crazyflie_ros2_controller_cpp/base_mpc.hpp"
#include "factor_graph_lib/factor_executor_factory.hpp"

#include <vector>
#include <string> 
#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>


using namespace std;
using namespace gtsam;
using symbol_t = gtsam::Symbol;
using json = nlohmann::json;


class GtsamCppTestNode : public BaseMpc
{
public:
    GtsamCppTestNode() : 
    BaseMpc(
        "/home/maryia/legacy/experiments/metrics/", 
        true, 
        false, 
        "/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/one_drone_with_ori_points.json",
        "gtsam_cpp_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "MPC with load orientation with GTSAM node has started.");
    }

    FactorExecutorResult run_factor_executor() {

        Vector4 initial_robot_state(
            position_[0][0], position_[0][1],
            position_[0][3], position_[0][4]
        );
        Vector6 initial_load_state(
            load_position_[0], load_position_[1], load_angles_[2], 
            load_position_[3], load_position_[4], load_angles_[3]
        );

        auto next_p = get_next_points();
        Vector6 final_load_goal(
            next_p[0], next_p[1], next_p[2],
            0.0,        0.0,        0.0 // doesn't matter
        );

        // record_datapoint(
        //     {
        //         {"init_load", initial_load_state},
        //         {"init_robot", initial_robot_state},
        //         {"goal_load", final_load_goal},
        //         {"height" , position_[2]},
        //     }
        // );

        log_cur_state(
            position_, 
            load_position_, 
            {load_angles_[2], load_angles_[3]}, 
            {next_p[0], next_p[1], next_p[2]}
        );

        auto executor = FactorExecutorFactory::create(
            "sim", 
            initial_load_state, 
            initial_robot_state, 
            final_load_goal, 
            position_[0][2], 
            desired_heights_[0], {}, {});
        map<string, double> factor_errors = {};
        return executor->run(factor_errors, pos_error_);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GtsamCppTestNode>());
    rclcpp::shutdown();
    return 0;
}
