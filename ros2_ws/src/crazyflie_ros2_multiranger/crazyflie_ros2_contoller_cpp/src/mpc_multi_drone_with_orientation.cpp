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
        "/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/four_drones_with_ori_points.json",
        "gtsam_cpp_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "MPC for multiple robots with orientation with GTSAM node has started.");

        last_robot_rm_ = this->create_subscription<std_msgs::msg::Bool>(
          "last_robot_rm",
          10, // QoS history depth
          std::bind(&GtsamCppTestNode::last_robot_rm_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_, last_robot_rm_;
    int last_drone_removed_ = 0;

    void last_robot_rm_callback(const std_msgs::msg::Bool msg) {
        if (last_drone_removed_) {
            cout << "Last drone was already removed; Ignoring the command!\n";
            return;
        }

        cout << "Removing Crazyflie_0" << robot_num_ + 1 << endl;
        last_drone_removed_ = 1;
    }

    FactorExecutorResult run_factor_executor() {
        std::vector<Vector4> initial_robot_states_;
        std::vector<double> heights;
        for (int i = 0; i < robot_num_ - last_drone_removed_; i++) {
            Vector4 state(
                position_[i][0], position_[i][1],
                position_[i][3], position_[i][4]
            );
            initial_robot_states_.push_back(state);
            heights.push_back(position_[i][2]);
        }
        Vector6 initial_load_state(
            load_position_[0], load_position_[1], load_angles_[2], 
            load_position_[3], load_position_[4], load_angles_[3]
        );

        auto next_p = get_next_points();
        Vector6 final_load_goal(
            next_p[0], next_p[1], next_p[2],
            0,          0,         0 // doesn't matter
        );

        // record_datapoint(
        //     {
        //         {"init_load", initial_load_state},
        //         {"init_robot1", initial_robot1_state},
        //         {"init_robot2", initial_robot2_state},
        //         {"goal_load", final_load_goal},
        //         {"height1" , position1_[2]},
        //         {"height2" , position2_[2]},
        //     }
        // );

        log_cur_state(position_, load_position_, {load_angles_[2], load_angles_[3]}, {next_p[0], next_p[1], next_p[2]});

        auto executor = FactorExecutorFactory::create("sim", robot_num_ - last_drone_removed_, initial_load_state, initial_robot_states_, final_load_goal, heights, desired_heights_, {}, {});
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
