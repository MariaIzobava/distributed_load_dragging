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
        true, 
        "/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/one_drone_with_height_and_ori_points.json",
        "gtsam_cpp_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "MPC for multiple robots with height and orientation with GTSAM node has started.");

        this->declare_parameter<std::string>("robot_prefix", "/crazyflie");
        this->declare_parameter<int>("robot_num", 4);

        std::string robot_prefix_ = this->get_parameter("robot_prefix").as_string();
        robot_num_ = this->get_parameter("robot_num").as_int();
        init_robot_num(robot_num_);

        cout << "Registered number of robots: " << robot_num_ << endl;

        position_.resize(robot_num_);
        angles_.resize(robot_num_);
        rot_.resize(robot_num_);

        for (int i = 0; i < robot_num_; i++) {
            position_[i].resize(6);
            angles_[i].resize(6);
        }
        load_position_.resize(6);
        load_angles_.resize(6);

        is_pulling_ = 0;
        desired_height_ = 0.5;

        for (int i = 0; i < robot_num_; i++) {
            string suffix = "_0" + to_string(i+1) + "/odom";
            odom_subscription_.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
            robot_prefix_ + suffix,
            10, // QoS history depth
            [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
                // Inside the lambda, call the member function with both arguments
                this->robot_odom_subscribe_callback(msg, i);
            }));
        }

        load_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "load/odom",
          10, // QoS history depth
          std::bind(&GtsamCppTestNode::load_odom_subscribe_callback, this, std::placeholders::_1));

        land_ = this->create_subscription<std_msgs::msg::Bool>(
          "land",
          10, // QoS history depth
          std::bind(&GtsamCppTestNode::land_subscribe_callback, this, std::placeholders::_1));

        for (int i = 0; i < robot_num_; i++) {
            string topic_name = "/cmd_vel_0" + to_string(i+1);
            twist_publisher_.push_back(this->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10));
        }
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 0.1 seconds = 100 milliseconds
        std::bind(&GtsamCppTestNode::timer_callback, this));
    }

private:
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> twist_publisher_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr load_odom_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::vector<double> > position_;
    std::vector<std::vector<double> > angles_;
    std::vector<double> load_position_;
    std::vector<double> load_angles_;
    std::vector<Eigen::Matrix3d> rot_;
    Eigen::Matrix3d rotl_;

    int is_pulling_, robot_num_;
    double desired_height_;
    double pos_error_;

    void land_subscribe_callback(const std_msgs::msg::Bool msg)
    {
        cout << "Factor Graph MPC: LANDING" << std::endl;
        timer_->cancel();
        geometry_msgs::msg::Twist new_cmd_msg;
        new_cmd_msg.linear.z = -0.2;
            
        for (int i = 0; i < robot_num_; i++) {
            twist_publisher_[i]->publish(new_cmd_msg);
        }
        return; 
    }

    void robot_odom_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg, int k)
    {
        odom_(msg, position_[k], angles_[k], rot_[k]);
    }

    void load_odom_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_(msg, load_position_, load_angles_, rotl_);
    }
    
    void timer_callback()
    {

        // If not flying --> takeoff
        if (is_pulling_ != (1 << robot_num_) - 1) {
            for (int i = 0; i < robot_num_; i++) {
                geometry_msgs::msg::Twist new_cmd_msg;
                new_cmd_msg.linear.z = 0.1;

                if (position_[i][2] > desired_height_) {
                    new_cmd_msg.linear.z = 0.0;
                    is_pulling_ |= (1 << i);
                }

                twist_publisher_[i]->publish(new_cmd_msg);
            }
            if (is_pulling_ == (1 << robot_num_) - 1) {
                start_time_ = std::chrono::high_resolution_clock::now();
                RCLCPP_INFO(this->get_logger(), "Takeoff completed");
            }
            return;
        }

        if (path_ == NULL) {
            return;
        }

        // Now the drone is in "pulling" mode, control with MPC
        auto next_velocity = get_next_velocity_();

        std::vector<double> tensions;
        std::vector<int> ap_directions;
        std::vector<std::vector<double> > controls;
        controls.resize(robot_num_);
        for (int i = 0; i < robot_num_; i++) {
            tensions.push_back(next_velocity[robot_num_ * 3 + i]);
            ap_directions.push_back(1);

            controls[i].push_back(next_velocity[robot_num_ * 4 + 3 * i]);
            controls[i].push_back(next_velocity[robot_num_ * 4 + 3 * i + 1]);
            controls[i].push_back(next_velocity[robot_num_ * 4 + 3 * i + 2]);
        }
        std::vector<double> load_pos = {load_position_[0], load_position_[1], load_angles_[2], 
            load_position_[3], load_position_[4], load_angles_[3]};
        record_metrics(load_pos, position_, tensions, ap_directions, controls, pos_error_);

        for (int i = 0; i < robot_num_; i++) {
            geometry_msgs::msg::Twist new_cmd_msg;

            convert_robot_velocity_to_local_frame(
                next_velocity[3 * i], next_velocity[3 * i + 1], next_velocity[3 * i + 2], 
                rot_[i], new_cmd_msg, 0.2
            );

            cout << "Next velocity (world) drone " << i+1 << ": " << next_velocity[3*i] << ' ' << next_velocity[3*i+1] << ' ' << next_velocity[3 * i + 2]  << endl;
            cout << "Next velocity (local) drone " << i+1 << ": " <<  new_cmd_msg.linear.x << ' ' << new_cmd_msg.linear.y << ' ' << new_cmd_msg.linear.z << endl;
            twist_publisher_[i]->publish(new_cmd_msg);
        }
    }

    std::vector<double> get_next_velocity_() {
        std::vector<Vector6> initial_robot_states_;
        for (int i = 0; i < robot_num_; i++) {
            Vector6 state(
                position_[i][0], position_[i][1], position_[i][2],
                position_[i][3], position_[i][4], position_[i][5]
            );
            initial_robot_states_.push_back(state);
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
        //         {"init_robot1", initial_robot_states_[0]},
        //         {"goal_load", final_load_goal},
        //     }
        // );

        cout << "\n===================" << std::endl;
        cout << "Cur load position: " << initial_load_state[0] <<  ", " << initial_load_state[1] << ", " << initial_load_state[2] <<  ", " << initial_load_state[3] << ", " << initial_load_state[4] <<  ", " << initial_load_state[5] << std::endl;
        for (int i = 0; i < robot_num_; i++) {
            cout << "Cur robot " << i + 1 << " position: " << initial_robot_states_[i][0] <<  ", " << initial_robot_states_[i][1] << ", " << initial_robot_states_[i][2] <<  ", " << initial_robot_states_[i][3] << ", " << initial_robot_states_[i][4] <<  ", " << initial_robot_states_[i][5] <<  std::endl;
        }
        cout << "Next load position: " << final_load_goal[0] <<  ", " << final_load_goal[1]<<  ", " << final_load_goal[2] << std::endl;

        auto executor = FactorExecutorFactory::create("sim", robot_num_, initial_load_state, initial_robot_states_, final_load_goal, {}, {});
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
