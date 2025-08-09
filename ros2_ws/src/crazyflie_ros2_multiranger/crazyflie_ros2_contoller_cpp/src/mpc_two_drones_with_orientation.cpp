#include "rclcpp/rclcpp.hpp"

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp" // For the Quaternion message type

// For quaternion to Euler conversion
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// My Factor Graph classes
#include "factor_graph_lib/factor_executor.hpp"

#include <vector>
#include <string> 
#include <chrono>

// Use gtsam namespace
using namespace std;
using namespace gtsam;

// Define symbols for different variable types for clarity
using symbol_t = gtsam::Symbol;


class GtsamCppTestNode : public rclcpp::Node
{
public:
    GtsamCppTestNode() : Node("gtsam_cpp_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "MPC with GTSAM node has started.");

        this->declare_parameter<std::string>("robot_prefix", "/crazyflie");

        std::string robot_prefix_ = this->get_parameter("robot_prefix").as_string();

        position1_.resize(6);
        angles1_.resize(6);
        position2_.resize(6);
        angles2_.resize(6);
        load_position_.resize(6);
        load_angles_.resize(6);

        is_pulling_ = 0;
        desired_height_ = 0.5;

        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
          "/desired_path",
          10, // QoS history depth
          std::bind(&GtsamCppTestNode::path_subscribe_callback, this, std::placeholders::_1));

        odom_subscription1_ = this->create_subscription<nav_msgs::msg::Odometry>(
          robot_prefix_ + "_01/odom",
          10, // QoS history depth
          std::bind(&GtsamCppTestNode::odom1_subscribe_callback, this, std::placeholders::_1));

        odom_subscription2_ = this->create_subscription<nav_msgs::msg::Odometry>(
          robot_prefix_ + "_02/odom",
          10, // QoS history depth
          std::bind(&GtsamCppTestNode::odom2_subscribe_callback, this, std::placeholders::_1));

        load_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "load/odom",
          10, // QoS history depth
          std::bind(&GtsamCppTestNode::load_odom_subscribe_callback, this, std::placeholders::_1));

        land_ = this->create_subscription<std_msgs::msg::Bool>(
          "land",
          10, // QoS history depth
          std::bind(&GtsamCppTestNode::land_subscribe_callback, this, std::placeholders::_1));

        twist_publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_01", 10);
        twist_publisher2_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_02", 10);
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 0.1 seconds = 100 milliseconds
        std::bind(&GtsamCppTestNode::timer_callback, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher1_, twist_publisher2_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription1_, odom_subscription2_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr load_odom_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> position1_, position2_;
    std::vector<double> angles1_, angles2_;
    std::vector<double> load_position_;
    std::vector<double> load_angles_;
    nav_msgs::msg::Path::SharedPtr path_;

    int is_pulling_;
    double desired_height_;
    std::chrono::high_resolution_clock::time_point start_time_;

    void land_subscribe_callback(const std_msgs::msg::Bool msg)
    {
        cout << "Factor Graph MPC: LANDING" << std::endl;
        timer_->cancel();
        geometry_msgs::msg::Twist new_cmd_msg;
        new_cmd_msg.linear.z = -0.2;
            
        twist_publisher1_->publish(new_cmd_msg);
        twist_publisher2_->publish(new_cmd_msg);
        return; 
    }

    void odom_(const nav_msgs::msg::Odometry::SharedPtr msg, std::vector<double>& position, std::vector<double>& angles) {
        position[0] = msg->pose.pose.position.x;
        position[1] = msg->pose.pose.position.y;
        position[2] = msg->pose.pose.position.z;
        position[3] = msg->twist.twist.linear.x;
        position[4] = msg->twist.twist.linear.y;
        position[5] = msg->twist.twist.linear.z;

        // Quaternion to Euler conversion
        tf2::Quaternion q(
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z,
          msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // getRPY returns roll, pitch, yaw in radians

        angles[0] = roll;
        angles[1] = pitch;
        angles[2] = atan2(sin(yaw), cos(yaw));
        angles[3] = msg->twist.twist.angular.z;  // only save Yaw velocity
    }

    // Callback functions
    void odom1_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_(msg, position1_, angles1_);
        // position1_[0] = msg->pose.pose.position.x;
        // position1_[1] = msg->pose.pose.position.y;
        // position1_[2] = msg->pose.pose.position.z;
        // position1_[3] = msg->twist.twist.linear.x;
        // position1_[4] = msg->twist.twist.linear.y;
        // position1_[5] = msg->twist.twist.linear.z;

        // // Quaternion to Euler conversion
        // tf2::Quaternion q(
        //   msg->pose.pose.orientation.x,
        //   msg->pose.pose.orientation.y,
        //   msg->pose.pose.orientation.z,
        //   msg->pose.pose.orientation.w);

        // tf2::Matrix3x3 m(q);
        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw); // getRPY returns roll, pitch, yaw in radians

        // angles1_[0] = roll;
        // angles1_[1] = pitch;
        // angles1_[2] = yaw;
    }

    void odom2_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_(msg, position2_, angles2_);
        // position2_[0] = msg->pose.pose.position.x;
        // position2_[1] = msg->pose.pose.position.y;
        // position2_[2] = msg->pose.pose.position.z;
        // position2_[3] = msg->twist.twist.linear.x;
        // position2_[4] = msg->twist.twist.linear.y;
        // position2_[5] = msg->twist.twist.linear.z;

        // // Quaternion to Euler conversion
        // tf2::Quaternion q(
        //   msg->pose.pose.orientation.x,
        //   msg->pose.pose.orientation.y,
        //   msg->pose.pose.orientation.z,
        //   msg->pose.pose.orientation.w);

        // tf2::Matrix3x3 m(q);
        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw); // getRPY returns roll, pitch, yaw in radians

        // angles2_[0] = roll;
        // angles2_[1] = pitch;
        // angles2_[2] = yaw;
    }

    void load_odom_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_(msg, load_position_, load_angles_);
        // load_position_[0] = msg->pose.pose.position.x;
        // load_position_[1] = msg->pose.pose.position.y;
        // load_position_[2] = msg->pose.pose.position.z;
        // load_position_[3] = msg->twist.twist.linear.x;
        // load_position_[4] = msg->twist.twist.linear.y;
        // load_position_[5] = msg->twist.twist.linear.z;

        // // Quaternion to Euler conversion
        // tf2::Quaternion q(
        //   msg->pose.pose.orientation.x,
        //   msg->pose.pose.orientation.y,
        //   msg->pose.pose.orientation.z,
        //   msg->pose.pose.orientation.w);

        // tf2::Matrix3x3 m(q);
        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw); // getRPY returns roll, pitch, yaw in radians

        // load_angles_[0] = roll;
        // load_angles_[1] = pitch;
        // load_angles_[2] = atan2(sin(yaw), cos(yaw));
        // load_angles_[3] = msg->twist.twist.angular.z;  // only save Yaw velocity
    }

    void path_subscribe_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_ = msg;
    }
    
    void timer_callback()
    {
        geometry_msgs::msg::Twist new_cmd_msg1, new_cmd_msg2;

        // If not flying --> takeoff
        if (is_pulling_ != 3) {
            new_cmd_msg1.linear.z = 0.1;
            new_cmd_msg2.linear.z = 0.1;
            if (position1_[2] > desired_height_) {
                new_cmd_msg1.linear.z = 0.0;
                is_pulling_ |= 1;   
            }
            if (position2_[2] > desired_height_) {
                new_cmd_msg2.linear.z = 0.0;
                is_pulling_ |= 2;   
            }
            twist_publisher1_->publish(new_cmd_msg1);
            twist_publisher2_->publish(new_cmd_msg2);
            if (is_pulling_ == 3) {
                start_time_ = std::chrono::high_resolution_clock::now();
                RCLCPP_INFO(this->get_logger(), "Takeoff completed");
            }
            return;
        }

        if (path_ == NULL) {
            return;
        }

        // Now the drone is in "pulling" mode, control with MPC

        // First, we're keeping the drones on the same height
        new_cmd_msg1.linear.z = desired_height_ - position1_[2];
        new_cmd_msg2.linear.z = desired_height_ - position2_[2];

        auto next_velocity = get_next_velocity_();

        // Limiting velocities for safety
        new_cmd_msg1.linear.x = next_velocity[0]; //max(min(next_velocity[0], 0.1), -0.1);
        new_cmd_msg1.linear.y = next_velocity[1]; //max(min(next_velocity[1], 0.1), -0.1);

        new_cmd_msg2.linear.x = next_velocity[2]; //max(min(next_velocity[2], 0.1), -0.1);
        new_cmd_msg2.linear.y = next_velocity[3]; // max(min(next_velocity[3], 0.1), -0.1);
        
        cout << "Next velocity drone 1: " << new_cmd_msg1.linear.x << ' ' << new_cmd_msg1.linear.y << endl;
        cout << "Next velocity drone 2: " << new_cmd_msg2.linear.x << ' ' << new_cmd_msg2.linear.y << endl;
        
        twist_publisher1_->publish(new_cmd_msg1);
        twist_publisher2_->publish(new_cmd_msg2);
    }

    std::vector<double> get_next_velocity_() {
        Vector4 initial_robot1_state(
            position1_[0], position1_[1],
            position1_[3], position1_[4]
        );
        Vector4 initial_robot2_state(
            position2_[0], position2_[1],
            position2_[3], position2_[4]
        );
        Vector6 initial_load_state(
            load_position_[0], load_position_[1], load_angles_[2], 
            load_position_[3], load_position_[4], load_angles_[3]
        );

        auto next_p = get_next_points();
        Vector6 final_load_goal(
            next_p[0], next_p[1], next_p[2],
            0,          0,         0 // doesn't matter
        );


        cout << "\n===================" << std::endl;
        cout << "Cur load position: " << initial_load_state[0] <<  ", " << initial_load_state[1] << ", " << initial_load_state[2] <<  ", " << initial_load_state[3] << ", " << initial_load_state[4] <<  ", " << initial_load_state[5] << std::endl;
        cout << "Cur robot 1 position: " << initial_robot1_state[0] <<  ", " << initial_robot1_state[1] << ", " << initial_robot1_state[2] <<  ", " << initial_robot1_state[3] <<  std::endl;
        cout << "Cur robot 2 position: " << initial_robot2_state[0] <<  ", " << initial_robot2_state[1] << ", " << initial_robot2_state[2] <<  ", " << initial_robot2_state[3] <<  std::endl;       
        cout << "Robots height: " << position1_[2] << ' ' << position2_[2] << std::endl;
        cout << "Next load position: " << final_load_goal[0] <<  ", " << final_load_goal[1]<<  ", " << final_load_goal[2] << std::endl;

        auto executor = FactorExecutorFactory::create(0, initial_load_state, initial_robot1_state, initial_robot2_state, final_load_goal, position1_[2], position2_[2]);
        return executor->run();
    }

    std::vector<double> get_next_points() {
        int num_p = 4000; // num_p is number of points
        
        // Get current time
        auto cur_time = std::chrono::high_resolution_clock::now();
        
        // Calculate seconds from the beginning
        auto cur_millisec = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - start_time_);

        int i = (cur_millisec.count() / 100) % num_p;
        cout << i << ' ' << path_->poses.size() << std::endl;
        std::vector<double> ans;
        ans.push_back(path_->poses[i].pose.position.x);
        ans.push_back(path_->poses[i].pose.position.y);
        ans.push_back(atan2(sin(path_->poses[i].pose.orientation.x), cos(path_->poses[i].pose.orientation.x)) );
        return ans; 
    }


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GtsamCppTestNode>());
    rclcpp::shutdown();
    return 0;
}
