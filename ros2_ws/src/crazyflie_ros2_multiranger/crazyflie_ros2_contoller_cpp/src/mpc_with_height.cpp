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
#include "geometry_msgs/msg/quaternion.hpp"
#include <Eigen/Geometry>
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
        "/home/maryia/legacy/experiments/metrics/one_drone_with_height.csv", 
        1, 
        false, 
        true, 
        "/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/one_drone_with_height_points.json",
        "gtsam_cpp_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "MPC with height with GTSAM node has started.");

        this->declare_parameter<std::string>("robot_prefix", "/crazyflie");

        std::string robot_prefix_ = this->get_parameter("robot_prefix").as_string();

        position_.resize(6);
        angles_.resize(3);
        load_position_.resize(6);
        load_angles_.resize(3);
        last_control_ << 0.0, 0.0, 0.0;
        last_tension_ << 0;

        is_pulling_ = false;
        desired_height_ = 0.5;

        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
          "/desired_path",
          10, // QoS history depth
          std::bind(&GtsamCppTestNode::path_subscribe_callback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          robot_prefix_ + "/odom",
          10, // QoS history depth
          std::bind(&GtsamCppTestNode::odom_subscribe_callback, this, std::placeholders::_1));

        load_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "load/odom",
          10, // QoS history depth
          std::bind(&GtsamCppTestNode::load_odom_subscribe_callback, this, std::placeholders::_1));

        land_ = this->create_subscription<std_msgs::msg::Bool>(
          "land",
          10, // QoS history depth
          std::bind(&GtsamCppTestNode::land_subscribe_callback, this, std::placeholders::_1));

        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_01", 10);
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 0.1 seconds = 100 milliseconds
        std::bind(&GtsamCppTestNode::timer_callback, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr load_odom_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> position_;
    std::vector<double> angles_;
    std::vector<double> load_position_;
    std::vector<double> load_angles_;
    Eigen::Matrix3d rot_l_, rot_r_;
    nav_msgs::msg::Path::SharedPtr path_;

    Vector3 last_control_;
    Vector1 last_tension_;

    bool is_pulling_;
    double desired_height_;
    std::chrono::high_resolution_clock::time_point start_time_;

    void land_subscribe_callback(const std_msgs::msg::Bool msg)
    {
        cout << "Factor Graph MPC: LANDING" << std::endl;
        timer_->cancel();
        geometry_msgs::msg::Twist new_cmd_msg;
        new_cmd_msg.linear.z = -0.2;
            
        twist_publisher_->publish(new_cmd_msg);
        return; 
    }

    // This function takes roll, pitch, and yaw (rx, ry, rz)
    // and returns a combined 3x3 rotation matrix.
    // The order of multiplication is Z-Y-X.
    Eigen::Matrix3d get_rot_from_euler(double rx, double ry, double rz) {
        // Eigen's AngleAxisd class represents a rotation by an angle around an axis.
        // The order of multiplication matters: yaw, then pitch, then roll.
        // The multiplication is Z * Y * X, which is a common convention for intrinsic rotations.
        Eigen::AngleAxisd rollAngle(rx, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(ry, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(rz, Eigen::Vector3d::UnitZ());

        // Multiplying the AngleAxis objects gives a combined rotation.
        // This product is internally stored as a quaternion, which is efficient and avoids Gimbal Lock.
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        
        // Convert the quaternion to a 3x3 rotation matrix
        return q.toRotationMatrix();
    }

    void odom_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_(msg, position_, angles_, rot_r_);
    }

    void load_odom_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_(msg, load_position_, load_angles_, rot_l_);
    }

    void path_subscribe_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_ = msg;
    }
    
    void timer_callback()
    {
        geometry_msgs::msg::Twist new_cmd_msg;

        // If not flying --> takeoff
        if (!is_pulling_) {
            new_cmd_msg.linear.z = 0.1;
            if (position_[2] > desired_height_) {
                new_cmd_msg.linear.z = 0.0;
                is_pulling_ = true;
                start_time_ = std::chrono::high_resolution_clock::now();
                RCLCPP_INFO(this->get_logger(), "Takeoff completed");
            }
            twist_publisher_->publish(new_cmd_msg);
            return;
        }

        if (path_ == NULL) {
            return;
        }

        // Now the drone is in "pulling" mode, control with MPC

        auto next_velocity = get_next_velocity_();
        last_control_ << next_velocity[3], next_velocity[4], next_velocity[5];
        last_tension_ << next_velocity[6];

        Eigen::Vector3d v_local;
        v_local << next_velocity[0], next_velocity[1], next_velocity[2]; 

        Eigen::Vector3d v_world = rot_r_.transpose() * v_local;

        new_cmd_msg.linear.x = max(min(v_world(0), 0.2), -0.2); 
        new_cmd_msg.linear.y = max(min(v_world(1), 0.2), -0.2);
        new_cmd_msg.linear.z = max(min(v_world(2), 0.2), -0.2);
        
        cout << "Next velocity: " << new_cmd_msg.linear.x << ' ' << new_cmd_msg.linear.y << ' ' << new_cmd_msg.linear.z << endl;
        cout << "Next control: " << last_control_[0] << ' ' << last_control_[1]<< ' ' << last_control_[2] << endl;
        cout << "Next tension: " << last_tension_[0] << endl;
        
        twist_publisher_->publish(new_cmd_msg);
    }

    std::vector<double> get_next_velocity_() {

        Vector6 initial_robot_state(
            position_[0], position_[1], position_[2],
            position_[3], position_[4], position_[5]
        );
        Vector4 initial_load_state(
            load_position_[0], load_position_[1],
            load_position_[3], load_position_[4]
        );

        auto next_p = get_next_points();
        Vector4 final_load_goal(
            next_p[0], next_p[1],
            0.0,       0.0  // doesn't matter
        );

        cout << "Cur load position: " << initial_load_state[0] <<  ", " << initial_load_state[1] << ", " << initial_load_state[2] <<  ", " << initial_load_state[3] << std::endl;
        cout << "Cur robot position: " << initial_robot_state[0] <<  ", " << initial_robot_state[1] << ", " << initial_robot_state[2] <<  ", " << initial_robot_state[3] <<  ", " << initial_robot_state[4] <<  ", " << initial_robot_state[5] <<  std::endl;
        cout << "Cur robot height: " << position_[2] << std::endl;
        cout << "Next load position: " << final_load_goal[0] <<  ", " << final_load_goal[1] << std::endl;

        auto executor = FactorExecutorFactory::create("sim", initial_load_state, initial_robot_state, final_load_goal, position_[2], last_control_, last_tension_, {}, {});
        map<string, double> factor_errors = {};
        double pos_error = 0.0;
        return executor->run(factor_errors, pos_error);
    }

    std::vector<double> get_next_points() {
        int num_p = 4000; // num_p is number of points
        
        // Get current time
        auto cur_time = std::chrono::high_resolution_clock::now();
        
        // Calculate seconds from the beginning
        auto cur_millisec = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - start_time_);

        int i = (cur_millisec.count() / 100 + 10) % num_p;
        cout << i << ' ' << path_->poses.size() << std::endl;
        std::vector<double> ans;
        ans.push_back(path_->poses[i].pose.position.x);
        ans.push_back(path_->poses[i].pose.position.y);
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
