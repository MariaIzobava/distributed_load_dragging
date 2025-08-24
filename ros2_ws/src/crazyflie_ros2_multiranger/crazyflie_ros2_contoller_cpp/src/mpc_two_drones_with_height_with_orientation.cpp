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
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// My Factor Graph classes
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


class GtsamCppTestNode : public rclcpp::Node
{
public:
    GtsamCppTestNode() : Node("gtsam_cpp_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "MPC with GTSAM node has started.");

        this->declare_parameter<std::string>("robot_prefix", "/crazyflie");

        std::string robot_prefix_ = this->get_parameter("robot_prefix").as_string();

        position1_.resize(6);
        angles1_.resize(3);
        position2_.resize(6);
        angles2_.resize(3);
        load_position_.resize(6);
        load_angles_.resize(3);

        last_u1_ << 0.0, 0.0;
        last_u2_ << 0.0, 0.0;

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

        twist_publisher11_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_stamped_vel_01", 10);
        twist_publisher22_= this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_stamped_vel_02", 10);
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 0.1 seconds = 100 milliseconds
        std::bind(&GtsamCppTestNode::timer_callback, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher1_, twist_publisher2_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher11_, twist_publisher22_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription1_, odom_subscription2_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr load_odom_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_;

    rclcpp::TimerBase::SharedPtr timer_;
    int kk_ = 0;

    std::vector<double> position1_, position2_;
    std::vector<double> angles1_, angles2_;
    std::vector<double> load_position_;
    std::vector<double> load_angles_;
    Eigen::Matrix3d rot1_, rot2_, rotl_;
    nav_msgs::msg::Path::SharedPtr path_;

    Vector2 last_u1_;
    Vector2 last_u2_;

    int is_pulling_;
    double desired_height_;
    std::chrono::high_resolution_clock::time_point start_time_;

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

    // Callback functions
    void odom1_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        position1_[0] = msg->pose.pose.position.x;
        position1_[1] = msg->pose.pose.position.y;
        position1_[2] = msg->pose.pose.position.z;

        // Quaternion to Euler conversion
        tf2::Quaternion q(
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z,
          msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // getRPY returns roll, pitch, yaw in radians

        angles1_[0] = roll;
        angles1_[1] = pitch;
        angles1_[2] = yaw;

        // Transforming velocity from local frame to world frame
        rot1_ = get_rot_from_euler(roll, pitch, yaw);
        Eigen::Vector3d v_local;
        v_local << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z; 

        Eigen::Vector3d v_world = rot1_ * v_local;
        position1_[3] = v_world(0);
        position1_[4] = v_world(1);
        position1_[5] = v_world(2);
    }

    void odom2_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        position2_[0] = msg->pose.pose.position.x;
        position2_[1] = msg->pose.pose.position.y;
        position2_[2] = msg->pose.pose.position.z;

        // Quaternion to Euler conversion
        tf2::Quaternion q(
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z,
          msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // getRPY returns roll, pitch, yaw in radians

        angles2_[0] = roll;
        angles2_[1] = pitch;
        angles2_[2] = yaw;

        // Transforming velocity from local frame to world frame
        rot2_ = get_rot_from_euler(roll, pitch, yaw);
        Eigen::Vector3d v_local;
        v_local << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z; 

        Eigen::Vector3d v_world = rot2_ * v_local;
        position2_[3] = v_world(0);
        position2_[4] = v_world(1);
        position2_[5] = v_world(2);
    }

    void load_odom_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        load_position_[0] = msg->pose.pose.position.x;
        load_position_[1] = msg->pose.pose.position.y;
        load_position_[2] = msg->pose.pose.position.z;

        // Quaternion to Euler conversion
        tf2::Quaternion q(
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z,
          msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // getRPY returns roll, pitch, yaw in radians

        load_angles_[0] = roll;
        load_angles_[1] = pitch;
        load_angles_[2] = yaw;

        // Transforming velocity from local frame to world frame
        rotl_ = get_rot_from_euler(roll, pitch, yaw);
        Eigen::Vector3d v_local;
        v_local << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z; 

        Eigen::Vector3d v_world = rotl_ * v_local;
        load_position_[3] = v_world(0);
        load_position_[4] = v_world(1);
        load_position_[5] = v_world(2);
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
            new_cmd_msg1.linear.z = 0.03;
            new_cmd_msg2.linear.z = 0.03;
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

        auto next_velocity = get_next_velocity_();
        last_u1_<< next_velocity[4], next_velocity[5];
        last_u2_<< next_velocity[6], next_velocity[7];

        cout << "Next velocity (world) drone 1: " << next_velocity[0] << ' ' << next_velocity[1] << endl;
        cout << "Next velocity (world) drone 2: " << next_velocity[2] << ' ' << next_velocity[3] << endl;
        cout << "Orientation 1: " << angles1_[0] << ' ' << angles1_[1] << ' ' << angles1_[2] << endl; 
        cout << "Orientation 2: " << angles2_[0] << ' ' << angles2_[1] << ' ' << angles2_[2] << endl;  
        double z1 = (desired_height_ - position1_[2] > 0.01) ? 0.01 : (desired_height_ - position1_[2] < -0.01) ? -0.01 : 0;
        double z2 = (desired_height_ - position2_[2] > 0.01) ? 0.01 : (desired_height_ - position2_[2] < -0.01) ? -0.01 : 0;

        Eigen::Vector3d v_local1, v_local2;
        v_local1 << next_velocity[0], next_velocity[1], desired_height_ - position1_[2]; 
        v_local2 << next_velocity[2], next_velocity[3], desired_height_ - position2_[2]; 

        Eigen::Vector3d v_world1 = rot1_.transpose() * v_local1;
        Eigen::Vector3d v_world2 = rot2_.transpose() * v_local2;

        new_cmd_msg1.linear.x = max(min(v_world1(0), 0.2), -0.2);
        new_cmd_msg1.linear.y = max(min(v_world1(1), 0.2), -0.2);
        new_cmd_msg1.linear.z = max(min(v_world1(2), 0.2), -0.2);

        new_cmd_msg2.linear.x = max(min(v_world2(0), 0.2), -0.2);
        new_cmd_msg2.linear.y = max(min(v_world2(1), 0.2), -0.2);
        new_cmd_msg2.linear.z = max(min(v_world2(2), 0.2), -0.2);

        cout << "Next velocity (local) drone 1: " << new_cmd_msg1.linear.x << ' ' << new_cmd_msg1.linear.y << endl;
        cout << "Next velocity (local) drone 2: " << new_cmd_msg2.linear.x << ' ' << new_cmd_msg2.linear.y << endl;

        auto twist_msg = geometry_msgs::msg::TwistStamped();;

        // Set the header
        // This is the most important part for RViz visualization.
        twist_msg.header.stamp = this->get_clock()->now();
        
        // Use the global frame, like "map" or "odom", as the frame_id.
        // Make sure this matches the fixed frame in your RViz configuration.
        twist_msg.header.frame_id = "odom"; 

        // Set the linear velocities
        twist_msg.twist.linear.x = new_cmd_msg1.linear.x;
        twist_msg.twist.linear.y = new_cmd_msg1.linear.y;
        twist_msg.twist.linear.z = new_cmd_msg1.linear.z;

        // Set the angular velocities
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;

        // Publish the message
        twist_publisher11_->publish(twist_msg);

        auto twist_msg2 = geometry_msgs::msg::TwistStamped();;

        // Set the header
        // This is the most important part for RViz visualization.
        twist_msg2.header.stamp = this->get_clock()->now();
        
        // Use the global frame, like "map" or "odom", as the frame_id.
        // Make sure this matches the fixed frame in your RViz configuration.
        twist_msg2.header.frame_id = "odom"; 

        // Set the linear velocities
        twist_msg2.twist.linear.x = new_cmd_msg2.linear.x;
        twist_msg2.twist.linear.y = new_cmd_msg2.linear.y;
        twist_msg2.twist.linear.z = new_cmd_msg2.linear.z;

        // Set the angular velocities
        twist_msg2.twist.angular.x = 0.0;
        twist_msg2.twist.angular.y = 0.0;
        twist_msg2.twist.angular.z = 0.0;

        // Publish the message
        twist_publisher22_->publish(twist_msg2);

        
        twist_publisher1_->publish(new_cmd_msg1);
        twist_publisher2_->publish(new_cmd_msg2);
    }

    std::vector<double> get_next_velocity_() {
        Vector6 initial_robot1_state(
            position1_[0], position1_[1], position1_[2],
            position1_[3], position1_[4], position1_[5]
        );
        Vector6 initial_robot2_state(
            position2_[0], position2_[1], position2_[2],
            position2_[3], position2_[4], position2_[5]
        );
        Vector4 initial_load_state(
            load_position_[0], load_position_[1],
            load_position_[3], load_position_[4]
        );

        auto next_p = get_next_points();
        Vector4 final_load_goal(
            next_p[0], next_p[1],
            0,          0 // doesn't matter
        );

        // kk_++;
        // if (kk_ % 10 == 0) {
        //     string filename = "/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/two_drones_no_ori_points.json";
        //     std::ifstream file(filename);
        //     if (!file.is_open()) {
        //         std::cerr << "Error: Could not open the JSON file!" << std::endl;
        //     }

        //     json jsonData;
        //     try {
        //         file >> jsonData;
        //     } catch (json::parse_error& e) {
        //         std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        //     }
        //     file.close();

        //     jsonData.push_back({
        //         {"init_load", initial_load_state},
        //         {"init_robot1", initial_robot1_state},
        //         {"init_robot2", initial_robot2_state},
        //         {"goal_load", final_load_goal},
        //         {"height1" , position1_[2]},
        //         {"height2" , position2_[2]},
        //     });

        //     std::ofstream output_file(filename);
        //     if (output_file.is_open()) {
        //         output_file << jsonData.dump(4);
                
        //         std::cout << "JSON object successfully written to " << filename << std::endl;
        //     } else {
        //         std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        //     }
        // }

        cout << "Cur load position: " << initial_load_state[0] <<  ", " << initial_load_state[1] << ", " << initial_load_state[2] <<  ", " << initial_load_state[3] << std::endl;
        cout << "Cur robot 1 position: " << initial_robot1_state[0] <<  ", " << initial_robot1_state[1] << ", " << initial_robot1_state[2] <<  ", " << initial_robot1_state[3] <<  std::endl;
        cout << "Cur robot 2 position: " << initial_robot2_state[0] <<  ", " << initial_robot2_state[1] << ", " << initial_robot2_state[2] <<  ", " << initial_robot2_state[3] <<  std::endl;       
        cout << "Robots height: " << position1_[2] << ' ' << position2_[2] << std::endl;
        cout << "Next load position: " << final_load_goal[0] <<  ", " << final_load_goal[1] << std::endl;

        auto executor = FactorExecutorFactory::create("sim", initial_load_state, initial_robot1_state, initial_robot2_state, final_load_goal, position1_[2], position2_[2], last_u1_, last_u2_, {}, {});
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

        int i = (cur_millisec.count() / 100 + 60) % num_p;
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
