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
        angles1_.resize(3);
        position2_.resize(6);
        angles2_.resize(3);
        load_position_.resize(6);
        load_angles_.resize(3);

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

    // Callback functions
    void odom1_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        position1_[0] = msg->pose.pose.position.x;
        position1_[1] = msg->pose.pose.position.y;
        position1_[2] = msg->pose.pose.position.z;
        position1_[3] = msg->twist.twist.linear.x;
        position1_[4] = msg->twist.twist.linear.y;
        position1_[5] = msg->twist.twist.linear.z;

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
    }

    void odom2_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        position2_[0] = msg->pose.pose.position.x;
        position2_[1] = msg->pose.pose.position.y;
        position2_[2] = msg->pose.pose.position.z;
        position2_[3] = msg->twist.twist.linear.x;
        position2_[4] = msg->twist.twist.linear.y;
        position2_[5] = msg->twist.twist.linear.z;

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
    }

    void load_odom_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        load_position_[0] = msg->pose.pose.position.x;
        load_position_[1] = msg->pose.pose.position.y;
        load_position_[2] = msg->pose.pose.position.z;
        load_position_[3] = msg->twist.twist.linear.x;
        load_position_[4] = msg->twist.twist.linear.y;
        load_position_[5] = msg->twist.twist.linear.z;

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
        new_cmd_msg1.linear.x = max(min(next_velocity[0], 0.2), -0.2);
        new_cmd_msg1.linear.y = max(min(next_velocity[1], 0.2), -0.2);

        new_cmd_msg2.linear.x = max(min(next_velocity[2], 0.2), -0.2);
        new_cmd_msg2.linear.y = max(min(next_velocity[3], 0.2), -0.2);
        
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
        Vector4 initial_load_state(
            load_position_[0], load_position_[1],
            load_position_[3], load_position_[4]
        );

        auto next_p = get_next_points();
        Vector4 final_load_goal(
            next_p[0], next_p[1],
            0,          0 // doesn't matter
        );

        cout << "Cur load position: " << initial_load_state[0] <<  ", " << initial_load_state[1] << ", " << initial_load_state[2] <<  ", " << initial_load_state[3] << std::endl;
        cout << "Cur robot 1 position: " << initial_robot1_state[0] <<  ", " << initial_robot1_state[1] << ", " << initial_robot1_state[2] <<  ", " << initial_robot1_state[3] <<  std::endl;
        cout << "Cur robot 2 position: " << initial_robot2_state[0] <<  ", " << initial_robot2_state[1] << ", " << initial_robot2_state[2] <<  ", " << initial_robot2_state[3] <<  std::endl;       
        cout << "Robots height: " << position1_[2] << ' ' << position2_[2] << std::endl;
        cout << "Next load position: " << final_load_goal[0] <<  ", " << final_load_goal[1] << std::endl;

        auto executor = FactorExecutorFactory::create(0, initial_load_state, initial_robot1_state, initial_robot2_state, final_load_goal, position1_[2], position2_[2]);
        return executor->run();

        // NonlinearFactorGraph graph;

        // // --- Define problem parameters ---
        // const int num_time_steps = 20;
        // const double dt = 0.005;
        // const double robot_mass = 0.025; // kg
        // const double load_mass = 0.01;   // kg
        // const double gravity = 9.81;
        // const double mu = 0.3;
        // const double cable_length = 1.02; // 0.98; // 1.0; // 0.8; // 1.056; // meters
        // const double u_upper_bound = 0.9;
        // const double u_lower_bound = 0.003;
        // double weight_tension_lower_bound = 1000000.0; // High weight to strongly enforce T >= 0
        // double weight_cable_stretch = 100.0;     // Very high weight to strongly prevent cable from over-stretching
        // double weight_tension_slack = 50.0;
        // const double cable_length1 = 0.20 + sqrt(1.03 * 1.03 - (position1_[2]) * (position1_[2]));
        // const double cable_length2 = 0.20 + sqrt(1.03 * 1.03 - (position2_[2]) * (position2_[2]));


        // // --- Define Noise Models ---
        // // These represent the uncertainty of each factor (1/covariance)
        // auto dynamics_cost = noiseModel::Diagonal::Sigmas(
        //     (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
        // auto traj_cost = noiseModel::Diagonal::Sigmas(
        //     (Vector(4) << 0.07, 0.07, 1000.1, 1000.1).finished());
        // auto goal_cost = noiseModel::Diagonal::Sigmas(
        //     (Vector(4) << 0.00005, 0.00005, 1000.1, 1000.1).finished());
        // auto init_cost = noiseModel::Diagonal::Sigmas(
        //     (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
        // auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
        // auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-2);

        // // --- Add Factors to the Graph ---

        // // Add prior factors for the initial state
        // Vector4 initial_robot1_state(
        //     position1_[0],
        //     position1_[1],
        //     position1_[3],
        //     position1_[4]);
        // Vector4 initial_robot2_state(
        //     position2_[0],
        //     position2_[1],
        //     position2_[3],
        //     position2_[4]);
        // Vector4 initial_load_state(
        //     load_position_[0],
        //     load_position_[1],
        //     load_position_[3],
        //     load_position_[4]);
        // graph.add(PriorFactor<Vector4>(symbol_t('x', 0), initial_robot1_state, init_cost));
        // graph.add(PriorFactor<Vector4>(symbol_t('X', 0), initial_robot2_state, init_cost));
        // graph.add(PriorFactor<Vector4>(symbol_t('l', 0), initial_load_state, init_cost));

        // auto next_p = get_next_points();
        // // cout << "Cur load position: " << load_position_[0] <<  ' ' << load_position_[1] << std::endl;
        // // cout << "Cur robot 1 position: " << position1_[0] <<  ' ' << position1_[1] << std::endl;
        // Vector4 final_load_goal(
        //     next_p[0],
        //     next_p[1],
        //     load_position_[3], // doesn't matter
        //     load_position_[4]);  // doesn't matter
        // graph.add(PriorFactor<Vector4>(symbol_t('l', num_time_steps), final_load_goal, goal_cost));

        // double xdiff = (final_load_goal(0) - initial_load_state(0)) / num_time_steps;
        // double ydiff = (final_load_goal(1) - initial_load_state(1)) / num_time_steps;
        // Vector4 diff(xdiff, ydiff, 0, 0);

        // for (int k = 0; k < num_time_steps; ++k) {
        //     graph.add(RobotDynamicsFactor(
        //         symbol_t('x', k), 
        //         symbol_t('l', k),
        //         symbol_t('u', k),
        //         symbol_t('t', k),
        //         symbol_t('x', k+1),
        //         dt,
        //         robot_mass,
        //         dynamics_cost
        //         ));

        //     graph.add(RobotDynamicsFactor(
        //         symbol_t('X', k), 
        //         symbol_t('l', k),
        //         symbol_t('U', k),
        //         symbol_t('T', k),
        //         symbol_t('X', k+1),
        //         dt,
        //         robot_mass,
        //         dynamics_cost
        //         ));

        //     graph.add(LoadDynamicsTwoRobotsFactor(
        //         symbol_t('l', k),
        //         symbol_t('x', k),
        //         symbol_t('t', k),
        //         symbol_t('X', k),
        //         symbol_t('T', k),
        //         symbol_t('l', k+1),
        //         dt,
        //         load_mass,
        //         mu,
        //         gravity,
        //         dynamics_cost
        //         ));

        //     // Factor 1: Enforce T_k >= 0
        //     graph.add(TensionLowerBoundFactor(symbol_t('t', k), weight_tension_lower_bound, tension_cost));
        //     graph.add(TensionLowerBoundFactor(symbol_t('T', k), weight_tension_lower_bound, tension_cost));
            
        //     // Factor 2: Penalize if ||p_r - p_l|| > cable_length
        //     //graph.add(CableStretchPenaltyFactor(symbol_t('x', k), symbol_t('l', k), cable_length1, weight_cable_stretch, tension_cost));
        //     //graph.add(CableStretchPenaltyFactor(symbol_t('X', k), symbol_t('l', k), cable_length2, weight_cable_stretch, tension_cost));
            
        //     // Factor 3: Penalize if T_k > 0 AND ||p_r - p_l|| < cable_length (i.e., tension in slack cable)
        //     graph.add(TensionSlackPenaltyFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length1 - 0.007, weight_tension_slack, tension_cost));
        //     graph.add(TensionSlackPenaltyFactor(symbol_t('T', k), symbol_t('X', k), symbol_t('l', k), cable_length2 - 0.007, weight_tension_slack, tension_cost));

        //     // Add a soft cost on control input to keep it constrained (prevents wild and too slow solutions).
        //     graph.add(MagnitudeUpperBoundFactor(symbol_t('u', k), u_upper_bound, control_cost));
        //     graph.add(MagnitudeLowerBoundFactor(symbol_t('u', k), u_lower_bound, control_cost));
        //     graph.add(MagnitudeUpperBoundFactor(symbol_t('U', k), u_upper_bound, control_cost));
        //     graph.add(MagnitudeLowerBoundFactor(symbol_t('U', k), u_lower_bound, control_cost));

        //     if (k > 0) {
        //         Vector4 mid_state(4);
        //         mid_state << initial_load_state + k * diff;
        //         graph.add(PriorFactor<Vector4>(symbol_t('l', k), mid_state, goal_cost));
        //     }
        // }

        // // --- 3. Create Initial Estimate ---
        // // The optimizer needs an initial guess for all variables.
        // Values initial_values;
        // Vector2 init_u(0.0, 0.0);
        // Vector1 init_t(0.0);
        // for (int k = 0; k <= num_time_steps; ++k) {
        //     // A simple initial guess: stay at the start position.
        //     initial_values.insert(symbol_t('x', k), initial_robot1_state);
        //     initial_values.insert(symbol_t('X', k), initial_robot2_state);
        //     initial_values.insert(symbol_t('l', k), initial_load_state);
        //     // Initial guess for controls is zero.
        //     if (k < num_time_steps) {
        //         initial_values.insert(symbol_t('u', k), init_u);
        //         initial_values.insert(symbol_t('t', k), init_t);
        //         initial_values.insert(symbol_t('U', k), init_u);
        //         initial_values.insert(symbol_t('T', k), init_t);
        //     }
        // }


        // // // --- 4. Optimize ---
        // LevenbergMarquardtParams params;
        // //params.setVerbosity("TERMINATION"); // Print info at the end
        // LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);

        // cout << "\nOptimizing..." << endl;
        // Values result = optimizer.optimize();
        // cout << "Optimization complete." << endl;
        // cout << "Initial Error: " << graph.error(initial_values) << endl;
        // cout << "Final Error: " << graph.error(result) << endl;

        // Vector4 next_state1 = result.at<Vector4>(symbol_t('x', 1));
        // Vector4 next_state2 = result.at<Vector4>(symbol_t('X', 1));
        // Vector2 next_ctrl1 = result.at<Vector2>(symbol_t('u', 1));
        // Vector2 next_ctrl2 = result.at<Vector2>(symbol_t('U', 1));

        // cout << "Next controls: " << next_ctrl1[0] << ' ' << next_ctrl1[1] << ' ' << next_ctrl2[0] << ' ' << next_ctrl2[1] << endl;
        // return {next_state1[2], next_state1[3], next_state2[2], next_state2[3]};
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
