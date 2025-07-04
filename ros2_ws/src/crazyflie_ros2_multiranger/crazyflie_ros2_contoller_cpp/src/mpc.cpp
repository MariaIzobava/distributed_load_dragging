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
#include "factor_graph_lib/cable_factors.hpp"
#include "factor_graph_lib/control_factors.hpp"
#include "factor_graph_lib/dynamics_factors.hpp"

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
        RCLCPP_INFO(this->get_logger(), "GTSAM C++ test node has started.");

        this->declare_parameter<std::string>("robot_prefix", "/crazyflie");

        std::string robot_prefix_ = this->get_parameter("robot_prefix").as_string();

        position_.resize(6);
        angles_.resize(3);
        load_position_.resize(6);
        load_angles_.resize(3);

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

        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 0.1 seconds = 100 milliseconds
        std::bind(&GtsamCppTestNode::timer_callback, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr load_odom_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> position_;
    std::vector<double> angles_;
    std::vector<double> load_position_;
    std::vector<double> load_angles_;
    nav_msgs::msg::Path::SharedPtr path_;

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

    // Callback functions
    void odom_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        position_[0] = msg->pose.pose.position.x;
        position_[1] = msg->pose.pose.position.y;
        position_[2] = msg->pose.pose.position.z;
        position_[3] = msg->twist.twist.linear.x;
        position_[4] = msg->twist.twist.linear.y;
        position_[5] = msg->twist.twist.linear.z;

        // Quaternion to Euler conversion
        tf2::Quaternion q(
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z,
          msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // getRPY returns roll, pitch, yaw in radians

        angles_[0] = roll;
        angles_[1] = pitch;
        angles_[2] = yaw;
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
        cout << "GOT THE PATH!" << std::endl;
        path_ = msg; // Store the shared pointer to the received path message
    }
    
    void timer_callback()
    {
        if (path_ == NULL) {
            return;
        }

        geometry_msgs::msg::Twist new_cmd_msg; // Create a new Twist message each time

        // If not flying --> takeoff
        if (!is_pulling_) {
            new_cmd_msg.linear.z = 0.5;
            if (position_[2] > desired_height_) {
                new_cmd_msg.linear.z = 0.0;
                is_pulling_ = true;
                start_time_ = std::chrono::high_resolution_clock::now();
                RCLCPP_INFO(this->get_logger(), "Takeoff completed");
            }
            twist_publisher_->publish(new_cmd_msg);
            return;
        }

        // Now the drone is in "pulling" mode, control with MPC

        // First, we're keeping the drone on the same height for now
        double error = desired_height_ - position_[2];
        new_cmd_msg.linear.z = error;

        auto next_velocity = get_next_velocity_();

        // Limiting velocities for safety
        new_cmd_msg.linear.x = max(min(next_velocity[0], 0.3), -0.3);
        new_cmd_msg.linear.y = max(min(next_velocity[1], 0.3), -0.3);
        cout << "Next velocity: " << new_cmd_msg.linear.x << ' ' << new_cmd_msg.linear.y << endl;
        
        twist_publisher_->publish(new_cmd_msg);
    }

    std::vector<double> get_next_velocity_() {
        NonlinearFactorGraph graph;

        // --- Define problem parameters ---
        const int num_time_steps = 20;
        const double dt = 0.005;
        const double robot_mass = 0.025; // kg
        const double load_mass = 0.001;   // kg
        const double gravity = -9.81;
        const double mu = 0.1;
        const double cable_length = 1.2; // meters
        double weight_tension_lower_bound = 1000000.0; // High weight to strongly enforce T >= 0
        double weight_cable_stretch = 1000000.0;     // Very high weight to strongly prevent cable from over-stretching
        double weight_tension_slack = 50.0;
    

        // --- Define Noise Models ---
        // These represent the uncertainty of each factor (1/covariance)
        auto dynamics_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
        auto goal_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.000005, 0.000005, 1000.1, 1000.1).finished());
        auto init_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
        auto control_cost = noiseModel::Diagonal::Sigmas(
            (Vector(2) << 0.1, 0.1).finished());
        auto tension_cost = noiseModel::Isotropic::Sigma(1, 0.001);

        // --- Add Factors to the Graph ---

        // Add prior factors for the initial state
        Vector4 initial_robot_state(
            position_[0],
            position_[1],
            position_[3],
            position_[4]);
        Vector4 initial_load_state(
            load_position_[0],
            load_position_[1],
            load_position_[3],
            load_position_[4]);
        graph.add(PriorFactor<Vector4>(symbol_t('x', 0), initial_robot_state, init_cost));
        graph.add(PriorFactor<Vector4>(symbol_t('l', 0), initial_load_state, init_cost));

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

            graph.add(LoadDynamicsFactor(
                symbol_t('l', k),
                symbol_t('x', k),
                symbol_t('t', k),
                symbol_t('l', k+1),
                dt,
                load_mass,
                mu,
                gravity,
                dynamics_cost
                ));

            // Factor 1: Enforce T_k >= 0
            graph.add(TensionLowerBoundFactor(symbol_t('t', k), weight_tension_lower_bound, tension_cost));
            
            // Factor 2: Penalize if ||p_r - p_l|| > cable_length
            graph.add(CableStretchPenaltyFactor(symbol_t('x', k), symbol_t('l', k), cable_length, weight_cable_stretch, tension_cost));
            
            // Factor 3: Penalize if T_k > 0 AND ||p_r - p_l|| < cable_length (i.e., tension in slack cable)
            graph.add(TensionSlackPenaltyFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length, weight_tension_slack, tension_cost));

            // Add a soft cost on control input to keep it constrained (prevents wild and too slow solutions).
            graph.add(MagnitudeUpperBoundFactor(symbol_t('u', k), 6, tension_cost));
            graph.add(MagnitudeLowerBoundFactor(symbol_t('u', k), 0.88, tension_cost));
        }

        auto next_p = get_next_points();
        cout << "Cur load position: " << load_position_[0] <<  ' ' << load_position_[1] << std::endl;
        cout << "Cur robot position: " << position_[0] <<  ' ' << position_[1] << std::endl;
        cout << "Next position: " << next_p[0] << ' ' << next_p[1] << std::endl;

        // Add a goal cost on the final load state
        Vector4 final_load_goal(
            next_p[0],
            next_p[1],
            load_position_[3], // doesn't matter
            load_position_[4]);  // doesn't matter
        graph.add(PriorFactor<Vector4>(symbol_t('l', num_time_steps), final_load_goal, goal_cost));


        // --- 3. Create Initial Estimate ---
        // The optimizer needs an initial guess for all variables.
        Values initial_values;
        Vector2 init_u(0.0, 0.0);
        Vector1 init_t(0.0);
        for (int k = 0; k <= num_time_steps; ++k) {
            // A simple initial guess: stay at the start position.
            initial_values.insert(symbol_t('x', k), initial_robot_state);
            initial_values.insert(symbol_t('l', k), initial_load_state);
            // Initial guess for controls is zero.
            if (k < num_time_steps) {
                initial_values.insert(symbol_t('u', k), init_u);
                initial_values.insert(symbol_t('t', k), init_t);
            }
        }


        // // --- 4. Optimize ---
        LevenbergMarquardtParams params;
        //params.setVerbosity("TERMINATION"); // Print info at the end
        LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);

        cout << "\nOptimizing..." << endl;
        Values result = optimizer.optimize();
        cout << "Optimization complete." << endl;
        cout << "Initial Error: " << graph.error(initial_values) << endl;
        cout << "Final Error: " << graph.error(result) << endl;


        // --- 5. Print Results ---
        Vector4 next_state = result.at<Vector4>(symbol_t('x', 1));
        Vector2 next_ctrl = result.at<Vector2>(symbol_t('u', 1));

        cout << "Next control: " << next_ctrl[0] << ' ' << next_ctrl[1] << endl;
        return {next_state[2], next_state[3]};
    }

    std::vector<double> get_next_points() {
        int num_p = 2500; // num_p is number of points
        
        // Get current time
        auto cur_time = std::chrono::high_resolution_clock::now();
        
        // Calculate seconds from the beginning
        auto cur_millisec = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - start_time_);

        int i = (cur_millisec.count() / 100 + 1) % num_p;
        cout << i << std::endl;
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
