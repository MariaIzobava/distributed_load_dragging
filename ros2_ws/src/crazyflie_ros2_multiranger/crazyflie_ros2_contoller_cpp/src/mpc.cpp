#include "rclcpp/rclcpp.hpp"

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp" // For the Quaternion message type

// For quaternion to Euler conversion
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <vector>
#include <string> 
#include <chrono>

// Use gtsam namespace
using namespace std;
using namespace gtsam;

// Define symbols for different variable types for clarity
using symbol_t = gtsam::Symbol;

class NonNegativeFactor: public NoiseModelFactor1<Vector1> {
public:
    NonNegativeFactor(Key key, const SharedNoiseModel& model): 
        NoiseModelFactor1<Vector1>(model, key) {}

    Vector evaluateError(const Vector1& x, gtsam::OptionalMatrixType H) const override {
        const double val = x(0);

        if (val >= 0) {
            if (H) {
                *H = Matrix11::Zero();

            }
            return Vector1::Zero();
        } else {
            if (H) {
                *H = -Matrix11::Identity();
            }
            return Vector1(-val);
        }
    }
};

/**
 * Custom factor to model the robot's dynamics.
 * Connects Xr_k, U_k, T_k, Xr_{k+1}
 * Error = Xr_{k+1} - (Xr_k + dt * f(Xr_k, U_k, T_k))
 */
class RobotDynamicsFactor: public NoiseModelFactor4<Vector4, Vector2, Vector1, Vector4> {
    double dt_;
    double robot_mass_;

public:
    // Standard constructor
    RobotDynamicsFactor(Key key_xr_k, Key key_u_k, Key key_tension_k, Key key_xr_k_plus_1,
                        double dt, double robot_mass, const SharedNoiseModel& model) :
        NoiseModelFactor4<Vector4, Vector2, Vector1, Vector4>(model, key_xr_k, key_u_k, key_tension_k, key_xr_k_plus_1),
        dt_(dt), robot_mass_(robot_mass) {}

    // The evaluateError function, which implements the factor's error calculation.
    Vector evaluateError(const Vector4& xr_k, 
                         const Vector2& u_k, 
                         const Vector1& tension_k,
                         const Vector4& xr_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4) const override {
        
        // Unpack state: xr = [px, py, vx, vy]
        Vector2 pos_k = xr_k.head<2>();
        Vector2 vel_k = xr_k.tail<2>();

        // Simple Euler integration for dynamics
        // next_pos = current_pos + dt * current_vel
        // next_vel = current_vel + dt * (control_force/mass - tension_force/mass)
        Vector2 next_pos = pos_k + vel_k * dt_;
        Vector2 t(2);
        t << tension_k, 0;
        Vector2 next_vel = vel_k + (u_k + t) / robot_mass_ * dt_;

        Vector4 predicted_xr_k_plus_1(4);
        predicted_xr_k_plus_1 << next_pos, next_vel;

        if (H1) {
            // H1 is a valid reference to a matrix, so you can assign to it.
            *H1 = (gtsam::Matrix(4, 4) << 
                -1,  0, -dt_, 0,
                 0, -1,  0,  -dt_,
                 0,  0, -1,   0,
                 0,  0,  0,  -1).finished();

        }
        if (H2) {
            *H2 = (gtsam::Matrix(4, 2) << 
                0,  0, 
                0, 0,  
                -(dt_/robot_mass_), 0, 
                0, -(dt_/robot_mass_)).finished();
        }
        if (H3) {
            *H3 = (gtsam::Matrix(4, 1) << 
                0, 
                0,  
                -(dt_/robot_mass_), 
                0).finished();
        }
        if (H4) {
            *H4 = gtsam::Matrix4::Identity();
        }

        return (Vector(4) << xr_k_plus_1 - predicted_xr_k_plus_1).finished();
    }
};


class LoadDynamicsFactor: public NoiseModelFactor3<Vector4, Vector1, Vector4> {
    double dt_;
    double load_mass_;
    double mu_;
    double g_;

public:
    // Standard constructor
    LoadDynamicsFactor(Key key_xl_k, Key key_tension_k, Key key_xl_k_plus_1,
                        double dt, double load_mass, double mu, double g, const SharedNoiseModel& model) :
        NoiseModelFactor3<Vector4, Vector1, Vector4>(model, key_xl_k, key_tension_k, key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), g_(g) {}

    // The evaluateError function, which implements the factor's error calculation.
    Vector evaluateError(const Vector4& xl_k, 
                         const Vector1& tension_k,
                         const Vector4& xl_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3) const override {
        
        // Unpack state: xr = [px, py, vx, vy]
        Vector2 pos_k = xl_k.head<2>();
        Vector2 vel_k = xl_k.tail<2>();
        double v_norm = vel_k.norm();
        Vector2 normed_vel_k = Vector2::Zero();
        if (v_norm > 1e-1) {
            normed_vel_k << vel_k / v_norm;
        }

        // Simple Euler integration for dynamics
        // next_pos = current_pos + dt * current_vel
        // next_vel = current_vel + dt * (control_force/mass - tension_force/mass)
        Vector2 next_pos = pos_k + vel_k * dt_;
        Vector2 t(2);
        t << tension_k, 0;

        //Vector2 next_vel = vel_k + (-t - mu_ * load_mass_ * g_ * normed_vel_k) / load_mass_ * dt_;
        Vector2 next_vel = vel_k + (-t - mu_ * load_mass_ * g_ * vel_k) / load_mass_ * dt_;

        Vector4 predicted_xl_k_plus_1(4);
        predicted_xl_k_plus_1 << next_pos, next_vel;

        if (H1) {
            
            double ax_der_x = 0;
            double ay_der_y = 0;
            double ax_der_y = 0;
            if (v_norm > 1e-1) {  
                ax_der_x = -mu_ * g_ * (1 / v_norm - vel_k(0) * vel_k(0) / (v_norm * v_norm * v_norm));
                ay_der_y = -mu_ * g_ * (1 / v_norm - vel_k(1) * vel_k(1) / (v_norm * v_norm * v_norm));
                ax_der_y = -mu_ * g_ * vel_k(0) * vel_k(1) / (v_norm * v_norm * v_norm);
            }

            // *H1 = (gtsam::Matrix(4, 4) << 
            //     -1,  0, -dt_, 0,
            //      0, -1,  0,  -dt_,
            //      0,  0, -1 - dt_ * ax_der_x, -dt_ * ax_der_y,
            //      0,  0,  -dt_ * ax_der_y,  -1 - dt_ * ay_der_y).finished();

            *H1 = (gtsam::Matrix(4, 4) << 
                -1,  0, -dt_, 0,
                 0, -1,  0,  -dt_,
                 0,  0, -1 - dt_ * (-mu_ * g_), 0,
                 0,  0,  0,  -1 - dt_ * (-mu_ * g_)).finished();
        }
        if (H2) {
            *H2 = (gtsam::Matrix(4, 1) << 
                0, 
                0,  
                (dt_/load_mass_), 
                0).finished();
        }
        if (H3) {
            *H3 = gtsam::Matrix4::Identity();
        }

        return (Vector(4) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();
    }
};

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
        desired_height_ = 1.0;

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

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> position_;
    std::vector<double> angles_;
    std::vector<double> load_position_;
    std::vector<double> load_angles_;
    nav_msgs::msg::Path::SharedPtr path_;

    bool is_pulling_;
    double desired_height_;
    rclcpp::Time start_time_;

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
        path_ = msg; // Store the shared pointer to the received path message
    }
    
    void timer_callback()
    {
        geometry_msgs::msg::Twist new_cmd_msg; // Create a new Twist message each time

        // If not flying --> takeoff
        if (!is_pulling_) {
            new_cmd_msg.linear.z = 0.5;
            if (position_[2] > desired_height_) {
                // stop going up if height is reached
                new_cmd_msg.linear.z = 0.0;
                is_pulling_ = true;
                start_time_ = this->now();
                RCLCPP_INFO(this->get_logger(), "Takeoff completed");
            }
            twist_publisher_->publish(new_cmd_msg);
            return;
        }

        // Now the drone is in "pulling" mode, control with MPC

        // First, we're keeping the drone on the same height for now
        double error = desired_height_ - position_[2];
        new_cmd_msg.linear.z = error;
        //new_cmd_msg.linear.x = -0.1;

        auto next_velocity = get_next_velocity_();

        new_cmd_msg.linear.x = next_velocity[0];
        new_cmd_msg.linear.y = next_velocity[1];
        
        twist_publisher_->publish(new_cmd_msg);
    }

    std::vector<double> get_next_velocity_() {
        NonlinearFactorGraph graph;

        // --- Define problem parameters ---
        const int num_time_steps = 20;
        const double dt = 0.1;
        const double robot_mass = 0.025; // kg
        const double load_mass = 0.0001;   // kg
        const double gravity = -9.81;
        const double mu = 0.1;
        // const double cable_length = 1.0; // meters
        // const double cable_stiffness = 500.0; // N/m
        // const double cable_damping = 20.0;
    

        // --- Define Noise Models ---
        // These represent the uncertainty of each factor (1/covariance)
        auto prior_noise = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.01, 0.01, 0.01, 0.01).finished());
        auto control_cost = noiseModel::Diagonal::Sigmas(
            (Vector(2) << 0.1, 0.1).finished());
        auto goal_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.05, 0.05, 0.1, 0.1).finished());
        auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-6);


        // --- Add Factors to the Graph ---

        // Add prior factors for the initial state (t=0)
        // Assume robot starts at origin, load is hanging below it.
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
        graph.add(PriorFactor<Vector4>(symbol_t('x', 0), initial_robot_state, prior_noise));
        graph.add(PriorFactor<Vector4>(symbol_t('l', 0), initial_load_state, prior_noise));

        for (int k = 0; k < num_time_steps; ++k) {
            graph.add(RobotDynamicsFactor(
                symbol_t('x', k), 
                symbol_t('u', k),
                symbol_t('t', k),
                symbol_t('x', k+1),
                dt,
                robot_mass,
                prior_noise
                ));

            graph.add(LoadDynamicsFactor(
                symbol_t('l', k),
                symbol_t('t', k),
                symbol_t('l', k+1),
                dt,
                load_mass,
                mu,
                gravity,
                prior_noise
                ));

            // Add a soft cost on control input to keep it small (prevents wild solutions)
            graph.add(PriorFactor<Vector2>(symbol_t('u', k), Vector2::Zero(), control_cost));
            graph.add(NonNegativeFactor(symbol_t('t', k), tension_cost));
        }

        // Add a goal cost on the final load state
        Vector4 final_load_goal(load_position_[0] - 0.2,
            load_position_[1],
            load_position_[3],
            load_position_[4]); 
        graph.add(PriorFactor<Vector4>(symbol_t('l', num_time_steps), final_load_goal, goal_cost));
        
        Vector4 final_robot_goal(position_[0] - 0.2,
            position_[1],
            position_[3],
            position_[4]);
        graph.add(PriorFactor<Vector4>(symbol_t('x', num_time_steps), final_robot_goal, goal_cost));


        // cout << "Factor Graph built." << endl;
        // graph.print("Factor Graph:\n");


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

        // cout << "\nInitial values created." << endl;
        // initial_values.print("Initial Values:\n");

        // // --- 4. Optimize ---
        LevenbergMarquardtParams params;
        params.setVerbosity("TERMINATION"); // Print info at the end
        LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);

        cout << "\nOptimizing..." << endl;
        Values result = optimizer.optimize();
        cout << "Optimization complete." << endl;
        cout << "Initial Error: " << graph.error(initial_values) << endl;
        cout << "Final Error: " << graph.error(result) << endl;


        // --- 5. Print Results ---
        Vector4 next_state = result.at<Vector4>(symbol_t('x', 1));
        return {next_state[2], next_state[3]};
    }


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GtsamCppTestNode>());
    rclcpp::shutdown();
    return 0;
}
