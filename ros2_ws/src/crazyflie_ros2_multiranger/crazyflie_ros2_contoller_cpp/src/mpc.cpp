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

/**
 * Custom factor to model the robot's dynamics.
 * Connects Xr_k, U_k, T_k, Xr_{k+1}
 * Error = Xr_{k+1} - (Xr_k + dt * f(Xr_k, U_k, T_k))
 */
class RobotDynamicsFactor: public NoiseModelFactor5<Vector4, Vector4, Vector2, Vector1, Vector4> {
    double dt_;
    double robot_mass_;

public:
    // Standard constructor
    RobotDynamicsFactor(Key key_xr_k, Key key_xl_k, Key key_u_k, Key key_tension_k, Key key_xr_k_plus_1,
                        double dt, double robot_mass, const SharedNoiseModel& model) :
        NoiseModelFactor5<Vector4, Vector4, Vector2, Vector1, Vector4>(model, key_xr_k, key_xl_k, key_u_k, key_tension_k, key_xr_k_plus_1),
        dt_(dt), robot_mass_(robot_mass) {}

    // The evaluateError function, which implements the factor's error calculation.
    Vector evaluateError(const Vector4& xr_k,
                         const Vector4& xl_k,
                         const Vector2& u_k, 
                         const Vector1& tension_k,
                         const Vector4& xr_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4,
                         gtsam::OptionalMatrixType H5) const override {
        
        // Unpack state: xr = [px, py, vx, vy]
        Vector2 pos_k = xr_k.head<2>();
        Vector2 vel_k = xr_k.tail<2>();

        // Simple Euler integration for dynamics
        // next_pos = current_pos + dt * current_vel
        // next_vel = current_vel + dt * (control_force/mass - tension_force/mass)
        Vector2 next_pos = pos_k + vel_k * dt_;

        double xd = xr_k(0) - xl_k(0);
        double yd = xr_k(1) - xl_k(1);
        Vector2 e(2);
        e << xd, yd;

        double norm = e.norm();
        Vector2 e_norm(2);
        e_norm << e / norm;

        Vector2 next_vel = vel_k + (u_k - tension_k(0) * e_norm) / robot_mass_ * dt_;

        Vector4 predicted_xr_k_plus_1(4);
        predicted_xr_k_plus_1 << next_pos, next_vel;

        double ONE = 1.0 / norm - xd * xd / (norm * norm * norm);
        double TWO = -1.0 / norm + xd * xd / (norm * norm * norm);
        double THREE = -xd * yd / (norm * norm * norm);
        double FOUR = xd * yd / (norm * norm * norm);
        double FIVE = 1.0 / norm - yd * yd / (norm * norm * norm);
        double SIX = -1.0 / norm + yd * yd / (norm * norm * norm);

        if (H1) {
            // H1 is a valid reference to a matrix, so you can assign to it.
            *H1 = (gtsam::Matrix(4, 4) << 
                -1,  0, -dt_, 0,
                 0, -1,  0,  -dt_,
                 dt_ * tension_k(0) * ONE / robot_mass_,  dt_ * tension_k(0) * THREE / robot_mass_, -1,   0,
                 dt_ * tension_k(0) * THREE / robot_mass_,  dt_ * tension_k(0) * FIVE / robot_mass_,  0,  -1).finished();

        }
        if (H2) {
            *H2 = (gtsam::Matrix(4, 4) << 
                0, 0, 0, 0,
                0, 0, 0, 0,
                 dt_ * tension_k(0) * TWO / robot_mass_,  dt_ * tension_k(0) * FOUR / robot_mass_, 0,   0,
                 dt_ * tension_k(0) * FOUR / robot_mass_,  dt_ * tension_k(0) * SIX / robot_mass_,  0,  0).finished();

        }

        if (H3) {
            *H3 = (gtsam::Matrix(4, 2) << 
                0, 0, 
                0, 0,  
                -(dt_/robot_mass_), 0, 
                0, -(dt_/robot_mass_)).finished();
        }
        if (H4) {
            *H4 = (gtsam::Matrix(4, 1) << 
                0,
                0,
                (dt_ * xd / (robot_mass_ * norm)), 
                (dt_ * yd / (robot_mass_ * norm))).finished();
        }
        if (H5) {
            *H5 = gtsam::Matrix4::Identity();
        }

        return (Vector(4) << xr_k_plus_1 - predicted_xr_k_plus_1).finished();
    }
};


class LoadDynamicsFactor: public NoiseModelFactor4<Vector4, Vector4, Vector1, Vector4> {
    double dt_;
    double load_mass_;
    double mu_;
    double g_;

public:
    // Standard constructor
    LoadDynamicsFactor(Key key_xl_k, Key key_xr_k, Key key_tension_k, Key key_xl_k_plus_1,
                        double dt, double load_mass, double mu, double g, const SharedNoiseModel& model) :
        NoiseModelFactor4<Vector4, Vector4, Vector1, Vector4>(model, key_xl_k, key_xr_k, key_tension_k, key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), g_(g) {}

    // The evaluateError function, which implements the factor's error calculation.
    Vector evaluateError(const Vector4& xl_k, 
                         const Vector4& xr_k,
                         const Vector1& tension_k,
                         const Vector4& xl_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4) const override {
        
        // Unpack state: xr = [px, py, vx, vy]
        Vector2 pos_k = xl_k.head<2>();
        Vector2 vel_k = xl_k.tail<2>();
        double v_norm = vel_k.norm();
        Vector2 normed_vel_k = Vector2::Zero();
        double SEVEN = 0.0;
        double EIGHT = 0.0;
        double NINE = 0.0;
        if (v_norm > 1e-12) {
            normed_vel_k << vel_k / v_norm;
            SEVEN = 1.0 / v_norm - vel_k(0) * vel_k(0) / (v_norm * v_norm * v_norm);
            EIGHT = 1.0 / v_norm - vel_k(1) * vel_k(1) / (v_norm * v_norm * v_norm);
            NINE = -1.0 * vel_k(0) * vel_k(1) / (v_norm * v_norm * v_norm);
        }

        // Simple Euler integration for dynamics
        // next_pos = current_pos + dt * current_vel
        // next_vel = current_vel + dt * (control_force/mass - tension_force/mass)
        Vector2 next_pos = pos_k + vel_k * dt_;

        double xd = xr_k(0) - xl_k(0);
        double yd = xr_k(1) - xl_k(1);
        Vector2 e(2);
        e << xd, yd;

        double norm = e.norm();
        Vector2 e_norm(2);
        e_norm << e / e.norm();

        Vector2 next_vel = vel_k + (tension_k(0) * e_norm - mu_ * load_mass_ * g_ * normed_vel_k) / load_mass_ * dt_;

        Vector4 predicted_xl_k_plus_1(4);
        predicted_xl_k_plus_1 << next_pos, next_vel;

        double ONE = 1.0 / norm - xd * xd / (norm * norm * norm);
        double TWO = -1.0 / norm + xd * xd / (norm * norm * norm);
        double THREE = -yd * xd / (norm * norm * norm);
        double FOUR = xd * yd / (norm * norm * norm);
        double FIVE = 1.0 / norm - yd * yd / (norm * norm * norm);
        double SIX = -1.0 / norm + yd * yd / (norm * norm * norm);

        if (H1) {
            *H1 = (gtsam::Matrix(4, 4) << 
                -1,  0, -dt_, 0,
                 0, -1,  0,  -dt_,
                 -dt_ * tension_k(0) * TWO / load_mass_,  -dt_ * tension_k(0) * FOUR / load_mass_, -1 + dt_ * mu_ * g_ * SEVEN, dt_ * mu_ * g_ * NINE,
                 -dt_ * tension_k(0) * FOUR / load_mass_,  -dt_ * tension_k(0) * SIX / load_mass_,  dt_ * mu_ * g_ * NINE,  -1 + dt_ * mu_ * g_ * EIGHT).finished();
        }
        if (H2) {
            *H2 = (gtsam::Matrix(4, 4) << 
                0,  0, 0, 0,
                 0, 0,  0,  0,
                 -dt_ * tension_k(0) * ONE / load_mass_,  -dt_ * tension_k(0) * THREE / load_mass_, 0, 0,
                 -dt_ * tension_k(0) * THREE / load_mass_,  -dt_ * tension_k(0) * FIVE / load_mass_, 0, 0).finished();
        }
        if (H3) {
            *H3 = (gtsam::Matrix(4, 1) << 
                0,
                0,
                -(dt_ * xd / (load_mass_ * norm)),
                -(dt_ * yd / (load_mass_ * norm))).finished();
        }
        if (H4) {
            *H4 = gtsam::Matrix4::Identity();
        }

        return (Vector(4) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();
    }
};


// Helper function for a smooth approximation of max(0, x)
// This is used to make the factors differentiable everywhere, which helps gradient-based optimizers.
double smooth_max_zero(double x, double epsilon = 1e-6) {
    return 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
}


class TensionLowerBoundFactor : public NoiseModelFactor1<Vector1> {
private:
    double weight_; // Weight for this factor

public:
    // Constructor: Takes the Key for the tension variable, a noise model, and the weight.
    TensionLowerBoundFactor(Key tensionKey, double weight, const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector1>(model, tensionKey), weight_(weight) {}

    // evaluateError: Calculates the residual (error) vector.
    // T_k: The current estimate of the tension variable.
    // H: Optional output for the Jacobian matrix.
    Vector evaluateError(const Vector1& T_k,
                         gtsam::OptionalMatrixType H) const override {
        // The error value is `weight_ * smooth_max_zero(-T_k)`.
        double error_val = weight_ * smooth_max_zero(-T_k(0));

        if (H) {
            // Calculate the Jacobian (derivative of the error with respect to T_k).
            // Let f(x) = smooth_max_zero(x). We want d(weight_ * f(-T_k))/dT_k.
            // Using the chain rule: weight_ * f'(-T_k) * d(-T_k)/dT_k = weight_ * f'(-T_k) * (-1).
            // The derivative of f(x) is f'(x) = 0.5 * (1 + x / sqrt(x^2 + epsilon^2)).
            // So, f'(-T_k) = 0.5 * (1 + (-T_k) / sqrt((-T_k)^2 + epsilon^2)).
            // d(error_val)/dT_k = weight_ * (-0.5) * (1 - T_k / sqrt(T_k^2 + epsilon^2)).
            double deriv_smooth_max_inner = 0.5 * (1.0 - T_k(0) / std::sqrt(T_k(0)*T_k(0) + 1e-6*1e-6));
            (*H) = (Vector(1) << -weight_ * deriv_smooth_max_inner).finished();
        }
        return (Vector(1) << error_val).finished(); // Return a 1x1 vector for scalar error
    }
};

// -------------------------------------------------------------------------
// 2. Custom Factor: Cable Stretch Penalty (||p_r - p_l|| <= L_cable)
// Error: w_stretch * max(0, ||p_r - p_l|| - L_cable)
// This factor penalizes if the cable is stretched beyond its nominal length.
// -------------------------------------------------------------------------
class CableStretchPenaltyFactor : public NoiseModelFactor2<Vector4, Vector4> {
private:
    double cable_length_; // The nominal length of the cable
    double weight_;       // Weight for this factor

public:
    // Constructor: Takes Keys for robot and load positions, cable length, noise model, and weight.
    CableStretchPenaltyFactor(Key robotPosKey, Key loadPosKey, double cableLength,
                              double weight, const SharedNoiseModel& model)
        : NoiseModelFactor2<Vector4, Vector4>(model, robotPosKey, loadPosKey),
          cable_length_(cableLength), weight_(weight) {}

    // evaluateError: Calculates the residual vector.
    // p_r: Robot position.
    // p_l: Load position.
    // H1, H2: Optional outputs for Jacobians with respect to p_r and p_l.
    Vector evaluateError(const Vector4& p_r, const Vector4& p_l,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2) const override {
        Vector2 diff(2);
        diff << p_r(0) - p_l(0), p_r(1) - p_l(1);
        double distance = diff.norm(); // Euclidean distance ||p_r - p_l||

        // The error value is `weight_ * smooth_max_zero(distance - cable_length_)`.
        double error_val = weight_ * smooth_max_zero(distance - cable_length_);

        if (H1 || H2) {
            // Calculate the derivative of `smooth_max_zero(x)` with respect to `x`.
            // Here, x = `distance - cable_length_`.
            double inner_term = distance - cable_length_;
            double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));

            // The derivative of distance `||p_r - p_l||` with respect to `p_r` is `(p_r - p_l) / ||p_r - p_l||`,
            // which is the unit vector `e` pointing from load to robot.
            Vector2 partial_deriv_distance_pr = diff / distance;
            // The derivative with respect to `p_l` is `-(p_r - p_l) / ||p_r - p_l||`, or `-e`.
            Vector2 partial_deriv_distance_pl = -diff / distance;

            if (H1) { // Jacobian with respect to p_r
                // Chain rule: d(error)/dp_r = weight * d(smooth_max_zero)/d(inner) * d(inner)/d(distance) * d(distance)/dp_r
                // d(inner)/d(distance) is 1.0.
                (*H1) = (gtsam::Matrix(1, 4) << weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(0), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(1), 0, 0).finished();
            }
            if (H2) { // Jacobian with respect to p_l
                (*H2) = (gtsam::Matrix(1, 4) << weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(0), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(1), 0, 0).finished();
            }
        }
        return (Vector(1) << error_val).finished();
    }
};

// -------------------------------------------------------------------------
// 3. Custom Factor: Tension-Slack Penalty (T_k * max(0, L_cable - ||p_r - p_l||))
// Error: w_T_slack * T_k * max(0, L_cable - ||p_r - p_l||)
// This factor discourages positive tension when the cable is slack.
// -------------------------------------------------------------------------
class TensionSlackPenaltyFactor : public NoiseModelFactor3<Vector1, Vector4, Vector4> {
private:
    double cable_length_; // The nominal length of the cable
    double weight_;       // Weight for this factor

public:
    // Constructor: Takes Keys for tension, robot position, load position, cable length, noise model, and weight.
    TensionSlackPenaltyFactor(Key tensionKey, Key robotPosKey, Key loadPosKey, double cableLength,
                               double weight, const SharedNoiseModel& model)
        : NoiseModelFactor3<Vector1, Vector4, Vector4>(model, tensionKey, robotPosKey, loadPosKey),
          cable_length_(cableLength), weight_(weight) {}

    // evaluateError: Calculates the residual vector.
    // T_k: Tension.
    // p_r: Robot position.
    // p_l: Load position.
    // H1, H2, H3: Optional outputs for Jacobians.
    Vector evaluateError(const Vector1& T_k, const Vector4& p_r, const Vector4& p_l,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3) const override {
        Vector2 diff(2);
        diff << p_r(0) - p_l(0), p_r(1) - p_l(1);
        double distance = diff.norm();
        // Calculate the slack term: max(0, L_cable - distance).
        // This term is positive if the cable is slack (distance < L_cable).
        double slack_term_val = smooth_max_zero(cable_length_ - distance);

        // The error value is `weight_ * T_k * slack_term_val`.
        // If T_k > 0 AND slack_term_val > 0, this factor incurs a penalty.
        double error_val = weight_ * T_k(0) * slack_term_val;

        if (H1 || H2 || H3) {
            // Derivatives for Jacobian calculations:
            // d(error_val)/dT_k = weight_ * slack_term_val
            // d(error_val)/dp_r = weight_ * T_k * d(slack_term_val)/dp_r
            // d(error_val)/dp_l = weight_ * T_k * d(slack_term_val)/dp_l

            // First, calculate derivative of smooth_max_zero(x) where x = (cable_length_ - distance).
            // d(smooth_max_zero(x))/dx = 0.5 * (1 + x / sqrt(x^2 + epsilon^2))
            double inner_term = cable_length_ - distance;
            double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));

            // Chain rule for d(slack_term_val)/d(distance): d(smooth_max_zero(C-D))/dD = d(smooth_max_zero)/d(inner) * d(inner)/dD
            // Here, d(inner)/dD = d(C-D)/dD = -1.
            double deriv_slack_wrt_distance = deriv_smooth_max_wrt_inner * (-1.0);

            // Derivatives of distance with respect to positions (unit vectors).
            Vector2 partial_deriv_distance_pr = diff / distance; // unit vector e
            Vector2 partial_deriv_distance_pl = -diff / distance; // -e vector

            if (H1) { // Jacobian with respect to T_k
                (*H1) = (Vector(1) << weight_ * slack_term_val).finished();
            }
            if (H2) { // Jacobian with respect to p_r
                // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_r
                (*H2) = (gtsam::Matrix(1, 4) << weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pr(0), weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pr(1), 0, 0).finished();
            }
            if (H3) { // Jacobian with respect to p_l
                // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_l
                (*H3) = (gtsam::Matrix(1, 4) << weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pl(0), weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pl(1), 0, 0).finished();
            }
        }
        return (Vector(1) << error_val).finished();
    }
};

class MagnitudeUpperBoundFactor : public NoiseModelFactor1<Vector2> { // Or Pose2 if u_x, u_y are part of a Pose
private:
    double max_magnitude_; // The 0.7 limit
public:
    MagnitudeUpperBoundFactor(Key key, double maxMagnitude,
                      const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector2>(model, key), max_magnitude_(maxMagnitude) {}

    // The evaluateError method calculates the residual
    Vector evaluateError(const Vector2& u_val,
                                OptionalMatrixType H) const override {
        // Calculate current magnitude
        double current_magnitude = u_val.norm(); // Vector2::norm() computes Euclidean norm

        // Calculate the residual. Only penalize if it exceeds max_magnitude.
        // We'll return a 1-dimensional residual vector.
        Vector residual(1);
        if (current_magnitude > max_magnitude_) {
            residual[0] = current_magnitude - max_magnitude_;
        } else {
            residual[0] = 0.0;
        }

        if (H) {
            // Compute Jacobian if requested
            // The Jacobian of residual w.r.t u_val. This can be tricky.
            // If residual[0] = current_magnitude - max_magnitude:
            //   d(current_magnitude)/du_x = u_x / current_magnitude
            //   d(current_magnitude)/du_y = u_y / current_magnitude
            // If residual[0] = 0: Jacobian is all zeros.

            if (current_magnitude > max_magnitude_ && current_magnitude > 1e-9) { // Avoid division by zero
                (*H) = (gtsam::Matrix(1, 2) << u_val[0] / current_magnitude, u_val[1] / current_magnitude).finished();
            } else {
                (*H) = (gtsam::Matrix(1, 2) << 0, 0).finished(); // Zero Jacobian
            }
        }
        return residual;
    }
};

class MagnitudeLowerBoundFactor : public NoiseModelFactor1<Vector2> { // Or Pose2 if u_x, u_y are part of a Pose
private:
    double min_magnitude_; // The 0.7 limit
public:
    MagnitudeLowerBoundFactor(Key key, double minMagnitude,
                      const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector2>(model, key), min_magnitude_(minMagnitude) {}

    // The evaluateError method calculates the residual
    Vector evaluateError(const Vector2& u_val,
                                OptionalMatrixType H) const override {
        // Calculate current magnitude
        double current_magnitude = u_val.norm(); // Vector2::norm() computes Euclidean norm

        // Calculate the residual. Only penalize if it exceeds max_magnitude.
        // We'll return a 1-dimensional residual vector.
        Vector residual(1);
        if (current_magnitude < min_magnitude_) {
            residual[0] = min_magnitude_ - current_magnitude;
        } else {
            residual[0] = 0.0;
        }

        if (H) {
            // Compute Jacobian if requested
            // The Jacobian of residual w.r.t u_val. This can be tricky.
            // If residual[0] = current_magnitude - max_magnitude:
            //   d(current_magnitude)/du_x = u_x / current_magnitude
            //   d(current_magnitude)/du_y = u_y / current_magnitude
            // If residual[0] = 0: Jacobian is all zeros.

            if (current_magnitude < min_magnitude_ && current_magnitude > 1e-9) { // Avoid division by zero
                (*H) = (gtsam::Matrix(1, 2) << -u_val[0] / current_magnitude, -u_val[1] / current_magnitude).finished();
            } else {
                (*H) = (gtsam::Matrix(1, 2) << 0, 0).finished(); // Zero Jacobian
            }
        }
        return residual;
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
    std::chrono::high_resolution_clock::time_point start_time_;

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
        const double dt = 0.005;
        const double robot_mass = 0.025; // kg
        const double load_mass = 0.0001;   // kg
        const double gravity = -9.81;
        const double mu = 0.1;
        const double cable_length = 1.2; // meters
    

        // --- Define Noise Models ---
        // These represent the uncertainty of each factor (1/covariance)
        auto dynamics_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
        auto goal_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.000005, 0.000005, 1000.1, 1000.1).finished());
        auto init_cost = noiseModel::Diagonal::Sigmas(
            (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
        auto tension_cost = noiseModel::Isotropic::Sigma(1, 0.001);

        auto control_cost = noiseModel::Diagonal::Sigmas(
            (Vector(2) << 0.1, 0.1).finished());
        //auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-6);

        double weight_tension_lower_bound = 1000000.0; // High weight to strongly enforce T >= 0
        double weight_cable_stretch = 1000000.0;     // Very high weight to strongly prevent cable from over-stretching
        double weight_tension_slack = 50.0;

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

            // Add a soft cost on control input to keep it small (prevents wild solutions)
            //graph.add(PriorFactor<Vector2>(symbol_t('u', k), Vector2::Zero(), control_cost));
            graph.add(MagnitudeUpperBoundFactor(symbol_t('u', k), 0.3, tension_cost));
            graph.add(MagnitudeLowerBoundFactor(symbol_t('u', k), 0.1, tension_cost));
        }

        auto next_p = get_next_points(num_time_steps);
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
        params.setVerbosity("TERMINATION"); // Print info at the end
        LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);

        cout << "\nOptimizing..." << endl;
        Values result = optimizer.optimize();
        cout << "Optimization complete." << endl;
        cout << "Initial Error: " << graph.error(initial_values) << endl;
        cout << "Final Error: " << graph.error(result) << endl;


        // --- 5. Print Results ---
        Vector4 next_state = result.at<Vector4>(symbol_t('x', 1));
        Vector2 next_ctrl = result.at<Vector2>(symbol_t('u', 1));

        cout << "Next velocity: " << next_state[2] << ' ' << next_state[3] << endl;
        cout << "Next control: " << next_ctrl[0] << ' ' << next_ctrl[1] << endl;
        return {next_state[2], next_state[3]};
    }

    std::vector<double> get_next_points(int n) {
        int num_p = 2500; // num_p is number of points
        
        // Get current time
        auto cur_time = std::chrono::high_resolution_clock::now();
        
        // Calculate seconds from the beginning
        auto cur_millisec = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - start_time_);
        //double sec_from_beginning = sec_from_beginning_duration.count();

        // Calculate start_point using modulo. 
        // Ensure num_p is an integer for the modulo operator.
        int i = (cur_millisec.count() / 100 + 1) % num_p;
        cout << "Next point is: " << i << ' ' << cur_millisec.count() << std::endl;
        std::vector<double> ans;
        ans.push_back(path_->poses[i].pose.position.x);
        ans.push_back(path_->poses[i].pose.position.y);
        return ans; 
        
        // std::vector<double> res;
        // res.reserve(n * 2); // Pre-allocate memory to improve performance

        // // First loop: from start_point to min(start_point + n, num_p)
        // for (int i = start_point; i < std::min(start_point + n, num_p); ++i) {
        //     // Check if i is a valid index
        //     if (i >= 0 && i < path.poses.size()) {
        //         Point p = path.poses[i].position;
        //         res.push_back(p.x);
        //         res.push_back(p.y);
        //     } else {
        //         // Handle out-of-bounds index if necessary, though ideally path.poses should have num_p elements
        //         // For example, you might want to break or log an error.
        //     }
        // }

        // // Second loop: if start_point + n wraps around num_p
        // if (start_point + n > num_p) {
        //     // The range for the second loop should be from 0 up to the remaining points needed.
        //     // The number of remaining points is (start_point + n) - num_p.
        //     // So, loop for i from 0 up to (start_point + n) - num_p.
        //     int remaining_points_to_add = (start_point + n) - num_p;
        //     for (int i = 0; i < remaining_points_to_add; ++i) {
        //          // Check if i is a valid index
        //         if (i >= 0 && i < path.poses.size()) {
        //             Point p = path.poses[i].position;
        //             res.push_back(p.x);
        //             res.push_back(p.y);
        //         } else {
        //             // Handle out-of-bounds index if necessary
        //         }
        //     }
        // }
        // return res;
    }


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GtsamCppTestNode>());
    rclcpp::shutdown();
    return 0;
}
