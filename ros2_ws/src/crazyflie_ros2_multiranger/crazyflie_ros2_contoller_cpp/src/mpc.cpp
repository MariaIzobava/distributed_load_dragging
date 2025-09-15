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
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "phasespace_msgs/msg/rigids.hpp"

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
        false, 
        false, 
        "/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/one_drone_no_ori_points.json",
        "gtsam_cpp_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "MPC with GTSAM node has started.");

        this->declare_parameter<std::string>("env", "sim");
        this->declare_parameter<std::string>("robot_prefix", "/crazyflie");
        init_robot_num(1);

        std::string robot_prefix_ = this->get_parameter("robot_prefix").as_string();
        std::string env_ = this->get_parameter("env").as_string();

        position_.resize(6);
        angles_.resize(3);
        load_position_.resize(6);
        load_angles_.resize(3);
        last_control_ << 0.0, 0.0;
        last_tension_ << 0;

        is_pulling_ = false;
        desired_height_ = 0.5;

        std::string twist_publisher_topic = "/cmd_vel";

        if (env_ == "sim") {

            twist_publisher_topic = "/cmd_vel_01";

            odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            robot_prefix_ + "/odom",
            10, // QoS history depth
            std::bind(&GtsamCppTestNode::odom_subscribe_callback, this, std::placeholders::_1));

            load_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "load/odom",
            10, // QoS history depth
            std::bind(&GtsamCppTestNode::load_odom_subscribe_callback, this, std::placeholders::_1));

        } else {
            odom_real_subscription_ = this->create_subscription<phasespace_msgs::msg::Rigids>(
            "/phasespace_rigids",
            10, // QoS history depth
            std::bind(&GtsamCppTestNode::odom_real_subscribe_callback, this, std::placeholders::_1));

            rclcpp::SensorDataQoS sensor_data_qos;
            sensor_data_qos.keep_last(1);
            sensor_data_qos.deadline(rclcpp::Duration(0/*s*/, (int)1e9/100 /*ns*/));

            robot_pose_publisher_ = this->create_publisher<motion_capture_tracking_interfaces::msg::NamedPoseArray>("/poses", sensor_data_qos);
        }

        land_ = this->create_subscription<std_msgs::msg::Bool>(
            "land",
            10, // QoS history depth
            std::bind(&GtsamCppTestNode::land_subscribe_callback, this, std::placeholders::_1));

        // force_subsc_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        //     "/ft_sensor_topic",
        //     10,
        //     std::bind(&GtsamCppTestNode::force_subscribe_callback, this, std::placeholders::_1)
        // );

        k_ = 0;

        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(twist_publisher_topic, 10);
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 0.1 seconds = 100 milliseconds
        std::bind(&GtsamCppTestNode::timer_callback, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr load_odom_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<phasespace_msgs::msg::Rigids>::SharedPtr odom_real_subscription_;

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_subsc_;

    rclcpp::Publisher<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr robot_pose_publisher_;

    int k_;
    std::vector<double> position_;
    std::vector<double> angles_;
    std::vector<double> load_position_;
    std::vector<double> load_angles_;
    Eigen::Matrix3d rot_l_, rot_r_;

    Vector2 last_control_;
    Vector1 last_tension_;

    bool is_pulling_;
    double desired_height_;
    double pos_error_;

    std::optional<rclcpp::Time> last_robot_time_, last_load_time_;

    // Assuming 'rigid' is a struct or class with the following members:
    // float x, y, z, qx, qy, qz, qw;
    struct RigidBody {
        float x, y, z;
        float qx, qy, qz, qw;
    };

    // Function to convert quaternion to rotation matrix
    Eigen::Matrix3f quatToRotMatrix(const Eigen::Quaternionf& q) {
        return q.normalized().toRotationMatrix();
    }

    // Function to convert rotation matrix to quaternion
    Eigen::Quaternionf rotMatrixToQuat(const Eigen::Matrix3f& R) {
        return Eigen::Quaternionf(R);
    }

    // PhaseSpace-related. Doesn't yet work!
    void convertPose(const phasespace_msgs::msg::Rigid& rigid, double& x, double& y, double& z, 
                    double& qx, double& qy, double& qz, double& qw) {

        // position conversion (millimeters to meters)
        x = rigid.x / 1000.0f;    // left -> east (adjust symbol according to actual situation)
        y = -rigid.z / 1000.0f;   // front -> north (adjust symbol according to actual situation)
        z = rigid.y / 1000.0f;    // up -> up
        
      
        // quaternion attitude conversion
        Eigen::Quaternionf q_ps(rigid.qw, rigid.qx, rigid.qy, rigid.qz);

        // create coordinate transformation matrix (PhaseSpace -> ENU)
        Eigen::Matrix3f T_ps_to_enu;
        T_ps_to_enu << 1, 0,  0,
                    0, 0, -1,
                    0, 1,  0;

        // convert quaternion to rotation matrix
        Eigen::Matrix3f R_ps = quatToRotMatrix(q_ps);

        // apply coordinate transformation: R_enu = T * R_ps * T^T
        Eigen::Matrix3f R_enu = T_ps_to_enu * R_ps * T_ps_to_enu.transpose();
        
        // convert back to quaternion
        Eigen::Quaternionf q_enu = rotMatrixToQuat(R_enu);

        // Eigen stores quaternions as (x, y, z, w)
        qx = q_enu.x();
        qy = q_enu.y();
        qz = q_enu.z();
        qw = q_enu.w();
    
    }

    // PhaseSpace-related. Doesn't yet work!
    void odom_real_subscribe_callback(const phasespace_msgs::msg::Rigids msg) {
        // 2 for attached drone, 6 for another one
        int robot_id = 6;
        int load_id = 3;

        for (const auto& m : msg.rigids) {
            if (m.id == robot_id || m.id == load_id) {

                double x, y, z, qx, qy, qz, qw;
                convertPose(m, x, y, z, qx, qy, qz, qw);

                if (m.id == robot_id) {
                    double prevx = position_[0];
                    double prevy = position_[1];
                    double prevz = position_[2];

                    position_[0] = x;
                    position_[1] = y;
                    position_[2] = z;

                    // Quaternion to Euler conversion
                    tf2::Quaternion q(
                        qx,
                        qy,
                        qz,
                        qw);

                    tf2::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);

                    angles_[0] = roll;
                    angles_[1] = pitch;
                    angles_[2] = yaw;

                    //cout << "Rotation angles: " << roll << ' ' << pitch << ' '<< yaw << endl;


                    rot_r_ = get_rot_from_euler(roll, pitch, yaw);
                    // Eigen::Vector3d v_local;
                    // v_local << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z; 
                    // Eigen::Vector3d v_world = rot_r_ * v_local;

                    const auto& current_time = this->now();
                    if (!last_robot_time_.has_value()) {
                        RCLCPP_INFO(this->get_logger(), "First position received. Initializing.");
                        last_robot_time_ = current_time;
                        return;
                    }

                    rclcpp::Duration dt_duration = current_time - last_robot_time_.value();
                    double dt_seconds = dt_duration.seconds();

                    position_[3] = (position_[0] - prevx) / dt_seconds;
                    position_[4] = (position_[1] - prevy) / dt_seconds;
                    position_[5] = (position_[2] - prevz) / dt_seconds;
                    last_robot_time_ = current_time;

                    //cout << "Robot pos: " << position_[0] << ' '<< position_[1] << ' '<<position_[2] << ' '<<position_[3] << ' '<<position_[4] << ' '<<position_[5] << endl; 
                } else {    
                    double prevx = load_position_[0];
                    double prevy = load_position_[1];
                    double prevz = load_position_[2];

                    load_position_[0] = x;
                    load_position_[1] = y;
                    load_position_[2] = z;

                    // Quaternion to Euler conversion
                    tf2::Quaternion q(
                        qx,
                        qy,
                        qz,
                        qw);

                    tf2::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);

                    load_angles_[0] = roll;
                    load_angles_[1] = pitch;
                    load_angles_[2] = yaw;

                    rot_l_ = get_rot_from_euler(roll, pitch, yaw);

                    const auto& current_time = this->now();
                    if (!last_load_time_.has_value()) {
                        RCLCPP_INFO(this->get_logger(), "First position received. Initializing.");
                        last_load_time_ = current_time;
                        return;
                    }

                    rclcpp::Duration dt_duration = current_time - last_load_time_.value();
                    double dt_seconds = dt_duration.seconds();

                    load_position_[3] = (load_position_[0] - prevx) / dt_seconds;
                    load_position_[4] = (load_position_[1] - prevy) / dt_seconds;
                    load_position_[5] = (load_position_[2] - prevz) / dt_seconds;
                    last_load_time_ = current_time;
                    //cout << "Load position: " << x <<' ' << y << ' ' <<z << ' ' << roll << ' '<< pitch << ' '<< yaw << endl; 
                }
            }
        }
    }

    void force_subscribe_callback(const geometry_msgs::msg::WrenchStamped msg) {
        Vector2 e(2);
        e << position_[0] - load_position_[0], position_[1] - load_position_[1];
        double norm = e.norm();
        e = e / norm;
        Vector3 f(3);
        f << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z;
        
        double t = e(0) * f(0) + e(1) * f(1);
        cout << "Tension value: " << t << endl;
        cout << "     Distance: " << norm << " height: " << position_[2] << endl;
    }

    void land_subscribe_callback(const std_msgs::msg::Bool msg)
    {
        cout << "Factor Graph MPC: LANDING" << std::endl;
        timer_->cancel();
        geometry_msgs::msg::Twist new_cmd_msg;
        //convert_robot_velocity_to_local_frame(0.0, 0.0, -0.25, rot_r_, new_cmd_msg, 0.25);
            
        twist_publisher_->publish(new_cmd_msg);
        return; 
    }

    void odom_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_(msg, position_, angles_, rot_r_);
    }

    void load_odom_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_(msg, load_position_, load_angles_, rot_l_);
    }
    
    void timer_callback()
    {
        geometry_msgs::msg::Twist new_cmd_msg;

        // If not flying --> takeoff
        if (!is_pulling_) {
            new_cmd_msg.linear.z = 0.1;
            //convert_robot_velocity_to_local_frame(0.0, 0.0, 0.1, rot_r_, new_cmd_msg, 0.15);
            //cout << "commanding velocity: " << new_cmd_msg.linear.x << ' ' << new_cmd_msg.linear.y << ' ' << new_cmd_msg.linear.z << endl;
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

        //Now the drone is in "pulling" mode, control with MPC
        auto next_velocity = get_next_velocity_();

        last_control_ << next_velocity[2], next_velocity[3];
        last_tension_ << next_velocity[4];
        record_metrics(load_position_, {position_}, {last_tension_[0]}, {}, {{last_control_[0], last_control_[1], 0.0}}, pos_error_);

        convert_robot_velocity_to_local_frame(
            next_velocity[0], next_velocity[1], desired_height_ - position_[2], 
            rot_r_, new_cmd_msg, 0.2
        );
        
        cout << "Next velocity: " << new_cmd_msg.linear.x << ' ' << new_cmd_msg.linear.y << endl;
        cout << "Next control: " << last_control_[0] << ' ' << last_control_[1] << endl;
        cout << "Next tension: " << last_tension_[0] << endl;
        
        twist_publisher_->publish(new_cmd_msg);
    }

    std::vector<double> get_next_velocity_() {

        Vector4 initial_robot_state(
            position_[0], position_[1],
            position_[3], position_[4]
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

        // record_datapoint(
        //     {
        //         {"init_load", initial_load_state},
        //         {"init_robot", initial_robot_state},
        //         {"goal_load", final_load_goal},
        //         {"height" , position_[2]},
        //         {"last_u" , last_control_},
        //     }
        // );

        cout << "Cur load position: " << initial_load_state[0] <<  ", " << initial_load_state[1] << ", " << initial_load_state[2] <<  ", " << initial_load_state[3] << std::endl;
        cout << "Cur robot position: " << initial_robot_state[0] <<  ", " << initial_robot_state[1] << ", " << initial_robot_state[2] <<  ", " << initial_robot_state[3] <<  std::endl;
        cout << "Cur robot height: " << position_[2] << std::endl;
        cout << "Next load position: " << final_load_goal[0] <<  ", " << final_load_goal[1] << std::endl;

        auto executor = FactorExecutorFactory::create("sim", initial_load_state, initial_robot_state, final_load_goal, position_[2], last_control_, last_tension_, {}, {});
        map<string, double> factor_errors = {};
        double pos_error = 0.0;
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
