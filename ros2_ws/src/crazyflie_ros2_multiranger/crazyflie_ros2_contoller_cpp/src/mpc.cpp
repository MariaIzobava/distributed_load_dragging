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
    GtsamCppTestNode() : BaseMpc(false, false, "gtsam_cpp_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "MPC with GTSAM node has started.");

        this->declare_parameter<std::string>("env", "sim");
        std::string env_ = this->get_parameter("env").as_string();

        last_control_ << 0.0, 0.0;
        last_tension_ << 0;

        // IMPORTANT: pass this topic name to BaseMpc so that for REAL use case
        // we use the correct topic name
        std::string twist_publisher_topic = "/cmd_vel";

        if (env_ == "sim") {

            twist_publisher_topic = "/cmd_vel_01";

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
    }

private:

    rclcpp::Subscription<phasespace_msgs::msg::Rigids>::SharedPtr odom_real_subscription_;
    rclcpp::Publisher<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr robot_pose_publisher_;

    Vector2 last_control_;
    Vector1 last_tension_;

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
                    double prevx = position_[0][0];
                    double prevy = position_[0][1];
                    double prevz = position_[0][2];

                    position_[0][0] = x;
                    position_[0][1] = y;
                    position_[0][2] = z;

                    // Quaternion to Euler conversion
                    tf2::Quaternion q(
                        qx,
                        qy,
                        qz,
                        qw);

                    tf2::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);

                    angles_[0][0] = roll;
                    angles_[0][1] = pitch;
                    angles_[0][2] = yaw;

                    //cout << "Rotation angles: " << roll << ' ' << pitch << ' '<< yaw << endl;


                    rot_[0] = get_rot_from_euler(roll, pitch, yaw);
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

                    position_[0][3] = (position_[0][0] - prevx) / dt_seconds;
                    position_[0][4] = (position_[0][1] - prevy) / dt_seconds;
                    position_[0][5] = (position_[0][2] - prevz) / dt_seconds;
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

                    rotl_ = get_rot_from_euler(roll, pitch, yaw);

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
        e << position_[0][0] - load_position_[0], position_[0][1] - load_position_[1];
        double norm = e.norm();
        e = e / norm;
        Vector3 f(3);
        f << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z;
        
        double t = e(0) * f(0) + e(1) * f(1);
        cout << "Tension value: " << t << endl;
        cout << "     Distance: " << norm << " height: " << position_[0][2] << endl;
    }

    FactorExecutorResult run_factor_executor() {

        Vector4 initial_robot_state(
            position_[0][0], position_[0][1],
            position_[0][3], position_[0][4]
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

        log_cur_state(
            position_, 
            load_position_, 
            {}, 
            {next_p[0], next_p[1]}
        );

        auto executor = FactorExecutorFactory::create("sim", initial_load_state, initial_robot_state, final_load_goal, position_[0][2], desired_heights_, last_control_, last_tension_, {}, {});
        map<string, double> factor_errors = {};
        auto res = executor->run(factor_errors, pos_error_);

        last_control_ << res[0].controls[0], res[0].controls[1];
        last_tension_ << res[0].tension;

        return res;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GtsamCppTestNode>());
    rclcpp::shutdown();
    return 0;
}
