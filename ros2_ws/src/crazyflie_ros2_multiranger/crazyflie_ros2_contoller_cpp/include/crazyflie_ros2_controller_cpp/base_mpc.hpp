#ifndef CRAZYFLIE_ROS2_CONTROLLER_CPP_BASE_MPC_HPP_
#define CRAZYFLIE_ROS2_CONTROLLER_CPP_BASE_MPC_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <Eigen/Geometry>
#include <gtsam/base/Vector.h>

#include "factor_graph_lib/common.hpp"

#include <vector>
#include <cstdio>
#include <nlohmann/json.hpp>
#include <math.h>

using json = nlohmann::json;


class BaseMpc : public rclcpp::Node
{
public:
  virtual ~BaseMpc() {}

protected:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  nav_msgs::msg::Path::SharedPtr path_;
  std::chrono::high_resolution_clock::time_point start_time_;

  explicit BaseMpc(
    const std::string& metrics_file,
    bool load_ori, 
    bool robot_height, 
    const std::string& datapoints_file,
    const std::string& node_name)
  : rclcpp::Node(node_name), metrics_file_(metrics_file), load_ori_(load_ori), robot_height_(robot_height), datapoints_file_(datapoints_file) {

    // Should be initialized via init_robot_num
    num_robots_ = 0;

    path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
          "/desired_path",
          10, // QoS history depth
          std::bind(&BaseMpc::path_subscribe_callback, this, std::placeholders::_1));
  }

  void init_robot_num(int robot_num) {
    num_robots_ = robot_num;

    cout << "here\n";

    std::vector<string> num_drones = {"one_drone", "two_drones", "three_drones", "four_drones"};
    std::string filename = num_drones[num_robots_ - 1];
    filename += robot_height_ ? "_with_height" : "";
    filename += load_ori_ ? "_with_ori" : "";
    filename += ".csv";

    metrics_file_ += filename;

    cout << "here\n";

    cout << "Metrics will be stored in: " << metrics_file_ << endl;
    // Clean up and initialize metrics file with correct headers
    initialize_metrics_file();
  }

  std::vector<double> get_next_points() {
      //int num_p = 4000; // num_p is number of points
      
      // Get current time
      auto cur_time = std::chrono::high_resolution_clock::now();
      
      // Calculate seconds from the beginning
      auto cur_millisec = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - start_time_);

      int i = (cur_millisec.count() / 100 + 10);
      cout << i << ' ' << path_->poses.size() << std::endl;
      std::vector<double> ans;
      ans.push_back(path_->poses[i].pose.position.x);
      ans.push_back(path_->poses[i].pose.position.y);
      ans.push_back(path_->poses[i].pose.orientation.x); // atan2(sin(path_->poses[i].pose.orientation.x), cos(path_->poses[i].pose.orientation.x))
      return ans; 
  }

  void odom_(const nav_msgs::msg::Odometry::SharedPtr msg, std::vector<double>& position, std::vector<double>& angles, Eigen::Matrix3d& rotation_m) {
      position[0] = msg->pose.pose.position.x;
      position[1] = msg->pose.pose.position.y;
      position[2] = msg->pose.pose.position.z;

      // Quaternion to Euler conversion
      tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      angles[0] = roll;
      angles[1] = pitch;

      if (have_prev_angle) {
        if ((angles[2] < -3.01 && yaw > 2.7) || (angles[2] < -3.14)) {
          angles[2] = -M_PI - (M_PI - yaw);
        } else {
          angles[2] = yaw;
        }
      } else {
        angles[2] = yaw;
        have_prev_angle = true;
      }
      // Transforming velocity from local frame to world frame
      rotation_m = get_rot_from_euler(roll, pitch, yaw);
      Eigen::Vector3d v_local;
      v_local << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z; 

      Eigen::Vector3d v_world = rotation_m * v_local;
      position[3] = v_world(0);
      position[4] = v_world(1);
      position[5] = v_world(2);
  }

  void record_metrics(
    const std::vector<double>& load,
    const std::vector<std::vector<double>>& robots, 
    std::vector<double> tensions,
    std::vector<int> ap_directions,
    std::vector<std::vector<double>> controls = {},
    double pos_error = 0.0) {
      // Open the file in append mode (std::ios::app)
      std::ofstream file(metrics_file_, std::ios::app);
      if (!file.is_open()) {
          std::cerr << "Error: Could not open the file " << metrics_file_ << std::endl;
          return;
      }

      std::vector<double> row;
      double currentTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      row.push_back(currentTime);
      row.push_back(load[0]);
      row.push_back(load[1]);

      for (int i = 0; i < num_robots_; i++) {
        for (int j = 0; j < 6; j++) {
            row.push_back(robots[i][j]);
        }
        row.push_back(tensions[i]);
        row.push_back(controls[i][0]);
        row.push_back(controls[i][1]);
        row.push_back(controls[i][2]);

        if (load_ori_) {

          Vector4 robot_state(
            robots[i][0], robots[i][1],
            robots[i][3], robots[i][4]
          );
          Vector6 load_state(
              load[0], load[1], load[2], 
              load[3], load[4], load[3]
          );
          CableVectorHelper h(robot_state, load_state);
          row.push_back(h.ap_x);
          row.push_back(h.ap_y);
        }
        row.push_back(pos_error);
      }

      std::string line = join(row, ",");
      file << line << std::endl;
      file.close();   
  }

  void record_datapoint(const std::map<std::string, json>& data) {
    kk_++;
    if (kk_ % 10 == 0) {
        std::ifstream file(datapoints_file_);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open the JSON file!" << std::endl;
        }

        json jsonData;
        try {
            file >> jsonData;
        } catch (json::parse_error& e) {
            std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        }
        file.close();

        jsonData.push_back(data);

        std::ofstream output_file(datapoints_file_);
        if (output_file.is_open()) {
            output_file << jsonData.dump(4);
            
            std::cout << "JSON object successfully written to " << datapoints_file_ << std::endl;
        } else {
            std::cerr << "Error: Could not open file " << datapoints_file_ << " for writing." << std::endl;
        }
    }
  }

  void convert_robot_velocity_to_local_frame(double x, double y, double z, const Eigen::Matrix3d& rotation_m, geometry_msgs::msg::Twist& msg, double limit = 0.2) {
    Eigen::Vector3d v_local;
    v_local << x, y, z; 

    Eigen::Vector3d v_world = rotation_m.transpose() * v_local;

    msg.linear.x = max(min(v_world(0), limit), -limit);
    msg.linear.y = max(min(v_world(1), limit), -limit);
    msg.linear.z = max(min(v_world(2), limit), -limit); 
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

private:

  void path_subscribe_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
      path_ = msg;
  }

  void initialize_metrics_file() {
      // First clean up the file if there is anything in it
      std::ofstream ofs;
      ofs.open(metrics_file_, std::ofstream::out | std::ofstream::trunc);
      ofs.close();
      
      std::ofstream header_file(metrics_file_);
      if (!header_file.is_open()) {
          std::cerr << "Error: Could not create the file " << metrics_file_ << std::endl;
          return;
      }

      // Now construct the required headers
      header_file << "timestamp,actual_load_x,actual_load_y";
      std::string drone_tmpl = "drone%d_pose_x,drone%d_pose_y,drone%d_pose_z,drone%d_velocity_x,drone%d_velocity_y,drone%d_velocity_z,cable%d_tension,drone%d_control_x,drone%d_control_y,drone%d_control_z";
      
      for (int i = 1; i <= num_robots_; i++) {
        char buffer[250];
        snprintf(buffer, sizeof(buffer), drone_tmpl.c_str(), i, i, i, i, i, i, i, i, i, i);
        std::string drone_str(buffer);
        header_file << "," << drone_str;

        if (load_ori_) {
          header_file << "," << "attachment_point" << i << "_x,attachment_point" << i << "_y";
        }
        header_file << ",pos_error";
      }
      cout << "here\n";
      header_file << std::endl;
      header_file.close();
  }

  // A simple function to join elements of a vector into a comma-separated string
  template <typename T>
  std::string join(const std::vector<T>& values, const std::string& delimiter) {
      std::string result = "";
      if (values.empty()) {
          return result;
      }
      result += std::to_string(values[0]);
      for (size_t i = 1; i < values.size(); ++i) {
          result += delimiter + std::to_string(values[i]);
      }
      return result;
  }

  std::string metrics_file_;
  std::string datapoints_file_;
  int num_robots_;
  bool load_ori_;  // If True then load orientation is considered in the optimization problem
  bool robot_height_;  // If True then robot height is considered in the optimization problem
  int kk_ = 0;
  bool have_prev_angle = false;
};

#endif // CRAZYFLIE_ROS2_CONTROLLER_CPP_BASE_MPC_HPP_