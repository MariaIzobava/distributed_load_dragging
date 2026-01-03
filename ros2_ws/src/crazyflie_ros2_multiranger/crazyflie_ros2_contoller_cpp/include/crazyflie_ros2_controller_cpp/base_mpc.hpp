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
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> twist_publisher_;
  
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr load_odom_subscription_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path::SharedPtr path_;
  std::chrono::high_resolution_clock::time_point start_time_;

  std::vector<double> load_position_;
  std::vector<double> load_angles_;
  std::vector<std::vector<double> > position_;
  std::vector<std::vector<double> > angles_;
  std::vector<double> desired_heights_;
  std::vector<Eigen::Matrix3d> rot_;
  Eigen::Matrix3d rotl_;

  int robot_num_;
  int is_pulling_;
  double pos_error_;

  explicit BaseMpc(
    const std::string& metrics_file,
    bool load_ori, 
    bool robot_height, 
    const std::string& datapoints_file,
    const std::string& node_name)
  : rclcpp::Node(node_name), metrics_file_(metrics_file), load_ori_(load_ori), robot_height_(robot_height), datapoints_file_(datapoints_file) {

    this->declare_parameter<std::string>("robot_prefix", "/crazyflie");
    this->declare_parameter<int>("robot_num", 1);

    robot_num_ = this->get_parameter("robot_num").as_int();
    std::string robot_prefix_ = this->get_parameter("robot_prefix").as_string();

    load_position_.resize(6);
    load_angles_.resize(4);
    position_.resize(robot_num_);
    angles_.resize(robot_num_);
    rot_.resize(robot_num_);

    for (int i = 0; i < robot_num_; i++) {
        position_[i].resize(6);
        angles_[i].resize(6);
        
        double sgn = (i % 2 == 0) ? 1: -1;
        desired_heights_.push_back(0.5 + sgn * 0.1 * i);
    }

    for (int i = 0; i < robot_num_; i++) {
        string suffix = "_0" + to_string(i+1) + "/odom";
        odom_subscription_.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
        robot_prefix_ + suffix,
        10, // QoS history depth
        [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->odom_subscribe_callback(msg, i);
        }));
    }

    load_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "load/odom",
          10, // QoS history depth
          [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->load_odom_subscribe_callback(msg); 
          });

    path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
          "/desired_path",
          10, // QoS history depth
          std::bind(&BaseMpc::path_subscribe_callback, this, std::placeholders::_1));

    land_ = this->create_subscription<std_msgs::msg::Bool>(
          "land",
          10, // QoS history depth
          [this](const std_msgs::msg::Bool msg) {
                this->land_subscribe_callback(msg); 
          });

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 0.1 seconds = 100 milliseconds
        [this]() {
            this->timer_callback(); 
        });

    for (int i = 0; i < robot_num_; i++) {
        std::string topic_name = "/cmd_vel_0" + to_string(i+1);
        twist_publisher_.push_back(this->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10));
    }

    initialize_metrics_file();
  }

  virtual FactorExecutorResult run_factor_executor() = 0;

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

  void log_cur_state(
    const std::vector<std::vector<double> >& drones, 
    const std::vector<double>& load,
    const std::vector<double>& load_angles,
    const std::vector<double>& final_load_points
    ) {
      cout << "\n===================\n";
      cout << "Cur load position: " << join(load, ", ") << std::endl;

      if (load_angles.size() > 0) {
        cout << "Cur load angles: " << join(load_angles, ", ") << std::endl;
      }

      cout << "Next load position: " << join(final_load_points, ", ") << std::endl;

      for (int i = 0; i < drones.size(); i++) {
        cout << "Cur robot " << i+1 << " position: " << join(drones[i], ", ") << std::endl;
      }
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

  void odom_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg, int k)
  {
      odom(msg, position_[k], angles_[k], rot_[k]);
  }

  void load_odom_subscribe_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
      odom(msg, load_position_, load_angles_, rotl_);
  }

  void odom(const nav_msgs::msg::Odometry::SharedPtr msg, std::vector<double>& position, std::vector<double>& angles, Eigen::Matrix3d& rotation_m) {
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

  void land_subscribe_callback(const std_msgs::msg::Bool msg)
  {
      cout << "Factor Graph MPC: LANDING" << std::endl;
      timer_->cancel();
      geometry_msgs::msg::Twist new_cmd_msg;
      new_cmd_msg.linear.z = -0.2;
          
      for (int i = 0; i < robot_num_; i++) {
        twist_publisher_[i]->publish(new_cmd_msg);
      }
  }

  void timer_callback()
  {
      if (!is_ready_to_pull() || path_ == NULL) {
          return;
      }

      // Now the drone is in "pulling" mode, control with MPC
      auto res = run_factor_executor();

      std::vector<double> load_state = {
          load_position_[0], load_position_[1], load_angles_[2], 
          load_position_[3], load_position_[4], load_angles_[3]
      };
      record_metrics(load_state, position_, res, pos_error_);

      for (int i = 0; i < robot_num_; i++) {
          geometry_msgs::msg::Twist new_cmd_msg;
          convert_robot_velocity_to_local_frame(
              res[i].drone_vel[0], res[i].drone_vel[1], res[i].drone_vel[2], 
              rot_[i], new_cmd_msg, 0.2
          );
          
          cout << "Next velocity " << i + 1 << ": " << new_cmd_msg.linear.x << ' ' << new_cmd_msg.linear.y << endl;
          twist_publisher_[i]->publish(new_cmd_msg);
      }
  }

  bool is_ready_to_pull() {
    if (is_pulling_ != (1 << robot_num_) - 1) {
        for (int i = 0; i < robot_num_; i++) {
            geometry_msgs::msg::Twist new_cmd_msg;
            new_cmd_msg.linear.z = 0.1;

            if (position_[i][2] > desired_heights_[i]) {
                new_cmd_msg.linear.z = 0.0;
                is_pulling_ |= (1 << i);
            }

            twist_publisher_[i]->publish(new_cmd_msg);
        }
        if (is_pulling_ == (1 << robot_num_) - 1) {
            start_time_ = std::chrono::high_resolution_clock::now();
            RCLCPP_INFO(this->get_logger(), "Takeoff completed");
        }
        return false;
    }
    return true;
  }

  void record_metrics(
    const std::vector<double>& load_state,
    const std::vector<std::vector<double>>& robots,
    const FactorExecutorResult& executor_res,
    double pos_error
    ) {
      // Open the file in append mode (std::ios::app)
      std::ofstream file(metrics_file_, std::ios::app);
      if (!file.is_open()) {
          std::cerr << "Error: Could not open the file " << metrics_file_ << std::endl;
          return;
      }

      std::vector<double> row;
      double currentTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      row.push_back(currentTime);
      row.push_back(load_state[0]);
      row.push_back(load_state[1]);

      for (int i = 0; i < executor_res.size(); i++) {
        for (int j = 0; j < 3; j++) {
          row.push_back(robots[i][j]);
        }
        for (int j = 0; j < 3; j++) {
          row.push_back(executor_res[i].drone_vel[j]);
        }
        row.push_back(executor_res[i].tension);

        for (int j = 0; j < 3; j++) {
          row.push_back(executor_res[i].controls[j]);
        }

        if (load_ori_) {
          Vector4 robot_state_v(
            robots[i][0], robots[i][1],
            robots[i][3], robots[i][4]
          );
          Vector6 load_state_v(
              load_state[0], load_state[1], load_state[2],
              load_state[3], load_state[4], load_state[5]
          );
          CableVectorHelper h(robot_state_v, load_state_v);
          row.push_back(h.ap_x);
          row.push_back(h.ap_y);
        }
      }
      row.push_back(pos_error);

      std::string line = join(row, ",");
      file << line << std::endl;
      file.close();   
  }

  void convert_robot_velocity_to_local_frame(double x, double y, double z, const Eigen::Matrix3d& rotation_m, geometry_msgs::msg::Twist& msg, double limit = 0.2) {
    Eigen::Vector3d v_local;
    v_local << x, y, z; 

    Eigen::Vector3d v_world = rotation_m.transpose() * v_local;

    msg.linear.x = max(min(v_world(0), limit), -limit);
    msg.linear.y = max(min(v_world(1), limit), -limit);
    msg.linear.z = max(min(v_world(2), limit), -limit); 
  }

  void initialize_metrics_file() {

    std::vector<string> num_drones = {"one_drone", "two_drones", "three_drones", "four_drones"};
    std::string filename = num_drones[robot_num_ - 1];
    filename += robot_height_ ? "_with_height" : "";
    filename += load_ori_ ? "_with_ori" : "";
    filename += ".csv";

    metrics_file_ += filename;
    cout << "Metrics will be stored in: " << metrics_file_ << endl;
    
    // Clean up and initialize metrics file with correct headers
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
    
    for (int i = 1; i <= robot_num_; i++) {
      char buffer[250];
      snprintf(buffer, sizeof(buffer), drone_tmpl.c_str(), i, i, i, i, i, i, i, i, i, i);
      std::string drone_str(buffer);
      header_file << "," << drone_str;

      if (load_ori_) {
        header_file << "," << "attachment_point" << i << "_x,attachment_point" << i << "_y";
      }
    }
    header_file << ",pos_error";
    header_file << std::endl;
    header_file.close();
  }

  // A function to join elements of a vector into a delimeter-separated string
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
  bool load_ori_;  // If True then load orientation is considered in the optimization problem
  bool robot_height_;  // If True then robot height is considered in the optimization problem
  int kk_ = 0;
  bool have_prev_angle = false;
};

#endif // CRAZYFLIE_ROS2_CONTROLLER_CPP_BASE_MPC_HPP_