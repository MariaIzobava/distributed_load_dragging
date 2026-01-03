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
    GtsamCppTestNode() : BaseMpc(false, true, "gtsam_cpp_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "MPC with height with GTSAM node has started.");

        last_control_ << 0.0, 0.0, 0.0;
        last_tension_ << 0;
    }

    Vector3 last_control_;
    Vector1 last_tension_;

    FactorExecutorResult run_factor_executor() {

        Vector6 initial_robot_state(
            position_[0][0], position_[0][1], position_[0][2],
            position_[0][3], position_[0][4], position_[0][5]
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

        log_cur_state(position_, load_position_, {}, {next_p[0], next_p[1]});

        auto executor = FactorExecutorFactory::create("sim", initial_load_state, initial_robot_state, final_load_goal, position_[0][2], last_control_, last_tension_, {}, {});
        map<string, double> factor_errors = {};
        auto res = executor->run(factor_errors, pos_error_);

        last_control_ << res[0].controls[0], res[0].controls[1], res[0].controls[2];
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
