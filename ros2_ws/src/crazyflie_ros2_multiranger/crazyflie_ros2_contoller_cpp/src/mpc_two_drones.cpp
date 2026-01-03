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
        RCLCPP_INFO(this->get_logger(), "MPC for two drones with GTSAM node has started.");

        last_u1_ << 0.0, 0.0;
        last_u2_ << 0.0, 0.0;
    }

private:
    Vector2 last_u1_;
    Vector2 last_u2_;

    FactorExecutorResult run_factor_executor() {
        Vector4 initial_robot1_state(
            position_[0][0], position_[0][1],
            position_[0][3], position_[0][4]
        );
        Vector4 initial_robot2_state(
            position_[1][0], position_[1][1],
            position_[1][3], position_[1][4]
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

        // record_datapoint(
        //     {
        //         {"init_load", initial_load_state},
        //         {"init_robot1", initial_robot1_state},
        //         {"init_robot2", initial_robot2_state},
        //         {"goal_load", final_load_goal},
        //         {"height1" , position1_[2]},
        //         {"height2" , position2_[2]},
        //     }
        // );

        log_cur_state(position_, load_position_, {}, {next_p[0], next_p[1]});

        auto executor = FactorExecutorFactory::create("sim", initial_load_state, initial_robot1_state, initial_robot2_state, final_load_goal, position_[0][2], position_[1][2], desired_heights_, last_u1_, last_u2_, {}, {});
        map<string, double> factor_errors = {};
        auto res = executor->run(factor_errors, pos_error_);

        last_u1_<< res[0].controls[0], res[0].controls[1];
        last_u2_<< res[1].controls[0], res[1].controls[1];

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
