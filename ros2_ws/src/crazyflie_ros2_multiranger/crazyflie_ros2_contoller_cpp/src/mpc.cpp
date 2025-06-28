#include "rclcpp/rclcpp.hpp"

// Include the necessary GTSAM headers
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <vector>

class GtsamCppTestNode : public rclcpp::Node
{
public:
    GtsamCppTestNode() : Node("gtsam_cpp_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "GTSAM C++ test node has started.");

        // --- Perform the GTSAM checks ---
        try
        {
            // 1. Create a simple GTSAM object to prove it works
            // A Pose2 represents a 2D position (x, y) and orientation (theta)
            gtsam::Pose2 initial_pose(1.0, 2.0, M_PI / 2.0);
            RCLCPP_INFO(this->get_logger(), "Successfully created a GTSAM Pose2 object.");

            // 2. Create an empty factor graph
            gtsam::NonlinearFactorGraph graph;
            RCLCPP_INFO(this->get_logger(), "Successfully created an empty NonlinearFactorGraph.");

            // // 3. Add a prior factor to the graph
            // // This factor represents a belief about the initial pose
            auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.05));
            graph.add(gtsam::PriorFactor<gtsam::Pose2>(1, initial_pose, prior_noise));
            RCLCPP_INFO(this->get_logger(), "Successfully added a prior factor to the graph. Graph size: %zu", graph.size());

            RCLCPP_INFO(this->get_logger(), "GTSAM C++ test node has finished its checks successfully!");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "An exception occurred during GTSAM operations: %s", e.what());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // The node's logic runs in the constructor for this simple test case.
    // For a real node, you would typically spin the node to keep it running.
    auto node = std::make_shared<GtsamCppTestNode>();
    rclcpp::shutdown();
    return 0;
}
