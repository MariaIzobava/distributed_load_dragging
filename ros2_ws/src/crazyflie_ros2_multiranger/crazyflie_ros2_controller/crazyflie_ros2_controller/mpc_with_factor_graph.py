import rclpy
from rclpy.node import Node
import gtsam
import numpy as np


class MpcGraphController(Node):
    def __init__(self):
        super().__init__(mpc_graph_controller)
        self.get_logger().info("GTSAM availability is ok")

        initial_pose = gtsam.Pose2(1.0, 2.0, np.pi / 2)
        graph = gtsam.NonlinearFactorGraph()

        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))
        graph.add(gtsam.PriorFactorPose2(1, initial_pose, prior_noise))

def main(args=None):
    rclpy.init(args=args)
    mpc_controller = MpcGraphController()
    rclpy.spin(mpc_controller)
    mpc_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()