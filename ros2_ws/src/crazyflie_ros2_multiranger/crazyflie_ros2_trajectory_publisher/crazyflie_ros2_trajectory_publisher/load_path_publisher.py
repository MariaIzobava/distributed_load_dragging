import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class LoadPathPublisher(Node):
    def __init__(self):
        super().__init__('load_path_publisher')

        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.path.header.stamp = self.get_clock().now().to_msg()

        self.load_odom_subscriber = self.create_subscription(
        Odometry, 'load/odom', self.publish_path, 10)

        self.publisher_ = self.create_publisher(Path, 'true_load_path', 10)
        self.get_logger().info('Load true path publisher node started.')

    def publish_path(self, msg):
        self.path.header.stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.path.header.stamp
        pose.pose.position.x = msg.pose.pose.position.x
        pose.pose.position.y = msg.pose.pose.position.y
        pose.pose.position.z = 0.0 # Assuming 2D path
        self.path.poses.append(pose)

        self.publisher_.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    path_publisher = LoadPathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()