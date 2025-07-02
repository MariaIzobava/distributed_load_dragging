import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(Path, 'desired_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path) # Publish every second (or once)
        self.get_logger().info('Trajectory publisher node started.')

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom' # Or 'odom', depending on your setup
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Example: A simple circular path
        radius = 2.0
        num_points = 2500
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points - math.pi / 2.0
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = -radius * math.cos(angle)
            pose.pose.position.y = radius * math.sin(angle) + 2
            pose.pose.position.z = 0.0 # Assuming 2D path
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)
        #self.get_logger().info('Published desired path.')

        # For a static path, you might only publish it once and then stop the timer
        #self.timer.cancel() # Uncomment if you only want to publish once

def main(args=None):
    rclpy.init(args=args)
    path_publisher = TrajectoryPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()