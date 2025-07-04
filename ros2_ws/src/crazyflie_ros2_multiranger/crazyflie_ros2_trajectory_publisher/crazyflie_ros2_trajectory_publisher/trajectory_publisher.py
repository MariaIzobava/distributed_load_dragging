import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.started = False
        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value
        self.start_subsc_ = self.create_subscription(Odometry, robot_prefix + '/odom', self.start_cb, 10)
        self.publisher_ = self.create_publisher(Path, 'desired_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path) # Publish every second (or once)
        self.get_logger().info('Trajectory publisher node started.')

    def start_cb(self, msg):
        if self.started:
            return
        
        if msg.pose.pose.position.z >= 0.49:
            self.started = True
            self.start_time = round(time.time() * 1000)


    def publish_path(self):
        if not self.started:
            return
        path_msg = Path()
        path_msg.header.frame_id = 'odom' # Or 'odom', depending on your setup
        path_msg.header.stamp = self.get_clock().now().to_msg()

        cur_time = round(time.time() * 1000)

        # Example: A simple circular path
        radius = 2.0
        num_points = 5000

        cur_points = int((cur_time - self.start_time) / 100 + 20 ) % num_points
        for i in range(cur_points):
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