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

        self.declare_parameter('traj_type', 'circle')
        self.traj_type = self.get_parameter('traj_type').value

        self.start_subsc_ = self.create_subscription(Odometry, robot_prefix + '/odom', self.start_cb, 10)
        self.publisher_ = self.create_publisher(Path, 'desired_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path) # Publish every second
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

        if (self.traj_type == 'circle'):
            self.publish_circle_()
        elif (self.traj_type == 'sin'):
            self.publish_sin_()
        elif (self.traj_type == 'complex'):
            self.publish_complex_()
        else:
            print('WRONG trajectory type: ', self.traj_type)
            return


    def publish_circle_(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        cur_time = round(time.time() * 1000)

        # Example: A simple circular path
        radius = 2.0
        num_points_per_circle = 4000

        cur_points = int((cur_time - self.start_time) / 100 + 120)
        for i in range(cur_points):
            angle = 2 * math.pi * i / num_points_per_circle - math.pi / 2.0
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = -radius * math.cos(angle)
            pose.pose.position.y = radius * math.sin(angle) + 2
            pose.pose.position.z = 0.0 # Assuming 2D path
            pose.pose.orientation.x = -angle - math.pi / 2.0 # saving orientation
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)


    def publish_sin_(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        cur_time = round(time.time() * 1000)

        step_y = 0.002

        cur_points = int((cur_time - self.start_time) / 100 + 120)
        for i in range(cur_points):
            y = step_y * i
            x = math.sin(y)
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = -x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0 # Assuming 2D path
            pose.pose.orientation.x = 0.0 # saving orientation
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)


    def publish_complex_(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        cur_time = round(time.time() * 1000)

        step_y = 0.001

        cur_points = int((cur_time - self.start_time) / 100 + 120)
        for i in range(cur_points):
            y = step_y * i
            x = math.sin(0.5 * y) + math.sin(0.3 * y) + math.sin(2.4 * y)
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = -x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0 # Assuming 2D path
            pose.pose.orientation.x = 0.0 # saving orientation
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    path_publisher = TrajectoryPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()