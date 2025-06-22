import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import math
import tf_transformations

import logging
import time
from typing import Optional

import cvxpy as cp
import dccp
import numpy as np


class MpcController(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        self.declare_parameter('desired_height', 0.7) # 45 degrees
        self.desired_height = self.get_parameter('desired_height').value
        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value

        self.max_abs_speed = 3.0
        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        self.load_position = [0.0, 0.0, 0.0]
        self.load_angles = [0.0, 0.0, 0.0]
        self.path = None
        self.is_pulling = False
        self.start_time = None
        self.mpc_horizon = 15
        self.dt = 0.02
        self.n_u = 2
        self.n_state = 4

        # Create MPC optimization problem
        self.problem = self.make_problem(1, self.n_state, self.mpc_horizon, self.dt)

        # Subscribers
        self.path_subscriber = self.create_subscription(
            Path, '/desired_path', self.path_subscribe_callback, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_subscribe_callback, 10)
        self.load_odom_subscriber = self.create_subscription(
            Odometry, 'load/odom', self.load_odom_subscribe_callback, 10)

        # Publisher
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('MPC controller node started.')

    def make_problem(self, n_drones, n_state, n, dt):

        # First 2 for drone, last 2 for load
        x0 = cp.Parameter(n_state, name="x0") 
        x_target = cp.Parameter(n * 2, name="x_target")
        direction = cp.Parameter(2, name="direction")
        
        # We can only optimize the load state
        Q = np.eye(2) * 10 * dt

        # cost matrix for control, linear velocity
        R = np.eye(n_drones * n_u) * 8 * dt

        # linear velocities of the drones
        num_controls_per_step = n_drones * n_u
        self.num_controls_per_step = num_controls_per_step
        u = cp.Variable(num_controls_per_step * n, name="u")
        num_states_per_step = n_state
        x = cp.Variable(num_states_per_step * (n + 1), name="x")

        self.last_x = np.zeros((num_states_per_step*(n+1)))
        self.last_u = np.zeros((num_controls_per_step * n))

        constraints = [x0[:num_states_per_step] == x[:num_states_per_step]]

        cost = 0.0

        for i in range(n):
            
            x_start_idx = num_states_per_step * i
            x_end_idx = x_start_idx + num_states_per_step
            u_start_idx = num_controls_per_step * i
            u_end_idx = u_start_idx + num_controls_per_step

            next_target = x_target[x_start_idx // 2:x_end_idx // 2]
            
            cur_speed = u[u_start_idx:u_end_idx]

            next_state_drone = x[x_start_idx:x_start_idx + 2] + dt * cur_speed
            next_state_load = x[x_start_idx:x_start_idx + 2] + dt * cur_speed - direction

            constraints.append(next_state_drone == x[x_end_idx:x_end_idx + 2])
            constraints.append(next_state_load == x[x_end_idx + 2:x_end_idx + 4])

            cost += cp.quad_form(next_state_load - next_target, Q) + cp.quad_form(u[u_start_idx:u_end_idx], R)

        constraints.append(cp.abs(u) <= self.max_abs_speed)
        prob = cp.Problem(cp.Minimize(cost), constraints)
        print("dccp:")
        print(dccp.is_dccp(prob))
        return prob

    def timer_callback(self):

        new_cmd_msg = Twist()

        # If not flying --> takeoff
        if not self.is_pulling:
            new_cmd_msg.linear.z = 0.5
            if self.position[2] > self.desired_height:
                # stop going up if height is reached
                new_cmd_msg.linear.z = 0.0
                self.is_pulling = True
                self.start_time = time.time()
                self.get_logger().info('Takeoff completed')
            
            self.twist_publisher.publish(new_cmd_msg)
            return
        
        # Now the drone is in "pulling" mode, control with MPC

        # First, we're keeping the drone on the same height for now
        error = self.desired_height - self.position[2]
        new_cmd_msg.linear.z = error

        # Get the reference points for the MPC horizon
        trajectory_points = self.get_next_points(self.mpc_horizon)

        # Get the current state of the drone and the load
        x0 = np.array([self.position[0], self.position[1], self.load_position[0], self.load_position[1]])
        
        # Calculate the distance vector between the drone and the load
        initial_relative_vec = x0[:2] - x0[2:]
        initial_link_direction = initial_relative_vec / np.linalg.norm(initial_relative_vec)
        x_target = trajectory_points

        x0_param = self.problem.param_dict["x0"]
        x_target_param = self.problem.param_dict["x_target"]
        dir_param = self.problem.param_dict["direction"]
        x0_param.value = x0
        x_target_param.value = x_target
        dir_param.value = initial_link_direction

        u_var = self.problem.var_dict["u"]
        u_var.value = self.last_u
        x_var = self.problem.var_dict["x"]
        x_var.value = self.last_x

        self.problem.solve(verbose=False, warm_start=True, enforce_dpp=True)

        if self.problem.status != "optimal":
            logging.error(f"error in opto, {self.problem.status}")

        next_control = np.array(u_var.value[:self.num_controls_per_step])

        self.last_u = u_var.value
        self.last_x = x_var.value

        new_cmd_msg.linear.x = next_control[0]
        new_cmd_msg.linear.y = next_control[1]

        self.twist_publisher.publish(new_cmd_msg)

    def get_next_points(self, n: int):
        num_p = 500
        cur_time = time.time()
        sec_from_beginning = cur_time - self.start_time
        start_point = int(sec_from_beginning % num_p) # num_p is number of points
        res = []

        for i in range(start_point, min(start_point + n, num_p)):
            p = self.path.poses[i].pose.position
            res.append(p.x)
            res.append(p.y)

        if start_point + n > num_p:
            for i in range(0, num_p - start_point - n):
                p = self.path.poses[i].pose.position
                res.append(p.x)
                res.append(p.y)
        return res

    def odom_subscribe_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[0] = euler[0]
        self.angles[1] = euler[1]
        self.angles[2] = euler[2]

    def load_odom_subscribe_callback(self, msg):
        self.load_position[0] = msg.pose.pose.position.x
        self.load_position[1] = msg.pose.pose.position.y
        self.load_position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.load_angles[0] = euler[0]
        self.load_angles[1] = euler[1]
        self.load_angles[2] = euler[2]

    def path_subscribe_callback(self, msg):
        self.path = msg

def main(args=None):
    rclpy.init(args=args)
    mpc_controller = MpcController()
    rclpy.spin(mpc_controller)
    mpc_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()