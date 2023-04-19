from collections import deque
from math import inf
from time import time
import rclpy
from rclpy.node import Node
import tf_transformations
import numpy as np

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from enum import Enum
from rclpy.task import Future

import sys
from thymio_sim.utils import mkrot, mktransl
from statistics import mean

class ControllerState(Enum):
    FORWARD_with_SENSOR = 1,
    ROTATING = 2,
    ALIGNING_START = 5,
    TASK_2_DONE = 3,
    STABILIZING = 4,
    FORWARD_with_ODOMETRY = 6,
    TASK_3_ROTATING = 7,
    TASK_3_FORWARD = 8,
    TASK_3_DONE = 9,
    WAITING = -1,


class ControllerThymioNode(Node):
    def __init__(self):
        super().__init__('controller_thymio_node')
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
        # Create attributes to keep track of trajectory in terms of timestamps and steps to follow
        self.n_updates= 0
        self.step = 0

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)

        proximity_keys = ["center_left", "center", "center_right", "rear_left", "rear_right"]
        self.proximity_readings = {
            key : deque(maxlen=15) for key in proximity_keys
        }

        def make_closure(sensor_name):
            return lambda msg: self.proximity_callback(sensor_name, msg)

        self.proximity_subs = {
            key : self.create_subscription(Range, '/thymio0/proximity/' + key, make_closure(key), 1)
            for key in proximity_keys
        }

        self.target_rotation_hist = deque(maxlen=5)
        self.stabilization_start = None
        self.theta_rm = 0.6117743042921571
        self.theta_lm = 0.3324939920952909
        self.alignment_target_pose = None
        self.alignment_rotation = None

        self.right_sensor_frame =  mktransl(0.056069172456885794, 0) @ mkrot(-0.5106135853892982)
        self.left_sensor_frame = mktransl(0.056070991608852434,0) @ mkrot(0.47216591557126225) 
        self.center_sensor_frame = mktransl(+5.4436e-02, +2.4643e-06)

        self.right_to_self = np.linalg.inv(self.right_sensor_frame)
        self.left_to_self = np.linalg.inv(self.left_sensor_frame)
        self.center_to_self = np.linalg.inv(self.center_sensor_frame)

        self.wait_start_time = None
        self.after_wait_state = None
        self.wait_duration = None
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
        self.done_future= Future()
        self.state = ControllerState.FORWARD_with_SENSOR
        return self.done_future
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    def proximity_callback(self, sensor : str, msg : Range):
        to_append = inf if msg.range == -1 or msg.range > 0.138 else msg.range
        self.proximity_readings[sensor].appendleft(to_append)

    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist
    
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion, axes='sxyz')
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2
        
    def get_last_proximities(self):
        return {k : list(v)[0] if len(list(v)) > 0 else inf for k, v in self.proximity_readings.items()}
    
    def get_stable_proximities(self):
        result = {}
        for k,v in self.proximity_readings.items():
            if len(list(v)) > 0:
                result[k] = mean(list(v))
            else:
                result[k] = inf
        return result
    
    def get_last_prox_without_noise(self):
        return
    def update_callback(self):
        if self.state == ControllerState.FORWARD_with_SENSOR:
            cmd_vel = Twist() 
            cmd_vel.linear.x  = .5 # [m/s]
            self.vel_publisher.publish(cmd_vel)
            proximity_readings = self.get_last_proximities()
            if np.min(list(proximity_readings.values())) <= .07:
                self.state = ControllerState.STABILIZING
        elif self.state == ControllerState.STABILIZING:
            if self.stabilization_start == None:
                stable_readings = self.get_stable_proximities()
                rotation = self.compute_rotation_tolerant(dist_R=stable_readings['center_right'],
                                             dist_L=stable_readings['center_left'],
                                             dist_M=stable_readings['center'])
                self.get_logger().info(f"Rotation: {rotation} ({np.rad2deg(rotation)})", throttle_duration_sec=1)
                cmd_vel = Twist() 
                cmd_vel.linear.x  = 0.0 # [m/s]
                self.vel_publisher.publish(cmd_vel)
                self.stabilization_start = time()
                self.get_logger().info("STATE: STABILIZING")

            elif time() - self.stabilization_start >= 3:
                self.stabilization_start = None
                self.state = ControllerState.ALIGNING_START
        elif self.state == ControllerState.ALIGNING_START:
            stable_readings = self.get_stable_proximities()
            rotation = self.compute_rotation_tolerant(dist_R=stable_readings['center_right'],
                                             dist_L=stable_readings['center_left'],
                                             dist_M=stable_readings['center'])
            pose = self.pose3d_to_2d(self.odom_pose)
            target_pose = list(pose)
            target_pose[-1] += rotation
            self.alignment_target_pose = target_pose
            self.alignment_rotation = rotation

            self.target_rotation_hist.appendleft(rotation)
            self.state = ControllerState.ROTATING
            self.get_logger().info("STATE: ROTATING")
            self.get_logger().info(f"Rotation: {rotation} ({np.rad2deg(rotation)})")
        elif self.state == ControllerState.ROTATING:
            pose = self.pose3d_to_2d(self.odom_pose)
            target_rotation = mean(list(self.target_rotation_hist))
            if abs(target_rotation) > np.deg2rad(3):
                cmd_vel = Twist() 
                cmd_vel.angular.z  = np.sign(target_rotation) * np.deg2rad(10)
                self.vel_publisher.publish(cmd_vel)
                stable_readings = self.get_stable_proximities()
                rotation = self.compute_rotation_tolerant(dist_R=stable_readings['center_right'],
                                                dist_L=stable_readings['center_left'],
                                                dist_M=stable_readings['center'])
                self.target_rotation_hist.appendleft(rotation)
                self.get_logger().info(f"Rotation: {rotation} ({np.rad2deg(rotation)})")

            else:
                self.get_logger().info("STATE: STOPPED")
                cmd_vel = Twist() 
                cmd_vel.angular.z  = 0.0
                cmd_vel.linear.x  = 0.0
                self.vel_publisher.publish(cmd_vel)
                self.state = ControllerState.TASK_2_DONE
        elif self.state == ControllerState.TASK_2_DONE:
            self.get_logger().info("STATE: STOPPED")
            self.done_future.set_result(True)
        elif self.state == ControllerState.WAITING:
            if time() - self.wait_start_time >= self.wait_duration:
                self.state = self.after_wait_state
                self.wait_duration = None
                self.wait_start_time = None
            
    def compute_rotation(self, dist_R, dist_M, dist_L):
        self.get_logger().info(f"R: {dist_R} L: {dist_L} M: {dist_M}")
        if dist_R < dist_L:
            h2 = dist_R**2 * np.tan(self.theta_rm)**2 / (np.tan(self.theta_rm) ** 2 + 1)
            h = np.sqrt(h2)
            m1 = h / np.tan(self.theta_rm)
            m2 = dist_M - m1
            theta_wr = np.arctan(h / m2)
            print(np.rad2deg(theta_wr))
            return -(np.pi/2 - theta_wr) # CW rotation
        else:
            h2 = dist_L**2 * np.tan(self.theta_lm)**2 / (np.tan(self.theta_lm) ** 2 + 1)
            h = np.sqrt(h2)
            m1 = h / np.tan(self.theta_lm)
            m2 = dist_M - m1
            theta_wr = np.arctan(h / m2)
            print(np.rad2deg(theta_wr))
            return np.pi/2 - theta_wr # CCW rotation
        
    def compute_rotation_tolerant(self, dist_R, dist_M, dist_L):
        r_to_self = self.right_sensor_frame @ np.array([dist_R, 0, 1]).T
        m_to_self = self.center_sensor_frame @ np.array([dist_M, 0, 1]).T
        l_to_self = self.left_sensor_frame @ np.array([dist_L, 0, 1]).T
        if dist_L < dist_R:
            deltas = m_to_self - l_to_self
            angular_coefficient = deltas[0] / -deltas[1]
        else:
            deltas = m_to_self - r_to_self
            angular_coefficient = deltas[0] / -deltas[1]
        self.get_logger().info(f"dR: {dist_R} dC: {dist_M} dL: {dist_L}", throttle_duration_sec=1)
        
        self.get_logger().info(f"R: {r_to_self} C: {m_to_self} L: {l_to_self}", throttle_duration_sec=1)

        return np.arctan(angular_coefficient)
    
    def send_move_command(self, forward_vel = 0.0, rotation_vel = 0.0):
        cmd_vel = Twist() 
        cmd_vel.angular.z  = float(rotation_vel)
        cmd_vel.linear.x  = float(forward_vel)
        self.vel_publisher.publish(cmd_vel)

    def wait_for(self, duration, next_state):
        self.state = ControllerState.WAITING
        self.wait_duration = duration
        self.wait_start_time = time()
        self.after_wait_state = next_state

def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerThymioNode()
    node.get_logger().info("Controller created")
    done=node.start()
    node.get_logger().info("Controller started")
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin_until_future_complete(node, done)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
