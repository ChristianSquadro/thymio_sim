from math import inf
import rclpy
import numpy as np

from thymio_sim.task_2_controller import ControllerThymioNode as Task2Controller, ControllerState
from geometry_msgs.msg import Twist, Pose

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

import sys
from thymio_sim.utils import pose_distance

class ControllerThymioNode(Task2Controller):
    def __init__(self):
        super().__init__()
        self.after_rotation_pose = None
        self.target_rotation= None

        self.new_proximity_keys = ["center_left", "center", "center_right","left", "right"]
        self.new_proximity_readings = {
            key : deque(maxlen=15) for key in self.new_proximity_keys
        }

        def make_closure(sensor_name):
            return lambda msg: self.new_proximity_callback(sensor_name, msg)

        self.new_proximity_subs = {
            key : self.create_subscription(Range, '/thymio0/proximity/' + key, make_closure(key), 1)
            for key in self.new_proximity_keys
        }


    def new_proximity_callback(self, sensor : str, msg : Range):
        to_append = inf if msg.range == -1 or msg.range > 0.138 else msg.range
        self.new_proximity_readings[sensor].appendleft(to_append)

    def get_last_proximities(self):
        return {k : list(v)[0] if len(list(v)) > 0 else inf for k, v in self.new_proximity_readings.items()}
        
    def update_callback(self):

        if self.state == ControllerState.FORWARD_with_SENSOR:
                cmd_vel = Twist() 
                cmd_vel.linear.x  = .5 # [m/s]
                self.vel_publisher.publish(cmd_vel)
                proximity_readings = self.get_last_proximities()
                if np.min(list(proximity_readings.values())) <= .07:
                    self.state = ControllerState.GENERATING_ANGLE

        elif self.state == ControllerState.GENERATING_ANGLE:
                self.get_logger().info("STATE: STOPPED")
                cmd_vel = Twist() 
                cmd_vel.angular.z  = 0.0
                cmd_vel.linear.x  = 0.0
                self.vel_publisher.publish(cmd_vel)
                self.state = ControllerState.ROTATING_RANDOM

                #Generating Random Angle
                rm_rot= [np.deg2rad(30),np.deg2rad(60),np.deg2rad(90),np.deg2rad(120),np.deg2rad(150),
                         np.deg2rad(-30),np.deg2rad(-60),np.deg2rad(-90),np.deg2rad(-120),np.deg2rad(-150)]
                self.rotation = np.random.choice(rm_rot, p=[0.30,0.15,0.04,0.008,0.002, 0.30,0.15,0.04,0.008,0.002])

                #TODO turn to 180 degree if the center sensor has a value greater than both center_right and center_left values,
                # moreover the center_right and center_left measures should be similiar

                #Clip to 180 degree (3,14 radiants)
                pose = self.pose3d_to_2d(self.odom_pose)
                if pose[-1] + self.rotation > 3.14:
                     self.target_rotation = -6.28 + (pose[-1] + self.rotation)
                elif pose[-1] + self.rotation < -3.14:
                     self.target_rotation = 6.28 + (pose[-1] + self.rotation)
                else:
                    self.target_rotation = pose[-1] + self.rotation 

                self.get_logger().info("STATE: ROTATING_RANDOM")
                self.get_logger().info(f"Difference rotation: {pose[-1] + self.rotation} and target rotation:{(self.target_rotation)} )")
        
        elif self.state == ControllerState.ROTATING_RANDOM:
            pose = self.pose3d_to_2d(self.odom_pose)
            if abs(pose[-1] - self.target_rotation) > np.deg2rad(10):
                cmd_vel = Twist() 
                cmd_vel.angular.z  = 7*(np.sign(self.target_rotation) * np.deg2rad(10))
                self.vel_publisher.publish(cmd_vel)
            else:
                self.get_logger().info("STATE: STOPPED")
                cmd_vel = Twist() 
                cmd_vel.angular.z  = 0.0
                cmd_vel.linear.x  = 0.0
                self.target_rotation=0
                self.vel_publisher.publish(cmd_vel)
                self.state = ControllerState.FORWARD_with_SENSOR
    

def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerThymioNode()
    done=node.start()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin_until_future_complete(node, done)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
