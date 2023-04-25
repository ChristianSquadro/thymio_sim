from asyncio import Future
from enum import Enum
from math import inf
from collections import deque
from statistics import mean
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import tf_transformations


class ControllerState(Enum):
    FORWARD_with_SENSOR = 1,
    ROTATING = 2,
    ROTATING_RANDOM = 10,
    GENERATING_ANGLE= 11,
    ALIGNING_START = 5,
    TASK_2_DONE = 3,
    STABILIZING = 4,
    FORWARD_with_ODOMETRY = 6,
    TASK_3_ROTATING = 7,
    TASK_3_FORWARD = 8,
    TASK_3_DONE = 9,
    WAITING = -1,


class ProximityModule():

    def __init__(self, keys, node : Node, upper_clamp_val = inf) -> None:
        self.proximity_keys = keys
        self.proximity_readings = {
            key : deque(maxlen=15) for key in self.proximity_keys
        }
        self.node = node
        self.upper_clamp_val = upper_clamp_val

        def make_closure(sensor_name):
            return lambda msg: self.proximity_callback(sensor_name, msg)

        self.proximity_subs = {
            key : self.node.create_subscription(Range, 'proximity/' + key, make_closure(key), 1)
            for key in self.proximity_keys
        }

    def proximity_callback(self, sensor : str, msg : Range):
        to_append = self.upper_clamp_val if msg.range == -1 or msg.range > 0.138 else msg.range
        self.proximity_readings[sensor].appendleft(to_append)

    def __del__(self):
        for sub in self.proximity_subs.values():
            sub.destroy()
    
    def last_proximities(self):
        return {k : list(v)[0] if len(list(v)) > 0 else inf for k, v in self.proximity_readings.items()}
    
    def stable_proximities(self):
        result = {}
        for k,v in self.proximity_readings.items():
            if len(list(v)) > 0:
                result[k] = mean(list(v))
            else:
                result[k] = inf
        return result


class OdometryModule:

    def __init__(self, node : Node):
        self.odom_pose = None
        self.odom_velocity = None
        self.odom_subscriber = node.create_subscription(Odometry, 'odom', self.odom_callback, 1)
        self.node = node
    
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
        
        _, __, yaw = tf_transformations.euler_from_quaternion(quaternion, axes='sxyz')
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2
    
    def last_pose(self):
        return self.pose3d_to_2d(self.odom_pose)
    
    def __del__(self):
        self.odom_subscriber.destroy()


class ThymioController(Node):
    
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odometry_module : OdometryModule = OdometryModule(self)
        self.proximity_module = None
        self.state = None

    def proximity_init(self, keys, clamp_val = inf):
        if not self.proximity_module == None:
            del self.proximity_module
        self.proximity_module = ProximityModule(keys, self, upper_clamp_val=clamp_val)

    def last_proximities(self):
        if not self.proximity_module == None:
            return self.proximity_module.last_proximities()
        else:
            return False
    
    def stable_proximities(self):
        if not self.proximity_module == None:
            return self.proximity_module.stable_proximities()
        else:
            return False

        
    def send_move_command(self, forward_vel = 0.0, rotation_vel = 0.0):
        cmd_vel = Twist() 
        cmd_vel.angular.z  = float(rotation_vel)
        cmd_vel.linear.x  = float(forward_vel)
        self.vel_publisher.publish(cmd_vel)

    def pose(self):
        return self.odometry_module.last_pose()

    def send_move_command(self, forward_vel = 0.0, rotation_vel = 0.0):
        cmd_vel = Twist() 
        cmd_vel.angular.z  = float(rotation_vel)
        cmd_vel.linear.x  = float(forward_vel)
        self.vel_publisher.publish(cmd_vel)

    def stop_moving(self):
        self.send_move_command(0, 0)

    def start(self):
        self.timer = self.create_timer(1/60, self.update_callback)
        self.done_future= Future()
        self.state = ControllerState.FORWARD_with_SENSOR
        return self.done_future
    
    def stop(self):
        self.stop_moving()
        self.timer.cancel()
        del self.proximity_module

