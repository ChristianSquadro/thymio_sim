from math import inf
import rclpy
import numpy as np

from thymio_sim.task_2_controller import ControllerThymioNode as Task2Controller, ControllerState
from geometry_msgs.msg import Twist

from collections import deque
from math import inf
import rclpy
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

import sys


class SteeringController():

    def __init__(self, K, I, D, prox_max_val=0.138, prox_min_val=.0215, steer_max_angle=np.pi / 36) -> None:
        self.K = K
        self.I = I
        self.D = D
        self.last_e = 0
        self.prox_max = prox_max_val
        self.angle_max = steer_max_angle
        self.prox_range = prox_max_val - prox_min_val
        self.prox_min = prox_min_val

    def step(self, prox_l, prox_r, prox_cl, prox_cr):
        norm_l = 1 - (prox_l - self.prox_min) / self.prox_range
        norm_r = 1 - (prox_r - self.prox_min) / self.prox_range
        norm_cl = 1 - (prox_cl - self.prox_min) / self.prox_range
        norm_cr = 1 - (prox_cr - self.prox_min) / self.prox_range

        norm_l = .3 * norm_l + .7 * norm_cl
        norm_r = .3 * norm_r + .7 * norm_cr
        diff = norm_r - norm_l
        e = diff * self.angle_max

        y = self.K * e
        self.last_e = e
        return y


class ControllerThymioNode(Task2Controller):
    def __init__(self):
        super().__init__()
        self.after_rotation_pose = None
        self.target_rotation = None

        self.new_proximity_keys = ["center_left",
                                   "center", "center_right", "left", "right"]
        self.new_proximity_readings = {
            key: deque(maxlen=15) for key in self.new_proximity_keys
        }

        def make_closure(sensor_name):
            return lambda msg: self.new_proximity_callback(sensor_name, msg)

        self.new_proximity_subs = {
            key: self.create_subscription(
                Range, 'proximity/' + key, make_closure(key), 1)
            for key in self.new_proximity_keys
        }

        self.prox_min_val = .0215
        self.prox_max_val = .138
        self.steer_controller = SteeringController(
            4, 0, 0, prox_max_val=self.prox_max_val, prox_min_val=self.prox_min_val, steer_max_angle=np.deg2rad(30))

    def new_proximity_callback(self, sensor: str, msg: Range):
        to_append = self.prox_max_val if msg.range == - \
            1 or msg.range > self.prox_max_val else msg.range
        self.new_proximity_readings[sensor].appendleft(to_append)

    def get_last_proximities(self):
        return {k: list(v)[0] if len(list(v)) > 0 else inf for k, v in self.new_proximity_readings.items()}

    def update_callback(self):

        if self.state == ControllerState.FORWARD_with_SENSOR:
            cmd_vel = Twist()
            cmd_vel.linear.x = .5  # [m/s]
            proximity_readings = self.get_last_proximities()

            left_prox = proximity_readings['left']
            right_prox = proximity_readings['right']
            center_left_prox = proximity_readings['center_left']
            center_right_prox = proximity_readings['center_right']
            steering = self.steer_controller.step(
                left_prox, right_prox, center_left_prox, center_right_prox)

            if not np.isnan(steering):
                if steering != 0:
                    cmd_vel.linear.x /= 5
                cmd_vel.angular.z = steering
            if proximity_readings['center'] <= .035:
                cmd_vel.linear.x = 0.0

            if cmd_vel.linear.x == 0.0 and cmd_vel.angular.z == 0.0 or (np.isclose(left_prox, self.prox_min_val) and np.isclose(right_prox, self.prox_min_val)):
                # Somehow we got stuck, nudge the robot a bit and hope for the best
                self.random_rotation = np.deg2rad(
                    90) * np.sign(np.random.random_sample() - .5)
                self.state = ControllerState.ROTATING
            self.vel_publisher.publish(cmd_vel)
        elif self.state == ControllerState.ROTATING:
            proximity_readings = self.get_last_proximities()
            left_prox = proximity_readings['left']
            right_prox = proximity_readings['right']
            if np.isclose(left_prox, 0.138) and np.isclose(right_prox, 0.138):
                self.state = ControllerState.FORWARD_with_SENSOR
            else:
                cmd_vel = Twist()
                cmd_vel.angular.z = self.random_rotation
                self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    # Create an instance of your node class
    node = ControllerThymioNode()
    done = node.start()

    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin_until_future_complete(node, done)
    except KeyboardInterrupt:
        pass

    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
