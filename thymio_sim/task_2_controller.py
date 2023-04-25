from collections import deque
from time import time
import rclpy
from thymio_sim.thymio_controller import ThymioController, ControllerState
import numpy as np

import sys
from thymio_sim.utils import mkrot, mktransl
from statistics import mean

class ControllerThymioNode(ThymioController):
    def __init__(self):
        super().__init__('controller_thymio_node')
        
        self.n_updates= 0
        self.step = 0

        self.target_rotation_hist = deque(maxlen=5)
        self.stabilization_start = None
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

        self.proximity_init(["center", "center_left", "center_right"])
        
    def update_callback(self):
        if self.state == ControllerState.FORWARD_with_SENSOR:
            self.send_move_command(forward_vel=.05, rotation_vel=0)
            proximity_readings = self.stable_proximities()
            if np.min(list(proximity_readings.values())) <= .07:
                self.state = ControllerState.STABILIZING
                self.stop_moving()
        elif self.state == ControllerState.STABILIZING:
            if self.stabilization_start == None:
                stable_readings = self.stable_proximities()
                rotation = self.compute_rotation(dist_R=stable_readings['center_right'],
                                             dist_L=stable_readings['center_left'],
                                             dist_M=stable_readings['center'])
                self.get_logger().info(f"Rotation: {rotation} ({np.rad2deg(rotation)})", throttle_duration_sec=1)
                self.stabilization_start = time()
                self.get_logger().info("STATE: STABILIZING")

            elif time() - self.stabilization_start >= 3:
                self.stabilization_start = None
                self.state = ControllerState.ALIGNING_START
        elif self.state == ControllerState.ALIGNING_START:
            stable_readings = self.stable_proximities()
            rotation = self.compute_rotation(dist_R=stable_readings['center_right'],
                                             dist_L=stable_readings['center_left'],
                                             dist_M=stable_readings['center'])
            self.alignment_rotation = rotation
            self.target_rotation_hist.appendleft(rotation)
            self.state = ControllerState.ROTATING
            self.get_logger().info("STATE: ROTATING")
            self.get_logger().info(f"Rotation: {rotation} ({np.rad2deg(rotation)})")
        elif self.state == ControllerState.ROTATING:
            target_rotation = mean(list(self.target_rotation_hist))
            if abs(target_rotation) > np.deg2rad(3):
                angular_speed  = np.sign(target_rotation) * np.deg2rad(10)
                self.send_move_command(forward_vel=0, rotation_vel=angular_speed)
                stable_readings = self.stable_proximities()
                rotation = self.compute_rotation(dist_R=stable_readings['center_right'],
                                                dist_L=stable_readings['center_left'],
                                                dist_M=stable_readings['center'])
                self.target_rotation_hist.appendleft(rotation)
                self.get_logger().info(f"Rotation: {rotation} ({np.rad2deg(rotation)})")

            else:
                self.get_logger().info("STATE: STOPPED")
                self.stop_moving()
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
