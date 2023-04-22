from math import inf
import rclpy
import numpy as np

from thymio_sim.task_2_controller import ControllerThymioNode as Task2Controller, ControllerState

import sys
from thymio_sim.utils import pose_distance

class ControllerThymioNode(Task2Controller):
    def __init__(self):
        super().__init__()
        self.after_rotation_pose = None
        
    def update_callback(self):

        if self.state == ControllerState.TASK_2_DONE:
            self.wait_for(2, ControllerState.TASK_3_ROTATING)
            self.starting_orientation = self.pose3d_to_2d(self.odom_pose)[-1]
        elif self.state == ControllerState.TASK_3_ROTATING:
            if not self.opposing_wall():
                self.send_move_command(rotation_vel=np.deg2rad(25))
            else:
                self.state = ControllerState.TASK_3_FORWARD
                self.send_move_command(0, 0)
                self.after_rotation_pose = self.pose3d_to_2d(self.odom_pose)
        elif self.state == ControllerState.TASK_3_FORWARD:
            current_pose = self.pose3d_to_2d(self.odom_pose)
            if pose_distance(self.after_rotation_pose, current_pose) <= 2.35:
                self.send_move_command(forward_vel=.5)
            else:
                self.send_move_command(forward_vel=0)
                self.state = ControllerState.TASK_3_DONE
                self.done_future.set_result(True)
        else:
            super().update_callback()

    def opposing_wall(self):
        prox_readings = self.get_stable_proximities()
        return abs(prox_readings['rear_left'] - prox_readings['rear_right']) <= 0.1 and (prox_readings['rear_left'] != inf or prox_readings['rear_right'] != inf)






            

        

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
