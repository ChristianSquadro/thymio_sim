import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

import sys

class ControllerThymioNode(Node):
    def __init__(self):
        super().__init__('controller_thymio_node')
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
        # Create attributes to keep track of trajectory in terms of timestamps and steps to follow
        self.n_updates= 0
        self.step = 0

        # the "8" trajectory of the thymio
        self.trajectory= {
        "linear": [0.08, 0.08, 0.08, 0.08],
        "angular": [-0.35, 0.55, 0.45, -0.60],
        "dt": [60, 170, 80, 180],
        "step": [0, 1, 2, 3]
        }
                    
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which Thymio should be controlled.
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
        
        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
    
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2
        
    def update_callback(self):

        if self.trajectory["dt"][self.step] != self.n_updates:
            #open-loop controller
            cmd_vel = Twist() 
            cmd_vel.linear.x  = self.trajectory["linear"][self.step] # [m/s]
            cmd_vel.angular.z = self.trajectory["angular"][self.step] # [rad/s]
            
            # Publish the command and increment the counter
            self.vel_publisher.publish(cmd_vel)
            self.n_updates = self.n_updates + 1
        else:
            #restart the counter for the next step
            self.n_updates=0
            #follow the trajectory with the next step
            if len(self.trajectory["step"]) != self.step + 1:
                self.step=self.trajectory["step"][self.step+1]
            else:
            #at the end of the trajectory repeat it
                self.step=0

def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerThymioNode()
    node.start()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
