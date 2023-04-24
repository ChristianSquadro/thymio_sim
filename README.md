## Launch instructions
You can find a single launch file to run each mandatory task and the compulsory.launch.xml file that runs the most advanced mandatory task. In addition, there is a launch file for the bonus task. To run the third and the bonus task, three terminals must be opened:

-In the first one write:
  1. cd dev_ws
  2. colcon build --packages-up-to thymio_sim
	
-In the second one (load manually Ros2InterfaceHelper if not already present in the scene):
  3. cd ~/apps/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu22_0
  4. bash coppeliaSim.sh
  5.1. For the third task: load scene wall_v2.ttt and start the simulation (with real-time mode enabled)
  5.2. For the bonus task: load HW2scene.ttt and start the simulation (with real-time mode enabled)

-In the third one (the launch file automatically starts the ROS bridge):
  6. cd dev_ws
  7. source ./install/setup.sh
  8.1. For the third task: ros2 launch thymio_sim compulsory.launch.xml thymio_name:={ROBOT_NAME}
  8.2. For the bonus task: ros2 launch thymio_sim bonus.launch.xml thymio_name:={ROBOT_NAME}

## Task 1
We were able to implement the task correctly. Considering that the controller is open-loop, we did not expect
accuracy in the executed trajectory.

## Task 2
We were able to implement the task correctly. We found a strange behavior when the wall is little inclined, where thymio adopted a less accurate position in front of the wall than when the wall is more inclined. This result makes us suspect that the poor accuracy is due to the noise coming from the proximity sensors.

## Task 3

## Task Bonus
