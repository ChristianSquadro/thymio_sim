## Launch instructions
You can find a single launch file to run each mandatory task and the compulsory.launch.xml file that runs the most advanced mandatory task. In addition, there is a launch file for the bonus task. To run the third and the bonus task, three terminals must be opened:

- In the first one write:
  1. cd dev_ws
  2. colcon build --packages-up-to thymio_sim
	
- In the second one (load manually Ros2InterfaceHelper if not already present in the scene):
  3. cd ~/apps/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu22_0
  4. bash coppeliaSim.sh
  5.1. For the third task: load scene wall_v2.ttt and start the simulation (with real-time mode enabled)
  5.2. For the bonus task: load HW2scene.ttt and start the simulation (with real-time mode enabled)

- In the third one (the launch file automatically starts the ROS bridge):
  6. cd dev_ws
  7. source ./install/setup.sh
  8.1. For the third task: ros2 launch thymio_sim compulsory.launch.xml thymio_name:={ROBOT_NAME}
  8.2. For the bonus task: ros2 launch thymio_sim bonus.launch.xml thymio_name:={ROBOT_NAME}

## Task 1
We were able to implement the task correctly. Considering that the controller is open-loop, we did not expect
accuracy in the executed trajectory. Indeed what happens is that, over time, the Thymio will drift away from its starting position.

## Task 2
We were able to implement the task correctly. The Thymio is generally able to align itself with the wall. Weirdly enough, we see some precision issues arising when the wall is just slightly (< 10 degs) rotated relative to the Thymio. We speculate this is related to the noise coming from sensor readings.

## Task 3
Task 3 was fully accomplished without any implementation issue. The only problems we met were of a technical nature and related to the simulation environment. More precisely, we found out that whenever the wall was spawned pretty far from the Thymio's starting position, the robot's odometry would start accumulating noticeable error. This caused the robot not to stop at the 2 meters mark (sometimes even falling off the scene). The problem was solved by applying the workaround suggested in the forum.  
[Left turn test video](https://drive.switch.ch/index.php/s/HfiSm9u9AyzmSCF)  
[Small turn test video](https://drive.switch.ch/index.php/s/YrZNf7pEkOvxcpi)

## Task Bonus
For this task we were able to get the Thymio to walk around the scene without colliding with any of the walls or obstacles. We did so by implementing a proportional steer controller based on the 4 lateral-front proximity sensors.
We are aware of a specific instance where the robot will fail its task by standing still and doing nothing: whenever the robot finds itself perfectly centered in front of a cilinder (i.e. with opposing lateral-front sensors having the same readings) the robot will just stop in front of the obstacle to avoid colliding with it.  
This issue has a trivial fix but we did not implement it because it was really hard to reproduce the issue and hence test this change.  
On a last note, the robot's logic does not implement navigation. This means that the path the robot follows is exclusively dictated by the obstacles it encounters.  
[Test run video](https://drive.switch.ch/index.php/s/2aOUH0o9SomDoEv)
