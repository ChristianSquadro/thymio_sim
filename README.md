## Launch instructions
You could file single launch file to run single compulsory task and the compulsory.launch.xml which runs the last one. Moreover, there
is a bonus task launch file. In order to run the nodes open three terminals: 

-In the first one write:
  1. cd dev_ws
  2. colcon build --packages-up-to thymio_sim
	
-In the second one:
  3. cd ~/apps/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu22_04
  4. bash coppeliaSim.sh
  5.1 For the compulsory tasks: load scene wall_v2.ttt and start the simulation (with real-time mode enabled)
  5.2 For the bonus tasks: load HW2scene.ttt and start the simulation (with real-time mode enabled)

-In the third one (the launch files automatically start the bridge):
  6. cd dev_ws
  7. source ./install/setup.sh
  8.1 For the compulsory tasks: ros2 launch thymio_sim compulsory.launch.xml thymio_name:={ROBOT_NAME}
  8.2 For the bonus task: ros2 launch thymio_sim bonus.launch.xml thymio_name:={ROBOT_NAME}

## Task 1

## Task 2

## Task 3

## Task 4
