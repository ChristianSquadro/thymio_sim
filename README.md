## Launch instructions
We created a compulsory.launch.xml file which will launch the most advanced mandatory task we implemented (task 3). Additionally, you can find a single launch file for every task (`task_{number}.launch.xml`), including the bonus task, which we call task 4.
Every launch file will also run the Thymio bridge and exposes one single parameter `thymio_name` which defaults to `thymio0`.  
For instance, running the bonus task is done like so:
```shell
ros2 launch thymio_sim task_4.launch.xml thymio_name:=thymio0
```

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
