**Task 1**

![image-20251002150448828](/home/jone/snap/typora/106/.config/Typora/typora-user-images/image-20251002150448828.png)

3 

	- ros2 node list, ros2 topic list -t
	- ros2 topic echo /turtle1/cmd_vel

Concatenation with ;

ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.0}}"; \
sleep 2; \
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.57}}"

Example script: 

source /home/jone/robo_400/exercise_answers/ex3_1.sh

**Task 2**

2.1.3

/cmd_vel

2.1.4.

Couldnt find script, but this does the same

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

**Task 3**

~/turtlebot3_ws/src/

export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo empty_world.launch.py

python3 motion_control.py (normal p-controller)

python3 motion_control_2.py (pure pursuit, for the recorded paths)



**Ways to improve robot control:**

- Use a differential drive model for more accurate kinematics (ros2 has this)

- Separate control actions (rotate first, then drive, then final orientation).
  - Annoying oscillation of approach angle with current system
- Improve control system:
  - PID instead of only P
  - Dynamic parameter tuning
  - Other control methods
- Use established controller libraries (e.g., ROS2 Navigation Stack).
- Make it work in worlds with obstacles:
  - Use ros2 navigation stack (supports slam, global/local path planning + following, recovery behaviors, obstacle avoidance, ...)
  - subscribe to lidar
  - Plan path with Djikstra or A* etc.
  -  Stop and turn away if obstacle is very close
- Apply sensor fusion for better state estimation (odometry is prone to drift).
- Add safety checks for acceleration, jerk, and stopping distance.
- Confirm goal achievement with dwell or repeated checks.
- This system only works in an empty world, add path generation

motion_control_2.py has pure pursuit!

**Task 4**

ONLY motion_control_2.py has error publishing

ros2 bag record /odom -o <output>

python3 rosbag_extract_path.py

python3 motion_control_2.py replay path_bag_squarish.csv #lookahead_distance = ?

python3 motion_control_2.py replay path_random.csv #lookahead_distance = 0.1

python3 motion_control_2.py replay path_bag_circle.csv #lookahead_distance = 0.3





ros2 run turtlebot3_teleop teleop_keyboard

ros2 run rqt_plot rqt_plot



STOP: 

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

python3 motion_control_2.py replay path_squarish.csv --record bag_pp_squarish

python3 visualize_path.py path_circle.csv path_bag_pp_circle.csv

