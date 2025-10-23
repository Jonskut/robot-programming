# Ex 5

## 1.

***instructions for launching world***
```bash
cd turtlebot3_ws/src
export TURTLEBOT3_MODEL=burger
source ~/turtlebot3_ws/install/setup.sh
source /usr/share/gazebo/setup.sh
ros2 launch turtlebot3_gazebo ex5.launch.py 
```

> [!WARNING]
> ros2 run turtlebot3_gazebo searches for WORLD AND .py SCRIPT from /install!!!
> So need to either put from src/ to there or colcon build!

## 2.

/home/jone/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/ex5.world

***Slam node*** (in new terminal)
```bash
cd turtlebot3_ws/src
export TURTLEBOT3_MODEL=burger
source ~/turtlebot3_ws/install/setup.sh
source /usr/share/gazebo/setup.sh
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

```
***Teleop*** (in new terminal)
```bash
cd turtlebot3_ws/src
export TURTLEBOT3_MODEL=burger
source ~/turtlebot3_ws/install/setup.sh
source /usr/share/gazebo/setup.sh
ros2 run turtlebot3_teleop teleop_keyboard
```


***save map*** (in new terminal)
```bash
cd turtlebot3_ws/src
export TURTLEBOT3_MODEL=burger
source ~/turtlebot3_ws/install/setup.sh
source /usr/share/gazebo/setup.sh
ros2 run nav2_map_server map_saver_cli -f ~/map
```

![map](./rviz-map.png)

![map from ros command](./map.pgm)
(Go with gf in neovim (no leader key))
![Turtlebot lua confit](/home/jone/turtlebot3_ws/src/turtlebot3/turtlebot3_cartographer/config/turtlebot3_lds_2d.lua)
**start with max_range=3.5**
![start](./map-start-large.png)
**start with max_range=1.0**, Couldn't even recognize anything at first so had to move a bit
![small](./map-start-small.png)

## 3.

```bash
cd turtlebot3_ws/src
export TURTLEBOT3_MODEL=burger
source ~/turtlebot3_ws/install/setup.sh
source /usr/share/gazebo/setup.sh
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

- 2D pose Rviz -> drag arrow from robot to facing instructions
- Teleop to narrow down position
- Nav2 Goal Rviz

```bash
jone@jone-ubuntu:~/turtlebot3_ws/src$ source check_sensors_nav2.sh
=== Checking active Navigation2 nodes and their sensor subscriptions ===

--- Node: /amcl ---
    /map: nav_msgs/msg/OccupancyGrid
    /scan: sensor_msgs/msg/LaserScan
    /tf: tf2_msgs/msg/TFMessage

--- Node: /controller_server ---
    /odom: nav_msgs/msg/Odometry

--- Node: /planner_server ---

--- Node: /bt_navigator ---
    /odom: nav_msgs/msg/Odometry
    /tf: tf2_msgs/msg/TFMessage
    /tf_static: tf2_msgs/msg/TFMessage

--- Node: /map_server ---
/map_server
    /map: nav_msgs/msg/OccupancyGrid
    /map_server/transition_event: lifecycle_msgs/msg/TransitionEvent
    /map_server/change_state: lifecycle_msgs/srv/ChangeState
    /map_server/describe_parameters: rcl_interfaces/srv/DescribeParameters```bash
cd turtlebot3_ws/src
export TURTLEBOT3_MODEL=burger
source ~/turtlebot3_ws/install/setup.sh
source /usr/share/gazebo/setup.sh
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```


    /map_server/get_available_states: lifecycle_msgs/srv/GetAvailableStates
    /map_server/get_available_transitions: lifecycle_msgs/srv/GetAvailableTransitions
    /map_server/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /map_server/get_parameters: rcl_interfaces/srv/GetParameters
    /map_server/get_state: lifecycle_msgs/srv/GetState
    /map_server/get_transition_graph: lifecycle_msgs/srv/GetAvailableTransitions
    /map_server/list_parameters: rcl_interfaces/srv/ListParameters
    /map_server/load_map: nav2_msgs/srv/LoadMap
    /map_server/map: nav_msgs/srv/GetMap
    /map_server/set_parameters: rcl_interfaces/srv/SetParameters
    /map_server/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically

--- Node: /local_costmap/local_costmap ---
    /scan: sensor_msgs/msg/LaserScan

--- Node: /global_costmap/global_costmap ---
    /map: nav_msgs/msg/OccupancyGrid
    /scan: sensor_msgs/msg/LaserScan

--- Node: /smoother_server ---

=== Summary of key sensor topics ===
If you see /scan, /odom, /imu, or /tf under 'Subscribed topics', that sensor is in use.

[burger.yaml](/home/jone/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/humble/burger.yaml)

default:
![image-20251022223719559](/home/jone/snap/typora/108/.config/Typora/typora-user-images/image-20251022223719559.png)
with  inflation_radius: 0.2
      cost_scaling_factor: 1.0
![hihi](./2025-10-23_00:01:19.png)

Those change the local costmap: "halo radius" and "halo gradient".

## 4.

**Cartographer**

- Range data, IMU/Odometer, fixed frame pose 
- Local SLAM -> Global SLAM -> fixed frame pose 
- scan-matching (local to global)

**ORB-SLAM** 

- Visual (needs camera) + depth (optional)
- Feature-based
- Needs good lighting

**RTAB-Map** 
- Visual, rgbd, lidar (flexible)
- bag-of-words loop closure (is image new location or old)


> [!WARNING]
> Cartographer has to be running! (see above for instructions)
> Also ex5c.world (closed room)

```bash
cd ~/turtlebot3_ws/src
export TURTLEBOT3_MODEL=burger
source ~/turtlebot3_ws/install/setup.sh
source /usr/share/gazebo/setup.sh
ros2 launch turtlebot3_gazebo ex5c.launch.py # Separate terminal 
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true # Separate terminal
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true autostart:=true map:=~/empty_map.yaml # Separate terminal
python3 run automap.py # Separate terminal
```

Frontier exploration, but stupid: picks a random frontier (no clustering or scoring)

https://awabot.com/en/autonomous-exploration-method-frontiers/

https://github.com/robo-friends/m-explore-ros2/tree/main

1: G ← Occupancy grid
 2: **Do**
 3: F = DetectBorder (G)
 4: D = SelectBorder (F)
 5: **While F** is not empty
 6: **End While**

- A good way to check completion would be to check unknown cells against total cells (-1 vs all)  (this updates dynamically?? full about 65-75% for my world)

- no frontiers found for N tries
- If enclosed occupancy grid (how to check this?)

![image-20251023055026363](/home/jone/snap/typora/108/.config/Typora/typora-user-images/image-20251023055026363.png)

![image-20251023060022617](/home/jone/snap/typora/108/.config/Typora/typora-user-images/image-20251023060022617.png)

![image-20251023060843577](/home/jone/snap/typora/108/.config/Typora/typora-user-images/image-20251023060843577.png)

