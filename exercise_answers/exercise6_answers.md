# Ex 6

Build is so slow: close all other apps and run

```bash
# REMEMBER ALWAYS
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash
export MAKEFLAGS="j -4" # or 1, if still doesnt work
colcon build --executor sequential --mixin debug

```

## Task 1

[rviz tuto](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)

> [!IMPORTANT]
> Robot visualization is both in motion planning AND planningScene
> no 'manipulator' or 'base-link', they have panda starting names, must have updated and tutorial hasn't. This causes issues in a lot of spots but just have to update variable names.

- What functionalities do you need to plan and execute robot end-effector motion?
  - A defined robot (kinematic and dynamic model)
  - A motion planner to solve the IK and path
  - A controller to execute the path
    - IRL these often need feedback, so sensors
  - For executing, a real robot or a simulated one
- How is motion planning via RViz limited, compared to utilizing Python/C++ (see task 2)?
  - No automation/multiple path execution
  - No integration with other software (other ros nodes)
  - No custom logic
  - No changing planner params at runtime
  - Result: Limited options, limited path options no obstacle avoidance, no adding objects to end-effector, etc.
  - So Rviz is just basic GUI motion planning place1 -> place2
- What is the robot’s Null space?
  - The robot's null space is the set of joint variables that don't change the end-effectors current pose.

## Task 2

[rviz tuto](https://moveit.picknik.ai/main/doc/tutorials/your_first_project/your_first_project.html)
Explain the following with knowledge from Task 1 and Task 2 :
- How is planning motions different for robot joint space and Cartesian (or work) space?
  - Joint space: FK, Path is planned with joint variables, path may be unintuitive
  - Cartesian space: IK, Path is defined by end-effector positions. Useful if path of end-effector from start -> finish is important
- Name few different constraints that apply in joint space
  - Limits of joint angles, acceleration, velocity
  - Path of end-effector can't be set (at least easily)
- Name few different constraints that apply in Cartesian (or work) space
  - Not all paths can be satisfied
  - Workspace boundaries
  - Obstacle avoidance (easier with cartesian)
  - Path continuity
  - velocity, acceleration

## Task 3

> [!IMPORTANT]
> Check that when demoing, task3 cpp has been loaded to hello_moveit.cpp

[tutorial](https://moveit.picknik.ai/main/doc/tutorials/visualizing_in_rviz/visualizing_in_rviz.html)
Follow the tutorial commands to demo if this doesnt work:
(remember to activate task3 rviz config from file -> recents)
```bash
cd ~/ws_moveit
source install/setup.bash
ros2 launch moveit2_tutorials demo.launch.py
```
```bash
cd ~/ws_moveit
source install/setup.bash
ros2 run hello_moveit hello_moveit
```

## Task 4

[tuto](https://moveit.picknik.ai/main/doc/tutorials/planning_around_objects/planning_around_objects.html)

```bash
cd ~/ws_moveit
source install/setup.bash
ros2 launch moveit2_tutorials demo.launch.py
```

> [!IMPORTANT]
> Remember 2 cpp files depending on tas3 or task4

**How the classes work**
- RobotState: Holds a snapshot of the robot state, needs RobotModel.
- CurrentStateMonitor: Monitors simulator publishing and updates RobotState
- PlanningScene: Has RobotState + environment, test if a candidate is valid
-PlanningSceneMonitor: Monitors the world via ROStopics (etc?) and updates PlanningScene
- PlanningSceneInterface: used to modify the scene (add stuff, remove stuff, attach stuff)

Topics -> CurrentStateMonitor -> RobotState -> PlanningSceneMonitor -> PlanningScene -> RViz / Planner

## Task 5

[tutor](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html) 

```bash
cd ~/ws_moveit
source install/setup.bash
ros2 launch mtc_tutorial pick_place_demo.launch.py
```


```bash
cd ~/ws_moveit
source install/setup.bash
ros2 launch moveit2_tutorials mtc_demo.launch.py
```

