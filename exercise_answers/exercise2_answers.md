**Ex.1**

What are the different attributes of the Lidar sensor and what difference do they make 
when set to different values?


- name and type of sensor and pose, self explanatory (xyzrpy)
- topic, where data will be published
- updaterate of data generation in Hz
- rays: H and V samples, how many rays.
	Resolution, multiplied by rays to get number range
	Angle, what the FOV of the lidar is (spread of rays)
- range:
	Min and max, min and max distance that the lidar can measure
	resolution, linear resolution of the distance measurement
- always_on flag if true the sensor will always be updated at update_rate (unused)
- visualize: if the sensor is visualized in GUI (unused)

**Ex.2**

1. Explain what information is being published over each of the ignition topics.

   jone@jone-ubuntu:~$ ign topic -l
iign topic -l | while read topic; do
    type=$(ign topic -i --topic "$topic" 2>/dev/null | awk -F',' 'NR==2 {print $2}' | xargs)
    echo "$topic : $type"
done

/clock : ignition.msgs.Clock
/gazebo/resource_paths : ignition.msgs.StringMsg_V
/gui/camera/pose : ignition.msgs.Pose
/panoptic/camera_info : ignition.msgs.CameraInfo
/panoptic/colored_map : ignition.msgs.Image
/panoptic/labels_map : ignition.msgs.Image
/semantic/camera_info : ignition.msgs.CameraInfo
/semantic/colored_map : ignition.msgs.Image
/semantic/labels_map : ignition.msgs.Image
/sensors/marker : ignition.msgs.Marker
/stats : ignition.msgs.WorldStatistics
/world/shapes/clock : ignition.msgs.Clock
/world/shapes/dynamic_pose/info : ignition.msgs.Pose_V
/world/shapes/model/realsense_d435/link/link/sensor/realsense_d435/camera_info : ignition.msgs.CameraInfo
/world/shapes/model/realsense_d435/link/link/sensor/realsense_d435/depth_image : ignition.msgs.Image
/world/shapes/model/realsense_d435/link/link/sensor/realsense_d435/image : ignition.msgs.Image
/world/shapes/model/realsense_d435/link/link/sensor/realsense_d435/points : ignition.msgs.PointCloudPacked
/world/shapes/pose/info : ignition.msgs.Pose_V
/world/shapes/scene/deletion : ignition.msgs.UInt32_V
/world/shapes/scene/info : ignition.msgs.Scene
/world/shapes/state : ignition.msgs.SerializedStepMap
/world/shapes/stats : ignition.msgs.WorldStatistics


2. What are the message types of them?

See above...

3. Do you see the same list of topics in ros2 topics list? If the answer is no, then why?

jone@jone-ubuntu:~$ ros2 topic list -t
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]

ANSWER: No, the topic list is not the same beacause they are different softwares. To share topics, ROS2 and ignition have to be bridged

![depth](/home/jone/robo_400/exercise_answers/depth.png)**

![rgb](/home/jone/robo_400/exercise_answers/rgb.png)

![Screenshot from 2025-09-10 12-59-11](/home/jone/robo_400/exercise_answers/Screenshot from 2025-09-10 12-59-11.png)

**Ex.3**

1. What are the different topics being published, what information do they publish? What are the 
message types? 

SHAPES: None, except the default rosout (central logging) and parameter_events (notifies about parameter changes)

AIR_PRESSURE publishes the air pressure:

header:
  stamp:
    sec: 141
    nanosec: 504000000
  frame_id: sensors_box/link/air_pressure
fluid_pressure: 101324.69180775555
variance: 0.31622776601683794

CAMERA:

- /camera [sensor_msgs/msg/Image]
  /camera/compressed [sensor_msgs/msg/CompressedImage]
  /camera/compressedDepth [sensor_msgs/msg/CompressedImage]
  /camera/theora [theora_image_transport/msg/Packet]

  - Theora is a codec that allows low-bandit streaming over ros topics

  /depth_camera [sensor_msgs/msg/Image]
  /depth_camera/compressed [sensor_msgs/msg/CompressedImage]
  /depth_camera/compressedDepth [sensor_msgs/msg/CompressedImage]
  /depth_camera/theora [theora_image_transport/msg/Packet]
  /parameter_events [rcl_interfaces/msg/ParameterEvent]
  /rgbd_camera/depth_image [sensor_msgs/msg/Image]
  /rgbd_camera/depth_image/compressed [sensor_msgs/msg/CompressedImage]
  /rgbd_camera/depth_image/compressedDepth [sensor_msgs/msg/CompressedImage]
  /rgbd_camera/depth_image/theora [theora_image_transport/msg/Packet]
  /rgbd_camera/image [sensor_msgs/msg/Image]
  /rgbd_camera/image/compressed [sensor_msgs/msg/CompressedImage]
  /rgbd_camera/image/compressedDepth [sensor_msgs/msg/CompressedImage]
  /rgbd_camera/image/theora [theora_image_transport/msg/Packet]
  /rosout [rcl_interfaces/msg/Log]

DIFF DRIVE:

- /clicked_point [geometry_msgs/msg/PointStamped]

  - used for interactive inputs

  /initialpose [geometry_msgs/msg/PoseWithCovarianceStamped]
  /model/vehicle_blue/cmd_vel [geometry_msgs/msg/Twist]
  /model/vehicle_blue/odometry [nav_msgs/msg/Odometry]
  /model/vehicle_green/cmd_vel [geometry_msgs/msg/Twist]
  /model/vehicle_green/odometry [nav_msgs/msg/Odometry]
  /move_base_simple/goal [geometry_msgs/msg/PoseStamped]

  - simple navigation goal

  /parameter_events [rcl_interfaces/msg/ParameterEvent]
  /rosout [rcl_interfaces/msg/Log]
  /tf [tf2_msgs/msg/TFMessage]

  - Dynamic transforms between coordinate frames

  /tf_static [tf2_msgs/msg/TFMessage]

DEPTH CAM:

- /camera [sensor_msgs/msg/Image]
  /camera/compressed [sensor_msgs/msg/CompressedImage]
  /camera/compressedDepth [sensor_msgs/msg/CompressedImage]
  /camera/theora [theora_image_transport/msg/Packet]
  /depth_camera [sensor_msgs/msg/Image]
  /depth_camera/compressed [sensor_msgs/msg/CompressedImage]
  /depth_camera/compressedDepth [sensor_msgs/msg/CompressedImage]
  /depth_camera/theora [theora_image_transport/msg/Packet]
  /parameter_events [rcl_interfaces/msg/ParameterEvent]
  /rgbd_camera/depth_image [sensor_msgs/msg/Image]
  /rgbd_camera/depth_image/compressed [sensor_msgs/msg/CompressedImage]
  /rgbd_camera/depth_image/compressedDepth [sensor_msgs/msg/CompressedImage]
  /rgbd_camera/depth_image/theora [theora_image_transport/msg/Packet]
  /rgbd_camera/image [sensor_msgs/msg/Image]
  /rgbd_camera/image/compressed [sensor_msgs/msg/CompressedImage]
  /rgbd_camera/image/compressedDepth [sensor_msgs/msg/CompressedImage]
  /rgbd_camera/image/theora [theora_image_transport/msg/Packet]
  - Pretty much the same as normal camera...

LIDAR

- /clicked_point [geometry_msgs/msg/PointStamped]
  /initialpose [geometry_msgs/msg/PoseWithCovarianceStamped]
  /lidar [sensor_msgs/msg/LaserScan]
  /lidar/points [sensor_msgs/msg/PointCloud2]
  /move_base_simple/goal [geometry_msgs/msg/PoseStamped]
  /parameter_events [rcl_interfaces/msg/ParameterEvent]
  /rosout [rcl_interfaces/msg/Log]
  /tf [tf2_msgs/msg/TFMessage]
  /tf_static [tf2_msgs/msg/TFMessage]
  - Pretty self-explanatory...

IMU

- /imu [sensor_msgs/msg/Imu]

  - Orientation, angular velocity, linear acceleration etc etc...

  /parameter_events [rcl_interfaces/msg/ParameterEvent]
  /rosout [rcl_interfaces/msg/Log]

  - Similar to air pressure sensor

MAGNETOMETER

header:
  stamp:
    sec: 103
    nanosec: 690000000
  frame_id: sensors_box/link/magnetometer
magnetic_field:
  x: -0.043516692530210205
  y: -0.10156736098297237
  z: -0.09647345572360673
magnetic_field_covariance:

​	- "Uncertainty"

- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0

GNSS

/navsat [sensor_msgs/msg/NavSatFix]

- Lat, long, altitude, covariance

/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]

RGBD

/camera/camera_info [sensor_msgs/msg/CameraInfo]
/camera/image [sensor_msgs/msg/Image]
/clicked_point [geometry_msgs/msg/PointStamped]
/initialpose [geometry_msgs/msg/PoseWithCovarianceStamped]
/move_base_simple/goal [geometry_msgs/msg/PoseStamped]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rgbd_camera/camera_info [sensor_msgs/msg/CameraInfo]

- Camera matrices (distortion, intrinsic, rectification, projection)
- etc...

/rgbd_camera/depth_image [sensor_msgs/msg/Image]
/rgbd_camera/image [sensor_msgs/msg/Image]
/rgbd_camera/points [sensor_msgs/msg/PointCloud2]
/rosout [rcl_interfaces/msg/Log]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]

BATTERY:

tate --once
header:
  stamp:
    sec: 169
    nanosec: 250000000
  frame_id: ''
voltage: 12.592000007629395
temperature: 0.0
current: 0.0
charge: 1.2008999586105347
capacity: 1.2008999586105347
design_capacity: .nan
percentage: 100.0
power_supply_status: 4
power_supply_health: 0
power_supply_technology: 0
present: true
cell_voltage: []
cell_temperature: []
location: ''
serial_number: ''

etc...

ROBOT DESCRIPTION PUBLISHER

/clicked_point [geometry_msgs/msg/PointStamped]
/clock [rosgraph_msgs/msg/Clock]
/initialpose [geometry_msgs/msg/PoseWithCovarianceStamped]
/joint_states [sensor_msgs/msg/JointState]
/move_base_simple/goal [geometry_msgs/msg/PoseStamped]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/robot_description [std_msgs/msg/String]
/rosout [rcl_interfaces/msg/Log]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]

JOINT STATES PUBLISHER

/clicked_point [geometry_msgs/msg/PointStamped]
/clock [rosgraph_msgs/msg/Clock]
/goal_pose [geometry_msgs/msg/PoseStamped]
/initialpose [geometry_msgs/msg/PoseWithCovarianceStamped]
/joint_states [sensor_msgs/msg/JointState]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/robot_description [std_msgs/msg/String]
/rosout [rcl_interfaces/msg/Log]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]

JOINT AND POSE PUBLISHER

/clicked_point [geometry_msgs/msg/PointStamped]
/goal_pose [geometry_msgs/msg/PoseStamped]
/initialpose [geometry_msgs/msg/PoseWithCovarianceStamped]
/joint_states [sensor_msgs/msg/JointState]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]

---

2. The rostopics were not being published in the example in section 2.2. However, the rostopics are 
being published over the ROS master in this implementation in section 3. What is the reason for 
this difference between the two implementations?

Because in section 3, ROS 2 and ignition have been bridged

**Ex. 4**

ros2 run ros_gz_bridge parameter_bridge   /world/shapes/model/realsense_d435/link/link/sensor/realsense_d435/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image

The following topics were bridged:

![image-20250910163059041](/home/jone/snap/typora/106/.config/Typora/typora-user-images/image-20250910163059041.png)

Topics in action: ![image-20250910163623196](/home/jone/snap/typora/106/.config/Typora/typora-user-images/image-20250910163623196.png)

