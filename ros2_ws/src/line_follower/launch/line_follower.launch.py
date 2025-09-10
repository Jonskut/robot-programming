from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('line_follower')
    world_path = os.path.join(pkg_share, 'worlds', 'line_follower.sdf')

    print("World path:", world_path)  # This line was added

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path],
       # output='screen'
    )

    # Line follower
    line_follower = Node(
        package='line_follower',
        executable='line_follower_node',
        name='line_follower',
        output='screen'
    )

    # Bridge
    bridge = Node(
       package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Camera image bridge
            '/world/line_follower/model/vehicle_blue/link/rgb_camera/sensor/camera_sensor/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            # Camera info bridge 
            '/world/line_follower/model/vehicle_blue/link/rgb_camera/sensor/camera_sensor/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            # Cmd_vel bridge
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        line_follower,
    ])
