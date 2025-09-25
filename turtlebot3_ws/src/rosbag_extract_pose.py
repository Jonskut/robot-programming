import rosbag2_py
from nav_msgs.msg import Odometry
from rosidl_runtime_py.utilities import get_message
import csv
from math import atan2
import sys
import os

if len(sys.argv) < 2:
    print("Usage: python3 rosbag_extract_pose.py <bag_folder>")
    sys.exit(1)

BAG_PATH = sys.argv[1].rstrip('/')
bag_name = os.path.basename(BAG_PATH)
output_csv = f'path_{bag_name}.csv'

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
reader.open(storage_options, converter_options)

with open(output_csv, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['x', 'y', 'theta'])
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/odom':
            msg_type = get_message('nav_msgs/msg/Odometry')
            from rclpy.serialization import deserialize_message
            msg = deserialize_message(data, msg_type)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            theta = atan2(siny_cosp, cosy_cosp)
            writer.writerow([x, y, theta])

print(f"Path extracted to {output_csv}")
