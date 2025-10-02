#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from math import atan2, sqrt, pi
import csv
import sys
import subprocess


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle


class PurePursuit(Node):
    def __init__(self, path=None, lookahead_distance=0.2):
        super().__init__('pure_pursuit_controller')
        # publishers / subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.update_pose, 10)
        self.error_pub = self.create_publisher(Float32, '/tracking_error', 10)

        # robot pose
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.pose_initialized = False

        # path and indices
        self.path = path if path else []
        self.waypoint_index = 0       # next CSV waypoint to reach (sequential)
        self.lookahead_index = 0      # index found by lookahead search
        self.lookahead_distance = lookahead_distance
        self.completed_flags = [False] * len(self.path)

        # Change to DEBUG for more detailed logs
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

    def update_pose(self, msg: Odometry):
        """Update robot pose from Odometry message."""
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # yaw from quaternion (assuming geometry_msgs Quaternion)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.pose['theta'] = atan2(siny_cosp, cosy_cosp)
        self.pose_initialized = True

    def distance(self, point):
        """Euclidean distance from current pose to a point (x,y)."""
        return sqrt((point[0] - self.pose['x'])**2 + (point[1] - self.pose['y'])**2)

    def find_lookahead_point(self):
        """
        Find the first path point at distance >= lookahead_distance starting from waypoint_index.
        If none found, return the last waypoint. Also sets self.lookahead_index.
        """
        if not self.path:
            return None
        start = max(0, self.waypoint_index)
        for i in range(start, len(self.path)):
            if self.distance(self.path[i]) >= self.lookahead_distance:
                self.lookahead_index = i
                return self.path[i]
        # fallback to last point
        self.lookahead_index = len(self.path) - 1
        return self.path[-1]

    def run(self):
        if not self.path:
            self.get_logger().error("Empty path: nothing to follow.")
            return

        # wait for first odom
        while not self.pose_initialized and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        # Info message for first goal
        if self.path:
            first_goal = self.path[0]
            self.get_logger().info(
                f"Finding first goal: x={first_goal[0]:.3f}, y={first_goal[1]:.3f}"
            )

        # control params
        max_linear = 0.5
        max_angular = 2.0
        linear_gain = 1.25
        angular_gain = 2.5
        tolerance = 0.05

        vel_msg = Twist()

        last_completed_count = 0

        while rclpy.ok() and self.waypoint_index < len(self.path):
            # find lookahead point (used for steering)
            lookahead_point = self.find_lookahead_point()
            if lookahead_point is None:
                self.get_logger().error("No lookahead point found.")
                break
            distance_to_lookahead = self.distance(lookahead_point)

            # Sequentially mark CSV waypoints as completed **in order only**
            waypoint_reached = False
            while (self.waypoint_index < len(self.path) and
                   self.distance(self.path[self.waypoint_index]) < tolerance):
                idx = self.waypoint_index
                self.completed_flags[idx] = True
                self.get_logger().info(
                    f"Waypoint {idx}/{len(self.path)} reached.\n"
                    f"x={self.path[idx][0]:.3f}, y={self.path[idx][1]:.3f}"
                )
                self.waypoint_index += 1

            # If all waypoints completed, stop
            if self.waypoint_index >= len(self.path):
                self.get_logger().info("All waypoints reached. Stopping.")
                break

            # Current target waypoint for progress reporting (next in sequence)
            current_wp = self.path[self.waypoint_index]
            dist_to_current_wp = self.distance(current_wp)

            # Logging (clear separation between waypoint target and lookahead target)
            self.get_logger().debug(
                f"Next waypoint idx={self.waypoint_index} pos=({current_wp[0]:.3f},{current_wp[1]:.3f}) "
                f"dist={dist_to_current_wp:.3f}"
            )
            self.get_logger().debug(
                f"Lookahead idx={self.lookahead_index} pos=({lookahead_point[0]:.3f},{lookahead_point[1]:.3f}) "
                f"dist={distance_to_lookahead:.3f}"
            )

            # Pure Pursuit control (steer toward lookahead point)
            alpha = normalize_angle(
                atan2(lookahead_point[1] - self.pose['y'], lookahead_point[0] - self.pose['x'])
                - self.pose['theta']
            )

            linear_speed = min(max_linear, linear_gain * distance_to_lookahead)
            angular_speed = max(-max_angular, min(max_angular, angular_gain * alpha))

            vel_msg.linear.x = linear_speed
            vel_msg.angular.z = angular_speed
            self.cmd_pub.publish(vel_msg)
            self.error_pub.publish(Float32(data=dist_to_current_wp))

            # let callbacks run
            rclpy.spin_once(self, timeout_sec=0.01)

        # stop robot
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.cmd_pub.publish(vel_msg)
        self.get_logger().info("Controller stopped and robot commanded to zero velocity.")


def load_path_from_csv(filename):
    path = []
    with open(filename, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        if 'x' not in reader.fieldnames or 'y' not in reader.fieldnames:
            raise ValueError("CSV must have 'x' and 'y' headers")
        for row in reader:
            x = float(row['x'])
            y = float(row['y'])
            path.append((x, y))
    return path


def main():
    rclpy.init()
    if len(sys.argv) < 3 or sys.argv[1] != 'replay':
        print("Usage: python3 motion_control_2.py replay path.csv [--record <bag_name>]")
        return

    path_file = sys.argv[2]
    record_bag = False
    bag_name = None

    # Parse optional --record argument
    if len(sys.argv) >= 5 and sys.argv[3] == "--record":
        record_bag = True
        bag_name = sys.argv[4]

    # Check path file before starting ros2 bag record
    try:
        path = load_path_from_csv(path_file)
    except Exception as e:
        print(f"Failed to load path: {e}")
        return

    # Start ros2 bag record if requested
    bag_proc = None
    if record_bag:
        print(f"Recording /odom to bag: {bag_name}")
        bag_proc = subprocess.Popen(
            ["ros2", "bag", "record", "/odom", "-o", bag_name]
        )
        import time
        time.sleep(2)  # Give ros2 bag time to start

    node = PurePursuit(path=path, lookahead_distance=0.1)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Stop ros2 bag record if running
        if bag_proc:
            bag_proc.terminate()
            bag_proc.wait()
            print("ros2 bag record stopped.")
            # Run rosbag_extract_path.py with the bag name
            print("Extracting path from bag...")
            subprocess.run(
                ["python3", "rosbag_extract_path.py", bag_name],
                check=False
            )


if __name__ == '__main__':
    main()
