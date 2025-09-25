#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from math import atan2, sqrt, pi, sin, cos
import csv

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle

class PurePursuit(Node):
    def __init__(self, path=None, lookahead_distance=0.2):
        super().__init__('pure_pursuit_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.update_pose, 10)
        self.error_pub = self.create_publisher(Float32, '/tracking_error', 10)
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.pose_initialized = False
        self.path = path if path else []
        self.current_index = 0
        self.lookahead_distance = lookahead_distance

    def update_pose(self, msg):
        """Update robot pose from Odometry."""
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.pose['theta'] = atan2(siny_cosp, cosy_cosp)
        self.pose_initialized = True

    def distance(self, point):
        return sqrt((point[0] - self.pose['x'])**2 + (point[1] - self.pose['y'])**2)

    def find_lookahead_point(self):
        """Return the first point ahead of the robot at >= lookahead_distance."""
        for i in range(self.current_index, len(self.path)):
            if self.distance(self.path[i]) >= self.lookahead_distance:
                self.current_index = i
                return self.path[i]
        # If no point is far enough, return last point
        return self.path[-1]

    def run(self):
        # Wait until first pose is received
        while not self.pose_initialized and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        vel_msg = Twist()
        max_linear = 0.5        
        max_angular = 2.0
        linear_gain = 1.25      
        angular_gain = 2.5
        tolerance = 0.05

        while rclpy.ok() and self.current_index < len(self.path):
            goal = self.find_lookahead_point()
            distance_to_goal = self.distance(goal)
            self.get_logger().info(
                f"New goal received: x={goal[0]:.3f}, y={goal[1]:.3f} | Current pose: x={self.pose['x']:.3f}, y={self.pose['y']:.3f}, theta={self.pose['theta']:.3f}"
            )

            if distance_to_goal < tolerance and self.current_index == len(self.path)-1:
                self.get_logger().info("Final goal reached.")
                break  # Finished path

            if distance_to_goal < tolerance:
                self.get_logger().info(
                    f"Reached goal: x={goal[0]:.3f}, y={goal[1]:.3f}"
                )

            # Pure pursuit: compute steering angle to lookahead point
            alpha = normalize_angle(atan2(goal[1] - self.pose['y'], goal[0] - self.pose['x']) - self.pose['theta'])

            # Linear speed proportional to distance (capped)
            linear_speed = min(max_linear, linear_gain * distance_to_goal)

            # Angular speed proportional to steering angle (capped)
            angular_speed = max(-max_angular, min(max_angular, angular_gain * alpha))

            vel_msg.linear.x = linear_speed
            vel_msg.angular.z = angular_speed
            self.publisher_.publish(vel_msg)
            self.error_pub.publish(Float32(data=distance_to_goal))
            rclpy.spin_once(self)

        # Stop robot
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.publisher_.publish(vel_msg)
        self.get_logger().info("Path completed.")

def load_path_from_csv(filename):
    path = []
    with open(filename, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            x = float(row['x'])
            y = float(row['y'])
            path.append((x, y))
    return path

def main():
    rclpy.init()
    import sys
    if len(sys.argv) < 3 or sys.argv[1] != 'replay':
        print("Usage: python3 pure_pursuit.py replay path.csv")
        return

    path_file = sys.argv[2]
    path = load_path_from_csv(path_file)
    node = PurePursuit(path=path, lookahead_distance=0.3)
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
