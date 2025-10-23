#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, pi, sin, cos

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle

class GoToGoal(Node):
    def __init__(self, goal_type="point", goals=None):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.update_pose, 10)
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.pose_initialized = False  # <-- Add this flag
        self.goal_type = goal_type
        self.goals = goals if goals else []
        self.current_goal_index = 0

    def update_pose(self, msg):
        """Callback: update robot pose from Odometry."""
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.pose['theta'] = atan2(siny_cosp, cosy_cosp)
        self.pose_initialized = True  # <-- Set flag when pose is received

    def euclidean_distance(self, goal):
        return sqrt((goal[0] - self.pose['x'])**2 + (goal[1] - self.pose['y'])**2)

    def steering_angle(self, goal):
        return atan2(goal[1] - self.pose['y'], goal[0] - self.pose['x'])

    def move_to_goal(self):
        # Wait for first pose update
        while not self.pose_initialized and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        vel_msg = Twist()

        # Modify tuning parameters for better performance
        tolerance = 0.05
        linear_gain = 1.2
        angular_gain = 3.0
        max_linear = 0.22
        max_angular = 2.0
        min_linear = 0.15
        min_angular = 0.3

        while rclpy.ok() and self.current_goal_index < len(self.goals):
            goal = self.goals[self.current_goal_index]

            if self.goal_type == "point":
                distance = self.euclidean_distance(goal)
                while distance >= tolerance and rclpy.ok():
                    target_angle = self.steering_angle(goal)
                    angle_error = normalize_angle(target_angle - self.pose['theta'])

                    # Linear speed with minimum threshold
                    linear_speed = linear_gain * distance
                    if abs(linear_speed) < min_linear:
                        linear_speed = min_linear if distance > tolerance else 0.0
                    vel_msg.linear.x = min(linear_speed, max_linear)

                    # Angular speed with minimum threshold
                    angular_speed = angular_gain * angle_error
                    if abs(angular_speed) < min_angular and abs(angle_error) > 0.05:
                        angular_speed = min_angular if angular_speed > 0 else -min_angular
                    vel_msg.angular.z = max(-max_angular, min(angular_speed, max_angular))

                    self.publisher_.publish(vel_msg)
                    rclpy.spin_once(self)
                    distance = self.euclidean_distance(goal)

            elif self.goal_type == "pose":
                gx, gy, gtheta = goal
                distance = self.euclidean_distance([gx, gy])
                while (distance >= tolerance or abs(normalize_angle(gtheta - self.pose['theta'])) > 0.05) and rclpy.ok():
                    target_angle = self.steering_angle([gx, gy])
                    angle_error = normalize_angle(target_angle - self.pose['theta'])
                    heading_error = normalize_angle(gtheta - self.pose['theta'])

                    # If close to goal, rotate in place to desired heading
                    if distance < tolerance:
                        vel_msg.linear.x = 0.0
                        angular_speed = angular_gain * heading_error
                        if abs(angular_speed) < min_angular and abs(heading_error) > 0.05:
                            angular_speed = min_angular if angular_speed > 0 else -min_angular
                        vel_msg.angular.z = max(-max_angular, min(angular_speed, max_angular))
                    else:
                        # Move towards goal
                        linear_speed = linear_gain * distance
                        if abs(linear_speed) < min_linear:
                            linear_speed = min_linear if distance > tolerance else 0.0
                        vel_msg.linear.x = min(linear_speed, max_linear)

                        angular_speed = angular_gain * angle_error
                        if abs(angular_speed) < min_angular and abs(angle_error) > 0.05:
                            angular_speed = min_angular if angular_speed > 0 else -min_angular
                        vel_msg.angular.z = max(-max_angular, min(angular_speed, max_angular))

                    self.publisher_.publish(vel_msg)
                    rclpy.spin_once(self)
                    distance = self.euclidean_distance([gx, gy])

            # Stop at goal
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.publisher_.publish(vel_msg)
            self.get_logger().info(f"Reached goal {self.current_goal_index+1}/{len(self.goals)}")
            self.get_logger().info(
                f"My current pose is: x={self.pose['x']:.3f}, y={self.pose['y']:.3f}, theta={self.pose['theta']:.3f}"
            )

            self.current_goal_index += 1

def main():
    rclpy.init()

    import sys
    if len(sys.argv) < 3:
        print("Usage:")
        print(" python3 motion_control.py point x y")
        print(" python3 motion_control.py pose x y theta")
        print(" python3 motion_control.py path square|circle")
        return

    mode = sys.argv[1]

    if mode == "point":
        goals = [(float(sys.argv[2]), float(sys.argv[3]))]
        node = GoToGoal("point", goals)

    elif mode == "pose":
        goals = [(float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))]
        node = GoToGoal("pose", goals)

    elif mode == "path":
        shape = sys.argv[2]
        if shape == "square":
            goals = [(0, 0, 0), (1, 0, pi/2), (1, 1, pi), (0, 1, -pi/2), (0, 0, 0)]
        elif shape == "circle":
            goals = [(1.0 * cos(t), 1.0 * sin(t)) for t in [i * pi/8 for i in range(16)]]
        else:
            print("Unknown path. Use square or circle.")
            return
        node = GoToGoal("pose", goals)

    else:
        print("Invalid mode. Use: point | pose | path")
        return

    node.move_to_goal()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
