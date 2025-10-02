#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pi, sin, cos
from std_msgs.msg import Float32

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle

class GoToGoal(Node):
    def __init__(self, goals=None):
        super().__init__('proportional_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.update_pose, 10)
        self.error_pub = self.create_publisher(Float32, '/tracking_error', 10)
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.pose_initialized = False
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
        self.pose_initialized = True

    def euclidean_distance(self, goal):
        return sqrt((goal[0] - self.pose['x'])**2 + (goal[1] - self.pose['y'])**2)

    def steering_angle(self, goal):
        return atan2(goal[1] - self.pose['y'], goal[0] - self.pose['x'])

    def move_to_goal(self):
        # Wait for first pose update
        while not self.pose_initialized and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        vel_msg = Twist()

        # Info message for first goal
        if self.goals:
            first_goal = self.goals[0]
            if len(first_goal) == 3:
                self.get_logger().info(
                    f"Finding first goal: x={first_goal[0]:.3f}, y={first_goal[1]:.3f}, theta={first_goal[2]:.3f}"
                )
            else:
                self.get_logger().info(
                    f"Finding first goal: x={first_goal[0]:.3f}, y={first_goal[1]:.3f}"
                )

        # Tuning parameters
        tolerance = 0.05
        linear_gain = 1.2
        angular_gain = 3.0
        max_linear = 0.22
        max_angular = 2.0
        min_linear = 0.15
        min_angular = 0.3

        total_goals = len(self.goals)

        while rclpy.ok() and self.current_goal_index < total_goals:
            goal = self.goals[self.current_goal_index]
            # Support both (x, y) and (x, y, theta)
            if len(goal) == 3:
                gx, gy, gtheta = goal
            else:
                gx, gy = goal
                gtheta = None
            distance = self.euclidean_distance([gx, gy])
            self.get_logger().debug(
                f"Next goal idx={self.current_goal_index} pos=({gx:.3f},{gy:.3f}" +
                (f",{gtheta:.3f})" if gtheta is not None else ")") +
                f" dist={distance:.3f}"
            )
            while (distance >= tolerance or (gtheta is not None and abs(normalize_angle(gtheta - self.pose['theta'])) > 0.05)) and rclpy.ok():
                target_angle = self.steering_angle([gx, gy])
                angle_error = normalize_angle(target_angle - self.pose['theta'])
                heading_error = normalize_angle(gtheta - self.pose['theta']) if gtheta is not None else 0.0

                # If close to goal, rotate in place to desired heading (if theta given)
                if distance < tolerance and gtheta is not None:
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
                self.error_pub.publish(Float32(data=distance))
                rclpy.spin_once(self)
                distance = self.euclidean_distance([gx, gy])
                self.get_logger().debug(
                    f"Current pose: x={self.pose['x']:.3f}, y={self.pose['y']:.3f}, theta={self.pose['theta']:.3f}, dist to goal={distance:.3f}"
                )

            # Stop at goal
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.publisher_.publish(vel_msg)

            completed_goals = self.current_goal_index + 1
            if len(goal) == 3:
                goal_str = f"x={goal[0]:.3f}, y={goal[1]:.3f}, theta={goal[2]:.3f}"
            else:
                goal_str = f"x={goal[0]:.3f}, y={goal[1]:.3f}"
            self.get_logger().info(
                f"Reached goal {completed_goals}/{total_goals}:\n{goal_str}\n"
                f"My current pose is: x={self.pose['x']:.3f}, y={self.pose['y']:.3f}, theta={self.pose['theta']:.3f}"
            )

            self.current_goal_index += 1

        self.get_logger().info("All goals completed. Controller stopped.")

def main():
    rclpy.init()

    import sys
    if len(sys.argv) < 3:
        print("Usage:")
        print(" python3 motion_control.py pose x y [theta]")
        print(" python3 motion_control.py path square|circle")
        return

    mode = sys.argv[1]

    if mode == "pose":
        # Accepts: python3 motion_control.py pose x y [theta]
        if len(sys.argv) == 5:
            goals = [(float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))]
        else:
            goals = [(float(sys.argv[2]), float(sys.argv[3]))]
        node = GoToGoal(goals)

    elif mode == "path":
        shape = sys.argv[2]
        if shape == "square":
            goals = [(0, 0, 0), (1, 0, pi/2), (1, 1, pi), (0, 1, -pi/2), (0, 0, 0)]
        elif shape == "circle":
            goals = [(1.0 * cos(t), 1.0 * sin(t)) for t in [i * pi/8 for i in range(16)]]
        else:
            print("Unknown path. Use square or circle.")
            return
        node = GoToGoal(goals)

    else:
        print("Invalid mode. Use: pose | path")
        return

    node.move_to_goal()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
