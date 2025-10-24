#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import math

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Nav2 action client used only to detect whether Nav2 is present so we yield control
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.desired_distance = 0.5  # meters from the wall
        self.kp = 1.0  # proportional gain for correction
        self.forward_speed = 0.2  # m/s

        # runtime flags
        self.enabled = True  # wall-following allowed
        self.nav2_present = False  # if true, yield control to nav2
        self.stopped_due_to_map = False  # true when stopped because map >= threshold

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def map_callback(self, msg: OccupancyGrid):
        # compute percent known (free=0, occupied=100)
        width = msg.info.width
        height = msg.info.height
        total = width * height
        if total == 0:
            return
        data = msg.data
        count_free = sum(1 for v in data if v == 0)
        count_occupied = sum(1 for v in data if v == 100)
        known = count_free + count_occupied
        percent_known = (known / total) * 100.0

        if percent_known >= 95.0:
            if not self.stopped_due_to_map:
                self.get_logger().info(
                    f"Map known {percent_known:.1f}% -> stopping wall follower and yielding to navigation/bringup."
                )
                self.stop_robot()
                self.enabled = False
                self.stopped_due_to_map = True
        else:
            if self.stopped_due_to_map:
                self.get_logger().info(
                    f"Map known dropped to {percent_known:.1f}% -> resuming wall follower."
                )
                self.enabled = True
                self.stopped_due_to_map = False

    def scan_callback(self, msg: LaserScan):
        # Detect if Nav2 is present; if so, yield control (don't publish cmd_vel)
        try:
            # non-blocking short wait to detect server quickly
            self.nav2_present = self.nav_client.wait_for_server(timeout_sec=0.01)
        except Exception:
            self.nav2_present = False

        if not self.enabled:
            # stopped due to map fullness: ensure robot is not moving
            self.stop_robot()
            return

        if self.nav2_present:
            # Nav2 is running: yield to it
            self.get_logger().debug("Nav2 present - yielding cmd_vel")
            self.stop_robot()
            return

        # Simple: use the right side of the robot to follow a wall
        if not msg.ranges:
            return

        right_idx = len(msg.ranges) // 4  # 90 degrees to the right
        try:
            right_dist = msg.ranges[right_idx]
        except Exception:
            right_dist = float('inf')

        # Avoid NaNs or infs
        if math.isinf(right_dist) or math.isnan(right_dist):
            right_dist = 1.0

        error = self.desired_distance - right_dist
        correction = self.kp * error

        cmd = Twist()
        cmd.linear.x = self.forward_speed
        cmd.angular.z = correction  # turn toward/away from wall
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
