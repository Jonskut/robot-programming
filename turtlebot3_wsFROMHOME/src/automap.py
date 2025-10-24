#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math
import random
import time
from collections import deque

FREE = 0
OCCUPIED = 100
UNKNOWN = -1
# Some occupancy/costmap publishers use 255 for unknown; accept both.
UNKNOWN_ALT = 255

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('python_frontier_explorer')

        # Parameters
        self.declare_parameter('planner_frequency', 1.0)
        self.declare_parameter('min_frontier_size', 3)  # cells
        self.declare_parameter('progress_timeout', 20.0)
        # treat cells with cost <= cost_threshold as traversable (helps through doorways/inflation)
        self.declare_parameter('cost_threshold', 50)

        self.planner_frequency = self.get_parameter('planner_frequency').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.cost_threshold = self.get_parameter('cost_threshold').value

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Map and robot pose
        self.map_data = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.frontier_blacklist = []
        self.prev_goal = None
        self.prev_distance = float('inf')
        self.last_progress_time = self.get_clock().now()

        # Subscribers and timers
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.map_cb, 10)
        self.timer = self.create_timer(1.0 / self.planner_frequency, self.make_plan)
        self.goal_active = False

    def map_cb(self, msg):
        self.map_data = msg

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.orientation = trans.transform.rotation
            return pose
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def make_plan(self):
        if self.map_data is None or self.goal_active:
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        # Convert map to 2D array
        width = self.map_data.info.width
        height = self.map_data.info.height
        data = list(self.map_data.data)
        grid = [data[y*width:(y+1)*width] for y in range(height)]

        # Convert world to map indices
        def world_to_map(x, y):
            mx = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            my = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
            return mx, my

        # cost/check helper: treat small-cost cells as free to allow passing narrow gaps
        def is_unknown_val(v):
            return v == UNKNOWN or v == UNKNOWN_ALT

        def is_free_val(v):
            # explicitly exclude 255 (UNKNOWN_ALT) from being treated as free
            if v == UNKNOWN_ALT:
                return False
            return (v >= 0) and (v <= self.cost_threshold)
        

        # Convert map indices to world coordinates
        def map_to_world(mx, my):
            x = mx * self.map_data.info.resolution + self.map_data.info.origin.position.x + self.map_data.info.resolution/2
            y = my * self.map_data.info.resolution + self.map_data.info.origin.position.y + self.map_data.info.resolution/2
            return x, y

        # BFS to find frontiers
        mx, my = world_to_map(robot_pose.pose.position.x, robot_pose.pose.position.y)
        if not (0 <= mx < width and 0 <= my < height):
            self.get_logger().info(f"Robot pose outside map bounds ({mx},{my})")
            return

        start_val = grid[my][mx]
        self.get_logger().debug(f"Start cell ({mx},{my}) value={start_val}")

        # ensure we start BFS from a free cell; if current cell isn't free search nearby
        if not is_free_val(start_val):
            def find_nearest_free_start(sx, sy, max_radius=50):
                q = deque([(sx, sy, 0)])
                seen = {(sx, sy)}
                while q:
                    cx, cy, d = q.popleft()
                    if 0 <= cx < width and 0 <= cy < height:
                        if is_free_val(grid[cy][cx]):
                            return (cx, cy)
                    if d >= max_radius:
                        continue
                    for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                        nb = (cx+dx, cy+dy)
                        if nb in seen:
                            continue
                        seen.add(nb)
                        q.append((nb[0], nb[1], d+1))
                return None

            new_start = find_nearest_free_start(mx, my, max_radius=40)
            if new_start is None:
                self.get_logger().info("No nearby free start cell found; cannot search for frontiers")
                return
            mx, my = new_start
            self.get_logger().info(f"Using nearest free start cell ({mx},{my})")
        
        # run BFS from the (possibly updated) start cell to find frontiers
        visited = set()
        queue = deque([(mx, my)])
        visited.add((mx, my))
        frontiers = []

        while queue:
            cx, cy = queue.popleft()
            # neighbors
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = cx+dx, cy+dy
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                if (nx, ny) in visited:
                    continue
                visited.add((nx, ny))
                cell = grid[ny][nx]
                if is_free_val(cell):
                    queue.append((nx, ny))
                elif is_unknown_val(cell):
                    # Check if neighbor of free cell → frontier
                    frontiers.append((nx, ny))

        if not frontiers:
            self.get_logger().info("No frontiers found")
            return

        # Cluster frontiers: for simplicity, pick random frontier not in blacklist
        candidates = [f for f in frontiers if f not in self.frontier_blacklist]
        if not candidates:
            self.get_logger().info("All frontiers blacklisted, clearing blacklist")
            self.frontier_blacklist.clear()
            candidates = frontiers

        fx, fy = random.choice(candidates)
        wx, wy = map_to_world(fx, fy)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wx
        pose.pose.position.y = wy
        pose.pose.orientation.w = 1.0

        self.send_goal(pose, frontier_cell=(fx, fy))

    def send_goal(self, pose, frontier_cell=None):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 action server not ready")
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        send_goal_future = self.nav_client.send_goal_async(goal)

        def goal_response_callback(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Goal rejected")
                return

            self.goal_active = True

            def result_callback(res_fut):
                result = res_fut.result()
                status = result.status
                if frontier_cell is not None and status == 4:  # SUCCEEDED
                    self.get_logger().info(f"Reached frontier {frontier_cell}")
                elif frontier_cell is not None:
                    self.frontier_blacklist.append(frontier_cell)
                    self.get_logger().info(f"Blacklisting frontier {frontier_cell}")
                self.goal_active = False

            goal_handle.get_result_async().add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
