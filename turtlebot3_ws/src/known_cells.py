#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from typing import Sequence

class MapMonitor(Node):
    def __init__(self):
        super().__init__('map_monitor')

        # parameters (tune as needed)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('occupied_threshold', 65)   # >= this considered occupied
        self.declare_parameter('free_threshold', 25)       # <= this considered free
        self.declare_parameter('treat_255_as_unknown', True)

        topic = self.get_parameter('map_topic').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.free_threshold = self.get_parameter('free_threshold').value
        self.treat_255_as_unknown = self.get_parameter('treat_255_as_unknown').value

        self.sub = self.create_subscription(OccupancyGrid, topic, self.map_callback, 10)
        self.get_logger().info(f"Map monitor started. Listening to {topic}...")

    def _classify_counts(self, data: Sequence[int]):
        total = len(data)
        if total == 0:
            return {
                'total': 0, 'unknown': 0, 'free': 0, 'occupied': 0, 'inflated': 0
            }

        # Normalize values to python int (handles bytes/bytearray)
        vals = [int(v) for v in data]

        # Unknowns: -1 for OccupancyGrid, some publishers use 255
        unknown_vals = {-1}
        if self.treat_255_as_unknown:
            unknown_vals.add(255)

        unknown = sum(1 for v in vals if v in unknown_vals)
        # Known cells are those not unknown
        known = total - unknown

        free = sum(1 for v in vals if (v not in unknown_vals) and (v >= 0) and (v <= self.free_threshold))
        occupied = sum(1 for v in vals if (v not in unknown_vals) and (v >= self.occupied_threshold))
        # any known but neither free nor occupied -> inflated / mid-cost / unknown-cost
        inflated = known - (free + occupied)

        return {
            'total': total,
            'unknown': unknown,
            'known': known,
            'free': free,
            'occupied': occupied,
            'inflated': inflated
        }

    def map_callback(self, msg: OccupancyGrid):
        counts = self._classify_counts(msg.data)
        total = counts['total']
        if total == 0:
            self.get_logger().warn("Received empty map")
            return

        discovered_pct = 100.0 * counts['known'] / total
        free_pct_of_known = (100.0 * counts['free'] / counts['known']) if counts['known'] > 0 else 0.0
        occupied_pct_of_known = (100.0 * counts['occupied'] / counts['known']) if counts['known'] > 0 else 0.0
        inflated_pct_of_known = (100.0 * counts['inflated'] / counts['known']) if counts['known'] > 0 else 0.0

        self.get_logger().info(
            f"Map: total={total} known={counts['known']} ({discovered_pct:.1f}%) "
            f"free={counts['free']} ({free_pct_of_known:.1f}% of known) "
            f"occupied={counts['occupied']} ({occupied_pct_of_known:.1f}% of known) "
            f"inflated={counts['inflated']} ({inflated_pct_of_known:.1f}% of known) "
            f"unknown={counts['unknown']}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MapMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
