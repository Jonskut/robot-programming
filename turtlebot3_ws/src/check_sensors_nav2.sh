#!/bin/bash
# Check which sensor topics Nav2 nodes are currently subscribing to.
# Works with ROS 2 (Humble, Iron, etc.)

echo "=== Checking active Navigation2 nodes and their sensor subscriptions ==="

# List of common Nav2 nodes to check
NAV2_NODES=(
  "/amcl"
  "/controller_server"
  "/planner_server"
  "/bt_navigator"
  "/map_server"
  "/local_costmap/local_costmap"
  "/global_costmap/global_costmap"
  "/smoother_server"
)

for NODE in "${NAV2_NODES[@]}"; do
  echo -e "\n--- Node: $NODE ---"
  if ros2 node info "$NODE" &>/dev/null; then
    ros2 node info "$NODE" | grep -E "Subscribed topics|/scan|/odom|/imu|/tf|/map" --color=always
  else
    echo "Node not found or not active."
  fi
done

echo -e "\n=== Summary of key sensor topics ==="
echo "If you see /scan, /odom, /imu, or /tf under 'Subscribed topics', that sensor is in use."
