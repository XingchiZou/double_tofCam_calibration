#!/usr/bin/env bash
# End-to-end calibration demo inside the ROS Noetic container.
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"

CONFIG_FILE="${1:-$(rospack find dual_cam_pivot_calib)/config/config.yaml}"

echo "Starting roscore..."
roscore &
ROSCORE_PID=$!
sleep 2

cleanup() {
  kill "$ROSCORE_PID" 2>/dev/null || true
  kill "$PUB_PID" 2>/dev/null || true
  kill "$NODE_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "Starting synthetic point cloud publisher..."
rosrun dual_cam_pivot_calib synthetic_pivot_publisher.py &
PUB_PID=$!
sleep 2

echo "Loading parameters from: $CONFIG_FILE"
rosparam load "$CONFIG_FILE" /dual_cam_pivot_calib

echo "Starting calibration node..."
rosrun dual_cam_pivot_calib dual_cam_pivot_calib_node __name:=dual_cam_pivot_calib &
NODE_PID=$!
sleep 2

echo "=== set_reference ==="
rosservice call /dual_cam_pivot_calib/set_reference "{}"

for i in 1 2 3; do
  echo "=== add_pivot_sample ($i) ==="
  sleep 1
  rosservice call /dual_cam_pivot_calib/add_pivot_sample "{}"
done

echo "=== calibrate ==="
rosservice call /dual_cam_pivot_calib/calibrate "{}"

echo "Demo complete."
