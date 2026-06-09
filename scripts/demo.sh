#!/usr/bin/env bash
# End-to-end full calibration demo inside the ROS Noetic container.
# Workflow: roscore → synthetic publisher (rest) → set_reference →
#           pivot ×3 → straight ×1 → calibrate
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"

CONFIG_FILE="${1:-$(rospack find dual_cam_full_calib)/config/config.yaml}"

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
rosrun dual_cam_full_calib synthetic_full_publisher.py &
PUB_PID=$!
sleep 2

echo "Loading parameters from: $CONFIG_FILE"
rosparam load "$CONFIG_FILE" /dual_cam_full_calib

echo "Starting calibration node..."
rosrun dual_cam_full_calib dual_cam_full_calib_node __name:=dual_cam_full_calib &
NODE_PID=$!
sleep 2

# --- Publisher starts in "rest" mode → capture reference ---
echo "=== set_reference (rest pose) ==="
rosservice call /dual_cam_full_calib/set_reference "{}"

# --- Pivot samples ---
rosservice call /synthetic_full_publisher/set_motion "data: true"
sleep 1

for i in 1 2 3; do
  echo "=== add_pivot_sample ($i) ==="
  sleep 1
  rosservice call /dual_cam_full_calib/add_pivot_sample "{}"
done

# --- Straight sample (same reference frame) ---
rosservice call /synthetic_full_publisher/set_motion "data: false"
sleep 1

echo "=== add_straight_sample ==="
rosservice call /dual_cam_full_calib/add_straight_sample "{}"

# --- Run 6-phase calibration ---
echo "=== calibrate ==="
rosservice call /dual_cam_full_calib/calibrate "{}"

echo "Demo complete."
