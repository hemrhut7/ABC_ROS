#!/bin/bash

source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
mkdir -p /workspaces/isaac_ros-dev/output

echo "Starting VINS-Fusion..."
ros2 launch my_robot_bringup vins_fusion.launch.py use_sim_time:=true > /tmp/vins.log 2>&1 &
VINS_PID=$!
sleep 4

echo "Playing Rosbag..."
ros2 bag play /workspaces/isaac_ros-dev/ros2_bag/rosbag2_total_20260903_203935 --clock -r 1.0 < /dev/null > /tmp/bag.log 2>&1 &
BAG_PID=$!

echo "Waiting for /image_track or /odometry..."
timeout 20 ros2 topic echo /image_track --once || {
    echo "Timeout on /image_track, checking /odometry..."
    timeout 5 ros2 topic echo /odometry --once || true
}

echo "--- VINS Log Output ---"
cat /tmp/vins.log | tail -n 40

kill $BAG_PID $VINS_PID 2>/dev/null || true
wait $VINS_PID 2>/dev/null || true
echo "Done."
