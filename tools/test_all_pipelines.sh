#!/bin/bash
set -e

# Setup library paths for GXF
find /opt/ros/humble/share/isaac_ros_gxf/gxf/lib/ -name '*.so' -exec dirname {} \; | sort -u > /etc/ld.so.conf.d/isaac_ros_gxf.conf
ldconfig

source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

BAG_PATH="/workspaces/isaac_ros-dev/ros2_bag/rosbag2_total_20260903_203935"

echo "=================================================="
echo "1. Testing Isaac ROS cuVSLAM with ROS 2 Bag Replay"
echo "=================================================="
ros2 launch my_robot_bringup isaac_visual_slam.launch.py use_sim_time:=true enable_imu_fusion:=false > /tmp/vslam.log 2>&1 &
VSLAM_PID=$!
sleep 4

# Play 5 seconds of rosbag
ros2 bag play "$BAG_PATH" --clock -r 1.0 -d 1 < /dev/null > /tmp/bag_vslam.log 2>&1 &
BAG_PID=$!

# Wait for odometry topic
echo "Waiting for /visual_slam/tracking/odometry..."
timeout 15 ros2 topic echo /visual_slam/tracking/odometry --once || {
    echo "VSLAM Log:"
    cat /tmp/vslam.log | tail -n 25
}

kill $BAG_PID $VSLAM_PID 2>/dev/null || true
wait $VSLAM_PID 2>/dev/null || true
echo "cuVSLAM Test Complete."

echo "=================================================="
echo "2. Testing Isaac ROS ESS with ROS 2 Bag Replay"
echo "=================================================="
ros2 launch my_robot_bringup isaac_ess.launch.py use_sim_time:=true > /tmp/ess.log 2>&1 &
ESS_PID=$!
sleep 6

ros2 bag play "$BAG_PATH" --clock -r 1.0 -d 1 < /dev/null > /tmp/bag_ess.log 2>&1 &
BAG_PID=$!

echo "Waiting for /stereo/disparity or /stereo/depth..."
timeout 15 ros2 topic echo /stereo/depth --once || {
    echo "ESS Log:"
    cat /tmp/ess.log | tail -n 25
}

kill $BAG_PID $ESS_PID 2>/dev/null || true
wait $ESS_PID 2>/dev/null || true
echo "ESS Test Complete."

echo "=================================================="
echo "3. Testing VINS-Fusion with ROS 2 Bag Replay"
echo "=================================================="
mkdir -p /workspaces/isaac_ros-dev/output
ros2 launch my_robot_bringup vins_fusion.launch.py use_sim_time:=true > /tmp/vins.log 2>&1 &
VINS_PID=$!
sleep 4

ros2 bag play "$BAG_PATH" --clock -r 1.0 -d 1 < /dev/null > /tmp/bag_vins.log 2>&1 &
BAG_PID=$!

echo "Waiting for /odometry or /image_track..."
timeout 15 ros2 topic echo /odometry --once || {
    echo "VINS Log:"
    cat /tmp/vins.log | tail -n 25
}

kill $BAG_PID $VINS_PID 2>/dev/null || true
wait $VINS_PID 2>/dev/null || true
echo "VINS-Fusion Test Complete."
