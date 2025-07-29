#!/bin/bash

echo "[INFO] Saving SLAM map..."
rosrun map_server map_saver -f ~/ros_files/maps/final_map

sleep 2

echo "[INFO] ROS bag recording..."
rosbag record -a -o ~/ros_files/bags/nav_run

sleep 5

echo "[INFO] Shutting down robot now..."
sudo poweroff