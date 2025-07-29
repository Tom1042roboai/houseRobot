#!/bin/bash

echo "[INFO] Saving SLAM map..."
rosrun map_server map_saver -f ~/maps/final_map

sleep 2

echo "[INFO] ROS bag recording..."
rosbag record -a -o ~/bags/nav_run

sleep 5

echo "[INFO] Shutting down robot now..."
sudo poweroff