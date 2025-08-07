#!/bin/bash

SESSION="robot_startup"

# Kill existing session
tmux kill-session -t $SESSION 2>/dev/null

# Start new session
tmux new-session -d -s $SESSION

# Window 1: roscore
tmux rename-window -t $SESSION:0 'roscore'
tmux send-keys -t $SESSION:0 'source /opt/ros/noetic/setup.bash' C-m
tmux send-keys -t $SESSION:0 'export ROS_MASTER_URI=http://<robot_ip>:11311' C-m
tmux send-keys -t $SESSION:0 'export ROS_HOSTNAME=<robot_ip>' C-m
tmux send-keys -t $SESSION:0 'source ~/houseRobo/house_robo_ros1_ws/devel/setup.bash' C-m
tmux send-keys -t $SESSION:0 'roscore' C-m

# Wait for roscore to start
sleep 3

# Window 2: roslaunch
tmux new-window -t $SESSION:1 -n 'roslaunch'
tmux send-keys -t $SESSION:1 'source /opt/ros/noetic/setup.bash' C-m
tmux send-keys -t $SESSION:1 'export ROS_MASTER_URI=http://<robot_ip>:11311' C-m
tmux send-keys -t $SESSION:1 'export ROS_HOSTNAME=<robot_ip>' C-m
tmux send-keys -t $SESSION:1 'source ~/houseRobo/house_robo_ros1_ws/devel/setup.bash' C-m
tmux send-keys -t $SESSION:1 'roslaunch houseRobo_package robot_bringup.launch' C-m

# Wait for roslaunch
sleep 5

# Attach to session
tmux attach-session -t $SESSION