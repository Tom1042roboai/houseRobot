# HouseRobot stage 1: Mini ROS 4wd Robotic Vehicle
In the first stage of the HouseRobot project, I aim to design and build a small ROS-based (Robot Operating System) vehicle capable of mapping and navigating around the house. Subsequent stages will build upon this foundation. Through this stage, I plan to gain further experience with the following technologies:
- ROS
- LiDAR
- SLAM
- Navigation
- Frontier Exploration
- Ubuntu for embedded devices

<p align="center">
  <img src="Robot_pic.png" alt="Robot Image" width="200">
  <img src="SLAM_map_1.png" alt="Map Image 1" width="265">
  <img src="SLAM_map_2.png" alt="Map Image 2" width="235">
  <img src="SLAM_map_3.png" alt="Map Image 3" width="250">
</p>

## Hardware
- Raspberry Pi 4 Computer Model B 2GB RAM
- RPLIDAR A1M8
- Arduino UNO R3
- L293D Motor Driver Shield
- DFRobot 4WD Mobile Platform for Arduino
- Power bank with 5V/3A power supply

## Software Requirements
### Remote Laptop
- Ubuntu 20.04 LTS WSL2 (Remote Laptop)
- ROS Noetic
- ROS Noetic packages:
  - Rviz
  - rosserial: https://github.com/ros-drivers/rosserial/tree/noetic-devel
- Arduino IDE:
  - If you want to edit the [houseRobo_control.ino](Arduino/houseRobo_control/houseRobo_control.ino) file
  - rosserial_arduino: generated and copied over from Ubuntu ROS Noetic setup (More details in [Project Setup](#project-setup) section)

### 4wd Robotic Vehicle
- Ubuntu Server 20.04 (Robot vehicle)
  - GUI components removed to save memory capacity
- ROS Noetic
- ROS Noetic Packages:
  - rplidar_ros: https://github.com/robopeak/rplidar_ros/tree/master
  - slam_gmapping: https://github.com/ros-perception/slam_gmapping/tree/melodic-devel
  - openslam_gmapping: https://github.com/ros-perception/openslam_gmapping/tree/melodic-devel
  - explore_lite: https://github.com/hrnr/m-explore/tree/noetic-devel
  - rosserial: https://github.com/ros-drivers/rosserial/tree/noetic-devel
  - navigation stack: sudo apt install ros-noetic-navigation

## High-level View of Project Components
![Alt text](high_level_diagram.png)

## Project Setup
### Remote Laptop
1. Install Arduino IDE for the laptop.
2. Install ROS Noetic on the Ubuntu laptop or terminal.
3. Install the ROS Noetic packages listed in [Software Requirements](#software-requirements) section.
4. Copy the [slam_remote_config.rviz.yaml](remote_laptop_files/slam_remote_config.rviz.yaml) file into your Ubuntu environment.

#### Generate rosserial libraries for Arduino IDE (optional)
If you want to edit and compile the rosserial arduino file.

1. Source your ROS Noetic workspace setup.
2. Generate rosserial libraries for arduino: The cmd will generate a ros_lib folder.
```bash
rosrun rosserial_arduino make_libraries.py
```  
3. Copy the ros_lib folder to the Arduino/libraries folder for your Arduino IDE.

### 4wd Robotic Vehicle

## Run Project

## Additional Files In Repo

## Additional Project Notes

## Limitations and Future Work