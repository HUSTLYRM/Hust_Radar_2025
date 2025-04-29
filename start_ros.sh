#!/bin/bash

# 启动第一个终端并运行 roscore
gnome-terminal -- bash -c "roscore; exec bash"

# 启动第二个终端并运行 livox_ros_driver
gnome-terminal -- bash -c "cd ~/catkin_ws/ws_livox && source devel/setup.sh && roslaunch livox_ros_driver livox_lidar.launch; exec bash"

# 启动第三个终端并运行 pclmatcher
gnome-terminal -- bash -c "cd ~/RadarWorkspace/code/PCLMATCHER && source devel/setup.sh && roslaunch pclmatcher pclmatcher.launch; exec bash"
