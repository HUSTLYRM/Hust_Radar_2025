#!/bin/bash
# 杀死已启动的 roscore 进程
echo "Killing existing roscore processes..."
pkill -f roscore

# 启动第一个终端并运行 roscore
echo "Starting roscore in terminal 1..."
gnome-terminal -- bash -c "roscore; exec bash"

sleep 5

# 启动第二个终端并运行 livox_ros_driver
echo "Starting livox_ros_driver in terminal 2..."
gnome-terminal -- bash -c "cd ~/catkin_ws/ws_livox && source devel/setup.sh && roslaunch livox_ros_driver livox_lidar.launch; exec bash"

sleep 5

# 启动第三个终端并运行 pclmatcher
echo "Starting pclmatcher in terminal 3..."
gnome-terminal -- bash -c "cd ~/RadarWorkspace/code/PCLMATCHER && source devel/setup.sh && roslaunch pclmatcher pclmatcher.launch; exec bash"
