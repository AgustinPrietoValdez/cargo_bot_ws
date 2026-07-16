#!/bin/bash
# Source this file in WSL2 before running any ROS 2 nodes for cargo_bot
# Usage: source /mnt/c/Users/agusp/Documentos/cargo_bot_ws/config/source_ros_wsl.sh

export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/mnt/c/Users/agusp/Documentos/cargo_bot_ws/config/fastdds_wsl.xml

source /opt/ros/humble/setup.bash

echo "[cargo_bot] ROS 2 environment ready"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  RMW=$RMW_IMPLEMENTATION"
echo "  DDS Profile=$FASTRTPS_DEFAULT_PROFILES_FILE"
