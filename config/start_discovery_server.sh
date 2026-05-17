#!/bin/bash
# Start FastDDS Discovery Server for cargo_bot project
# Run this in WSL2 BEFORE launching Isaac Sim or any ROS 2 nodes

set -e

export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash

echo "=== Cargo Bot Discovery Server ==="
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "RMW=$RMW_IMPLEMENTATION"
echo "WSL IP: $(hostname -I | awk '{print $1}')"
echo "Listening on port 11811 (server ID 0)"
echo "Press Ctrl+C to stop"
echo "==================================="

fastdds discovery -i 0 -p 11811
