@echo off
REM Test: Launch Isaac Sim with Simple Discovery (no custom XML)
set ROS_DOMAIN_ID=1
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set FASTRTPS_DEFAULT_PROFILES_FILE=
call "C:\isaacsim_51_ga\setup_ros_env.bat"
echo ROS_DOMAIN_ID=%ROS_DOMAIN_ID%
echo RMW=%RMW_IMPLEMENTATION%
echo FASTRTPS_DEFAULT_PROFILES_FILE=%FASTRTPS_DEFAULT_PROFILES_FILE%
echo Launching Isaac Sim (Simple Discovery test)...
"C:\isaacsim_51_ga\isaac-sim.bat" --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge
