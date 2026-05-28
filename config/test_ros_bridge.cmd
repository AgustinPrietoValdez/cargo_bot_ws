@echo off
set ROS_DOMAIN_ID=1
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set FASTRTPS_DEFAULT_PROFILES_FILE=%~dp0fastdds_isaac.xml
set PATH=%PATH%;C:\isaacsim_51_ga\exts\isaacsim.ros2.bridge\humble\lib
call "C:\isaacsim_51_ga\setup_ros_env.bat"
echo Starting test publisher...
"C:\isaacsim_51_ga\python.bat" "%~dp0test_ros_bridge.py" > "%~dp0test_output.log" 2>&1
echo Exit code: %ERRORLEVEL% >> "%~dp0test_output.log"
