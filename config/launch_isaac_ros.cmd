@echo off
REM Launch script for Isaac Sim with ROS 2 Discovery Server (cargo_bot)
REM Detects WSL2 IP, patches fastdds_isaac.xml, sets env vars, launches Isaac Sim

setlocal enabledelayedexpansion

echo === Cargo Bot: Isaac Sim ROS 2 Launcher ===

REM Detect WSL2 IP address
for /f "tokens=*" %%i in ('wsl -d Ubuntu-22.04 -- hostname -I') do set WSL_RAW=%%i
for /f "tokens=1" %%a in ("%WSL_RAW%") do set WSL_IP=%%a

if "%WSL_IP%"=="" (
    echo ERROR: Could not detect WSL2 IP address. Is WSL running?
    exit /b 1
)
echo WSL2 IP detected: %WSL_IP%

REM Patch the FastDDS XML with current WSL IP
set CONFIG_DIR=%~dp0
set XML_FILE=%CONFIG_DIR%fastdds_isaac.xml
set XML_TEMPLATE=%CONFIG_DIR%fastdds_isaac.xml

REM Use PowerShell to replace the IP placeholder or previous IP
powershell -Command "(Get-Content '%XML_FILE%') -replace '<address>[^<]+</address>', '<address>%WSL_IP%</address>' | Set-Content '%XML_FILE%' -Encoding UTF8"

echo FastDDS XML updated with WSL IP: %WSL_IP%

REM Set environment variables
set ROS_DOMAIN_ID=1
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set FASTRTPS_DEFAULT_PROFILES_FILE=%XML_FILE%

echo ROS_DOMAIN_ID=%ROS_DOMAIN_ID%
echo RMW_IMPLEMENTATION=%RMW_IMPLEMENTATION%
echo FASTRTPS_DEFAULT_PROFILES_FILE=%FASTRTPS_DEFAULT_PROFILES_FILE%

REM Source Isaac Sim ROS env
call "C:\isaac-sim\setup_ros_env.bat"

echo.
echo Environment ready. Launching Isaac Sim...
echo (Make sure Discovery Server is running in WSL2 first!)
echo.

REM Launch Isaac Sim with ROS 2 bridge enabled
"C:\isaac-sim\isaac-sim.bat" --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge

endlocal
