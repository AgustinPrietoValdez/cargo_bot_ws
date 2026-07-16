@echo off
REM Launches the bundled rtx_lidar.py standalone example with the SAME ROS env
REM as launch_isaac_ros.cmd (DOMAIN_ID=1, Discovery Server via fastdds_isaac.xml).
REM Purpose: sanity-check that RTX Lidar can publish at all on this Isaac install.

setlocal EnableDelayedExpansion

echo === RTX Lidar Standalone Sanity Test ===

REM Detect WSL2 IP
for /f "usebackq tokens=*" %%i in (`powershell -NoProfile -Command "(wsl.exe -d Ubuntu-22.04 -- hostname -I).Trim().Split(' ')[0]"`) do (
  set "WSL_IP=%%i"
)
if "!WSL_IP!"=="" (
    echo ERROR: Could not detect WSL2 IP.
    endlocal & exit /b 1
)
echo WSL2 IP detected: !WSL_IP!

REM Patch fastdds_isaac.xml RemoteServer address (same regex as launch_isaac_ros.cmd)
set "XML_FILE=C:\Users\agusp\Documentos\cargo_bot_ws\config\fastdds_isaac.xml"
powershell -NoProfile -Command "$p='%XML_FILE%'; $c=[IO.File]::ReadAllText($p); $new=[regex]::Replace($c, '(<RemoteServer[^>]*?>\s*<metatrafficUnicastLocatorList>\s*<locator>\s*<udpv4>\s*<address>)[^<]+', '${1}!WSL_IP!'); [IO.File]::WriteAllText($p, $new, (New-Object System.Text.UTF8Encoding $false))"

set ROS_DOMAIN_ID=1
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set FASTRTPS_DEFAULT_PROFILES_FILE=%XML_FILE%
echo ROS_DOMAIN_ID=%ROS_DOMAIN_ID%
echo FASTRTPS_DEFAULT_PROFILES_FILE=%FASTRTPS_DEFAULT_PROFILES_FILE%

call "C:\isaacsim_51_ga\setup_ros_env.bat"

echo.
echo Launching standalone... will take 30-90s to initialize.
echo You should see Isaac open a headless window or just terminal output.
echo Watch for "Simulation App Started" then publication of /scan + /point_cloud.
echo Ctrl-C in this terminal to stop.
echo.

"C:\isaacsim_51_ga\python.bat" "C:\isaacsim_51_ga\standalone_examples\api\isaacsim.ros2.bridge\rtx_lidar.py"

endlocal
