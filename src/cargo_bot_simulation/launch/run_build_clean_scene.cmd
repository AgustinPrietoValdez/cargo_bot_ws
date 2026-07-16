@echo off
REM ====================================================================
REM One-shot builder: creates scene_clean.usda by referencing /cargo_bot
REM from scene.usda and adding a fresh OmniLidar via the canonical API
REM (NOT the GUI Create menu, which adds a confusing `cargo_bot` wrapper).
REM
REM PREREQUISITE: Close the Isaac Sim GUI before running this.
REM
REM After this finishes, the user must:
REM   1) Update SCENE_PATH and LIDAR_PRIM_PATH in
REM        ../scripts/standalone_lidar_publisher.py
REM      to point at the new scene_clean.usda and lidar_sensor path
REM      (the script prints the exact values to use).
REM   2) Then run run_standalone_lidar.cmd as usual.
REM ====================================================================

setlocal

echo === cargo_bot_ws clean scene builder ===

set ROS_DOMAIN_ID=1
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set FASTRTPS_DEFAULT_PROFILES_FILE=C:\Users\agusp\Documentos\cargo_bot_ws\config\fastdds_isaac.xml

call "C:\isaacsim_51_ga\setup_ros_env.bat"

echo.
echo Launching build_clean_scene.py ...
echo (one-shot ~30-60s)
echo.

"C:\isaacsim_51_ga\python.bat" "C:\Users\agusp\Documentos\cargo_bot_ws\src\cargo_bot_simulation\scripts\build_clean_scene.py"

echo.
echo === done ===
pause
endlocal
