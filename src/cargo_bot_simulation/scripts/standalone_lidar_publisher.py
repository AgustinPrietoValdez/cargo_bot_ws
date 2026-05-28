# SPDX-License-Identifier: Apache-2.0
# ----------------------------------------------------------------------------------
# standalone_lidar_publisher.py  --  cargo_bot_ws / Isaac Sim 5.1.0-rc.19
#
# Loads scene_v2.usda (the user's diff-drive robot scene) in a HEADLESS
# SimulationApp, then ADDS an RTX Lidar at world root using the canonical
# pattern from the bundled rtx_lidar.py example -- the only path proven to
# publish /scan in 5.1.0-rc.19.
#
# Run from Windows terminal:
#   src/cargo_bot_simulation/launch/run_standalone_lidar.cmd
#
# Or directly:
#   C:\isaacsim_51_ga\python.bat <this file>
#
# Output: /scan_py topic in ROS 2 (DOMAIN_ID=1), frame_id=lidar_link.
#
# IMPORTANT: The OmniLidar lives at /lidar_sensor (world root), NOT under
# /cargo_bot/lidar_link.  The scan still appears at lidar_link in TF because
# we publish with frame_id=lidar_link explicitly.  RViz/Nav2 use TF to
# position the scan correctly -- the USD prim location is irrelevant.
# ----------------------------------------------------------------------------------

import os
import sys

# ---------------------------------------------------------------------------
# 1. SimulationApp -- MUST come before any other omni.* imports.
# ---------------------------------------------------------------------------
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,   # show GUI so we can see what's happening (use True for prod)
    "renderer": "RaytracedLighting",
})

import carb
import omni.usd
import omni.kit.commands
import omni.kit.app
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.stage import is_stage_loading
from pxr import Gf

# ---------------------------------------------------------------------------
# 1b. Load required extensions BEFORE importing replicator (so writers
# register).  In a headless SimulationApp, these are NOT auto-loaded the way
# the bundled standalone example expects.
# ---------------------------------------------------------------------------
ext_mgr = omni.kit.app.get_app().get_extension_manager()
for ext_id in (
    "isaacsim.ros2.bridge",        # registers RtxLidarROS2PublishLaserScan, etc
    "isaacsim.sensors.rtx",        # registers OmniLidar prim type + sensor configs
    "omni.replicator.core",        # base replicator
):
    if not ext_mgr.is_extension_enabled(ext_id):
        print(f"[standalone_lidar] Loading extension {ext_id}...")
        ext_mgr.set_extension_enabled_immediate(ext_id, True)
    else:
        print(f"[standalone_lidar] Extension {ext_id} already loaded")

simulation_app.update()
simulation_app.update()

import omni.replicator.core as rep

# ---------------------------------------------------------------------------
# 2. Open scene_v2.usda
# ---------------------------------------------------------------------------
SCENE_PATH = "C:/Users/agusp/cargo_bot_ws/src/cargo_bot_simulation/scenes/scene_v2.usda"
LIDAR_PATH = "/lidar_sensor"   # world root, single component (proven pattern)
LIDAR_CONFIG = "Example_Rotary_2D"   # bundled config, proven to work
TOPIC_NAME = "/scan_py"
FRAME_ID = "lidar_link"

# The lidar's world position: roughly above the robot.  cargo_bot starts at
# world origin with base_footprint on the ground, lidar_link at z=0.32 above.
# Place lidar at z=0.35 to be just above lidar_link (avoids self-intersection
# with the chassis mesh).  When the robot drives, this lidar STAYS in place
# (limitation of placing at world root) but TF frame_id=lidar_link is what
# ROS uses to position the scan -- the USD position is just where the rays
# physically originate in the sim.
LIDAR_TRANSLATE = (0.0, 0.0, 0.35)

if not os.path.isfile(SCENE_PATH):
    carb.log_error(f"Scene file not found: {SCENE_PATH}")
    simulation_app.close()
    sys.exit(1)

print(f"[standalone_lidar] Opening stage: {SCENE_PATH}")
omni.usd.get_context().open_stage(SCENE_PATH, None)
simulation_app.update()
simulation_app.update()
while is_stage_loading():
    simulation_app.update()
print("[standalone_lidar] Stage loaded.")

stage = omni.usd.get_context().get_stage()

# ---------------------------------------------------------------------------
# 3. SimulationContext (required before creating sensors)
# ---------------------------------------------------------------------------
simulation_context = SimulationContext(
    physics_dt=1.0 / 60.0,
    rendering_dt=1.0 / 60.0,
    stage_units_in_meters=1.0,
)
simulation_app.update()

# ---------------------------------------------------------------------------
# 4. Create the OmniLidar at world root.  EXACT pattern from
#    C:\isaacsim_51_ga\standalone_examples\api\isaacsim.ros2.bridge\rtx_lidar.py
#    lines 57-64.  This is the only path proven to fire rays in 5.1.0-rc.19.
# ---------------------------------------------------------------------------
print(f"[standalone_lidar] Creating OmniLidar at {LIDAR_PATH} (config={LIDAR_CONFIG})...")
_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path=LIDAR_PATH,
    parent=None,
    config=LIDAR_CONFIG,
    translation=LIDAR_TRANSLATE,
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
)
if not sensor or not sensor.IsValid():
    carb.log_error("Failed to create OmniLidar")
    simulation_app.close()
    sys.exit(2)
print(f"[standalone_lidar] OmniLidar created: {sensor.GetPath()} type={sensor.GetTypeName()}")

# ---------------------------------------------------------------------------
# 5. Create render product (Replicator + SDG pipeline).  Same pattern as
#    rtx_lidar.py:67.
# ---------------------------------------------------------------------------
hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="CargoBotLidarRP")
print(f"[standalone_lidar] RenderProduct: {hydra_texture.path}")

# ---------------------------------------------------------------------------
# 6. Attach the LaserScan writer (rtx_lidar.py:95-97 pattern)
# ---------------------------------------------------------------------------
laser_writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
laser_writer.initialize(topicName=TOPIC_NAME, frameId=FRAME_ID)
laser_writer.attach([hydra_texture])
print(f"[standalone_lidar] LaserScan writer attached: topic={TOPIC_NAME} frame={FRAME_ID}")

# Debug-draw (optional but useful for verification)
try:
    debug_writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
    debug_writer.attach([hydra_texture])
    print("[standalone_lidar] DebugDrawPointCloud attached -- expect ring of rays in viewport")
except Exception as e:
    print(f"[standalone_lidar] (info) DebugDraw not available: {e}")

simulation_app.update()

# ---------------------------------------------------------------------------
# 7. Play the timeline.  This is what actually starts physics + ROS publishing.
# ---------------------------------------------------------------------------
print("[standalone_lidar] Starting timeline...")
simulation_context.play()
print("[standalone_lidar] PLAYING.  Verify in WSL:")
print(f"[standalone_lidar]   ros2 topic hz {TOPIC_NAME}")

# ---------------------------------------------------------------------------
# 8. Main loop -- step the sim forever (Ctrl-C to stop)
# ---------------------------------------------------------------------------
try:
    while simulation_app.is_running():
        simulation_app.update()
except KeyboardInterrupt:
    print("[standalone_lidar] Interrupted by user, shutting down...")

simulation_app.close()
print("[standalone_lidar] DONE.")
