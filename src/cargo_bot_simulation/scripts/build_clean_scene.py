# Builds a CLEAN scene file by referencing the existing /cargo_bot subtree
# and adding an OmniLidar via the canonical Python API (NOT the GUI Create menu,
# which adds a confusing `cargo_bot` wrapper Xform).
#
# RUN AS STANDALONE (via python.bat, not from Script Editor):
#   C:\isaacsim_51_ga\python.bat C:\Users\agusp\cargo_bot_ws\src\cargo_bot_simulation\scripts\build_clean_scene.py
#
# It is idempotent: re-running it overwrites scene_clean.usda from scratch.
#
# After running, run_standalone_lidar.cmd will load scene_clean.usda (we update
# SCENE_PATH to point there).

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
import omni.kit.commands
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

# ---- configuration ----------------------------------------------------------
ORIG_SCENE = "C:/Users/agusp/cargo_bot_ws/src/cargo_bot_simulation/scenes/scene.usda"
OUT_SCENE  = "C:/Users/agusp/cargo_bot_ws/src/cargo_bot_simulation/scenes/scene_clean.usda"

# Where to attach the new OmniLidar (parent prim, in the FRESH stage).
LIDAR_PARENT = "/cargo_bot/lidar_link"

# Slamtec RPLIDAR S2E asset bundled with Isaac
LIDAR_CONFIG = "Slamtec/RPLIDAR_S2E"

# Final OmniLidar prim path (we control the name -- no auto-named wrapper).
LIDAR_NAME = "lidar_sensor"

# ---- 1. Create a fresh stage ------------------------------------------------
print(f"[build_clean] creating new stage at {OUT_SCENE}")
ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()

# Set stage units = meters, Z-up
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
stage.SetEditTarget(stage.GetRootLayer())

# Create /World and /cargo_bot xforms as the structural root
world = UsdGeom.Xform.Define(stage, "/World")
print(f"[build_clean] /World created")

# ---- 2. Add `/cargo_bot` as a REFERENCE to the original scene ---------------
# This brings in the robot, articulation root, joints, and the existing
# ActionGraph (cmd_vel chain) WITHOUT copying their bytes -- they live in
# scene.usda and we read-through-reference.
# IMPORTANT: we only reference the /cargo_bot subtree (NOT /Render, which is
# where the corrupted render product cache lives).
print(f"[build_clean] adding reference to /cargo_bot from {ORIG_SCENE}")
cargo_bot = stage.OverridePrim("/cargo_bot")
cargo_bot.GetReferences().AddReference(
    assetPath=ORIG_SCENE,
    primPath="/cargo_bot",
)
stage.SetDefaultPrim(cargo_bot)
print(f"[build_clean] /cargo_bot referenced OK")

# ---- 3. Add the OmniLidar fresh via the canonical Isaac command -------------
# This is the SAME command the GUI Create menu uses, but called directly so
# we control the path and avoid the auto-rename to `cargo_bot`.
lidar_full_path = f"{LIDAR_PARENT}/{LIDAR_NAME}"
print(f"[build_clean] creating OmniLidar at {lidar_full_path}")
result, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path=lidar_full_path,
    parent=None,                  # path is absolute already
    config=LIDAR_CONFIG,
    translation=(0.0, 0.0, 0.0),  # local to lidar_link
    orientation=(1.0, 0.0, 0.0, 0.0),
)
if not result or not sensor:
    print(f"[build_clean] ERROR: IsaacSensorCreateRtxLidar returned ok={result} sensor={sensor}")
    simulation_app.close()
    raise RuntimeError("failed to create OmniLidar")
print(f"[build_clean] OmniLidar prim returned: {sensor.GetPath()}")

# Verify type
lidar_prim = stage.GetPrimAtPath(lidar_full_path)
if lidar_prim and lidar_prim.GetTypeName() == "OmniLidar":
    print(f"[build_clean] verified Type=OmniLidar at {lidar_full_path}")
else:
    actual_type = lidar_prim.GetTypeName() if lidar_prim else "MISSING"
    print(f"[build_clean] WARN: expected OmniLidar at {lidar_full_path}, got {actual_type}")

# ---- 4. Save the clean scene ------------------------------------------------
print(f"[build_clean] exporting to {OUT_SCENE}")
stage.GetRootLayer().Export(OUT_SCENE)
print(f"[build_clean] DONE.  Clean scene at: {OUT_SCENE}")
print(f"[build_clean] Next: update SCENE_PATH in standalone_lidar_publisher.py to:")
print(f"[build_clean]   {OUT_SCENE}")
print(f"[build_clean] And update LIDAR_PRIM_PATH to:")
print(f"[build_clean]   {lidar_full_path}")

simulation_app.close()
