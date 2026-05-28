# SPDX-License-Identifier: Apache-2.0
# ----------------------------------------------------------------------------------
# add_lidar.py  --  cargo_bot_ws / Isaac Sim 5.1.0-rc.19
#
# PURPOSE
#   Cleanup-and-create RTX Lidar in the current scene.  Pragmatic approach:
#   we no longer try to force the prim into /cargo_bot/lidar_link/lidar_sensor
#   (multiple Isaac 5.1.0-rc.19 bugs block that).  Instead we follow the bundled
#   standalone rtx_lidar.py pattern: create at a top-level path, accept whatever
#   path the command produces, and surface it so publish_lidar.py + the user
#   know what to use.
#
#   The lidar PRIM's position in the USD hierarchy is FUNCTIONALLY irrelevant.
#   ROS 2 / RViz / Nav2 position the scan via frame_id="lidar_link" published
#   on /tf -- not via the prim path in Isaac.
#
# HOW TO RUN
#   1. Window -> Script Editor in Isaac Sim
#   2. File -> Open ... -> this file
#   3. Click Run
#
# WHAT IT DOES
#   step 0  delete any prior OmniLidar prims in the scene (idempotent)
#   step 1  create one fresh OmniLidar via the standalone pattern
#   step 2  print the resulting prim path -- use this in publish_lidar.py
# ----------------------------------------------------------------------------------

import omni.kit.commands
import omni.usd
from pxr import Gf

# Use the BUNDLED Example_Rotary_2D config -- proven working in the standalone
# example (publishes /scan at 15.5 Hz).  The Slamtec preset has issues in
# 5.1.0-rc.19 where the resolver doesn't fully apply the sensor config and the
# RTX renderer returns 0 rays.  We'll tune ranges/scan rate post-creation if
# needed.
CONFIG = "Example_Rotary_2D"
# New scene namespace: robot is under /World/cargo_bot (NOT /cargo_bot)
PARENT_LINK = "/World/cargo_bot/lidar_link"
LIDAR_LEAF = "lidar_sensor"


def _log(msg):
    print(f"[add_lidar] {msg}")


def main():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        _log("FATAL: no stage open.  Open or create a scene first.")
        return
    _log("step 0a stage acquired")

    # ---- step 0: delete ALL existing OmniLidar / Camera leftovers ----
    # (idempotent; cleans Camera-fallback from URDF importer + any prior OmniLidar)
    to_delete = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        tname = prim.GetTypeName()
        if tname == "OmniLidar":
            to_delete.append(path)
        # URDF importer's `<sensor type="ray">` fallback creates a Camera under
        # lidar_link.  Kill it so we can replace with a real OmniLidar.
        if tname == "Camera" and "lidar_link" in path:
            to_delete.append(path)
        # Old wrappers from previous attempts
        name = prim.GetName()
        if "lidar_sensor" in name and tname not in ("OmniLidar", "Camera"):
            for child in prim.GetChildren():
                if child.GetTypeName() == "OmniLidar":
                    to_delete.append(path)
                    break

    # Also delete known historic paths
    for candidate in (
        "/cargo_bot_lidar_link_lidar_sensor",
        "/lidar_sensor",
        "/cargo_bot/lidar_sensor",
        "/cargo_bot/lidar_link/lidar_sensor",
        "/World/cargo_bot/lidar_link/lidar_sensor",
        "/World/cargo_bot/lidar_link/rplidar",
    ):
        if stage.GetPrimAtPath(candidate) and stage.GetPrimAtPath(candidate).IsValid():
            to_delete.append(candidate)

    # Dedupe & delete
    to_delete = sorted(set(to_delete))
    for p in to_delete:
        try:
            omni.kit.commands.execute("DeletePrims", paths=[p])
            _log(f"step 0b deleted {p}")
        except Exception as e:
            _log(f"step 0b could not delete {p}: {e}")

    # ---- step 1: create the lidar nested under /World/cargo_bot/lidar_link --
    # With /World as defaultPrim and the URDF imported there, IsaacSensorCreateRtxLidar
    # should resolve paths correctly (no underscore-mangling).
    # We pass `path=LEAF_NAME` and `parent=PARENT_LINK` per Isaac convention.
    parent_prim = stage.GetPrimAtPath(PARENT_LINK)
    if not parent_prim or not parent_prim.IsValid():
        _log(f"FATAL: parent missing: {PARENT_LINK}")
        return
    _log(f"step 0c parent {PARENT_LINK} OK")
    result, sensor_prim = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=LIDAR_LEAF,
        parent=PARENT_LINK,
        config=CONFIG,
        translation=Gf.Vec3d(0.0, 0.0, 0.0),   # inherit lidar_link's transform
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    if not result:
        _log(f"FATAL: command returned result=False.")
        return

    # ---- step 2: find the actual OmniLidar prim in the stage ----
    # (the command may have placed it under a different path due to the
    #  default-prim auto-prefix behavior in 5.1.0-rc.19)
    omnilidar_paths = []
    for prim in stage.Traverse():
        if prim.GetTypeName() == "OmniLidar":
            omnilidar_paths.append(str(prim.GetPath()))

    if not omnilidar_paths:
        _log("FATAL: no OmniLidar prim found in stage after creation.")
        return

    if len(omnilidar_paths) > 1:
        _log(f"WARNING: found multiple OmniLidars (cleanup missed some?):")
        for p in omnilidar_paths:
            _log(f"           {p}")

    actual_path = omnilidar_paths[0]
    _log(f"step 1 created OmniLidar at {actual_path}")
    _log(f"step 2 use THIS path in publish_lidar.py's LIDAR_PRIM_PATH:")
    _log(f"         LIDAR_PRIM_PATH = \"{actual_path}\"")
    _log("DONE.  Now press Play and run publish_lidar.py with the path above.")


main()
