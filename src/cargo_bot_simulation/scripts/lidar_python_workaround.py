# =============================================================================
# Python-based RTX Lidar -> ROS 2 /laser_scan publisher (bypasses ActionGraph)
# =============================================================================
# WHEN TO USE: when the ActionGraph's `isaac_create_render_product.outputs:
# renderProductPath` stays None at runtime and `ros2 topic hz /laser_scan`
# reports 0 msg/s despite the helper showing publisher_count=1.
#
# HOW TO USE:
#   1. In Isaac Sim, open Window -> Script Editor.
#   2. Make sure simulation is STOPPED (square Stop button on the timeline).
#   3. Paste the ENTIRE contents of this file into the Script Editor.
#   4. Click Run.  Watch for the "[lidar_workaround] ... DONE." line.
#   5. Press Play in the timeline.
#   6. In WSL:   ros2 topic hz /laser_scan_py
#      Expect: ~10 Hz (S2E preset scanRateBaseHz=10).
#      Echo:   ros2 topic echo /laser_scan_py --once
#
# WHAT IT DOES:
#   - Disables the existing helper OG node (so we don't fight it).
#   - Creates a fresh render product on the OmniLidar EXPLICITLY requesting
#     `render_vars=["GenericModelOutput", "RtxSensorMetadata"]`  -- the
#     canonical pair documented in
#     C:\isaacsim_51_ga\exts\isaacsim.sensors.rtx\isaacsim\sensors\rtx\tests\test_annotators.py:255
#     This is the AOV set the RTXSensor renderer writes into for OmniLidar.
#   - Acquires the registered writer `RtxLidarROS2PublishLaserScan` (from
#     `isaacsim.ros2.bridge`), initializes it with our topic/frame, and
#     attaches it to the new render product.
#   - Also attaches the `RtxLidarDebugDrawPointCloud` writer so you see rays
#     in the viewport (matches the official rtx_lidar.py standalone example
#     line 100-101).
#
# WHY this bypasses the bug:
#   The diag showed that `outputs:renderProductPath` on the OG
#   IsaacCreateRenderProduct node stays None at runtime.  Possible causes
#   (any of them is enough -- the workaround removes all three):
#     (a) The `inputs:cameraPrim` relationship on the OG node fails to
#         resolve at compute() time, so compute() returns early.
#     (b) The OG `RunOneSimulationFrame` node's `step` exec never fires
#         because of an evaluator/pipeline ordering bug in 5.1.0-rc.19.
#     (c) The 3-then-1 leftover Replicator prims that auto-rematerialize
#         on Play poison the render-product cache.
#   This workaround creates the render product directly via the Replicator
#   Python API, naming it deterministically so it does NOT collide with
#   any auto-Replicators, and attaches writers explicitly.
#
# IF YOU SEE NOTHING ON /laser_scan_py AFTER 5 SECONDS OF PLAY:
#   - In Script Editor:   print(rep.WriterRegistry.get_writers(category="isaacsim.ros2.bridge"))
#     If the list is empty, the ROS 2 bridge extension didn't load.
#     Window -> Extensions -> search "ROS 2 Bridge" -> enable, restart Play.
#   - Confirm WSL is on ROS_DOMAIN_ID=1 and Discovery Server is up.
#     Run on Isaac side from a powershell:
#       ros2 daemon stop ; ros2 topic list   (use the Isaac bundled ROS 2)
#     Should also see /laser_scan_py.  If not, it's a DDS issue, not Isaac.
#
# =============================================================================

import carb
import omni
import omni.usd
import omni.replicator.core as rep
import omni.graph.core as og
from pxr import Sdf, Usd

# ---- configuration ----------------------------------------------------------
LIDAR_PRIM_PATH = "/cargo_bot/lidar_link/cargo_bot/RPLIDAR_S2E/RPLidar_S2E"
TOPIC_NAME      = "/laser_scan_py"   # new topic name so it doesn't collide
                                     # with the (broken) OG-attached one
FRAME_ID        = "lidar_link"
HELPER_OG_PATH  = "/cargo_bot/ActionGraph/ros2_rtx_lidar_helper"
# -----------------------------------------------------------------------------

print("[lidar_workaround] starting...")

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("[lidar_workaround] No stage open.  Open scene.usda first.")

# --- 0. Sanity check the lidar prim --------------------------------------------
lidar_prim = stage.GetPrimAtPath(LIDAR_PRIM_PATH)
if not lidar_prim or not lidar_prim.IsValid():
    raise RuntimeError(
        f"[lidar_workaround] No prim at {LIDAR_PRIM_PATH}.  "
        f"Open the cargo_bot scene first."
    )
if lidar_prim.GetTypeName() != "OmniLidar":
    raise RuntimeError(
        f"[lidar_workaround] Prim at {LIDAR_PRIM_PATH} is type "
        f"{lidar_prim.GetTypeName()}, expected OmniLidar."
    )
if not lidar_prim.HasAPI("OmniSensorGenericLidarCoreAPI"):
    raise RuntimeError(
        f"[lidar_workaround] Prim at {LIDAR_PRIM_PATH} lacks "
        f"OmniSensorGenericLidarCoreAPI -- the OmniGraph helper validator "
        f"on line 82-87 of OgnROS2RtxLidarHelper.py requires it."
    )
print(f"[lidar_workaround] OmniLidar prim verified: {LIDAR_PRIM_PATH}")

# --- 1. Disable the (broken) OG lidar chain so it doesn't fight us -----------
# Disables: ros2_rtx_lidar_helper, isaac_create_render_product, isaac_run_one_simulation_frame
# isaac_create_render_product errors with "No valid sensor paths provided" if its
# cameraPrim relationship target was deleted (stale ref) -- killing the whole graph eval.
og_paths_to_disable = [
    HELPER_OG_PATH,
    "/cargo_bot/ActionGraph/isaac_create_render_product",
    "/cargo_bot/ActionGraph/isaac_run_one_simulation_frame",
]
for og_path in og_paths_to_disable:
    prim = stage.GetPrimAtPath(og_path)
    if prim and prim.IsValid():
        # SetActive(False) deactivates the prim entirely so OG evaluator skips it.
        prim.SetActive(False)
        # Also set inputs:enabled = False as a belt-and-suspenders.
        en = prim.GetAttribute("inputs:enabled")
        if not en:
            en = prim.CreateAttribute("inputs:enabled", Sdf.ValueTypeNames.Bool)
        en.Set(False)
        print(f"[lidar_workaround] Disabled OG node {og_path}")
    else:
        print(f"[lidar_workaround] (info) No OG node at {og_path} -- skipping")

# --- 2. Make sure ROS 2 bridge extension is loaded ---------------------------
import omni.kit.app
ext_mgr = omni.kit.app.get_app().get_extension_manager()
if not ext_mgr.is_extension_enabled("isaacsim.ros2.bridge"):
    print("[lidar_workaround] enabling isaacsim.ros2.bridge ...")
    ext_mgr.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
# Force registration of writers in the registry (idempotent)
if not ext_mgr.is_extension_enabled("isaacsim.sensors.rtx"):
    print("[lidar_workaround] enabling isaacsim.sensors.rtx ...")
    ext_mgr.set_extension_enabled_immediate("isaacsim.sensors.rtx", True)

# --- 3. Create render product with EXPLICIT AOVs -----------------------------
# This is the canonical pattern from test_annotators.py line 255-260 and
# is the only way to GUARANTEE the GenericModelOutput AOV is bound.
# `name="CargoBotLidarRP"` -> deterministic, doesn't collide with auto-Replicator_NN
print("[lidar_workaround] creating render product on OmniLidar with explicit AOVs...")
hydra_texture = rep.create.render_product(
    LIDAR_PRIM_PATH,
    [1, 1],                                                # resolution -- 1x1 is fine for lidar
    name="CargoBotLidarRP",
    render_vars=["GenericModelOutput", "RtxSensorMetadata"],
)
print(f"[lidar_workaround] render product path = {hydra_texture.path}")

# --- 4. Attach LaserScan writer ---------------------------------------------
print("[lidar_workaround] acquiring writer RtxLidarROS2PublishLaserScan ...")
writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
if writer is None:
    raise RuntimeError(
        "[lidar_workaround] Writer 'RtxLidarROS2PublishLaserScan' not in "
        "registry. The isaacsim.ros2.bridge extension probably didn't fully "
        "initialize.  In the Console you should see "
        "'[isaacsim.ros2.bridge] Extension started'.  If not, disable then "
        "re-enable the extension via Window -> Extensions."
    )

writer.initialize(
    topicName  = TOPIC_NAME,
    frameId    = FRAME_ID,
    nodeNamespace = "",
    queueSize  = 10,
    qosProfile = "",
    context    = 0,            # 0 = default ROS context
)
writer.attach([hydra_texture])
print(f"[lidar_workaround] LaserScan writer attached -> topic '{TOPIC_NAME}'")

# --- 5. Attach DebugDraw writer for visual confirmation -----------------------
print("[lidar_workaround] acquiring debug-draw writer RtxLidarDebugDrawPointCloud ...")
debug_writer = rep.writers.get("RtxLidarDebugDrawPointCloud")
if debug_writer is not None:
    try:
        debug_writer.initialize(doTransform=True)
        debug_writer.attach([hydra_texture])
        print("[lidar_workaround] DebugDraw writer attached -- you should see rays after Play.")
    except Exception as e:
        carb.log_warn(f"[lidar_workaround] DebugDraw attach failed (non-fatal): {e}")
else:
    print("[lidar_workaround] (info) DebugDraw writer not registered -- skipping")

# --- 6. Stash references on the carb settings so Python GC doesn't kill them -
#       (the writers/render product are not USD prims; if the Python wrapper
#        falls out of scope, the underlying Hydra texture is destroyed)
import builtins
if not hasattr(builtins, "_cargo_bot_lidar_keepalive"):
    builtins._cargo_bot_lidar_keepalive = {}
builtins._cargo_bot_lidar_keepalive["render_product"] = hydra_texture
builtins._cargo_bot_lidar_keepalive["writer_laser"]   = writer
builtins._cargo_bot_lidar_keepalive["writer_debug"]   = debug_writer

print("[lidar_workaround] DONE.  Press Play.  Then in WSL:")
print(f"[lidar_workaround]   ros2 topic hz {TOPIC_NAME}")
print(f"[lidar_workaround]   ros2 topic echo {TOPIC_NAME} --once")
print("[lidar_workaround] If hz is 0:")
print("[lidar_workaround]   - Confirm timeline is PLAYING.")
print("[lidar_workaround]   - Confirm ROS_DOMAIN_ID=1 and Discovery Server is up in WSL.")
print("[lidar_workaround]   - Run, in Script Editor while playing:")
print("[lidar_workaround]       hp = builtins._cargo_bot_lidar_keepalive['render_product'].path")
print("[lidar_workaround]       print('render product:', hp)")
print("[lidar_workaround]       print('writers attached to it: see writer registry')")
