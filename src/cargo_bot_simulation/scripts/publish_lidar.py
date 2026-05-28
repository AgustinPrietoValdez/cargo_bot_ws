# SPDX-License-Identifier: Apache-2.0
# ----------------------------------------------------------------------------------
# publish_lidar.py  --  cargo_bot_ws / Isaac Sim 5.1.0-rc.19
#
# PURPOSE
#   Attach a CLEAN Replicator render product to /cargo_bot/lidar_link/lidar_sensor
#   and a RtxLidarROS2PublishLaserScan writer that publishes to /scan_py with
#   frame_id "lidar_link".  Bypasses the buggy "Tools -> Robotics -> ROS 2
#   OmniGraphs -> RTX Lidar" shortcut whose IsaacCreateRenderProduct OG node
#   silently spawns a new Replicator render product (with the wrong LdrColor AOV)
#   on every Play, every Save, and every shortcut invocation.
#
#   Pattern lifted verbatim from the working bundled example at
#       C:\isaacsim_51_ga\standalone_examples\api\isaacsim.ros2.bridge\rtx_lidar.py
#   and the official annotator test at
#       C:\isaacsim_51_ga\exts\isaacsim.sensors.rtx\isaacsim\sensors\rtx\tests\test_annotators.py:255-260
#
# WHEN TO RUN THIS
#   AFTER you have:
#     (a) loaded the scene,
#     (b) added the lidar prim via add_lidar.py,
#     (c) PRESSED PLAY (the timeline must be playing for the writer to spin up).
#
# HOW TO RUN
#   1. In Isaac Sim GUI:  Window -> Script Editor
#   2. File -> Open ... -> this file
#   3. Press the viewport Play button first
#   4. Click "Run" (or Ctrl-Enter) in the Script Editor
#
# EXPECTED LOGS
#   [publish_lidar] step 1 lidar prim OK at /cargo_bot/lidar_link/lidar_sensor
#   [publish_lidar] step 2 render product created at /Render/.../CargoBotLidar_RP
#   [publish_lidar] step 3 orderedVars clean: ['GenericModelOutput', 'RtxSensorMetadata']
#   [publish_lidar] step 4 RtxLidarROS2PublishLaserScan attached  topic=/scan_py  frame=lidar_link
#   [publish_lidar] step 5 debug-draw attached (optional)
#   [publish_lidar] step 6 stashed keepalive refs in builtins._cargo_bot_lidar_keepalive
#   [publish_lidar] DONE.  /scan_py should be live in WSL within ~1 second.
#
# VERIFICATION FROM WSL  (terminal sourced with config/source_ros_wsl.sh)
#   ros2 topic hz /scan_py            # expected ~10 Hz  (S2E default scanRateBaseHz=10)
#   ros2 topic echo /scan_py --once   # expected frame_id="lidar_link", ~1066 ranges
#
# IDEMPOTENT
#   Re-running the script:
#     - Destroys any keepalive refs from a previous run (builtins.<key>).
#     - Asks Replicator for a NEW render product with the same `name=` (rep
#       will append _01, _02 etc).  This is intentional: if you re-run, the
#       previous RP becomes orphaned but no longer publishes.  To fully wipe
#       all RPs on the lidar, stop the timeline and use the cleanup snippet
#       documented in BUILD_SCENE_FROM_SCRATCH.md, Step 8.
# ----------------------------------------------------------------------------------

import builtins

import omni.replicator.core as rep
import omni.usd
from pxr import Usd


# Path of the actual OmniLidar prim (NOT the Xform wrapper).
# In Isaac Sim 5.1.0-rc.19 the IsaacSensorCreateRtxLidar command places the
# sensor at /cargo_bot/lidar_sensor with the actual OmniLidar nested inside
# the Slamtec USD reference as /cargo_bot/lidar_sensor/RPLidar_S2E.  See
# add_lidar.py for the gory details.  The sensor's USD position doesn't
# matter functionally -- ROS 2 uses frame_id=lidar_link via TF.
# After reparent_lidar.py, the wrapper Xform lives at
# /cargo_bot/lidar_link/lidar_sensor and the actual OmniLidar is at
# /cargo_bot/lidar_link/lidar_sensor/RPLidar_S2E.  ROS 2 frame_id stays
# `lidar_link` -- so RViz/Nav2 see the scan at the correct robot mount.
LIDAR_PATH = "/cargo_bot/lidar_sensor"
RP_NAME = "CargoBotLidar_RP"
TOPIC_NAME = "/scan_py"
FRAME_ID = "lidar_link"
KEEPALIVE_KEY = "_cargo_bot_lidar_keepalive"


def _log(msg):
    print(f"[publish_lidar] {msg}")


def main():
    # ---- step 0: drop previous keepalive refs (idempotent re-run) ----
    if hasattr(builtins, KEEPALIVE_KEY):
        try:
            delattr(builtins, KEEPALIVE_KEY)
            _log("step 0 cleared previous keepalive refs")
        except Exception as e:
            _log(f"step 0 (warn) could not clear keepalive: {e}")

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        _log("FATAL: no stage open.")
        return

    # ---- step 1: verify lidar prim exists ----
    lidar = stage.GetPrimAtPath(LIDAR_PATH)
    if not lidar or not lidar.IsValid():
        _log(f"FATAL: no lidar prim at {LIDAR_PATH}.  Run add_lidar.py first.")
        return
    _log(f"step 1 lidar prim OK at {LIDAR_PATH}")

    # ---- step 2: create a clean render product with ONLY the two RTX-Sensor AOVs ----
    # NOTE: replicator/create.py:1605-1606 only calls AddTarget(render_var_path)
    # for the requested AOVs -- it does NOT clear whatever viewport_manager
    # auto-added.  In an in-GUI run that often means LdrColor sneaks in.  We
    # scrub it at step 3 below.  See BUILD_SCENE_FROM_SCRATCH.md, "Why the
    # shortcut menu is poison" for the full story.
    hydra_texture = rep.create.render_product(
        LIDAR_PATH,
        resolution=(1, 1),
        name=RP_NAME,
        render_vars=["GenericModelOutput", "RtxSensorMetadata"],
        force_new=True,
    )
    _log(f"step 2 render product created at {hydra_texture.path}")

    # ---- step 3: defensive scrub of orderedVars -- strip LdrColor if present ----
    rp_prim = stage.GetPrimAtPath(hydra_texture.path)
    cleaned = []
    if rp_prim and rp_prim.IsValid():
        ov_rel = rp_prim.GetRelationship("orderedVars")
        if ov_rel:
            targets = list(ov_rel.GetTargets())
            kept = [t for t in targets if "LdrColor" not in str(t)]
            if len(kept) != len(targets):
                with Usd.EditContext(stage, stage.GetSessionLayer()):
                    ov_rel.SetTargets(kept)
                _log(f"step 3 scrubbed LdrColor: orderedVars now {[str(t) for t in kept]}")
            else:
                _log(f"step 3 orderedVars clean: {[str(t).rsplit('/', 1)[-1] for t in targets]}")
            cleaned = [str(t) for t in kept]

    # ---- step 4: laser-scan writer ----
    writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
    writer.initialize(topicName=TOPIC_NAME, frameId=FRAME_ID)
    writer.attach([hydra_texture])
    _log(f"step 4 RtxLidarROS2PublishLaserScan attached  topic={TOPIC_NAME}  frame={FRAME_ID}")

    # ---- step 5: optional viewport debug draw (visual ring of rays) ----
    debug = None
    try:
        debug = rep.writers.get("RtxLidarDebugDrawPointCloud")
        debug.attach([hydra_texture])
        _log("step 5 debug-draw attached (optional)")
    except Exception as e:
        _log(f"step 5 (info) debug-draw unavailable: {e}")

    # ---- step 6: stash references so Python GC does NOT tear them down
    #             between Script Editor runs.  Without this, the writer is
    #             collected and stops publishing within seconds. ----
    setattr(
        builtins,
        KEEPALIVE_KEY,
        {
            "hydra_texture": hydra_texture,
            "writer": writer,
            "debug": debug,
            "orderedVars": cleaned,
            "topic": TOPIC_NAME,
            "frame": FRAME_ID,
        },
    )
    _log(f"step 6 stashed keepalive refs in builtins.{KEEPALIVE_KEY}")

    _log("DONE.  /scan_py should be live in WSL within ~1 second.")
    _log("       Verify: ros2 topic hz /scan_py   (expect ~10 Hz)")


main()
