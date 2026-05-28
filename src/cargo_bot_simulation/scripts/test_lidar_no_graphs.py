# Last-ditch test: disable ALL OmniGraphs in the scene BEFORE creating the
# lidar.  Hypothesis: one of the Action Graphs (Clock/TF/Odom/cmd_vel) is
# competing with the RTX-Sensor pipeline for simulation step or scheduler
# resources, causing the sensor to never produce frames despite publisher
# being registered.
#
# This script:
#   1. Finds every ROS2 OmniGraph in scene_v2 and DISABLES them (sets
#      `inputs:enabled=False` on every output OG node).
#   2. Force-cleans any existing lidar prims + render products from prior
#      attempts.
#   3. Creates a single OmniLidar at world root path /lidar_sensor (the
#      proven-working location from the bundled rtx_lidar.py example).
#   4. Creates a render product with explicit AOVs.
#   5. Attaches RtxLidarROS2PublishLaserScan writer to /scan_isolated topic
#      (distinct name so we can compare to /scan from any other publisher).
#   6. Prints PLAY instructions.
#
# Run after STOP, with scene_v2 loaded in GUI Isaac.

import omni.usd
import omni.kit.commands
import omni.kit.app
from pxr import Sdf, Usd

# Load extensions if not already loaded
ext_mgr = omni.kit.app.get_app().get_extension_manager()
for ext_id in ("isaacsim.ros2.bridge", "isaacsim.sensors.rtx", "omni.replicator.core"):
    if not ext_mgr.is_extension_enabled(ext_id):
        print(f"[isolated] Loading extension {ext_id}...")
        ext_mgr.set_extension_enabled_immediate(ext_id, True)

import omni.replicator.core as rep

stage = omni.usd.get_context().get_stage()


def _log(msg):
    print(f"[isolated] {msg}")


# -- 1) Disable ALL OmniGraph nodes' execution
_log("Step 1: disabling all OmniGraphs...")
graphs_found = 0
nodes_disabled = 0
for prim in stage.Traverse():
    tname = prim.GetTypeName()
    if tname == "OmniGraph":
        # The graph prim itself; can be deactivated wholesale
        try:
            prim.SetActive(False)
            graphs_found += 1
            _log(f"  deactivated OmniGraph {prim.GetPath()}")
        except Exception as e:
            _log(f"  failed to deactivate {prim.GetPath()}: {e}")
    elif tname == "OmniGraphNode":
        # Per-node disable as well, for nodes that aren't under an OmniGraph parent
        en = prim.GetAttribute("inputs:enabled")
        if not en:
            en = prim.CreateAttribute("inputs:enabled", Sdf.ValueTypeNames.Bool)
        en.Set(False)
        nodes_disabled += 1
_log(f"  disabled {graphs_found} OmniGraph(s) and {nodes_disabled} OmniGraphNode(s)")


# -- 2) Cleanup any existing OmniLidar / RenderProduct prims
_log("Step 2: cleaning existing OmniLidar + RenderProduct prims...")
killed = 0
to_kill = []
for prim in stage.Traverse():
    tname = prim.GetTypeName()
    path = str(prim.GetPath())
    if tname == "OmniLidar":
        to_kill.append(path)
    elif tname == "RenderProduct":
        if "ViewportTexture" in path or "Persp" in path:
            continue
        to_kill.append(path)
for p in to_kill:
    try:
        omni.kit.commands.execute("DeletePrims", paths=[p])
        killed += 1
        _log(f"  killed {p}")
    except Exception as e:
        _log(f"  failed to kill {p}: {e}")
_log(f"  killed {killed} prim(s)")


# -- 3) Create OmniLidar at world root /lidar_sensor
_log("Step 3: creating OmniLidar at /lidar_sensor (world root)...")
from pxr import Gf
result, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/lidar_sensor",
    parent=None,
    config="Example_Rotary_2D",
    translation=Gf.Vec3d(0.0, 0.0, 0.35),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
)
if not sensor or not sensor.IsValid():
    _log(f"FATAL: failed to create OmniLidar. result={result}")
    raise SystemExit(1)
_log(f"  OmniLidar created at {sensor.GetPath()}  type={sensor.GetTypeName()}")


# -- 4) Render product
_log("Step 4: creating render product...")
hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="IsolatedLidar")
_log(f"  render product: {hydra_texture.path}")


# -- 5) Writer
_log("Step 5: attaching LaserScan writer...")
laser_writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
laser_writer.initialize(topicName="/scan_isolated", frameId="lidar_link")
laser_writer.attach([hydra_texture])
_log("  writer attached: topic=/scan_isolated frame=lidar_link")

# Debug-draw
try:
    debug_writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
    debug_writer.attach([hydra_texture])
    _log("  debug-draw attached (expect rays in viewport)")
except Exception as e:
    _log(f"  debug-draw not available: {e}")


# -- 6) Stash refs so GC doesn't kill the writer
import builtins
if not hasattr(builtins, "_isolated_lidar_keepalive"):
    builtins._isolated_lidar_keepalive = {}
builtins._isolated_lidar_keepalive["rp"] = hydra_texture
builtins._isolated_lidar_keepalive["writer"] = laser_writer


_log("DONE.")
_log("Next: press Play.  Then in WSL:")
_log("  ros2 topic hz /scan_isolated   (expect ~10 Hz)")
_log("")
_log("If /scan_isolated publishes (and rays appear in viewport), the bug is")
_log("DEFINITIVELY caused by competing OmniGraphs in scene_v2.  Solution:")
_log("keep the OG graphs disabled during Play OR re-enable them ONE AT A TIME")
_log("to find which one breaks the sensor pipeline.")
