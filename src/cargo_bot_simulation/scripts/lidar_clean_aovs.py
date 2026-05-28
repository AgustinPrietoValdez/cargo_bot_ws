# Aggressive cleanup + minimal-AOV render product creation.
# Run in Isaac Script Editor with timeline STOPPED.
# After running, press Play.  Verify ros2 topic hz /laser_scan_py from WSL.
import omni.usd
import omni.replicator.core as rep
from pxr import Sdf, Usd

stage = omni.usd.get_context().get_stage()

LIDAR_PRIM_PATH = "/cargo_bot/lidar_link/cargo_bot/RPLIDAR_S2E/RPLidar_S2E"
TOPIC_NAME      = "/laser_scan_py"
FRAME_ID        = "lidar_link"

# ---- 1. Verify OmniLidar exists ---------------------------------------------
lidar = stage.GetPrimAtPath(LIDAR_PRIM_PATH)
if not lidar or lidar.GetTypeName() != "OmniLidar":
    raise RuntimeError(
        f"OmniLidar not found at {LIDAR_PRIM_PATH} (got {lidar.GetTypeName() if lidar else 'None'})"
    )
print(f"[clean_aovs] OmniLidar verified: {LIDAR_PRIM_PATH}")

# ---- 2. Find ALL existing RenderProducts and KILL them ----------------------
#       (except the legitimate viewport one)
# Collect PATHS first as strings, then remove in second pass to avoid invalidating
# the iterator during traversal.
rp_paths_to_kill = []
for prim in stage.Traverse():
    if prim.GetTypeName() != "RenderProduct":
        continue
    path = str(prim.GetPath())
    if "ViewportTexture" in path or "Persp" in path:
        continue
    rp_paths_to_kill.append(path)
print(f"[clean_aovs] found {len(rp_paths_to_kill)} stale RenderProducts to kill")

killed = 0
root_layer = stage.GetRootLayer()
for path in rp_paths_to_kill:
    p = Sdf.Path(path)
    try:
        with Usd.EditContext(stage, root_layer):
            ok = stage.RemovePrim(p)
        if ok:
            killed += 1
            print(f"[clean_aovs] killed RenderProduct {path}")
        else:
            print(f"[clean_aovs] RemovePrim returned False for {path}")
    except Exception as e:
        print(f"[clean_aovs] exception killing {path}: {e}")
print(f"[clean_aovs] killed {killed} stale render products total")

# ---- 3. Clear OmniverseGlobalRenderSettings.products -----------------------
gs = stage.GetPrimAtPath("/Render/OmniverseGlobalRenderSettings")
if gs:
    rel = gs.GetRelationship("products")
    if rel:
        targets = rel.GetTargets()
        keep = [t for t in targets if ("ViewportTexture" in str(t) or "Persp" in str(t))]
        rel.SetTargets(keep)
        print(f"[clean_aovs] GlobalRenderSettings.products now: {keep}")

# ---- 4. Make sure ROS 2 bridge extension is loaded -------------------------
import omni.kit.app
ext_mgr = omni.kit.app.get_app().get_extension_manager()
for ext_id in ("isaacsim.ros2.bridge", "isaacsim.sensors.rtx", "omni.replicator.core"):
    if not ext_mgr.is_extension_enabled(ext_id):
        print(f"[clean_aovs] Loading extension {ext_id}...")
        ext_mgr.set_extension_enabled_immediate(ext_id, True)

# ---- 5. Create a fresh render product with ONLY the lidar AOVs --------------
print("[clean_aovs] creating fresh render product...")
hydra_texture = rep.create.render_product(
    LIDAR_PRIM_PATH,
    resolution=(1, 1),
    name="CargoBotLidarMinimal",
    render_vars=["GenericModelOutput", "RtxSensorMetadata"],
    force_new=True,
)
rp_path = hydra_texture.path
print(f"[clean_aovs] render product created at {rp_path}")

# ---- 6. FORCE remove LdrColor from orderedVars if Isaac auto-added it -------
rp_prim = stage.GetPrimAtPath(rp_path)
if rp_prim:
    ov_rel = rp_prim.GetRelationship("orderedVars")
    if ov_rel:
        targets = list(ov_rel.GetTargets())
        before = [str(t) for t in targets]
        # Keep only sensor AOVs
        kept = [t for t in targets if "LdrColor" not in str(t)]
        if len(kept) != len(targets):
            ov_rel.SetTargets(kept)
            after = [str(t) for t in kept]
            print(f"[clean_aovs] orderedVars BEFORE: {before}")
            print(f"[clean_aovs] orderedVars AFTER:  {after}")
        else:
            print(f"[clean_aovs] orderedVars already clean: {before}")

# ---- 7. Get the writer & attach --------------------------------------------
WRITER_NAME = "RtxLidarROS2PublishLaserScan"
try:
    writer = rep.writers.get(WRITER_NAME)
except Exception:
    # Try alternative name
    writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
writer.initialize(topicName=TOPIC_NAME, frameId=FRAME_ID)
writer.attach([hydra_texture])
print(f"[clean_aovs] writer attached, publishing to {TOPIC_NAME}")

# Debug-draw
try:
    debug_writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
    debug_writer.initialize(doTransform=True)
    debug_writer.attach([hydra_texture])
    print("[clean_aovs] Debug draw attached -- expect rays on Play")
except Exception as e:
    print(f"[clean_aovs] (info) debug draw not available: {e}")

# ---- 8. Keep alive across GC -----------------------------------------------
import builtins
builtins._cargo_bot_lidar_keepalive = {
    "render_product": hydra_texture,
    "writer_laser":   writer,
}

print(f"[clean_aovs] DONE.  Press Play.  In WSL: ros2 topic hz {TOPIC_NAME}")
