# Runtime diagnostic for the RTX Lidar pipeline.
# Writes output to C:\Users\agusp\cargo_bot_ws\src\cargo_bot_simulation\scripts\diag_output.txt
# Run in Isaac Sim Script Editor with Isaac in Play.
import omni.usd
from pxr import Usd, UsdRender

stage = omni.usd.get_context().get_stage()

OUT = "C:/Users/agusp/cargo_bot_ws/src/cargo_bot_simulation/scripts/diag_output.txt"
lines = []

def w(s):
    lines.append(str(s))

w("=" * 60)
w("DIAGNOSTIC: RTX Lidar pipeline state")
w("=" * 60)

# 1. List all RenderProduct prims in the live stage
w("")
w("--- All RenderProduct prims (active in stage right now) ---")
for prim in stage.Traverse():
    if prim.GetTypeName() == "RenderProduct":
        path = str(prim.GetPath())
        cam_rel = prim.GetRelationship("camera")
        cam_targets = [str(t) for t in cam_rel.GetTargets()] if cam_rel else []
        ov_rel = prim.GetRelationship("orderedVars")
        ov_targets = [str(t) for t in ov_rel.GetTargets()] if ov_rel else []
        w("  " + path)
        w("    camera: " + str(cam_targets))
        w("    orderedVars: " + str(ov_targets))
        w("    active: " + str(prim.IsActive()))

# 2. OmniLidar prim API status
w("")
w("--- OmniLidar prim APIs ---")
lidar_path = "/cargo_bot/lidar_link/RPLIDAR_S2E/RPLidar_S2E"
lidar = stage.GetPrimAtPath(lidar_path)
if lidar:
    w("  path: " + str(lidar.GetPath()))
    w("  type: " + lidar.GetTypeName())
    w("  applied APIs: " + str(lidar.GetAppliedSchemas()))
    w("  active: " + str(lidar.IsActive()))
    xform = lidar.GetAttribute("xformOp:translate")
    if xform:
        w("  translate (local): " + str(xform.Get()))
    w("  sensor attributes:")
    for attr in lidar.GetAttributes():
        n = attr.GetName()
        if "sensor" in n.lower() or "Core" in n or n.startswith("omni:"):
            v = attr.Get()
            if v is not None:
                w("    " + n + " = " + str(v))
else:
    w("  NOT FOUND at " + lidar_path)

# 3. OG nodes state
w("")
w("--- ActionGraph lidar chain nodes ---")
ag_paths = [
    "/cargo_bot/ActionGraph/ros2_rtx_lidar_helper",
    "/cargo_bot/ActionGraph/isaac_create_render_product",
    "/cargo_bot/ActionGraph/isaac_run_one_simulation_frame",
]
for p in ag_paths:
    node = stage.GetPrimAtPath(p)
    if node:
        w("  " + p)
        for attr in node.GetAttributes():
            n = attr.GetName()
            if n.startswith("inputs:") and not n.endswith(":connect"):
                v = attr.Get()
                if v is not None and str(v) != "":
                    w("    " + n + " = " + repr(v))
        out = node.GetAttribute("outputs:renderProductPath")
        if out:
            w("    [out] renderProductPath = " + repr(out.Get()))

# 4. Try to read the OG controller for runtime state
try:
    import omni.graph.core as og
    w("")
    w("--- OG graph runtime state ---")
    g = og.get_graph_by_path("/cargo_bot/ActionGraph")
    if g:
        try:
            w("  graph eval count: " + str(g.get_evaluation_count()))
        except Exception:
            pass
        for node in g.get_nodes():
            np = node.get_prim_path()
            if "lidar" in np.lower() or "render" in np.lower() or "run_once" in np.lower():
                try:
                    w("  node " + np + " enabled=" + str(node.is_enabled()))
                except Exception:
                    w("  node " + np)
except Exception as e:
    w("  OG runtime introspection failed: " + str(e))

w("")
w("=" * 60)
w("END OF DIAGNOSTIC")
w("=" * 60)

# Write to file
with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines))

print("Diagnostic written to: " + OUT)
print("Lines: " + str(len(lines)))
