# Cleanup legacy prims from previous failed lidar attempts so the new
# OmniLidar at /cargo_bot/lidar_sensor has a clean Hydra pipeline.
# Run with timeline STOPPED.
import omni.usd
import omni.kit.commands
from pxr import Sdf, Usd

stage = omni.usd.get_context().get_stage()
root_layer = stage.GetRootLayer()

# Prims to nuke -- explicit list
to_kill = [
    # Orphan Camera wrapper from previous Example_Rotary_2D attempt
    "/cargo_bot/lidar_link/lidar_sensor",
    # All Replicator/CargoBotLidar render products + their SDG pipeline nodes
]

# Scan for render products to delete too
for prim in stage.Traverse():
    p = str(prim.GetPath())
    if "ViewportTexture" in p or "Persp" in p:
        continue
    if "CargoBotLidar" in p or "Replicator" in p:
        to_kill.append(p)

# Dedupe & sort
to_kill = sorted(set(to_kill))
print(f"[cleanup_legacy] nuking {len(to_kill)} prims:")
for p in to_kill:
    print(f"  - {p}")

killed = 0
for p in to_kill:
    if not stage.GetPrimAtPath(p) or not stage.GetPrimAtPath(p).IsValid():
        print(f"  skip (already gone): {p}")
        continue
    try:
        omni.kit.commands.execute("DeletePrims", paths=[p])
    except Exception:
        pass
    # Force
    try:
        with Usd.EditContext(stage, root_layer):
            stage.RemovePrim(Sdf.Path(p))
    except Exception as e:
        pass
    if not stage.GetPrimAtPath(p) or not stage.GetPrimAtPath(p).IsValid():
        killed += 1
        print(f"  killed {p}")
    else:
        print(f"  SURVIVED: {p}")

print(f"[cleanup_legacy] killed {killed}/{len(to_kill)}")
print("[cleanup_legacy] DONE.  Re-run publish_lidar.py next.")
