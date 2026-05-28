# Force-delete all lidar/camera prims that linger after failed creation attempts.
# Run in Isaac Script Editor with timeline STOPPED.
import omni.usd
import omni.kit.commands
from pxr import Sdf, Usd

stage = omni.usd.get_context().get_stage()
root_layer = stage.GetRootLayer()

# Find every candidate to nuke
to_kill = []
for prim in stage.Traverse():
    path = str(prim.GetPath())
    tname = prim.GetTypeName()
    name = prim.GetName().lower()
    is_lidar = (
        tname == "OmniLidar"
        or tname == "Camera"
        or "lidar_sensor" in name
        or "rplidar" in name
        or name == "lidar_sensor"
    )
    # Skip /cargo_bot/lidar_link itself (the URDF link)
    if path == "/cargo_bot/lidar_link":
        continue
    if is_lidar:
        to_kill.append(path)

print(f"[cleanup] found {len(to_kill)} candidates")
for p in to_kill:
    print(f"  -> {p}")

# Try the standard delete command first
for p in to_kill:
    try:
        omni.kit.commands.execute("DeletePrims", paths=[p])
        print(f"[cleanup] DeletePrims OK for {p}")
    except Exception as e:
        print(f"[cleanup] DeletePrims failed for {p}: {e}")

# Then force-remove any survivors via Sdf layer manipulation
with Usd.EditContext(stage, root_layer):
    for p in to_kill:
        if stage.GetPrimAtPath(p) and stage.GetPrimAtPath(p).IsValid():
            try:
                stage.RemovePrim(Sdf.Path(p))
                print(f"[cleanup] RemovePrim OK for {p}")
            except Exception as e:
                print(f"[cleanup] RemovePrim failed for {p}: {e}")

# Verify
remaining = []
for prim in stage.Traverse():
    path = str(prim.GetPath())
    tname = prim.GetTypeName()
    if tname in ("OmniLidar", "Camera") and "lidar" in path.lower():
        remaining.append(f"{path} ({tname})")

print("---")
print(f"[cleanup] remaining lidar-like prims: {len(remaining)}")
for r in remaining:
    print(f"  -> {r}")
print("[cleanup] DONE.")
