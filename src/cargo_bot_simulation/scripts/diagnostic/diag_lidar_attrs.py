# Quick diagnostic: dump the OmniLidar's sensor attributes to see if the
# Slamtec config actually loaded.  If all the omni:sensor:Core:* fields are
# default/empty, the config didn't apply and the sensor won't fire.
import omni.usd
OUT = "C:/Users/agusp/Documentos/cargo_bot_ws/src/cargo_bot_simulation/scripts/diag_lidar_attrs.txt"

stage = omni.usd.get_context().get_stage()
lines = []

# Find all OmniLidar prims
for prim in stage.Traverse():
    if prim.GetTypeName() == "OmniLidar":
        lines.append(f"=== OmniLidar at {prim.GetPath()} ===")
        lines.append(f"Applied APIs: {prim.GetAppliedSchemas()}")
        lines.append("--- omni:sensor:* attributes ---")
        for attr in prim.GetAttributes():
            n = attr.GetName()
            if n.startswith("omni:sensor:"):
                v = attr.Get()
                lines.append(f"  {n} = {v}")
        lines.append("--- transform ---")
        for attr in prim.GetAttributes():
            n = attr.GetName()
            if n.startswith("xformOp:"):
                v = attr.Get()
                lines.append(f"  {n} = {v}")
        lines.append("")

# Also dump the parent wrapper's transform
parent_paths = [
    "/cargo_bot/lidar_link/lidar_sensor",
    "/cargo_bot/lidar_link",
]
for p in parent_paths:
    prim = stage.GetPrimAtPath(p)
    if prim and prim.IsValid():
        lines.append(f"=== parent {p} type={prim.GetTypeName()} ===")
        for attr in prim.GetAttributes():
            n = attr.GetName()
            if n.startswith("xformOp:"):
                v = attr.Get()
                lines.append(f"  {n} = {v}")
        lines.append("")

# Find all RenderProducts on the lidar
lines.append("=== RenderProducts ===")
for prim in stage.Traverse():
    if prim.GetTypeName() == "RenderProduct":
        path = str(prim.GetPath())
        cam_rel = prim.GetRelationship("camera")
        cam_targets = [str(t) for t in cam_rel.GetTargets()] if cam_rel else []
        ov_rel = prim.GetRelationship("orderedVars")
        ov_targets = [str(t) for t in ov_rel.GetTargets()] if ov_rel else []
        lines.append(f"  {path}")
        lines.append(f"    camera: {cam_targets}")
        lines.append(f"    orderedVars: {ov_targets}")
        lines.append(f"    active: {prim.IsActive()}")

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines))

print("Diagnostic written to: " + OUT)
print("Lines: " + str(len(lines)))
