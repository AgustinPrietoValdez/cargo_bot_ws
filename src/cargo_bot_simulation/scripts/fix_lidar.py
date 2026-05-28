# Cleanup leftover Replicator render products that corrupt the RTX-Sensor pipeline.
# Run from Isaac Sim: Window > Script Editor > File > Open > pick this file > Run
import omni.usd
from pxr import Sdf, Usd

stage = omni.usd.get_context().get_stage()

paths_to_remove = [
    "/Render/OmniverseKit/HydraTextures/Replicator",
    "/Render/OmniverseKit/HydraTextures/Replicator_01",
    "/Render/OmniverseKit/HydraTextures/Replicator_02",
]

edit_target = stage.GetRootLayer()
with Usd.EditContext(stage, edit_target):
    for p in paths_to_remove:
        if stage.GetPrimAtPath(p):
            edit_target.GetPrimAtPath(p) and Sdf.CreatePrimInLayer(edit_target, p)
            stage.RemovePrim(p)
            print("removed " + p)
        else:
            print("not present: " + p)

gsettings = stage.GetPrimAtPath("/Render/OmniverseGlobalRenderSettings")
if gsettings:
    products_rel = gsettings.GetRelationship("products")
    if products_rel:
        keep = [t for t in products_rel.GetTargets() if "/Replicator" not in str(t)]
        products_rel.SetTargets(keep)
        print("products now: " + str(keep))

print("=== done ===")
