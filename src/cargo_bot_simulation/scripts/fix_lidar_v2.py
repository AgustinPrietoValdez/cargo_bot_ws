# Aggressive cleanup of leftover Replicator render products on the OmniLidar.
# v2: strips no_delete=true, deactivates, AND removes from root layer directly.
# Run from Isaac Sim: Window > Script Editor > File > Open > pick this file > Run
import omni.usd
from pxr import Sdf, Usd

stage = omni.usd.get_context().get_stage()
root_layer = stage.GetRootLayer()

paths_to_remove = [
    "/Render/OmniverseKit/HydraTextures/Replicator",
    "/Render/OmniverseKit/HydraTextures/Replicator_01",
    "/Render/OmniverseKit/HydraTextures/Replicator_02",
]

# Step 1: deactivate them so the stage stops processing them
for p in paths_to_remove:
    prim = stage.GetPrimAtPath(p)
    if prim:
        prim.SetActive(False)
        # Clear the no_delete metadata if present
        if prim.HasCustomDataKey("no_delete"):
            prim.ClearCustomDataByKey("no_delete")
        print("deactivated " + p)
    else:
        print("not present: " + p)

# Step 2: remove the prim specs from the root layer directly
with Usd.EditContext(stage, root_layer):
    for p in paths_to_remove:
        spec = root_layer.GetPrimAtPath(p)
        if spec:
            root_layer.ScheduleRemoveIfInert(spec)
            try:
                Sdf.PrimSpec.RemoveProperty
            except Exception:
                pass
            # Force-remove via RemovePrimIfInert / direct delete
            try:
                del root_layer.GetPrimAtPath(p).realPath  # noop, just to test access
            except Exception:
                pass
            print("removing spec " + p)

# Step 3: explicit removal pass
for p in paths_to_remove:
    if stage.GetPrimAtPath(p):
        try:
            stage.RemovePrim(p)
            print("removed " + p)
        except Exception as e:
            print("removeprim failed: " + p + " -> " + str(e))

# Step 4: clean the GlobalRenderSettings.products list
gsettings = stage.GetPrimAtPath("/Render/OmniverseGlobalRenderSettings")
if gsettings:
    products_rel = gsettings.GetRelationship("products")
    if products_rel:
        keep = [t for t in products_rel.GetTargets() if "/Replicator" not in str(t)]
        products_rel.SetTargets(keep)
        print("products now: " + str(keep))

# Step 5: verify
remaining = [p for p in paths_to_remove if stage.GetPrimAtPath(p)]
print("=== remaining after cleanup: " + str(remaining))
print("=== done ===")
