# Find all OmniLidar prims + similar names in the stage so we know
# where add_lidar.py actually put the sensor.
import omni.usd
stage = omni.usd.get_context().get_stage()
print("[find_lidar] scanning stage...")
for prim in stage.Traverse():
    name = prim.GetName()
    tname = prim.GetTypeName()
    if tname == "OmniLidar" or "lidar" in name.lower() or "sensor" in name.lower():
        print(f"  path={prim.GetPath()}  type={tname}  active={prim.IsActive()}")
print("[find_lidar] DONE.")
