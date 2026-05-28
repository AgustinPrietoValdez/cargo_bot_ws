# Re-parent the lidar wrapper under lidar_link so it follows the robot.
# After add_lidar.py, the OmniLidar wrapper ends up at /cargo_bot/lidar_sensor
# (a sibling of /cargo_bot/lidar_link).  This means it does NOT inherit the
# articulation pose -- when physics moves the robot, the lidar stays behind.
#
# Fix: MovePrim /cargo_bot/lidar_sensor -> /cargo_bot/lidar_link/lidar_sensor
# so it becomes a child of lidar_link and inherits its world transform.
#
# RUN: Script Editor -> Open this file -> Run.  Isaac must be STOPPED.
import omni.usd
import omni.kit.commands

stage = omni.usd.get_context().get_stage()

SRC = "/cargo_bot/lidar_sensor"
DST = "/cargo_bot/lidar_link/lidar_sensor"

# Sanity check source
src_prim = stage.GetPrimAtPath(SRC)
if not src_prim or not src_prim.IsValid():
    print(f"[reparent] FATAL: source prim missing: {SRC}")
else:
    # Sanity check destination parent
    parent = stage.GetPrimAtPath("/cargo_bot/lidar_link")
    if not parent or not parent.IsValid():
        print(f"[reparent] FATAL: parent missing: /cargo_bot/lidar_link")
    else:
        # Also reset the local translation of the lidar_sensor to (0,0,0)
        # so it sits at lidar_link's exact pose (no offset).
        from pxr import Gf, UsdGeom
        xf = UsdGeom.Xformable(src_prim)
        if xf:
            for op in xf.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(0, 0, 0))
                    print(f"[reparent] reset translate -> (0,0,0)")

        # Now move
        result = omni.kit.commands.execute(
            "MovePrim",
            path_from=SRC,
            path_to=DST,
            keep_world_transform=False,
        )
        if result:
            print(f"[reparent] moved {SRC} -> {DST} OK")
            # Verify
            new_prim = stage.GetPrimAtPath(DST)
            if new_prim and new_prim.IsValid():
                print(f"[reparent] verified: {DST} exists")
            else:
                print(f"[reparent] WARNING: {DST} not found after move")
        else:
            print(f"[reparent] MovePrim returned {result}")

print("[reparent] DONE.  Now Play and the lidar should follow the robot.")
