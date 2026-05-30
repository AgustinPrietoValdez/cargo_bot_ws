# SPDX-License-Identifier: Apache-2.0
# ----------------------------------------------------------------------------------
# fix_lidar_range.py  --  cargo_bot_ws / Isaac Sim 5.1.0 GA
#
# PURPOSE
#   Fix the RTX Lidar's near/far range so the published /scan reports a usable
#   range_min. As shipped in scene_v4 the OmniLidar has:
#       omni:sensor:Core:nearRangeM = 1.0    -> /scan range_min = 1.0 m  (BAD)
#       omni:sensor:Core:farRangeM  = 200.0  -> /scan range_max = 200 m  (generic default)
#
#   nearRangeM=1.0 blinds SLAM to anything within 1 m of the robot -- catastrophic
#   for indoor mapping in small spaces (slam_toolbox even WARNs about it:
#   "minimum laser range setting (0.0 m) exceeds the capabilities of the used
#    Lidar (1.0 m)").
#
#   This sets:
#       nearRangeM = 0.15  (matches the sensor's focusDistM=0.15; ~RPLidar S2E min)
#       farRangeM  = 30.0  (realistic S2E max; slam_toolbox.yaml caps at 12 anyway,
#                           but 30 makes the LaserScan message honest)
#
# ROOT-CAUSE NOTE
#   range_min/range_max in the LaserScan map 1:1 to nearRangeM/farRangeM on the
#   OmniLidar prim (verified: scan reported 1.0/200.0, prim had 1.0/200.0).
#   These are plain float attributes -- settable directly, no relationship/target
#   gotcha (unlike ArtCtrl.targetPrim or ReadIMU.imuPrim).
#
# IDEMPOTENT: re-running just re-asserts the same values.
#
# HOW TO RUN
#   1. Window -> Script Editor -> File -> Open -> this file -> Run
#      (can run in Play or Stop)
#   2. File -> Save  (persist into scene_v4.usda)
#   3. Verify in WSL:  ros2 topic echo /scan --field range_min --once   (expect 0.15)
#      If it still shows 1.0, Stop + Play to regenerate the scan pipeline,
#      then restart slam.launch.py.
# ----------------------------------------------------------------------------------

import omni.usd

NEAR_RANGE_M = 0.15
FAR_RANGE_M  = 30.0

NEAR_ATTR = "omni:sensor:Core:nearRangeM"
FAR_ATTR  = "omni:sensor:Core:farRangeM"


def _log(msg):
    print(f"[fix_lidar_range] {msg}")


def main():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        _log("FATAL: no stage open. Open scene_v4.usda first.")
        return

    # Find every prim that actually has the nearRangeM attribute (robust against
    # the lidar prim living at a different path than expected).
    targets = []
    for prim in stage.Traverse():
        if prim.HasAttribute(NEAR_ATTR):
            targets.append(prim)

    if not targets:
        _log(f"FATAL: no prim with attribute '{NEAR_ATTR}' found in stage.")
        _log("       Is the RTX Lidar present? Run diagnostic/diag_lidar_attrs.py.")
        return

    for prim in targets:
        path = str(prim.GetPath())
        near = prim.GetAttribute(NEAR_ATTR)
        far  = prim.GetAttribute(FAR_ATTR)

        old_near = near.Get()
        near.Set(NEAR_RANGE_M)
        _log(f"{path}  nearRangeM: {old_near} -> {NEAR_RANGE_M}")

        if far and far.IsValid():
            old_far = far.Get()
            far.Set(FAR_RANGE_M)
            _log(f"{path}  farRangeM:  {old_far} -> {FAR_RANGE_M}")
        else:
            _log(f"{path}  (no farRangeM attr -- skipped)")

    _log("DONE.  File -> Save scene_v4, then verify:")
    _log("  ros2 topic echo /scan --field range_min --once   (expect 0.15)")


main()
