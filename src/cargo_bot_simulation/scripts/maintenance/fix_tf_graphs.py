# SPDX-License-Identifier: Apache-2.0
# ----------------------------------------------------------------------------------
# fix_tf_graphs.py  --  cargo_bot_ws / Isaac Sim 5.1.0 GA
#
# PURPOSE
#   Surgically fix the ROS_TF and ROS_Odometry OmniGraphs in scene_v4 so that:
#     (1) PublisherTF (in ROS_TF) emits the FULL URDF tree
#         base_footprint -> base_link -> {wheels, lidar_link, imu_link}
#         instead of the self-loop cargo_bot -> cargo_bot it does today.
#     (2) The duplicate `odom -> base_footprint` TF (from TFOdom2Robot in
#         ROS_Odometry) is REMOVED so the EKF (robot_localization) becomes the
#         sole authority of that TF in Fase 3.
#     (3) The non-standard `world -> odom` TF (from TFWorld2Odom in
#         ROS_Odometry) is REMOVED. SLAM will publish `map -> odom` instead.
#
# WHAT IT CHANGES
#   ROS_TF/PublisherTF:
#     inputs:parentPrim   /World/cargo_bot         ->  /World/cargo_bot/base_footprint
#     inputs:targetPrims  /World/cargo_bot         ->  /World/cargo_bot/base_link
#   ROS_Odometry:
#     TFOdom2Robot        DELETED
#     TFWorld2Odom        DELETED
#     ComputeOdometry     unchanged (publishes /odom topic still)
#     PublisherOdometry   unchanged (still publishes /odom topic)
#
# RESULT (post-fix, expected on WSL):
#   /tf  publishes:
#     base_footprint -> base_link
#     base_link -> left_wheel_link
#     base_link -> right_wheel_link
#     base_link -> caster_wheel_link
#     base_link -> lidar_link
#     base_link -> imu_link
#   /odom topic still publishes nav_msgs/Odometry as before.
#   No `world -> odom` TF, no `odom -> base_footprint` TF (EKF fills that gap).
#
# HOW TO RUN
#   1. Save scene_v4 first (File -> Save) so you can roll back via Open if needed
#   2. Stop Play (if running)
#   3. Window -> Script Editor -> File -> Open this file -> Run
#   4. File -> Save scene_v4
#   5. Press Play, verify on WSL:
#        ros2 topic echo /tf --once       (should show base_footprint -> base_link)
#        ros2 run tf2_tools view_frames   (should show URDF tree)
#
# IDEMPOTENT: re-running after the fix is safe (no-op if already applied).
# ----------------------------------------------------------------------------------

import omni.usd
import omni.kit.commands
from pxr import Sdf

# ---- paths (relative to the existing scene_v4 layout) -----------------------
GRAPH_TF_ROOT     = "/World/cargo_bot/Graph/ROS_TF"
GRAPH_ODOM_ROOT   = "/World/cargo_bot/Graph/ROS_Odometry"

PUBLISHER_TF      = f"{GRAPH_TF_ROOT}/PublisherTF"
TF_WORLD_ODOM     = f"{GRAPH_ODOM_ROOT}/TFWorld2Odom"
TF_ODOM_ROBOT     = f"{GRAPH_ODOM_ROOT}/TFOdom2Robot"

NEW_PARENT_PRIM   = "/World/cargo_bot/base_footprint"
NEW_TARGET_PRIM   = "/World/cargo_bot/base_link"


def _log(msg):
    print(f"[fix_tf] {msg}")


def _delete_if_exists(stage, path):
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        try:
            omni.kit.commands.execute("DeletePrims", paths=[path])
            _log(f"deleted {path}")
            return True
        except Exception as e:
            _log(f"FAILED to delete {path}: {e}")
            return False
    else:
        _log(f"already absent: {path}")
        return True


def _set_relationship_targets(prim, rel_name, new_targets):
    """Replace the targets of a USD relationship on the given prim."""
    rel = prim.GetRelationship(rel_name)
    if not rel or not rel.IsValid():
        _log(f"FAILED: relationship '{rel_name}' not found on {prim.GetPath()}")
        return False
    # SetTargets takes a list of Sdf.Path
    target_paths = [Sdf.Path(t) for t in new_targets]
    try:
        rel.SetTargets(target_paths)
        return True
    except Exception as e:
        _log(f"FAILED to set targets on {rel_name}: {e}")
        return False


def main():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        _log("FATAL: no stage open. Open scene_v4.usda first.")
        return

    # ------------------------------------------------------------------
    # Step 1: Fix PublisherTF parentPrim + targetPrims
    # ------------------------------------------------------------------
    publisher_tf = stage.GetPrimAtPath(PUBLISHER_TF)
    if not publisher_tf or not publisher_tf.IsValid():
        _log(f"FATAL: {PUBLISHER_TF} not found. Is the graph at the expected path?")
        return

    _log(f"step 1 fixing PublisherTF at {PUBLISHER_TF}")

    ok1 = _set_relationship_targets(publisher_tf, "inputs:parentPrim",
                                     [NEW_PARENT_PRIM])
    if ok1:
        _log(f"  parentPrim  -> {NEW_PARENT_PRIM}")
    ok2 = _set_relationship_targets(publisher_tf, "inputs:targetPrims",
                                     [NEW_TARGET_PRIM])
    if ok2:
        _log(f"  targetPrims -> [{NEW_TARGET_PRIM}]")

    if not (ok1 and ok2):
        _log("WARNING: PublisherTF fix partial — review above")

    # ------------------------------------------------------------------
    # Step 2: Delete TFOdom2Robot (EKF will replace odom -> base_footprint)
    # ------------------------------------------------------------------
    _log(f"step 2 removing TFOdom2Robot (EKF will publish odom -> base_footprint)")
    _delete_if_exists(stage, TF_ODOM_ROBOT)

    # ------------------------------------------------------------------
    # Step 3: Delete TFWorld2Odom (non-standard, SLAM will publish map -> odom)
    # ------------------------------------------------------------------
    _log(f"step 3 removing TFWorld2Odom (non-standard world -> odom)")
    _delete_if_exists(stage, TF_WORLD_ODOM)

    # ------------------------------------------------------------------
    # Done
    # ------------------------------------------------------------------
    _log("")
    _log("DONE. Save scene_v4 (File -> Save), then Play and verify on WSL:")
    _log("  ros2 topic echo /tf --once")
    _log("  ros2 run tf2_tools view_frames")
    _log("Expected new TF tree (no map/odom yet, those come with EKF + SLAM):")
    _log("  base_footprint")
    _log("  └── base_link")
    _log("      ├── left_wheel_link")
    _log("      ├── right_wheel_link")
    _log("      ├── caster_wheel_link")
    _log("      ├── lidar_link")
    _log("      └── imu_link")
    _log("And on /tf you should NO LONGER see the cargo_bot -> cargo_bot error.")


main()
