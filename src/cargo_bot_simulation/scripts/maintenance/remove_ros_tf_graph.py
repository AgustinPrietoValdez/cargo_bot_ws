# SPDX-License-Identifier: Apache-2.0
# ----------------------------------------------------------------------------------
# remove_ros_tf_graph.py  --  cargo_bot_ws / Isaac Sim 5.1.0 GA
#
# PURPOSE
#   Delete the ROS_TF OmniGraph from scene_v4.
#
# WHY
#   We are switching to robot_state_publisher (running on WSL) as the sole
#   authority for the URDF tree (base_footprint -> base_link -> wheels/lidar/imu).
#   Isaac's ROS_TF graph would conflict with robot_state_publisher by publishing
#   the same transforms on /tf -> non-deterministic tree.
#
# WHAT REMAINS (intentionally untouched)
#   * ROS_Lidar_RTX, ROS_Clock, cmd_vel_graph, imu_graph -> all kept
#   * ROS_Odometry kept (publishes /odom topic only; TF branches already
#     deleted by fix_tf_graphs.py)
#
# IDEMPOTENT: re-running after deletion is a no-op.
#
# HOW TO RUN
#   1. Save scene_v4 first
#   2. Stop Play if running
#   3. Window -> Script Editor -> File -> Open this file -> Run
#   4. File -> Save scene_v4
# ----------------------------------------------------------------------------------

import omni.usd
import omni.kit.commands

GRAPH_TF_PATH = "/World/cargo_bot/Graph/ROS_TF"


def _log(msg):
    print(f"[remove_ros_tf] {msg}")


def main():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        _log("FATAL: no stage open. Open scene_v4.usda first.")
        return

    prim = stage.GetPrimAtPath(GRAPH_TF_PATH)
    if not prim or not prim.IsValid():
        _log(f"already absent: {GRAPH_TF_PATH}")
        _log("DONE (no-op).")
        return

    try:
        omni.kit.commands.execute("DeletePrims", paths=[GRAPH_TF_PATH])
        _log(f"deleted {GRAPH_TF_PATH}")
        _log("DONE. Save scene_v4 (File -> Save).")
        _log("")
        _log("Next: robot_state_publisher (WSL) will publish the URDF tree.")
        _log("That setup happens in localization.launch.py (section 5+).")
    except Exception as e:
        _log(f"FAILED to delete: {e}")


main()
