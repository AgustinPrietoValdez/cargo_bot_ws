# SPDX-License-Identifier: Apache-2.0
# ----------------------------------------------------------------------------------
# inspect_tf_graphs.py  --  cargo_bot_ws / Isaac Sim 5.1.0 GA
#
# PURPOSE
#   Diagnose ROS_TF and ROS_Odometry OmniGraphs in scene_v4 to identify:
#     (1) why the TF tree under base_link (wheels/lidar/imu) is not being
#         published (only odom -> base_footprint and a cargo_bot -> cargo_bot
#         self-loop appear on /tf), and
#     (2) which branch of ROS_Odometry publishes the odom -> base_footprint
#         TF that we need to disable so the EKF (robot_localization) becomes
#         the sole authority for that TF.
#
# WHAT IT DOES
#   * Finds ROS_TF and ROS_Odometry by name anywhere in the stage.
#   * Lists every node inside each graph.
#   * For nodes related to Transform / Odometry / Publish / Read, dumps every
#     non-empty `inputs:*` attribute so we can see how they are configured.
#
# HOW TO RUN
#   1. Window -> Script Editor
#   2. File -> Open -> this file
#   3. Click Run (or Ctrl+Enter)
#   4. Copy the entire console output and paste it back in chat.
#
# READ-ONLY: this script does NOT modify the scene. Safe to run at any time.
# ----------------------------------------------------------------------------------

import omni.usd

GRAPHS_TO_INSPECT = ["ROS_TF", "ROS_Odometry"]


def _log(msg):
    print(f"[inspect] {msg}")


def main():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        _log("FATAL: no stage open. Open scene_v4.usda first.")
        return

    for graph_name in GRAPHS_TO_INSPECT:
        print("\n" + "=" * 72)
        print(f"GRAPH: {graph_name}")
        print("=" * 72)

        # Find the graph anywhere in the stage by name
        found = None
        for prim in stage.Traverse():
            if prim.GetName() == graph_name and prim.GetTypeName() == "OmniGraph":
                found = prim
                break

        if not found:
            print(f"  NOT FOUND")
            continue

        print(f"  path: {found.GetPath()}")

        # List child nodes and dump ALL their non-trivial inputs
        # (don't filter by type name — Isaac exposes all as "OmniGraphNode")
        for node_prim in found.GetAllChildren():
            node_type = node_prim.GetTypeName()
            node_name = node_prim.GetName()
            print(f"\n  NODE: {node_name}  [{node_type}]")

            # Try to read the actual node type from the OmniGraph metadata
            # (often stored as a custom attribute)
            for hint_attr in ("nodeTypeName", "node:type"):
                try:
                    a = node_prim.GetAttribute(hint_attr)
                    if a and a.IsValid():
                        v = a.Get()
                        if v:
                            print(f"    [hint nodeType={v}]")
                except Exception:
                    pass

            for attr in node_prim.GetAttributes():
                attr_name = attr.GetName()
                if not attr_name.startswith("inputs:"):
                    continue
                try:
                    val = attr.Get()
                except Exception:
                    continue
                if val is None:
                    continue
                val_str = str(val)
                # Skip truly empty/default
                if val_str in ("", "[]", "()", "None"):
                    continue
                if len(val_str) > 250:
                    val_str = val_str[:250] + "..."
                print(f"    {attr_name} = {val_str}")

            # Also check relationships (targetPrim is a relationship, not an attribute)
            for rel in node_prim.GetRelationships():
                rel_name = rel.GetName()
                if not rel_name.startswith("inputs:"):
                    continue
                try:
                    targets = rel.GetTargets()
                except Exception:
                    continue
                if not targets:
                    continue
                targets_str = ", ".join(str(t) for t in targets)
                if len(targets_str) > 250:
                    targets_str = targets_str[:250] + "..."
                print(f"    {rel_name} (rel) = [{targets_str}]")

    print("\n" + "=" * 72)
    print("Self-loop hunt: any prim NAMED 'cargo_bot' (excluding the root Xform)")
    print("=" * 72)
    root_cargo = "/World/cargo_bot"
    for prim in stage.Traverse():
        if prim.GetName() == "cargo_bot":
            path = str(prim.GetPath())
            # Skip the legitimate robot root
            if path == root_cargo:
                continue
            print(f"  suspicious: {path}  [{prim.GetTypeName()}]")
    print("\n[inspect] DONE. Paste the full output above back in chat.")


main()
