# Builds the cmd_vel ActionGraph programmatically.
# Bypasses the OmniGraph editor GUI (which crashes Isaac in some versions).
#
# Creates the full chain:
#   OnPlaybackTick -> ROS2SubscribeTwist -> BreakVec3(linear) + BreakVec3(angular)
#                                        -> DifferentialController -> ArticulationController
#
# Run with Isaac STOPPED.  Idempotent: re-running deletes the old cmd_vel_graph
# first and rebuilds.

import omni.usd
import omni.kit.commands
import omni.graph.core as og
from pxr import Sdf
import usdrt.Sdf

GRAPH_PATH       = "/World/cargo_bot/cmd_vel_graph"
ARTICULATION     = "/World/cargo_bot/base_footprint"
WHEEL_JOINTS     = ["left_wheel_joint", "right_wheel_joint"]
WHEEL_DISTANCE   = 0.29
WHEEL_RADIUS     = 0.10
MAX_LINEAR       = 1.0
MAX_ANGULAR      = 3.0
MAX_WHEEL_SPEED  = 10.0
TOPIC_CMD_VEL    = "cmd_vel"


def _log(msg):
    print(f"[cmd_vel_graph] {msg}")


def main():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        _log("FATAL: no stage open"); return

    # Idempotent: delete existing graph
    existing = stage.GetPrimAtPath(GRAPH_PATH)
    if existing and existing.IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[GRAPH_PATH])
        _log(f"deleted existing {GRAPH_PATH}")

    # Build graph via og.Controller (canonical Isaac OmniGraph API)
    keys = og.Controller.Keys
    (graph_handle, nodes, _, _) = og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick",         "omni.graph.action.OnPlaybackTick"),
                ("SubscribeTwist",         "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinear",            "omni.graph.nodes.BreakVector3"),
                ("BreakAngular",           "omni.graph.nodes.BreakVector3"),
                ("DiffCtrl",               "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArtCtrl",                "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.CONNECT: [
                # Exec chain
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("SubscribeTwist.outputs:execOut", "DiffCtrl.inputs:execIn"),
                ("SubscribeTwist.outputs:execOut", "ArtCtrl.inputs:execIn"),
                # Data flow: linear (Twist vec3 -> x scalar)
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
                ("BreakLinear.outputs:x",                  "DiffCtrl.inputs:linearVelocity"),
                # Data flow: angular (Twist vec3 -> z scalar)
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
                ("BreakAngular.outputs:z",                 "DiffCtrl.inputs:angularVelocity"),
                # DiffCtrl -> ArtCtrl
                ("DiffCtrl.outputs:velocityCommand",       "ArtCtrl.inputs:velocityCommand"),
            ],
            keys.SET_VALUES: [
                # Subscribe Twist
                ("SubscribeTwist.inputs:topicName",     TOPIC_CMD_VEL),
                # DiffCtrl tuning
                ("DiffCtrl.inputs:wheelDistance",       WHEEL_DISTANCE),
                ("DiffCtrl.inputs:wheelRadius",         WHEEL_RADIUS),
                ("DiffCtrl.inputs:maxLinearSpeed",      MAX_LINEAR),
                ("DiffCtrl.inputs:maxAngularSpeed",     MAX_ANGULAR),
                ("DiffCtrl.inputs:maxWheelSpeed",       MAX_WHEEL_SPEED),
                # ArtCtrl
                ("ArtCtrl.inputs:robotPath",            ARTICULATION),
                ("ArtCtrl.inputs:jointNames",           WHEEL_JOINTS),
            ],
        },
    )
    _log(f"created graph {GRAPH_PATH} with {len(nodes)} nodes")
    for n in nodes:
        _log(f"  node: {n.get_prim_path()}")

    # CRITICAL: inputs:targetPrim on IsaacArticulationController is a USD
    # RELATIONSHIP, not an attribute. Different from IsaacReadIMU.imuPrim,
    # which is an attribute with type=target (bindable via og.Controller.set
    # with [usdrt.Sdf.Path(...)]). Relationships need the USD API directly:
    # prim.GetRelationship(name).SetTargets([Sdf.Path(...)]).
    # Confirm with prim.GetAttributes() vs prim.GetRelationships() -- targetPrim
    # appears ONLY in the second. keys.SET_VALUES and og.Controller.set both
    # silently fail (no exception) when called on a relationship, leaving the
    # ArtCtrl subscribed to cmd_vel but unable to drive any joint.
    art_ctrl_prim = stage.GetPrimAtPath(f"{GRAPH_PATH}/ArtCtrl")
    if not art_ctrl_prim or not art_ctrl_prim.IsValid():
        _log(f"FATAL: ArtCtrl prim missing at {GRAPH_PATH}/ArtCtrl")
        return
    target_rel = art_ctrl_prim.GetRelationship("inputs:targetPrim")
    if not target_rel.IsValid():
        target_rel = art_ctrl_prim.CreateRelationship("inputs:targetPrim")
    target_rel.SetTargets([Sdf.Path(ARTICULATION)])
    _log(f"bound ArtCtrl.targetPrim (USD rel) -> {ARTICULATION}")

    _log("DONE.  Press Play.  Test with:")
    _log(f"  ros2 topic pub /{TOPIC_CMD_VEL} geometry_msgs/Twist '{{linear: {{x: 0.2}}}}' -r 10")


main()
