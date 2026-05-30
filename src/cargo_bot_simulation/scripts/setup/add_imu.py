# SPDX-License-Identifier: Apache-2.0
# ----------------------------------------------------------------------------------
# add_imu.py  --  cargo_bot_ws / Isaac Sim 5.1.0 GA
#
# PURPOSE
#   Cleanup-and-create an IMU sensor at /World/cargo_bot/imu_link/imu_sensor
#   and build an OmniGraph that publishes sensor_msgs/Imu on topic "imu/data"
#   with frame_id "imu_link" at ~100 Hz.
#
#   Follows the same pattern as add_lidar.py + build_cmd_vel_graph.py:
#     * omni.kit.commands.execute(...) for sensor prim creation
#     * og.Controller.edit(...) for the OmniGraph
#     * idempotent (deletes existing prim + graph before recreating)
#
# AUTHORITATIVE SOURCES (Isaac Sim 5.1.0 GA, installed at C:\isaacsim_51_ga\)
#   * Command spec:
#       exts/isaacsim.sensors.physics/isaacsim/sensors/physics/impl/commands.py
#       -> class IsaacSensorCreateImuSensor (line 112)
#   * IsaacReadIMU node spec:
#       exts/isaacsim.sensors.physics/ogn/docs/OgnIsaacReadIMU.rst
#       node type "isaacsim.sensors.physics.IsaacReadIMU"
#         inputs:  execIn (execution), imuPrim (target), readGravity (bool),
#                  useLatestData (bool)
#         outputs: execOut, linAcc (vec3d), angVel (vec3d),
#                  orientation (quatd[IJKR]), sensorTime (float)
#   * ROS2PublishImu node spec:
#       exts/isaacsim.ros2.bridge/ogn/docs/OgnROS2PublishImu.rst
#       node type "isaacsim.ros2.bridge.ROS2PublishImu"
#         inputs:  execIn, context (uint64), topicName, frameId,
#                  linearAcceleration, angularVelocity, orientation,
#                  publishLinearAcceleration/AngularVelocity/Orientation,
#                  timeStamp (double seconds), queueSize, qosProfile,
#                  nodeNamespace
#   * Canonical OG usage reference:
#       exts/isaacsim.sensors.physics/isaacsim/sensors/physics/tests/
#         test_imu_sensor_ogn.py
#       -> shows imuPrim must be set via og.Controller.set(...) with a
#          LIST of usdrt.Sdf.Path -- it is a `target` attribute, NOT a
#          plain string; cannot be set via keys.SET_VALUES.
#
# GOTCHAS (equivalent of the lidar's RenderProduct + path-mangling traps)
#   1. inputs:imuPrim is a target attribute. Setting it requires
#        og.Controller.set(<attr>, [usdrt.Sdf.Path("/World/cargo_bot/imu_link/imu_sensor")])
#      Passing a bare string via keys.SET_VALUES silently fails (no error,
#      but readings stay at 0).
#   2. IMU uses the PHYSICS pipeline (not RTX render). No render product is
#      needed (unlike the RTX Lidar).
#   3. The IMU prim must sit under a rigid body in the articulation. With our
#      URDF import, /World/cargo_bot/imu_link is a rigid Xform child of
#      base_footprint, which is correct.
#   4. timeStamp is in SECONDS (double). We wire IsaacReadSimulationTime ->
#      ROS2PublishImu.timeStamp so the message stamp tracks sim time -- this
#      matches what /scan + /odom do and keeps RViz happy under use_sim_time.
#   5. sensor_period=-1 means "every physics step" (no decimation). The 100 Hz
#      rate is governed by the physics dt, not by sensor_period.
#
# HOW TO RUN
#   1. Open scene_v3.usda in Isaac Sim 5.1
#   2. Window -> Script Editor
#   3. File -> Open -> this file -> Run (sim STOPPED)
#   4. Press Play
#   5. Verify in WSL:
#        ros2 topic hz /imu/data         (~100 Hz)
#        ros2 topic echo /imu/data --once
# ----------------------------------------------------------------------------------

import omni.kit.commands
import omni.usd
import omni.graph.core as og
import usdrt.Sdf
from pxr import Gf

# ---- config -----------------------------------------------------------------
# PARENT_LINK is auto-detected because URDF importer can nest imu_link
# under base_link (case observed in scene_v4 after merge_fixed_joints handling):
#   * Sometimes at top-level: /World/cargo_bot/imu_link
#   * Sometimes nested:       /World/cargo_bot/base_link/imu_link
# The find_imu_link() function below traverses the stage and uses whichever
# location actually exists.
ROBOT_ROOT      = "/World/cargo_bot"
LINK_NAME       = "imu_link"
IMU_LEAF        = "imu_sensor"
GRAPH_PATH      = "/World/cargo_bot/imu_graph"

TOPIC_NAME      = "imu/data"
FRAME_ID        = "imu_link"
PHYSICS_HZ      = 100.0
SENSOR_PERIOD   = 1.0 / PHYSICS_HZ  # used by sensor; -1 = every physics step
QOS_PROFILE     = ""                # "" => sensor_data default in ros2_bridge
QUEUE_SIZE      = 10

# Filter widths (1 = no filter, raw reading)
LIN_ACC_FILTER  = 1
ANG_VEL_FILTER  = 1
ORI_FILTER      = 1


def _log(msg):
    print(f"[add_imu] {msg}")


def _delete_if_exists(stage, path):
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        try:
            omni.kit.commands.execute("DeletePrims", paths=[path])
            _log(f"deleted existing {path}")
        except Exception as e:
            _log(f"could not delete {path}: {e}")


def find_imu_link(stage):
    """Find the imu_link prim path anywhere in the stage.

    URDF importer's USD hierarchy varies:
      - Sometimes top-level:  /World/cargo_bot/imu_link
      - Sometimes nested:     /World/cargo_bot/base_link/imu_link
      - Sometimes under different root (e.g. /World/cargo_bot_v4)
    Returns the first prim path whose leaf name matches LINK_NAME (case-insensitive
    fallback), or None if not found.
    """
    # Pass 1: exact case-sensitive match anywhere
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.split("/")[-1] == LINK_NAME:
            return path
    # Pass 2: case-insensitive fallback
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.split("/")[-1].lower() == LINK_NAME.lower():
            return path
    return None


def dump_stage_tree(stage, indent=0, max_depth=4):
    """Dump the stage hierarchy (limited depth) for diagnostic purposes."""
    pseudo_root = stage.GetPseudoRoot()
    def _walk(prim, depth):
        if depth > max_depth:
            return
        path = str(prim.GetPath())
        if path == "/":
            name = "/"
        else:
            name = path.split("/")[-1]
        _log(f"  {'  ' * depth}{name}  ({prim.GetTypeName()})  [{path}]")
        for child in prim.GetChildren():
            _walk(child, depth + 1)
    _walk(pseudo_root, 0)


def main():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        _log("FATAL: no stage open. Open scene_v4.usda first.")
        return
    _log("step 0a stage acquired")

    # ---- step 0b: auto-detect imu_link in the stage ----
    parent_link = find_imu_link(stage)
    if parent_link is None:
        _log(f"FATAL: could not find any prim named '{LINK_NAME}' anywhere in the stage")
        _log("       Dumping full stage tree (depth=4) for diagnosis:")
        dump_stage_tree(stage)
        _log("")
        _log("       Look for: any prim with 'imu' in its name. Edit LINK_NAME")
        _log("       at the top of this script if your prim is named differently.")
        return
    _log(f"step 0b found imu_link at: {parent_link}")
    imu_prim_path = f"{parent_link}/{IMU_LEAF}"

    # ---- step 0c: cleanup any prior IMU sensor + graph (idempotent) -----
    _delete_if_exists(stage, imu_prim_path)
    _delete_if_exists(stage, GRAPH_PATH)
    # Also sweep any stray IsaacImuSensor prims anywhere in the stage
    for prim in stage.Traverse():
        if prim.GetTypeName() == "IsaacImuSensor":
            path = str(prim.GetPath())
            if path != imu_prim_path:
                _delete_if_exists(stage, path)

    # ---- step 1: create the IMU sensor prim ----
    # Authoritative command name: "IsaacSensorCreateImuSensor"
    # Signature (from impl/commands.py):
    #   path: str = "/Imu_Sensor"
    #   parent: str = None
    #   sensor_period: float = -1          (-1 = every physics step)
    #   translation: Gf.Vec3d = (0,0,0)
    #   orientation: Gf.Quatd = (1,0,0,0)
    #   linear_acceleration_filter_size: int = 1
    #   angular_velocity_filter_size: int = 1
    #   orientation_filter_size: int = 1
    result, sensor_prim = omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path=f"/{IMU_LEAF}",                       # leaf only; parent prepended below
        parent=parent_link,
        sensor_period=SENSOR_PERIOD,
        translation=Gf.Vec3d(0.0, 0.0, 0.0),       # inherit imu_link transform
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # identity
        linear_acceleration_filter_size=LIN_ACC_FILTER,
        angular_velocity_filter_size=ANG_VEL_FILTER,
        orientation_filter_size=ORI_FILTER,
    )
    if not result or sensor_prim is None:
        _log("FATAL: IsaacSensorCreateImuSensor returned failure.")
        return

    # Confirm where it actually landed (defensive vs path-mangling, like lidar)
    found_path = None
    for prim in stage.Traverse():
        if prim.GetTypeName() == "IsaacImuSensor":
            found_path = str(prim.GetPath())
            break
    if not found_path:
        _log("FATAL: no IsaacImuSensor prim found in stage after creation.")
        return
    if found_path != imu_prim_path:
        _log(f"WARNING: IMU landed at {found_path} (expected {imu_prim_path})")
        _log("         Using actual path below.")
    actual_imu_path = found_path
    _log(f"step 1 created IsaacImuSensor at {actual_imu_path}")

    # ---- step 2: build the OmniGraph ----
    # Pipeline:
    #   OnPlaybackTick -> IsaacReadIMU -> ROS2PublishImu
    #                     IsaacReadSimulationTime -> ROS2PublishImu.timeStamp
    #                     ROS2Context -> ROS2PublishImu.context
    keys = og.Controller.Keys
    (graph_handle, nodes, _, _) = og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context",    "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime",    "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadIMU",        "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishIMU",     "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.CONNECT: [
                # Exec chain
                ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut",     "PublishIMU.inputs:execIn"),
                # Data flow: IMU readings -> publisher
                ("ReadIMU.outputs:linAcc",      "PublishIMU.inputs:linearAcceleration"),
                ("ReadIMU.outputs:angVel",      "PublishIMU.inputs:angularVelocity"),
                ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
                # Timestamp (sim time in seconds, matches use_sim_time=true)
                ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
                # ROS2 context plumbing
                ("ROS2Context.outputs:context", "PublishIMU.inputs:context"),
            ],
            keys.SET_VALUES: [
                # IsaacReadIMU
                ("ReadIMU.inputs:readGravity",   True),
                ("ReadIMU.inputs:useLatestData", False),
                # ROS2PublishImu
                ("PublishIMU.inputs:topicName",                 TOPIC_NAME),
                ("PublishIMU.inputs:frameId",                   FRAME_ID),
                ("PublishIMU.inputs:nodeNamespace",             ""),
                ("PublishIMU.inputs:queueSize",                 QUEUE_SIZE),
                ("PublishIMU.inputs:qosProfile",                QOS_PROFILE),
                ("PublishIMU.inputs:publishLinearAcceleration", True),
                ("PublishIMU.inputs:publishAngularVelocity",    True),
                ("PublishIMU.inputs:publishOrientation",        True),
                # ROS2Context: prefer ROS_DOMAIN_ID env var (DOMAIN_ID=1 in our stack)
                ("ROS2Context.inputs:useDomainIDEnvVar", True),
                # IsaacReadSimulationTime: keep monotonic across stop/play
                ("ReadSimTime.inputs:resetOnStop", False),
            ],
        },
    )
    _log(f"step 2 created graph {GRAPH_PATH} with {len(nodes)} nodes")
    for n in nodes:
        _log(f"  node: {n.get_prim_path()}")

    # ---- step 3: bind imuPrim target ----
    # CRITICAL: inputs:imuPrim is a `target` attribute, not a string.
    # The canonical pattern (from test_imu_sensor_ogn.py) is:
    #   og.Controller.set(<attr>, [usdrt.Sdf.Path("/...")])
    # passing a LIST. keys.SET_VALUES does NOT work for target attrs.
    try:
        og.Controller.set(
            og.Controller.attribute(f"{GRAPH_PATH}/ReadIMU.inputs:imuPrim"),
            [usdrt.Sdf.Path(actual_imu_path)],
        )
        _log(f"step 3 bound ReadIMU.imuPrim -> {actual_imu_path}")
    except Exception as e:
        _log(f"FATAL: could not bind imuPrim target: {e}")
        return

    _log("DONE.")
    _log(f"  Press Play, then in WSL:")
    _log(f"    ros2 topic hz /{TOPIC_NAME}         (expect ~{PHYSICS_HZ:.0f} Hz)")
    _log(f"    ros2 topic echo /{TOPIC_NAME} --once")
    _log(f"  Expected frame_id in message header: '{FRAME_ID}'")


main()
