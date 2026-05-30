# ============================================================================
# fix_wheel_drives.py
#
# PROBLEM
# -------
# cargo_bot acepta /cmd_vel pero las ruedas no giran. La cadena OmniGraph
# (SubscribeTwist -> DifferentialController -> IsaacArticulationController)
# esta correctamente cableada y la articulacion existe, pero PhysX no aplica
# torque a los joints.
#
# ROOT CAUSE
# ----------
# Los UsdPhysics.RevoluteJoint left_wheel_joint y right_wheel_joint tienen el
# PhysicsDriveAPI:angular configurado como type="force" con:
#   drive:angular:physics:stiffness = 0
#   drive:angular:physics:damping   = 0
#
# DifferentialController.forward() devuelve un ArticulationAction con
# joint_velocities (ver
#   C:\isaacsim_51_ga\exts\isaacsim.robot.wheeled_robots\isaacsim\robot\wheeled_robots\controllers\differential_controller.py
# linea 93), e IsaacArticulationController llama set_dof_velocity_targets()
# (ver C:\isaacsim_51_ga\exts\isaacsim.core.prims\isaacsim\core\prims\impl\articulation.py
# linea 1695). PhysX, para un drive en velocity-control con stiffness=0 y
# damping=0, aplica torque = 0 sin importar el targetVelocity. Cita textual
# de la docstring de Articulation.apply_action (lineas 1639-1641 del archivo
# articulation.py de Isaac 5.1 GA):
#
#     For position control, set relatively high stiffness and low damping
#     For velocity control, stiffness must be set to zero with a non-zero damping
#     For effort control, stiffness and damping must be set to zero
#
# scene_v3 (reportada como "funcional" el 2026-05-25) tambien tenia damping=0
# guardado en USD; cualquier comportamiento previo se debio a un tweak manual
# en el Property panel que NO fue persistido al .usda.
#
# FIX
# ---
# Para cada wheel joint:
#   drive:angular:physics:stiffness     = 0.0     (mantener: control de velocidad puro)
#   drive:angular:physics:damping       = 1000.0  (ganancia Kv; convierte error
#                                                  de velocidad en torque)
#   drive:angular:physics:maxForce      = 1.0e6   (tope sano para evitar inf)
#   drive:angular:physics:targetVelocity= 0.0     (initial; el controller lo
#                                                  sobreescribe cada tick)
#   drive:angular:physics:type          = "force" (mantener)
#
# Valor de damping = 1000.0 elegido para una rueda diff-drive ligera
# (wheel_radius=0.1 m, wheel_distance=0.29 m). Es el orden de magnitud que usa
# el sample Nova_Carter de Isaac y el suficiente para que un cmd_vel de
# linear.x=0.5 m/s produzca rotation real visible sin patinar.
#
# USO
# ---
# Isaac Sim 5.1.0 GA -> Window > Script Editor -> pegar este archivo > Run.
# Idempotente: re-ejecutar reaplica los mismos valores.
# Despues: Ctrl+S la escena, Stop+Play, y en WSL:
#   ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}' -r 10
# El robot debe moverse hacia adelante.
# ============================================================================

from pxr import Usd, UsdPhysics, Sdf
import omni.usd

WHEEL_JOINT_PATHS = [
    "/World/cargo_bot/joints/left_wheel_joint",
    "/World/cargo_bot/joints/right_wheel_joint",
]

# Tuning -- ver header comment para justification
DAMPING       = 1000.0
STIFFNESS     = 0.0
MAX_FORCE     = 1.0e6
DRIVE_TYPE    = "force"

def _fix_one(stage: Usd.Stage, joint_path: str) -> None:
    prim = stage.GetPrimAtPath(joint_path)
    if not prim or not prim.IsValid():
        print(f"[fix_wheel_drives] SKIP {joint_path}: prim not found")
        return

    joint = UsdPhysics.RevoluteJoint(prim)
    if not joint:
        print(f"[fix_wheel_drives] SKIP {joint_path}: not a RevoluteJoint")
        return

    # PhysicsDriveAPI:angular  (UsdPhysicsDriveAPI con instance name "angular")
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")

    # type = "force"  (control directo en fuerzas, NO acceleration)
    type_attr = drive.GetTypeAttr() or drive.CreateTypeAttr()
    type_attr.Set(DRIVE_TYPE)

    # stiffness = 0  (NO position control -- vel-only)
    stiff_attr = drive.GetStiffnessAttr() or drive.CreateStiffnessAttr()
    stiff_attr.Set(STIFFNESS)

    # damping > 0  (EL fix -- Kv para velocity drive)
    damp_attr = drive.GetDampingAttr() or drive.CreateDampingAttr()
    damp_attr.Set(DAMPING)

    # maxForce sano (no inf)
    max_attr = drive.GetMaxForceAttr() or drive.CreateMaxForceAttr()
    max_attr.Set(MAX_FORCE)

    # targetVelocity inicial = 0 (el OG controller lo sobreescribe cada tick)
    tgt_attr = drive.GetTargetVelocityAttr() or drive.CreateTargetVelocityAttr()
    tgt_attr.Set(0.0)

    print(
        f"[fix_wheel_drives] OK {joint_path}: "
        f"type={DRIVE_TYPE} stiffness={STIFFNESS} damping={DAMPING} "
        f"maxForce={MAX_FORCE}"
    )

def main() -> None:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("[fix_wheel_drives] ERROR: no stage abierta")
        return

    print("[fix_wheel_drives] start")
    for jp in WHEEL_JOINT_PATHS:
        _fix_one(stage, jp)
    print("[fix_wheel_drives] done -- Ctrl+S y luego Stop+Play")

main()
