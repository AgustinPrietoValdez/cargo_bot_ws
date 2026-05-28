# Flip the rotation axis on both wheel joints so positive linear.x = forward.
# Some URDF Importer configurations leave the wheel `physics:axis` pointing
# such that positive wheel velocity makes the robot go BACKWARD.  This script
# inverts the axis on both wheel joints so positive cmd_vel.linear.x advances
# the robot forward.
#
# RUN: Script Editor -> Open this file -> Run.  Isaac can be in Stop or Play.
import omni.usd
from pxr import Gf

stage = omni.usd.get_context().get_stage()

WHEEL_JOINT_PATHS = [
    "/cargo_bot/joints/left_wheel_joint",
    "/cargo_bot/joints/right_wheel_joint",
]

for jpath in WHEEL_JOINT_PATHS:
    prim = stage.GetPrimAtPath(jpath)
    if not prim or not prim.IsValid():
        # Try alternative paths just in case
        for alt in (jpath.replace("/joints/", "/"),):
            prim = stage.GetPrimAtPath(alt)
            if prim and prim.IsValid():
                jpath = alt
                break
    if not prim or not prim.IsValid():
        print(f"[flip_axis] FATAL: joint not found at {jpath}")
        continue

    # The PhysX revolute joint stores its axis as a token ("X", "Y", "Z") OR
    # as a 3-vector.  Most URDF Importer outputs use a `physics:axis` Token
    # attribute.  Flipping = swapping the joint's localRot/localPos OR more
    # robustly, negating the rotation direction by flipping the joint's
    # body0/body1 references.  Simplest: flip the joint's localRot quaternion
    # so the wheel spins the opposite way under the same angular command.

    # Read the current localRot1 (the rotation of body1 in joint frame).
    # Flipping body1's orientation 180deg around the joint's spin axis
    # inverts the perceived sign of joint angular velocity.
    rot_attr = prim.GetAttribute("physics:localRot1")
    if not rot_attr:
        print(f"[flip_axis] {jpath}: no physics:localRot1 attribute, trying body0...")
        rot_attr = prim.GetAttribute("physics:localRot0")
    if rot_attr:
        current = rot_attr.Get()
        # Flip rotation 180deg around X (the spin axis of revolute joints in URDF)
        flip = Gf.Quatf(0, 1, 0, 0)  # 180deg around X (w=0, i=1, j=0, k=0)
        new_rot = current * flip
        rot_attr.Set(new_rot)
        print(f"[flip_axis] {jpath}: localRot {current} -> {new_rot}")
    else:
        print(f"[flip_axis] {jpath}: no localRot attribute found")

print("[flip_axis] DONE.  Test with: ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}}' -r 10")
