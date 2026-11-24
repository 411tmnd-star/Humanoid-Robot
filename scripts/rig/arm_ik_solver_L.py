"""
arm_ik_solver_L.py -- v1.0
Left-side mirror of arm_ik_solver.py -- see that file for the full design
rationale (why analytic instead of generic IK, per docs/ENGINEERING_RULES.md Sec7/Sec11)
and for the shoulder-build update (specifications/arm/shoulder_complex.md):
the elbow's rotation plane now depends on DEF_UpperArm_L's LIVE pose (the
shoulder ball-socket above it), read fresh each call instead of assumed
fixed. This file only re-targets every bone/empty reference to the
left-side names created during the mirror phase (docs/phase_mirror.md); the
math is identical.

Inputs: IK_Target.arm.L (desired wrist position), IK_Pole.arm.L (forearm
roll/orientation reference).
Outputs: CTRL_Elbow_L.rotation_euler (X only), CTRL_Forearm_Twist_L.rotation_euler
(Y only). Feeds CTRL_Elbow_L -> MECH_Elbow_L -> DEF_Elbow_L and
CTRL_Forearm_Twist_L -> MECH_Forearm_Twist_L -> DEF_Forearm_Twist_L, the
same Copy Rotation -> Limit Rotation stack mirrored onto the left side.

Authoring-time tool (docs/ENGINEERING_RULES.md Sec5), not a live constraint/driver: call
solve() after repositioning IK_Target.arm.L / IK_Pole.arm.L.
"""

import bpy
import math
from mathutils import Vector

ARMATURE_NAME = 'RobotArmature'
TARGET_NAME = 'IK_Target.arm.L'
POLE_NAME = 'IK_Pole.arm.L'

ELBOW_MIN_DEG, ELBOW_MAX_DEG = 0.0, 130.0
TWIST_MIN_DEG, TWIST_MAX_DEG = -50.0, 50.0


def _rodrigues(v, k, theta):
    """Rotate vector v by angle theta (radians) around unit axis k, general form
    (does not assume v is perpendicular to k)."""
    k = k.normalized()
    return (v * math.cos(theta)
            + k.cross(v) * math.sin(theta)
            + k * (k.dot(v)) * (1 - math.cos(theta)))


def solve(report=True):
    """
    Recompute CTRL_Elbow_L / CTRL_Forearm_Twist_L from the current
    IK_Target.arm.L / IK_Pole.arm.L world positions.

    Returns a dict:
      elbow_deg, elbow_clamped, twist_deg, twist_clamped, residual_reach_error
    """
    if ARMATURE_NAME not in bpy.data.objects:
        raise RuntimeError(f"arm_ik_solver_L: armature '{ARMATURE_NAME}' not found -- aborting, nothing changed.")
    arm = bpy.data.objects[ARMATURE_NAME]
    db = arm.data.bones
    pb = arm.pose.bones

    for name in (TARGET_NAME, POLE_NAME):
        if name not in bpy.data.objects:
            raise RuntimeError(f"arm_ik_solver_L: required empty '{name}' not found -- aborting, nothing changed.")
    for name in ('CTRL_Elbow_L', 'CTRL_Forearm_Twist_L', 'DEF_UpperArm_L'):
        if name not in pb:
            raise RuntimeError(f"arm_ik_solver_L: required bone '{name}' not found -- aborting, nothing changed.")

    ctrl_forearm = pb['CTRL_Forearm_Twist_L']
    for c in ctrl_forearm.constraints:
        if c.type == 'IK' and not c.mute:
            raise RuntimeError(
                "arm_ik_solver_L: an enabled Blender IK constraint is present on "
                "CTRL_Forearm_Twist_L. This module replaces that mechanism -- mute "
                "or remove it before calling solve(), rather than layering both."
            )

    ik_target = bpy.data.objects[TARGET_NAME]
    pole_target = bpy.data.objects[POLE_NAME]
    ctrl_elbow = pb['CTRL_Elbow_L']
    elbow_bone = db['CTRL_Elbow_L']
    forearm_bone = db['CTRL_Forearm_Twist_L']

    # Object.matrix_world is not guaranteed synced immediately after a plain
    # `.location = ...` write in the same script -- confirmed empirically on
    # the right-side solver. Force a sync before reading target/pole world positions.
    bpy.context.view_layer.update()

    # --- elbow geometry: rotation axis/plane now depend on the shoulder's LIVE
    # pose -- see arm_ik_solver.py's equivalent section for the full
    # derivation. DEF_UpperArm_L's pose matrix already reflects the shoulder
    # (via its own Copy Rotation from MECH_Shoulder_L_Rot); compose the
    # elbow's fixed rest-relative offset on top of that instead of reading
    # the elbow bone's static rest transform directly.
    upper_arm_bone = db['DEF_UpperArm_L']
    elbow_rest_relative = upper_arm_bone.matrix_local.inverted() @ elbow_bone.matrix_local
    elbow_rest_world = arm.matrix_world @ (pb['DEF_UpperArm_L'].matrix @ elbow_rest_relative)
    elbow_head_world = elbow_rest_world.translation
    flex_axis_world = (elbow_rest_world.to_3x3() @ Vector((1, 0, 0))).normalized()

    forearm_len = (forearm_bone.head_local - forearm_bone.tail_local).length
    forearm_tail_offset_local = (elbow_bone.matrix_local.to_3x3().inverted()
                                  @ (forearm_bone.tail_local - elbow_bone.head_local))
    rest_forearm_dir_world = elbow_rest_world.to_3x3() @ forearm_tail_offset_local
    rest_forearm_dir_world = (rest_forearm_dir_world
                               - rest_forearm_dir_world.dot(flex_axis_world) * flex_axis_world).normalized()

    basis_u = rest_forearm_dir_world
    basis_v = flex_axis_world.cross(basis_u).normalized()

    target_world = ik_target.matrix_world.translation
    to_target = target_world - elbow_head_world
    to_target_planar = to_target - to_target.dot(flex_axis_world) * flex_axis_world

    if to_target_planar.length < 1e-6:
        elbow_angle_raw = 0.0
    else:
        to_target_planar.normalize()
        elbow_angle_raw = math.atan2(basis_v.dot(to_target_planar), basis_u.dot(to_target_planar))

    elbow_min, elbow_max = math.radians(ELBOW_MIN_DEG), math.radians(ELBOW_MAX_DEG)
    elbow_clamped = not (elbow_min <= elbow_angle_raw <= elbow_max)
    elbow_angle = max(elbow_min, min(elbow_max, elbow_angle_raw))

    forearm_dir_current = _rodrigues(basis_u, flex_axis_world, elbow_angle)
    wrist_world = elbow_head_world + forearm_dir_current * forearm_len
    residual = (wrist_world - target_world).length

    # --- forearm twist: orient local-X reference toward the pole, around the twist (local Y) axis ---
    # Bug fix -- see arm_ik_solver.py for the full derivation. CTRL_Forearm_Twist_L's rest
    # matrix is identical to CTRL_Elbow_L's (mirrored straight-arm rest pose), so deriving the
    # reference from bone roll collapsed onto flex_axis_world itself, invariant to elbow_angle
    # and unrelated to the pole -- always clamped to +-50. Use basis_v (already perpendicular
    # to flex_axis_world and basis_u, already computed above) as the non-degenerate,
    # "palm forward at rest" twist-zero reference instead.
    forearm_ref_axis_world_pretwist = _rodrigues(basis_v, flex_axis_world, elbow_angle)

    twist_axis_world = forearm_dir_current

    pole_world = pole_target.matrix_world.translation
    forearm_mid_world = elbow_head_world + forearm_dir_current * (forearm_len * 0.5)
    to_pole = pole_world - forearm_mid_world
    to_pole_planar = to_pole - to_pole.dot(twist_axis_world) * twist_axis_world

    ref_planar = (forearm_ref_axis_world_pretwist
                  - forearm_ref_axis_world_pretwist.dot(twist_axis_world) * twist_axis_world)

    if to_pole_planar.length < 1e-6 or ref_planar.length < 1e-6:
        twist_angle_raw = 0.0
    else:
        to_pole_planar.normalize()
        ref_planar.normalize()
        perp = twist_axis_world.cross(ref_planar).normalized()
        twist_angle_raw = math.atan2(to_pole_planar.dot(perp), to_pole_planar.dot(ref_planar))

    twist_min, twist_max = math.radians(TWIST_MIN_DEG), math.radians(TWIST_MAX_DEG)
    twist_clamped = not (twist_min <= twist_angle_raw <= twist_max)
    twist_angle = max(twist_min, min(twist_max, twist_angle_raw))

    # --- write result as plain channel values (matches the validated FK path) ---
    ctrl_elbow.rotation_mode = 'XYZ'
    ctrl_elbow.rotation_euler = (elbow_angle, 0.0, 0.0)
    ctrl_forearm.rotation_mode = 'XYZ'
    ctrl_forearm.rotation_euler = (0.0, twist_angle, 0.0)
    bpy.context.view_layer.update()

    result = {
        'elbow_deg': math.degrees(elbow_angle),
        'elbow_clamped': elbow_clamped,
        'twist_deg': math.degrees(twist_angle),
        'twist_clamped': twist_clamped,
        'residual_reach_error': residual,
    }
    if report:
        print(f"arm_ik_solver_L.solve(): elbow={result['elbow_deg']:.2f} deg "
              f"(clamped={result['elbow_clamped']}), twist={result['twist_deg']:.2f} deg "
              f"(clamped={result['twist_clamped']}), residual_reach_error={residual:.4f}")
    return result
