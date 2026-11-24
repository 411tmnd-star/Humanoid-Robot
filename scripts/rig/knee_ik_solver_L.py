"""
knee_ik_solver_L.py -- v1.0
Left-side mirror of knee_ik_solver.py -- see that file for the full design
rationale (Compound classification, why no twist half, per
specifications/legs/knee.md) and for the hip-build update
(specifications/legs/hip.md): the knee's rotation plane now depends on
DEF_Thigh_L's LIVE pose (the hip ball-socket above it), read fresh each
call instead of assumed fixed. This file only re-targets every bone/empty
reference to the left-side names created during the mirror phase
(docs/phase_mirror.md); the math is identical.

Input: IK_Target.leg.L (desired ankle position).
Output: CTRL_Knee_L.rotation_euler (X only). Feeds the mirrored
CTRL_Knee_L -> MECH_Knee_L (Copy Rotation -> Limit Rotation) -> DEF_Shin_L
(Copy Rotation) chain.

Authoring-time tool (docs/ENGINEERING_RULES.md Sec5), not a live constraint/driver: call
solve() after repositioning IK_Target.leg.L.
"""

import bpy
import math
from mathutils import Vector

ARMATURE_NAME = 'RobotArmature'
TARGET_NAME = 'IK_Target.leg.L'

KNEE_MIN_DEG, KNEE_MAX_DEG = 0.0, 120.0


def _rodrigues(v, k, theta):
    """Rotate vector v by angle theta (radians) around unit axis k, general form
    (does not assume v is perpendicular to k)."""
    k = k.normalized()
    return (v * math.cos(theta)
            + k.cross(v) * math.sin(theta)
            + k * (k.dot(v)) * (1 - math.cos(theta)))


def solve(report=True):
    """
    Recompute CTRL_Knee_L from the current IK_Target.leg.L world position.

    Returns a dict: knee_deg, knee_clamped, residual_reach_error
    """
    if ARMATURE_NAME not in bpy.data.objects:
        raise RuntimeError(f"knee_ik_solver_L: armature '{ARMATURE_NAME}' not found -- aborting, nothing changed.")
    arm = bpy.data.objects[ARMATURE_NAME]
    db = arm.data.bones
    pb = arm.pose.bones

    if TARGET_NAME not in bpy.data.objects:
        raise RuntimeError(f"knee_ik_solver_L: required empty '{TARGET_NAME}' not found -- aborting, nothing changed.")
    for name in ('CTRL_Knee_L', 'MECH_Knee_L', 'DEF_Thigh_L', 'DEF_Shin_L'):
        if name not in pb:
            raise RuntimeError(f"knee_ik_solver_L: required bone '{name}' not found -- aborting, nothing changed.")

    ctrl_knee = pb['CTRL_Knee_L']
    for c in ctrl_knee.constraints:
        if c.type == 'IK' and not c.mute:
            raise RuntimeError(
                "knee_ik_solver_L: an enabled Blender IK constraint is present on "
                "CTRL_Knee_L. This module replaces that mechanism -- mute or remove "
                "it before calling solve()."
            )

    ik_target = bpy.data.objects[TARGET_NAME]
    thigh_bone = db['DEF_Thigh_L']
    shin_bone = db['DEF_Shin_L']

    # Object.matrix_world is not guaranteed synced immediately after a plain
    # `.location = ...` write in the same script -- confirmed empirically on
    # the right-side solver. Force a sync before reading target world position.
    bpy.context.view_layer.update()

    # --- knee geometry: rotation axis/plane now depend on the hip's LIVE pose
    # -- see knee_ik_solver.py's equivalent section for the full derivation.
    knee_rest_world = arm.matrix_world @ pb['DEF_Thigh_L'].matrix
    knee_head_world = arm.matrix_world @ pb['DEF_Thigh_L'].tail
    flex_axis_world = (knee_rest_world.to_3x3() @ Vector((1, 0, 0))).normalized()

    shin_len = (shin_bone.head_local - shin_bone.tail_local).length
    shin_tail_offset_local = (thigh_bone.matrix_local.to_3x3().inverted()
                               @ (shin_bone.tail_local - thigh_bone.tail_local))
    rest_shin_dir_world = knee_rest_world.to_3x3() @ shin_tail_offset_local
    rest_shin_dir_world = (rest_shin_dir_world
                            - rest_shin_dir_world.dot(flex_axis_world) * flex_axis_world).normalized()

    basis_u = rest_shin_dir_world
    basis_v = flex_axis_world.cross(basis_u).normalized()

    target_world = ik_target.matrix_world.translation
    to_target = target_world - knee_head_world
    to_target_planar = to_target - to_target.dot(flex_axis_world) * flex_axis_world

    if to_target_planar.length < 1e-6:
        knee_angle_raw = 0.0
    else:
        to_target_planar.normalize()
        knee_angle_raw = math.atan2(basis_v.dot(to_target_planar), basis_u.dot(to_target_planar))

    knee_min, knee_max = math.radians(KNEE_MIN_DEG), math.radians(KNEE_MAX_DEG)
    knee_clamped = not (knee_min <= knee_angle_raw <= knee_max)
    knee_angle = max(knee_min, min(knee_max, knee_angle_raw))

    shin_dir_current = _rodrigues(basis_u, flex_axis_world, knee_angle)
    ankle_world = knee_head_world + shin_dir_current * shin_len
    residual = (ankle_world - target_world).length

    # --- write result as a plain channel value (matches the ground-truth-verified FK path) ---
    ctrl_knee.rotation_mode = 'XYZ'
    ctrl_knee.rotation_euler = (knee_angle, 0.0, 0.0)
    bpy.context.view_layer.update()

    result = {
        'knee_deg': math.degrees(knee_angle),
        'knee_clamped': knee_clamped,
        'residual_reach_error': residual,
    }
    if report:
        print(f"knee_ik_solver_L.solve(): knee={result['knee_deg']:.2f} deg "
              f"(clamped={result['knee_clamped']}), residual_reach_error={residual:.4f}")
    return result
