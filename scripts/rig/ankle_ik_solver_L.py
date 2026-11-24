"""
ankle_ik_solver_L.py -- v1.0
Left-side mirror of ankle_ik_solver.py -- see that file for the full
design rationale (Compound classification, sequential 2R decomposition,
known coupling-error limitation, ROM values) and for the hip/knee-aware
update (specifications/legs/hip.md): the ankle's rotation plane depends on
DEF_Shin_L's LIVE pose (hip + knee both above it now), read fresh each
call. CTRL_Ankle_Dorsi_L's parent is DEF_Shin_L (confirmed live) -- call
this AFTER knee_ik_solver_L.solve() for the same frame. This file only
re-targets every bone/empty reference to the left-side names created during
docs/phase_ankle_mirror.md; the math is identical.

Input: IK_Target.foot.L (desired toe/foot-tip position).
Output: CTRL_Ankle_Dorsi_L.rotation_euler (X only), CTRL_Ankle_Inversion_L.rotation_euler
(Z only). Feeds CTRL_Ankle_Dorsi_L -> MECH_Ankle_Dorsi_L -> DEF_Foot_L and
CTRL_Ankle_Inversion_L -> MECH_Ankle_Inversion_L -> DEF_Foot_L, the same
two independent, additive Copy Rotation constraints mirrored onto the
left side.

Authoring-time tool (docs/ENGINEERING_RULES.md Sec5), not a live constraint/driver: call
solve() after repositioning IK_Target.foot.L.
"""

import bpy
import math
from mathutils import Vector

ARMATURE_NAME = 'RobotArmature'
TARGET_NAME = 'IK_Target.foot.L'

DORSI_MIN_DEG, DORSI_MAX_DEG = -30.0, 20.0
INVERSION_MIN_DEG, INVERSION_MAX_DEG = -15.0, 15.0


def _rodrigues(v, k, theta):
    """Rotate vector v by angle theta (radians) around unit axis k, general form
    (does not assume v is perpendicular to k)."""
    k = k.normalized()
    return (v * math.cos(theta)
            + k.cross(v) * math.sin(theta)
            + k * (k.dot(v)) * (1 - math.cos(theta)))


def _solve_axis(to_target, axis_world, rest_forward_world, rom_min_deg, rom_max_deg):
    """Project to_target onto the plane perpendicular to axis_world, measure
    the signed angle from rest_forward_world (also projected), clamp to ROM.
    Returns (angle_rad, clamped_bool)."""
    basis_u = (rest_forward_world
               - rest_forward_world.dot(axis_world) * axis_world).normalized()
    basis_v = axis_world.cross(basis_u).normalized()

    planar = to_target - to_target.dot(axis_world) * axis_world
    if planar.length < 1e-6:
        angle_raw = 0.0
    else:
        planar.normalize()
        angle_raw = math.atan2(basis_v.dot(planar), basis_u.dot(planar))

    rom_min, rom_max = math.radians(rom_min_deg), math.radians(rom_max_deg)
    clamped = not (rom_min <= angle_raw <= rom_max)
    angle = max(rom_min, min(rom_max, angle_raw))
    return angle, clamped


def solve(report=True):
    """
    Recompute CTRL_Ankle_Dorsi_L / CTRL_Ankle_Inversion_L from the current
    IK_Target.foot.L world position.

    Returns a dict: dorsi_deg, dorsi_clamped, inversion_deg, inversion_clamped,
    residual_reach_error
    """
    if ARMATURE_NAME not in bpy.data.objects:
        raise RuntimeError(f"ankle_ik_solver_L: armature '{ARMATURE_NAME}' not found -- aborting, nothing changed.")
    arm = bpy.data.objects[ARMATURE_NAME]
    db = arm.data.bones
    pb = arm.pose.bones

    if TARGET_NAME not in bpy.data.objects:
        raise RuntimeError(f"ankle_ik_solver_L: required empty '{TARGET_NAME}' not found -- aborting, nothing changed.")
    for name in ('CTRL_Ankle_Dorsi_L', 'CTRL_Ankle_Inversion_L', 'DEF_Shin_L', 'DEF_Foot_L'):
        if name not in pb:
            raise RuntimeError(f"ankle_ik_solver_L: required bone '{name}' not found -- aborting, nothing changed.")

    for bone_name in ('CTRL_Ankle_Dorsi_L', 'CTRL_Ankle_Inversion_L'):
        for c in pb[bone_name].constraints:
            if c.type == 'IK' and not c.mute:
                raise RuntimeError(
                    f"ankle_ik_solver_L: an enabled Blender IK constraint is present on "
                    f"{bone_name}. This module replaces that mechanism -- mute or remove it."
                )

    ik_target = bpy.data.objects[TARGET_NAME]
    ctrl_dorsi = pb['CTRL_Ankle_Dorsi_L']
    ctrl_inv = pb['CTRL_Ankle_Inversion_L']
    ankle_bone = db['CTRL_Ankle_Dorsi_L']  # co-located with DEF_Foot_L; head = ankle joint
    foot_bone = db['DEF_Foot_L']

    bpy.context.view_layer.update()

    # See ankle_ik_solver.py's equivalent section for the full derivation.
    shin_bone = db['DEF_Shin_L']
    ankle_rest_relative = shin_bone.matrix_local.inverted() @ ankle_bone.matrix_local
    ankle_rest_world = arm.matrix_world @ (pb['DEF_Shin_L'].matrix @ ankle_rest_relative)
    ankle_head_world = ankle_rest_world.translation
    dorsi_axis_world = (ankle_rest_world.to_3x3() @ Vector((1, 0, 0))).normalized()
    inv_axis_world = (ankle_rest_world.to_3x3() @ Vector((0, 0, 1))).normalized()

    foot_forward_offset_local = (ankle_bone.matrix_local.to_3x3().inverted()
                                  @ (foot_bone.tail_local - foot_bone.head_local))
    rest_forward_world = (ankle_rest_world.to_3x3() @ foot_forward_offset_local).normalized()
    foot_len = (foot_bone.head_local - foot_bone.tail_local).length

    target_world = ik_target.matrix_world.translation
    to_target = target_world - ankle_head_world

    dorsi_angle, dorsi_clamped = _solve_axis(to_target, dorsi_axis_world, rest_forward_world,
                                              DORSI_MIN_DEG, DORSI_MAX_DEG)
    inv_angle, inv_clamped = _solve_axis(to_target, inv_axis_world, rest_forward_world,
                                          INVERSION_MIN_DEG, INVERSION_MAX_DEG)

    dir_after_dorsi = _rodrigues(rest_forward_world, dorsi_axis_world, dorsi_angle)
    dir_after_both = _rodrigues(dir_after_dorsi, inv_axis_world, inv_angle)
    toe_world = ankle_head_world + dir_after_both * foot_len
    residual = (toe_world - target_world).length

    ctrl_dorsi.rotation_mode = 'XYZ'
    ctrl_dorsi.rotation_euler = (dorsi_angle, 0.0, 0.0)
    ctrl_inv.rotation_mode = 'XYZ'
    ctrl_inv.rotation_euler = (0.0, 0.0, inv_angle)
    bpy.context.view_layer.update()

    result = {
        'dorsi_deg': math.degrees(dorsi_angle),
        'dorsi_clamped': dorsi_clamped,
        'inversion_deg': math.degrees(inv_angle),
        'inversion_clamped': inv_clamped,
        'residual_reach_error': residual,
    }
    if report:
        print(f"ankle_ik_solver_L.solve(): dorsi={result['dorsi_deg']:.2f} deg "
              f"(clamped={result['dorsi_clamped']}), inversion={result['inversion_deg']:.2f} deg "
              f"(clamped={result['inversion_clamped']}), residual_reach_error={residual:.4f}")
    return result
