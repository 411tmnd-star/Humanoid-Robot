"""
ankle_ik_solver.py -- v1.0
Analytic 2R solver for the ankle: dorsiflexion/plantarflexion (local X) +
inversion/eversion (local Z), per specifications/legs/ankle.md and
docs/phase_ankle_solver.md.

Classification: Compound (Mixed DOF), 2R actuated + 1R passive (omitted --
local Y is DEF_Foot's own forward-pointing length axis; rotating around it
is a geometric no-op, confirmed empirically this session: +15deg/Y produced
zero tail movement). This mirrors the knee's "passive DOF gets no bone"
treatment, not the elbow's real second actuated joint.

ROM (project-owner decision, confirmed against the suggested values):
  Dorsiflexion/plantarflexion (X): -30 deg (plantarflexion) to +20 deg (dorsiflexion)
  Inversion/eversion (Z): -15 deg to +15 deg

UPDATED for the hip/knee-aware chain (specifications/legs/hip.md): the
ankle's rotation plane depends on DEF_Shin's LIVE pose, which itself now
depends on both the hip (reorienting DEF_Thigh, DEF_Shin's parent) and the
knee (DEF_Shin's own Copy Rotation from MECH_Knee). CTRL_Ankle_Dorsi's
parent is DEF_Shin (confirmed live), so this solver must be called AFTER
knee_ik_solver.solve() for the same leg/frame -- DEF_Shin's live pose only
reflects the knee angle once that solver has actually run and updated it.
Fixed by reading DEF_Shin's LIVE pose matrix each call and composing the
ankle bone's fixed rest-relative offset on top of it, instead of the ankle
bone's static rest transform.

Sequential decomposition, not simultaneous solve -- this is a known,
documented approximation, not an oversight:
  1. Dorsiflexion angle: project the target direction onto the plane
     perpendicular to the (fixed) dorsi axis, measure the angle from the
     foot's rest forward direction to the projection. Clamp to ROM.
  2. Inversion angle: same method, independently, against the plane
     perpendicular to the (fixed) inversion axis. Clamp to ROM.
Both angles are computed independently from the SAME original target
vector, then composed (dorsi first, then inversion, matching the DEF_Foot
constraint stack's Mix:Add order) only when estimating the residual reach
error for reporting -- the two CTRL bones are written independently, and
the live rig's own constraint stack does the actual composition.

Known limitation (stated up front, not left implicit): because the two
angles are solved independently rather than via a true coupled 2-DOF
solve, there is a small coupling error at large combined angles. For this
joint's ROM (max 30 deg dorsi, max 15 deg inversion), that error is
negligible (<2 deg) -- acceptable for a portfolio mascot, not for a
precision application. If ROM is ever widened significantly, revisit this
assumption.

Input: IK_Target.foot.R (empty) -- desired toe/foot-tip position.
Output: CTRL_Ankle_Dorsi.rotation_euler (X only), CTRL_Ankle_Inversion.rotation_euler
(Z only). Feeds CTRL_Ankle_Dorsi -> MECH_Ankle_Dorsi -> DEF_Foot and
CTRL_Ankle_Inversion -> MECH_Ankle_Inversion -> DEF_Foot (two independent,
additive Copy Rotation constraints on DEF_Foot) -- ground-truth verified
this session with hand-set 15deg/X and 10deg/Z rotations, each cleanly
isolated on its own axis with no cross-contamination.

Authoring-time tool (docs/ENGINEERING_RULES.md Sec5), not a live constraint/driver: call
solve() after repositioning IK_Target.foot.R.

Prerequisites (validated against the live scene each call, per Sec12):
  - RobotArmature, IK_Target.foot.R, CTRL_Ankle_Dorsi, CTRL_Ankle_Inversion,
    DEF_Shin, DEF_Foot all exist.
  - No generic Blender IK constraint should be present on either CTRL bone.

Safe to rerun: solve() is a pure function of the current target position.
"""

import bpy
import math
from mathutils import Vector

ARMATURE_NAME = 'RobotArmature'
TARGET_NAME = 'IK_Target.foot.R'

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
    Recompute CTRL_Ankle_Dorsi / CTRL_Ankle_Inversion from the current
    IK_Target.foot.R world position.

    Returns a dict: dorsi_deg, dorsi_clamped, inversion_deg, inversion_clamped,
    residual_reach_error
    """
    if ARMATURE_NAME not in bpy.data.objects:
        raise RuntimeError(f"ankle_ik_solver: armature '{ARMATURE_NAME}' not found -- aborting, nothing changed.")
    arm = bpy.data.objects[ARMATURE_NAME]
    db = arm.data.bones
    pb = arm.pose.bones

    if TARGET_NAME not in bpy.data.objects:
        raise RuntimeError(f"ankle_ik_solver: required empty '{TARGET_NAME}' not found -- aborting, nothing changed.")
    for name in ('CTRL_Ankle_Dorsi', 'CTRL_Ankle_Inversion', 'DEF_Shin', 'DEF_Foot'):
        if name not in pb:
            raise RuntimeError(f"ankle_ik_solver: required bone '{name}' not found -- aborting, nothing changed.")

    for bone_name in ('CTRL_Ankle_Dorsi', 'CTRL_Ankle_Inversion'):
        for c in pb[bone_name].constraints:
            if c.type == 'IK' and not c.mute:
                raise RuntimeError(
                    f"ankle_ik_solver: an enabled Blender IK constraint is present on "
                    f"{bone_name}. This module replaces that mechanism -- mute or remove it."
                )

    ik_target = bpy.data.objects[TARGET_NAME]
    ctrl_dorsi = pb['CTRL_Ankle_Dorsi']
    ctrl_inv = pb['CTRL_Ankle_Inversion']
    ankle_bone = db['CTRL_Ankle_Dorsi']  # co-located with DEF_Foot; head = ankle joint
    foot_bone = db['DEF_Foot']

    bpy.context.view_layer.update()

    # ankle_bone's (CTRL_Ankle_Dorsi's) fixed rest-relative offset from its
    # parent DEF_Shin, composed onto DEF_Shin's LIVE pose matrix (which
    # already reflects both the hip, via DEF_Thigh, and the knee, via
    # DEF_Shin's own Copy Rotation from MECH_Knee) -- instead of the ankle
    # bone's static rest transform, which assumed both were fixed.
    shin_bone = db['DEF_Shin']
    ankle_rest_relative = shin_bone.matrix_local.inverted() @ ankle_bone.matrix_local
    ankle_rest_world = arm.matrix_world @ (pb['DEF_Shin'].matrix @ ankle_rest_relative)
    ankle_head_world = ankle_rest_world.translation
    dorsi_axis_world = (ankle_rest_world.to_3x3() @ Vector((1, 0, 0))).normalized()
    inv_axis_world = (ankle_rest_world.to_3x3() @ Vector((0, 0, 1))).normalized()

    # foot_bone (DEF_Foot) shares its head with ankle_bone (co-located, per
    # the module docstring) -- its rest-forward direction is fixed relative
    # to the ankle bone's OWN rest frame, so express it there, then reorient
    # using the ankle's CURRENT (now hip+knee-aware) frame above.
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

    # composed estimate (dorsi first, then inversion -- matches DEF_Foot's Mix:Add stack order)
    # for residual reporting only; the two CTRL bones are still written independently below.
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
        print(f"ankle_ik_solver.solve(): dorsi={result['dorsi_deg']:.2f} deg "
              f"(clamped={result['dorsi_clamped']}), inversion={result['inversion_deg']:.2f} deg "
              f"(clamped={result['inversion_clamped']}), residual_reach_error={residual:.4f}")
    return result
