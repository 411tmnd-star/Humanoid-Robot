"""
knee_ik_solver.py -- v1.0
Analytic 1-DOF hinge solver for the knee.

Per specifications/legs/knee.md: KINEMATICS.md classifies the knee as
Compound (Mixed DOF), not simple Hinge, with ONE actuated DOF (flexion,
local X, 0-120 deg per project-owner decision 2026-09-02) and a passive
secondary rotation that is OMITTED entirely -- no bone, not solved for.
Unlike the elbow, there is no twist companion joint here, so unlike
arm_ik_solver.py this solver has no pole-target / twist-solving half at
all. That reuse assumption was made once, checked against KINEMATICS.md's
actual Compound-joint methodology, and explicitly rejected during the
knee's classification phase (docs/phase_knee_spec_and_ankle_check.md) --
this file reflects the corrected scope, not the original assumption.

Geometry: the shin's length is fixed, so this is a single-DOF hinge sweeping
a fixed-RADIUS arc -- the same shape of problem as the elbow's flexion half,
solved the same way (Rodrigues projection, explicit clamp, best-effort-aim
fallback).

UPDATED for the hip build (specifications/legs/hip.md): the knee's rotation
PLANE is no longer fixed in world space -- a 3R hip ball-socket now sits
above DEF_Thigh and can reorient the whole thigh. Fixed by reading
DEF_Thigh's LIVE pose matrix (already evaluated by Blender, hip constraint
included) each call instead of its static rest transform. DEF_Thigh is
used directly as this solver's own rotational reference (no intermediate
stub-bone offset the way the elbow uses CTRL_Elbow relative to
DEF_UpperArm), so this fix is a direct swap: `thigh_bone.matrix_local`
(static) -> `pb['DEF_Thigh'].matrix` (live).

Input: IK_Target.leg.R (empty) -- desired ankle position.
Output: CTRL_Knee.rotation_euler (X only). Feeds the already-wired
CTRL_Knee -> MECH_Knee (Copy Rotation -> Limit Rotation) -> DEF_Shin
(Copy Rotation) chain -- ground-truth verified this session with a plain
45 deg hand-set rotation (MECH_Knee and DEF_Shin both showed exactly 45 deg
around local X).

Authoring-time tool (docs/ENGINEERING_RULES.md Sec5), not a live constraint/driver: call
solve() after repositioning IK_Target.leg.R.

Design decisions (explicit, matching arm_ik_solver.py's pattern):
- Knee angle solved by projecting the target onto the knee's fixed
  rotation plane and finding the angle that points the shin segment
  (fixed length) as close to the target direction as possible.
- Explicit clamping to [0, 120] deg after the trig, unconditionally --
  not trusted to fall in range because the geometry "shouldn't" produce
  an out-of-range value.
- Unreachable target fallback: fixed-length, fixed-axis hinge, cannot
  stretch. Always aims for minimum angular error to the target direction,
  then clamps to ROM if that aim falls outside 0-120 deg. Residual
  distance between the resulting ankle position and the actual target is
  returned so the caller can see how far off the reach was -- nothing
  fails silently.

Prerequisites (validated against the live scene each call, per Sec12):
  - RobotArmature, IK_Target.leg.R, CTRL_Knee, MECH_Knee, DEF_Thigh,
    DEF_Shin all exist.
  - No generic Blender IK constraint should be present on CTRL_Knee or
    MECH_Knee -- this module is the whole mechanism, not a layer under one.

Safe to rerun: solve() is a pure function of the current target position
and always recomputes the angle from scratch.
"""

import bpy
import math
from mathutils import Vector

ARMATURE_NAME = 'RobotArmature'
TARGET_NAME = 'IK_Target.leg.R'

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
    Recompute CTRL_Knee from the current IK_Target.leg.R world position.

    Returns a dict: knee_deg, knee_clamped, residual_reach_error
    """
    if ARMATURE_NAME not in bpy.data.objects:
        raise RuntimeError(f"knee_ik_solver: armature '{ARMATURE_NAME}' not found -- aborting, nothing changed.")
    arm = bpy.data.objects[ARMATURE_NAME]
    db = arm.data.bones
    pb = arm.pose.bones

    if TARGET_NAME not in bpy.data.objects:
        raise RuntimeError(f"knee_ik_solver: required empty '{TARGET_NAME}' not found -- aborting, nothing changed.")
    for name in ('CTRL_Knee', 'MECH_Knee', 'DEF_Thigh', 'DEF_Shin'):
        if name not in pb:
            raise RuntimeError(f"knee_ik_solver: required bone '{name}' not found -- aborting, nothing changed.")

    ctrl_knee = pb['CTRL_Knee']
    for c in ctrl_knee.constraints:
        if c.type == 'IK' and not c.mute:
            raise RuntimeError(
                "knee_ik_solver: an enabled Blender IK constraint is present on "
                "CTRL_Knee. This module replaces that mechanism -- mute or remove "
                "it before calling solve()."
            )

    ik_target = bpy.data.objects[TARGET_NAME]
    thigh_bone = db['DEF_Thigh']
    shin_bone = db['DEF_Shin']

    # Object.matrix_world is not guaranteed synced immediately after a plain
    # `.location = ...` write in the same script -- confirmed empirically on
    # the arm solver. Force a sync before reading target world position.
    bpy.context.view_layer.update()

    # --- knee geometry: rotation axis/plane now depend on the hip's LIVE pose.
    # DEF_Thigh's pose matrix already reflects the hip (via its own Copy
    # Rotation constraint from MECH_Hip_Rot, evaluated by Blender on the
    # view_layer.update() above) -- read it directly instead of the thigh
    # bone's static rest transform.
    knee_rest_world = arm.matrix_world @ pb['DEF_Thigh'].matrix
    knee_head_world = arm.matrix_world @ pb['DEF_Thigh'].tail   # LIVE knee joint location
    flex_axis_world = (knee_rest_world.to_3x3() @ Vector((1, 0, 0))).normalized()

    shin_len = (shin_bone.head_local - shin_bone.tail_local).length
    # Shin's offset from the knee is fixed in the thigh's OWN rest-relative
    # frame (neither bone's rest geometry changed) -- express it there, then
    # reorient using the thigh's CURRENT (now hip-aware) frame above, instead
    # of the old flat `arm.matrix_world.to_3x3() @ (...)`.
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
        print(f"knee_ik_solver.solve(): knee={result['knee_deg']:.2f} deg "
              f"(clamped={result['knee_clamped']}), residual_reach_error={residual:.4f}")
    return result
