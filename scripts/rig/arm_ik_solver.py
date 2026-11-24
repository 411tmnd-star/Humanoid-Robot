"""
arm_ik_solver.py -- v1.0
Purpose: Analytic 2-link hinge+twist IK for the elbow / forearm-twist chain.

Why this exists instead of a Blender IK constraint (docs/ENGINEERING_RULES.md Sec7/Sec11):
elbow_forearm.md classifies the elbow as a Hinge (1R, local X, 0-130 deg) and
the forearm as a Twist (1R, local Y, +-50 deg) -- a locked-axis serial-revolute
chain, not a ball-joint case. Blender's generic IK constraint was tried on
this chain and found, empirically this session, to not respect a chain
bone's lock_ik_y/lock_ik_z/ik_min_x/ik_max_x settings: with the constraint
solving, CTRL_Elbow's rotation contained a 118.69 deg component around an
axis nowhere near its local X, reproduced identically under both LEGACY and
ITASC solvers, independent of chain_count, mute/rebuild, or pose mode. Once
baked into CTRL_Elbow's own channel and passed through MECH_Elbow's already-
validated Copy Rotation -> Limit Rotation stack, the out-of-range result
collapsed to identity, so the elbow never visibly moved. Per Sec11, this is
exactly the "looks like it needs IK but is really a locked-axis chain" case,
so the solve is done here analytically instead: the elbow is a 1-DOF hinge
sweeping a fixed-RADIUS arc (forearm length never changes), solved directly
and clamped explicitly to the spec'd ROM rather than trusted to fall in range.

UPDATED for the shoulder build (specifications/arm/shoulder_complex.md): the
elbow's rotation PLANE is no longer fixed in world space -- a 3R shoulder
ball-socket now sits above DEF_UpperArm and can reorient the whole upper arm.
The original assumption ("no shoulder joint exists yet") is false as soon as
CTRL_Shoulder_* is posed off zero. Fixed by reading DEF_UpperArm's LIVE pose
matrix (already evaluated by Blender, shoulder constraint included) each
call instead of the elbow bone's static rest transform -- see the elbow
geometry section below. This makes the solver correct for ANY current
shoulder pose, not just rest; it does not require the shoulder to be at
zero.

Inputs (unchanged from the original setup -- only the math changed):
  - IK_Target.arm.R (empty)  -- desired wrist position
  - IK_Pole.arm.R   (empty)  -- forearm roll/orientation reference

Outputs: CTRL_Elbow.rotation_euler (X only), CTRL_Forearm_Twist.rotation_euler
(Y only). These feed the existing, validated CTRL -> MECH Local-space Copy
Rotation link (elbow_forearm.md) -- confirmed working this session with a
plain 45 deg hand-set rotation on CTRL_Elbow, reproduced end to end through
MECH_Elbow and DEF_Elbow, as long as no live Blender IK constraint is present
on the chain.

This is an authoring-time tool (docs/ENGINEERING_RULES.md Sec5), not a live constraint or
driver: call solve() after repositioning IK_Target.arm.R / IK_Pole.arm.R.
Nothing here runs at animation playback time or survives glTF export --
per Sec5, the deliverable is baked DEF-only keyframes.

Design decisions (explicit, stated up front rather than left implicit):
  - Elbow angle: found by projecting the target onto the elbow's fixed
    rotation plane (fixed because there is no shoulder joint yet to
    reorient it) and solving the angle that points the rigid forearm
    segment as close to the target direction as possible.
  - Forearm twist: does not affect wrist position (rotation about the
    forearm's own length axis), so it is used to orient the forearm's
    local-X reference axis toward the pole target, projected perpendicular
    to the twist axis.
  - Explicit clamping: both computed angles are clamped to their spec'd
    ROM (0-130 deg elbow, +-50 deg twist) after the trig, unconditionally
    -- not merely assumed safe because the geometry "shouldn't" produce an
    out-of-range value. Floating point and edge-case target placements are
    handled the same way as a genuinely unreachable target.
  - Unreachable target fallback: this is a fixed-length, fixed-axis hinge,
    not a stretchy chain -- it cannot lengthen to close a gap. When the
    target cannot be exactly reached (the normal case: a single hinge angle
    only reaches a 1-parameter arc of points, not a 3D volume), the solver
    always aims for minimum angular error to the target direction, then
    clamps to ROM if that aim falls outside 0-130 deg. It never stretches
    and never leaves the pose undefined. The residual distance between the
    resulting wrist position and the actual target is returned so the
    caller can see how far off the reach was; nothing fails silently.

Prerequisites (validated against the live scene each call, per Sec12):
  - RobotArmature, IK_Target.arm.R, IK_Pole.arm.R, CTRL_Elbow,
    CTRL_Forearm_Twist, DEF_UpperArm all exist.
  - No live Blender IK constraint should be enabled on CTRL_Forearm_Twist
    when solve() is called -- this module's whole purpose is to replace
    that constraint, not layer under it. solve() checks and warns (does
    not silently mute it) if it finds one enabled.

Safe to rerun: solve() is a pure function of the current target/pole
positions and always recomputes both angles from scratch; it does not
depend on the previous pose.
"""

import bpy
import math
from mathutils import Vector

ARMATURE_NAME = 'RobotArmature'
TARGET_NAME = 'IK_Target.arm.R'
POLE_NAME = 'IK_Pole.arm.R'

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
    Recompute CTRL_Elbow / CTRL_Forearm_Twist from the current IK_Target.arm.R
    / IK_Pole.arm.R world positions.

    Returns a dict:
      elbow_deg, elbow_clamped, twist_deg, twist_clamped, residual_reach_error
    """
    if ARMATURE_NAME not in bpy.data.objects:
        raise RuntimeError(f"arm_ik_solver: armature '{ARMATURE_NAME}' not found -- aborting, nothing changed.")
    arm = bpy.data.objects[ARMATURE_NAME]
    db = arm.data.bones
    pb = arm.pose.bones

    for name in (TARGET_NAME, POLE_NAME):
        if name not in bpy.data.objects:
            raise RuntimeError(f"arm_ik_solver: required empty '{name}' not found -- aborting, nothing changed.")
    for name in ('CTRL_Elbow', 'CTRL_Forearm_Twist', 'DEF_UpperArm'):
        if name not in pb:
            raise RuntimeError(f"arm_ik_solver: required bone '{name}' not found -- aborting, nothing changed.")

    ctrl_forearm = pb['CTRL_Forearm_Twist']
    for c in ctrl_forearm.constraints:
        if c.type == 'IK' and not c.mute:
            raise RuntimeError(
                "arm_ik_solver: an enabled Blender IK constraint is still present on "
                "CTRL_Forearm_Twist. This module replaces that mechanism -- mute or "
                "remove it before calling solve(), rather than layering both."
            )

    ik_target = bpy.data.objects[TARGET_NAME]
    pole_target = bpy.data.objects[POLE_NAME]
    ctrl_elbow = pb['CTRL_Elbow']
    elbow_bone = db['CTRL_Elbow']
    forearm_bone = db['CTRL_Forearm_Twist']

    # Object.matrix_world is not guaranteed synced immediately after a plain
    # `.location = ...` write in the same script -- confirmed empirically this
    # session (a pole reposition immediately followed by solve() in one call
    # read the *old* pole position). Force a sync before reading target/pole
    # world positions so solve() is correct even called right after a move.
    bpy.context.view_layer.update()

    # --- elbow geometry: rotation axis/plane now depend on the shoulder's LIVE
    # pose. DEF_UpperArm's pose matrix already reflects the shoulder (via its
    # own Copy Rotation constraint from MECH_Shoulder_Rot, evaluated by
    # Blender on the view_layer.update() above) -- compose the elbow's FIXED
    # rest-relative offset from DEF_UpperArm on top of that LIVE matrix,
    # instead of reading the elbow bone's static rest transform directly.
    upper_arm_bone = db['DEF_UpperArm']
    elbow_rest_relative = upper_arm_bone.matrix_local.inverted() @ elbow_bone.matrix_local
    elbow_rest_world = arm.matrix_world @ (pb['DEF_UpperArm'].matrix @ elbow_rest_relative)
    elbow_head_world = elbow_rest_world.translation
    flex_axis_world = (elbow_rest_world.to_3x3() @ Vector((1, 0, 0))).normalized()

    forearm_len = (forearm_bone.head_local - forearm_bone.tail_local).length
    # Same fix, one link further down the chain: the forearm-twist stub's
    # offset from the elbow is fixed in the elbow's OWN rest-relative frame
    # (neither bone's rest geometry changed), so express it there, then
    # reorient using the elbow's CURRENT (now shoulder-aware) frame above,
    # instead of the old flat `arm.matrix_world.to_3x3() @ (...)`.
    forearm_tail_offset_local = (elbow_bone.matrix_local.to_3x3().inverted()
                                  @ (forearm_bone.tail_local - elbow_bone.head_local))
    rest_forearm_dir_world = elbow_rest_world.to_3x3() @ forearm_tail_offset_local
    # project out any component along the rotation axis so the arc basis is clean
    rest_forearm_dir_world = (rest_forearm_dir_world
                               - rest_forearm_dir_world.dot(flex_axis_world) * flex_axis_world).normalized()

    basis_u = rest_forearm_dir_world
    basis_v = flex_axis_world.cross(basis_u).normalized()

    target_world = ik_target.matrix_world.translation
    to_target = target_world - elbow_head_world
    to_target_planar = to_target - to_target.dot(flex_axis_world) * flex_axis_world

    if to_target_planar.length < 1e-6:
        # target sits on the elbow's own rotation axis -- no well-defined direction to aim for
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
    # Bug fix (was: derive the reference from CTRL_Forearm_Twist's own bone roll relative to
    # CTRL_Elbow's). On this rig CTRL_Forearm_Twist's rest matrix is identical to CTRL_Elbow's
    # -- a straight-arm rest pose with no distinct roll ever authored between them -- so that
    # derivation collapsed onto flex_axis_world itself: a vector parallel to the very axis it
    # then got rotated around, making it invariant to elbow_angle and unrelated to the pole.
    # Confirmed live: dot(old reference, flex_axis_world) == 1.0000000028. That produced
    # nonsense raw angles (~161 deg observed at a near-rest pose) which always clamped to the
    # +-50 boundary -- the reported "saturates even at rest" bug.
    # Fix: basis_v is already perpendicular to both flex_axis_world and the rest forearm
    # direction (basis_u), and already computed above for the elbow-plane solve. It is
    # non-degenerate (not parallel to the rotation axis) and geometrically matches the spec's
    # "0 deg = neutral, palm forward" convention -- it's the front-back direction in the
    # elbow's swing plane at rest. Use it as the twist-zero reference instead.
    forearm_ref_axis_world_pretwist = _rodrigues(basis_v, flex_axis_world, elbow_angle)

    twist_axis_world = forearm_dir_current  # forearm's own length direction, post elbow-solve

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
        print(f"arm_ik_solver.solve(): elbow={result['elbow_deg']:.2f} deg "
              f"(clamped={result['elbow_clamped']}), twist={result['twist_deg']:.2f} deg "
              f"(clamped={result['twist_clamped']}), residual_reach_error={residual:.4f}")
    return result
