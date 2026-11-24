"""
shoulder_hip_builder_v1.py -- v1.0
Purpose: Build the shoulder and hip ball-socket joints (3R each, "three
serial revolutes" per docs/ENGINEERING_RULES.md Sec7's classification table) and splice
them into the existing DEF_UpperArm/DEF_Thigh chains, per
specifications/arm/shoulder_complex.md and specifications/legs/hip.md.

Design (stated up front, matches the specs): copied directly from
specifications/arm/wrist.md's validated 2-axis pattern, extended to three
axes. A serial MECH chain of single-axis stub bones (each spanning the same
head/tail as the existing DEF_UpperArm/DEF_Thigh bone they sit in front of)
collapses into ONE new constraint on the EXISTING DEF bone -- a Copy
Rotation, POSE space both sides, targeting the chain's terminal MECH bone.
Confirmed live before writing this script: DEF_Wrist's parent is
DEF_Forearm_Twist (unchanged) with exactly one such constraint targeting
MECH_Wrist_Dev. This script does the same thing for DEF_UpperArm(_L) and
DEF_Thigh(_L):
  - Their PARENT does not change.
  - Their REST geometry does not change.
  - Nothing downstream (elbow, wrist, hand, fingers / knee, ankle) is
    touched, re-parented, or re-skinned.
  - Exactly one new constraint is added to each.
This is deliberately the lowest-risk way to splice in a new proximal joint:
no Edit-Mode reparenting of the existing, already-validated arm/leg chains.

Prerequisites (validated live before running):
  - RobotArmature exists.
  - DEF_UpperArm, DEF_UpperArm_L, DEF_Thigh, DEF_Thigh_L all exist, each
    currently parented directly to DEF_Chest / DEF_Pelvis respectively with
    ZERO existing constraints (if any of them already has a constraint,
    this script aborts rather than assuming it's safe to layer under it --
    same philosophy as arm_ik_solver.py's live IK-constraint check).
  - None of the new bone names already exist (idempotency check).

Axis convention (per the specs, matching this rig's project-wide pattern --
X for the primary hinge-like axis, Z for the secondary/deviation axis, Y for
the long-axis twist -- HYPOTHESIS until verify_axes() below confirms it
empirically against THIS live rig, per docs/ENGINEERING_RULES.md Sec8. Do not trust the
ROM signs/directions as correct until verify_axes() has been run and its
report read.

Safe to rerun: aborts with a clear error if any target bone name already
exists rather than duplicating. Call teardown() first to remove a previous
build.
"""

import bpy
import math

ARMATURE_NAME = 'RobotArmature'

# (def_bone, parent_def_bone, ctrl_parent_def_bone, joint_prefix, axes)
# axes: list of (suffix, axis_letter, min_deg, max_deg)
# Empirically found, not assumed (docs/ENGINEERING_RULES.md Sec8): verify_axes() showed
# FlexExt's local-X sign is physically MIRRORED between R/L on both
# shoulder and hip -- +15deg CTRL moved the right side's DEF-bone tail
# forward (-Y) but the left side's tail BACKWARD (+Y), for the same nominal
# axis/ROM. AbdAdd's mirrored sign is expected (abduction is genuinely
# outward-opposite on each side); Rot's handedness was checked separately
# (via a perpendicular reference vector, since twisting a bone around its
# own length axis can't move its own tail) and found CONSISTENT between
# sides, so it needs no correction. Root cause: DEF_UpperArm_L/DEF_Thigh_L's
# own rest roll (authored during the original mirror phase, unrelated to
# this script) doesn't preserve "same positive angle = same physical
# direction" for this specific axis. Fixed here via Copy Rotation's
# built-in axis invert -- NOT by changing the new stub bones' own roll,
# which must stay identical to the target DEF bone's rest frame or the
# POSE-space collapse constraint would snap the rest pose to a different
# orientation the moment it's added.
MIRROR_SIGN_FIX = {('Shoulder_L', 'FlexExt'), ('Hip_L', 'FlexExt')}

JOINT_DEFS = [
    {
        'def_bone': 'DEF_UpperArm', 'parent_def': 'DEF_Chest', 'prefix': 'Shoulder',
        'axes': [('FlexExt', 'X', -40.0, 140.0), ('AbdAdd', 'Z', -20.0, 140.0), ('Rot', 'Y', -60.0, 60.0)],
    },
    {
        'def_bone': 'DEF_UpperArm_L', 'parent_def': 'DEF_Chest', 'prefix': 'Shoulder_L',
        'axes': [('FlexExt', 'X', -40.0, 140.0), ('AbdAdd', 'Z', -20.0, 140.0), ('Rot', 'Y', -60.0, 60.0)],
    },
    {
        'def_bone': 'DEF_Thigh', 'parent_def': 'DEF_Pelvis', 'prefix': 'Hip',
        'axes': [('FlexExt', 'X', -20.0, 115.0), ('AbdAdd', 'Z', -15.0, 40.0), ('Rot', 'Y', -30.0, 30.0)],
    },
    {
        'def_bone': 'DEF_Thigh_L', 'parent_def': 'DEF_Pelvis', 'prefix': 'Hip_L',
        'axes': [('FlexExt', 'X', -20.0, 115.0), ('AbdAdd', 'Z', -15.0, 40.0), ('Rot', 'Y', -30.0, 30.0)],
    },
]


def _mech_name(prefix, suffix):
    return f"MECH_{prefix}_{suffix}"


def _ctrl_name(prefix, suffix):
    return f"CTRL_{prefix}_{suffix}"


def _all_new_bone_names():
    names = []
    for j in JOINT_DEFS:
        for suffix, axis, lo, hi in j['axes']:
            names.append(_mech_name(j['prefix'], suffix))
            names.append(_ctrl_name(j['prefix'], suffix))
    return names


def teardown():
    arm = bpy.data.objects[ARMATURE_NAME]
    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.mode_set(mode='EDIT')
    eb = arm.data.edit_bones
    removed = []
    for name in _all_new_bone_names():
        if name in eb:
            eb.remove(eb[name])
            removed.append(name)
    bpy.ops.object.mode_set(mode='OBJECT')
    # Remove the DEF-bone constraints this script adds, if present.
    pb = arm.pose.bones
    for j in JOINT_DEFS:
        p = pb[j['def_bone']]
        for c in list(p.constraints):
            if c.type == 'COPY_ROTATION' and c.subtarget == _mech_name(j['prefix'], j['axes'][-1][0]):
                p.constraints.remove(c)
    print(f"shoulder_hip_builder_v1.teardown(): removed {len(removed)} bones and any matching DEF constraints.")


def _validate_prerequisites(arm):
    pb = arm.pose.bones
    db = arm.data.bones
    for j in JOINT_DEFS:
        if j['def_bone'] not in db:
            raise RuntimeError(f"shoulder_hip_builder_v1: required bone '{j['def_bone']}' not found -- aborting.")
        b = db[j['def_bone']]
        if b.parent is None or b.parent.name != j['parent_def']:
            raise RuntimeError(
                f"shoulder_hip_builder_v1: expected {j['def_bone']}.parent == {j['parent_def']}, "
                f"got {b.parent.name if b.parent else None} -- aborting, live rig doesn't match the spec's stated insertion point."
            )
        existing_cons = list(pb[j['def_bone']].constraints)
        if existing_cons:
            raise RuntimeError(
                f"shoulder_hip_builder_v1: {j['def_bone']} already has {len(existing_cons)} constraint(s) -- "
                f"aborting rather than assuming it's safe to layer a new one under/over them."
            )
    for name in _all_new_bone_names():
        if name in db:
            raise RuntimeError(
                f"shoulder_hip_builder_v1: '{name}' already exists -- call teardown() first if you want to rebuild."
            )


def build(report=True):
    if ARMATURE_NAME not in bpy.data.objects:
        raise RuntimeError(f"shoulder_hip_builder_v1: armature '{ARMATURE_NAME}' not found -- aborting.")
    arm = bpy.data.objects[ARMATURE_NAME]
    _validate_prerequisites(arm)

    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.mode_set(mode='EDIT')
    eb = arm.data.edit_bones

    created_mech = []
    created_ctrl = []

    for j in JOINT_DEFS:
        source = eb[j['def_bone']]
        head, tail, roll = source.head.copy(), source.tail.copy(), source.roll

        # MECH chain: serial, each parented to the previous (or parent_def for the first).
        prev_mech_parent_name = j['parent_def']
        prev_ctrl_parent_name = j['parent_def']
        for suffix, axis, lo, hi in j['axes']:
            mname = _mech_name(j['prefix'], suffix)
            mbone = eb.new(mname)
            mbone.head, mbone.tail, mbone.roll = head, tail, roll
            mbone.parent = eb[prev_mech_parent_name]
            mbone.use_connect = False
            mbone.use_inherit_rotation = True
            mbone.use_deform = False
            created_mech.append(mname)
            prev_mech_parent_name = mname

            cname = _ctrl_name(j['prefix'], suffix)
            cbone = eb.new(cname)
            cbone.head, cbone.tail, cbone.roll = head, tail, roll
            cbone.parent = eb[prev_ctrl_parent_name]
            cbone.use_connect = False
            cbone.use_inherit_rotation = True
            cbone.use_deform = False
            created_ctrl.append(cname)
            prev_ctrl_parent_name = cname

    bpy.ops.object.mode_set(mode='OBJECT')

    # Pose-mode wiring: Copy Rotation (Local) + Limit Rotation per MECH bone,
    # in that exact order -- Copy Rotation first, Limit Rotation second. This
    # is the precise bug found and fixed on the elbow build: reversing this
    # order makes Limit Rotation a no-op, since Copy Rotation in REPLACE mode
    # overwrites whatever Limit Rotation just clamped.
    pb = arm.pose.bones
    axis_field = {'X': ('use_limit_x', 'min_x', 'max_x'),
                  'Y': ('use_limit_y', 'min_y', 'max_y'),
                  'Z': ('use_limit_z', 'min_z', 'max_z')}

    for j in JOINT_DEFS:
        for suffix, axis, lo, hi in j['axes']:
            mname = _mech_name(j['prefix'], suffix)
            cname = _ctrl_name(j['prefix'], suffix)
            mpb = pb[mname]

            needs_invert = (j['prefix'], suffix) in MIRROR_SIGN_FIX

            copy_rot = mpb.constraints.new('COPY_ROTATION')
            copy_rot.name = 'Copy Rotation'
            copy_rot.target = arm
            copy_rot.subtarget = cname
            copy_rot.target_space = 'LOCAL'
            copy_rot.owner_space = 'LOCAL'
            copy_rot.influence = 1.0
            if needs_invert:
                setattr(copy_rot, f'invert_{axis.lower()}', True)

            limit_rot = mpb.constraints.new('LIMIT_ROTATION')
            limit_rot.name = 'Limit Rotation'
            limit_rot.owner_space = 'LOCAL'
            for ax in ('X', 'Y', 'Z'):
                use_field, min_field, max_field = axis_field[ax]
                setattr(limit_rot, use_field, True)
                if ax == axis:
                    # The Limit Rotation clamps the MECH bone's ACTUAL local
                    # value, which is evaluated AFTER Copy Rotation (with its
                    # invert already applied). So when invert is on, the
                    # physically-real range is the negated/swapped (-hi,-lo),
                    # not the conceptual (lo,hi) an animator thinks in terms of.
                    real_lo, real_hi = (-hi, -lo) if needs_invert else (lo, hi)
                    setattr(limit_rot, min_field, math.radians(real_lo))
                    setattr(limit_rot, max_field, math.radians(real_hi))
                else:
                    setattr(limit_rot, min_field, 0.0)
                    setattr(limit_rot, max_field, 0.0)

        # Collapse onto the existing DEF bone: one new Copy Rotation, POSE
        # space both sides, targeting the chain's terminal MECH bone --
        # exactly DEF_Wrist's confirmed-live pattern.
        terminal_mech = _mech_name(j['prefix'], j['axes'][-1][0])
        def_pb = pb[j['def_bone']]
        collapse = def_pb.constraints.new('COPY_ROTATION')
        collapse.name = 'Copy Rotation'
        collapse.target = arm
        collapse.subtarget = terminal_mech
        collapse.target_space = 'POSE'
        collapse.owner_space = 'POSE'
        collapse.influence = 1.0

    bpy.context.view_layer.update()

    result = {
        'mech_bones_created': created_mech,
        'ctrl_bones_created': created_ctrl,
        'def_bones_wired': [j['def_bone'] for j in JOINT_DEFS],
    }
    if report:
        print("shoulder_hip_builder_v1.build():", result)
    return result


def verify_axes(test_angle_deg=15.0, report=True):
    """Empirically confirm each axis's physical direction, per docs/ENGINEERING_RULES.md Sec8
    -- never trust axis-to-letter assumptions from a spec/diagram. For each
    new MECH bone, rotate it by test_angle_deg on its assigned axis, read the
    resulting world-space shift of the associated DEF bone's tail, then
    reset to zero. Does not leave any test rotation applied."""
    arm = bpy.data.objects[ARMATURE_NAME]
    pb = arm.pose.bones
    db = arm.data.bones
    bpy.context.view_layer.update()

    results = {}
    for j in JOINT_DEFS:
        def_bone = db[j['def_bone']]
        rest_tail_world = arm.matrix_world @ def_bone.tail_local
        for suffix, axis, lo, hi in j['axes']:
            mname = _mech_name(j['prefix'], suffix)
            cname = _ctrl_name(j['prefix'], suffix)
            cpb = pb[cname]
            cpb.rotation_mode = 'XYZ'
            idx = {'X': 0, 'Y': 1, 'Z': 2}[axis]
            euler = [0.0, 0.0, 0.0]
            euler[idx] = math.radians(test_angle_deg)
            cpb.rotation_euler = euler
            bpy.context.view_layer.update()

            # pose_bone.tail is already in armature space with every
            # constraint evaluated; matrix_world converts to world space.
            posed_tail_world = arm.matrix_world @ arm.pose.bones[j['def_bone']].tail
            delta = posed_tail_world - rest_tail_world
            results[f"{j['prefix']}.{suffix}"] = {
                'axis': axis, 'test_deg': test_angle_deg,
                'def_bone_tail_delta_world': [round(v, 4) for v in delta],
            }

            cpb.rotation_euler = (0.0, 0.0, 0.0)
            bpy.context.view_layer.update()

    if report:
        import json
        print("shoulder_hip_builder_v1.verify_axes():", json.dumps(results, indent=2))
    return results
