"""
blocking_v1_polish.py -- v1.0
Purpose: Graph-editor easing pass on the 'self_discovery_blocking_v1' action
-- Bezier/auto-clamped interpolation by default, plus a few targeted handle
overrides for specific beats (heavy push-off settle, shoulder-uncoil apex
ease, a sharp awakening-twitch snap).

Rewritten from a draft script that iterated `action.fcurves` directly --
that attribute does not exist on this Blender version's action data model.
Confirmed directly this session (hit the exact AttributeError earlier while
fixing the forearm-twist solver): Blender 5.2's actions store curves under
a layered structure, `action.layers[*].strips[*].channelbags[*].fcurves`,
not a flat `action.fcurves`. This script walks that structure instead.

Prerequisites (validated live before running):
  - RobotArmature exists.
  - Action 'self_discovery_blocking_v1' exists (built by
    self_discovery_blocking_v1.py) -- aborts otherwise rather than
    silently doing nothing.

Safe to rerun: every keyframe's interpolation/handle types are set
unconditionally each call (not toggled/accumulated), so rerunning just
reapplies the same result -- no teardown() needed.
"""

import bpy

ARMATURE_NAME = 'RobotArmature'
ACTION_NAME = 'self_discovery_blocking_v1'


def _iter_fcurves(action):
    for layer in action.layers:
        for strip in layer.strips:
            for channelbag in strip.channelbags:
                for fcurve in channelbag.fcurves:
                    yield fcurve


def apply_polish(report=True):
    if ARMATURE_NAME not in bpy.data.objects:
        raise RuntimeError(f"blocking_v1_polish: armature '{ARMATURE_NAME}' not found -- aborting.")
    if ACTION_NAME not in bpy.data.actions:
        raise RuntimeError(f"blocking_v1_polish: action '{ACTION_NAME}' not found -- aborting.")

    action = bpy.data.actions[ACTION_NAME]
    fcurve_count = 0
    override_count = 0

    for fcurve in _iter_fcurves(action):
        fcurve_count += 1
        data_path = fcurve.data_path

        # Step 1: default every keyframe to Bezier/auto-clamped -- smooth,
        # no manual overshoot risk, matches how the rest of this rig's
        # existing hello_sequence action was authored.
        for kp in fcurve.keyframe_points:
            kp.interpolation = 'BEZIER'
            kp.handle_left_type = 'AUTO_CLAMPED'
            kp.handle_right_type = 'AUTO_CLAMPED'

        # Step 2: targeted overrides for specific beats.
        if 'DEF_Pelvis' in data_path and 'location' in data_path:
            for kp in fcurve.keyframe_points:
                frame = round(kp.co.x)
                if frame == 300:
                    kp.handle_left_type = 'ALIGNED'
                    kp.handle_right_type = 'ALIGNED'
                    override_count += 1
                elif frame == 450:
                    kp.handle_left_type = 'FREE'
                    kp.handle_left = (400.0, kp.co.y)
                    override_count += 1

        if 'CTRL_Shoulder_FlexExt' in data_path and '_L' not in data_path:
            for kp in fcurve.keyframe_points:
                if round(kp.co.x) == 120:
                    kp.handle_left_type = 'FREE'
                    kp.handle_left = (90.0, kp.co.y)
                    override_count += 1

        if 'DEF_Head' in data_path:
            for kp in fcurve.keyframe_points:
                if round(kp.co.x) == 60:
                    kp.handle_left_type = 'VECTOR'
                    kp.handle_right_type = 'AUTO_CLAMPED'
                    override_count += 1

    result = {'fcurve_count': fcurve_count, 'targeted_overrides_applied': override_count}
    if report:
        print("blocking_v1_polish.apply_polish():", result)
    return result
