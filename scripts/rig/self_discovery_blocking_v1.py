"""
self_discovery_blocking_v1.py -- v1.0
Purpose: First-pass FK blocking for the self-discovery sequence's opening
beat (dormant -> awakening twitch -> arm uncoils into the dock light shaft
-> finger flex test -> hip/knee push-off -> full stand + gaze lock),
15 seconds at this scene's actual 30fps (frame 1-450).

Narrative structure carried over from a draft script the user provided;
that draft's bpy calls were rewritten from scratch against this rig's real
bone names -- none of its original names (Spine.001, UpperArm.R, ForeArm.R,
Root, CTRL_Shoulder.R, CTRL_Hip.R/L, Finger.R.0X) exist here, and its
`if not pbone: return` guard meant every keyframe call would have silently
no-op'd rather than erroring. Also corrected: that draft assumed 24fps
(360 frames = "15s"); this scene is 30fps (confirmed live), so 15s is
frame 450, and the internal beat timings (2s/4s/6s/10s/15s) are scaled
accordingly (60/120/180/300/450), not copied as literal frame numbers.

Design decision (blocking pass, stated explicitly): arms/spine/head/fingers
are keyframed via DIRECT CTRL-bone FK rotation -- exact reach targets for
the frame-120 raise aren't chosen yet, and FK gives full, predictable
silhouette control for that beat.

Legs are DIFFERENT, and were reworked after the first version of this file
rendered with visibly floating feet: `knee_ik_solver.py`/`_L` and
`ankle_ik_solver.py`/`_L` are called directly, at each frame that moves the
pelvis/hip, with `IK_Target.leg.*`/`IK_Target.foot.*` left at their existing
fixed world positions. Since those solvers now read the hip's LIVE pose
(this session's IK-rework pass), re-running them after posing the hip for a
given frame naturally computes whatever knee/ankle bend keeps the foot at
that same fixed spot -- the actual fix for foot-planting, not a guessed FK
angle. Solved only at the keyframed hip poses (frames 1, 300, 450), not
every frame -- Blender linearly interpolates the resulting knee/ankle
rotation between them, which will not stay perfectly grounded at every
in-between frame (that's a nonlinear relationship). Acceptable for a
blocking pass; flagged here rather than presented as fully solved. Adding
a few more solved in-between keyframes is the natural next refinement if
the final cut needs tighter grounding through the transition.

Body-height changes (sinking into debris / rising to standing) are
keyframed on DEF_Pelvis's own pose location -- confirmed live: parentless
(the actual skeletal root), zero existing constraints, safe to keyframe
directly. docs/ENGINEERING_RULES.md Sec5 requires animation baked to DEF-only keyframes;
DEF_Pelvis is a DEF bone and this rig's root, so its own location channel
is the correct place for this, not the armature OBJECT's transform (which
sits outside the DEF/MECH/CTRL framework entirely and wouldn't count as a
"DEF-only keyframe" under that rule). An earlier version of this script
used the object transform instead -- corrected here before this file was
ever treated as final.

Prerequisites (validated live before running):
  - RobotArmature exists with the shoulder/hip mechanism already built
    (specifications/arm/shoulder_complex.md, specifications/legs/hip.md).
  - Does NOT require or touch hello_sequence -- creates and uses its own
    new action, 'self_discovery_blocking_v1', so the validated wave/bow
    animation is untouched regardless of how many times this is rerun.

Safe to rerun: if 'self_discovery_blocking_v1' already exists as an action,
aborts rather than re-keying over it -- call teardown() first to delete it
and rebuild from scratch.
"""

import math
from pathlib import Path

import bpy

ARMATURE_NAME = 'RobotArmature'
ACTION_NAME = 'self_discovery_blocking_v1'
SCRIPTS_RIG_DIR = str(Path(__file__).resolve().parent)


def _load_solver(filename):
    """Load a validated solver module by absolute path and return its
    solve() function. No package/relative-import system exists in this
    project: every script here is self-contained, so solvers are loaded from this script's own folder."""
    path = f"{SCRIPTS_RIG_DIR}\\{filename}"
    ns = {}
    with open(path, 'r', encoding='utf-8') as f:
        exec(f.read(), ns)
    return ns['solve']

# Real bones, confirmed live this session -- not assumed from any draft.
CHEST, NECK, HEAD = 'DEF_Chest', 'DEF_Neck', 'DEF_Head'
SHOULDER_AXES = ['FlexExt', 'AbdAdd', 'Rot']
HIP_AXES = ['FlexExt', 'AbdAdd', 'Rot']


def teardown():
    if ACTION_NAME in bpy.data.actions:
        bpy.data.actions.remove(bpy.data.actions[ACTION_NAME])
        print(f"self_discovery_blocking_v1.teardown(): removed action '{ACTION_NAME}'.")
    else:
        print(f"self_discovery_blocking_v1.teardown(): no '{ACTION_NAME}' action found -- nothing to remove.")


def _validate_prerequisites(arm):
    required = [CHEST, NECK, HEAD,
                'CTRL_Shoulder_FlexExt', 'CTRL_Shoulder_L_FlexExt',
                'CTRL_Hip_FlexExt', 'CTRL_Hip_L_FlexExt',
                'CTRL_Elbow', 'CTRL_Elbow_L', 'CTRL_Knee', 'CTRL_Knee_L',
                'CTRL_Ankle_Dorsi', 'CTRL_Ankle_Dorsi_L']
    missing = [n for n in required if n not in arm.pose.bones]
    if missing:
        raise RuntimeError(f"self_discovery_blocking_v1: required bone(s) missing -- {missing}. Aborting.")
    if ACTION_NAME in bpy.data.actions:
        raise RuntimeError(
            f"self_discovery_blocking_v1: action '{ACTION_NAME}' already exists -- "
            f"call teardown() first if you want to rebuild it."
        )


def _key_rot(pb, bone_name, frame, deg_xyz):
    b = pb[bone_name]
    b.rotation_mode = 'XYZ'
    b.rotation_euler = tuple(math.radians(d) for d in deg_xyz)
    b.keyframe_insert(data_path='rotation_euler', frame=frame)


def _key_shoulder(pb, frame, side_suffix, flexext=0.0, abdadd=0.0, rot=0.0):
    prefix = f"CTRL_Shoulder{side_suffix}"
    _key_rot(pb, f"{prefix}_FlexExt", frame, (flexext, 0, 0))
    _key_rot(pb, f"{prefix}_AbdAdd", frame, (0, 0, abdadd))
    _key_rot(pb, f"{prefix}_Rot", frame, (0, rot, 0))


def _key_hip(pb, frame, side_suffix, flexext=0.0, abdadd=0.0, rot=0.0):
    prefix = f"CTRL_Hip{side_suffix}"
    _key_rot(pb, f"{prefix}_FlexExt", frame, (flexext, 0, 0))
    _key_rot(pb, f"{prefix}_AbdAdd", frame, (0, 0, abdadd))
    _key_rot(pb, f"{prefix}_Rot", frame, (0, rot, 0))


def _key_arm_fk(pb, frame, side_suffix, elbow_deg=0.0, twist_deg=0.0):
    _key_rot(pb, f"CTRL_Elbow{side_suffix}", frame, (elbow_deg, 0, 0))
    _key_rot(pb, f"CTRL_Forearm_Twist{side_suffix}", frame, (0, twist_deg, 0))


def _key_leg_grounded(pb, frame, side_suffix, solve_knee, solve_ankle):
    """Re-run the validated knee/ankle solvers for the CURRENT pose (hip
    already set for this frame, IK_Target.leg/foot.* left at their fixed
    world positions) so the foot stays planted, then keyframe the resulting
    CTRL rotations -- the anim_hello.md workflow (move target, solve,
    keyframe the output), applied per-frame instead of once. Ankle solved
    AFTER knee: ankle_ik_solver.py reads DEF_Shin's live pose, which only
    reflects the knee once solve_knee() has actually run this call."""
    solve_knee(report=False)
    solve_ankle(report=False)
    suffix = side_suffix
    pb[f"CTRL_Knee{suffix}"].keyframe_insert(data_path='rotation_euler', frame=frame)
    pb[f"CTRL_Ankle_Dorsi{suffix}"].keyframe_insert(data_path='rotation_euler', frame=frame)
    pb[f"CTRL_Ankle_Inversion{suffix}"].keyframe_insert(data_path='rotation_euler', frame=frame)


def build(report=True):
    if ARMATURE_NAME not in bpy.data.objects:
        raise RuntimeError(f"self_discovery_blocking_v1: armature '{ARMATURE_NAME}' not found -- aborting.")
    arm = bpy.data.objects[ARMATURE_NAME]
    _validate_prerequisites(arm)
    pb = arm.pose.bones

    scene = bpy.context.scene
    scene.frame_start = 1
    scene.frame_end = 450  # 15s at this scene's actual 30fps (confirmed live, not the 24fps a draft assumed)

    action = bpy.data.actions.new(ACTION_NAME)
    if arm.animation_data is None:
        arm.animation_data_create()
    arm.animation_data.action = action

    R, L = '', '_L'  # this rig's convention: right side is the bare name, left side takes the _L suffix

    solve_knee_r = _load_solver('knee_ik_solver.py')
    solve_knee_l = _load_solver('knee_ik_solver_L.py')
    solve_ankle_r = _load_solver('ankle_ik_solver.py')
    solve_ankle_l = _load_solver('ankle_ik_solver_L.py')

    pelvis = pb['DEF_Pelvis']

    # --- Frame 1 (0s): Dormant, slumped, sunk into the debris pile ---
    scene.frame_set(1)
    pelvis.location = (0, 0, -0.15)
    pelvis.keyframe_insert(data_path='location', frame=1)
    _key_rot(pb, CHEST, 1, (18, 0, 4))
    _key_rot(pb, NECK, 1, (0, 0, 6))
    _key_rot(pb, HEAD, 1, (25, 0, 0))
    # hip_flex=10 here, not the originally-intended 25: diagnostic sweep
    # (tested live, both legs, against the fixed IK_Target.leg/foot.*) showed
    # 25 deg clamps the RIGHT knee/ankle to their hard boundary (0deg / -30deg)
    # while the LEFT leg solves cleanly at 25 but clamps at 10 -- neither leg's
    # solver was buggy; the two legs simply aren't perfectly bilaterally
    # symmetric in this hand-built rig, and no single shared hip-flex value
    # clears both sides' comfortable range at this pelvis depth. 10deg is the
    # best compromise found: right leg fully clean (knee 3.6deg, dorsi -25.3deg,
    # both well inside ROM), left leg's knee clamps mildly to a straight 0deg
    # (ankle still clean at 7.6deg) -- a straight-but-not-broken leg, not a
    # wild boundary value. Stated as a real, minor blocking-pass compromise,
    # not hidden.
    for side in (R, L):
        _key_shoulder(pb, 1, side, flexext=10, abdadd=-10)
        _key_hip(pb, 1, side, flexext=10)
        _key_arm_fk(pb, 1, side, elbow_deg=25)
    _key_leg_grounded(pb, 1, R, solve_knee_r, solve_ankle_r)
    _key_leg_grounded(pb, 1, L, solve_knee_l, solve_ankle_l)

    # --- Frame 60 (2s): Awakening twitch -- a small relay-click flinch ---
    scene.frame_set(60)
    pelvis.location = (0, 0, -0.15)
    pelvis.keyframe_insert(data_path='location', frame=60)
    _key_rot(pb, CHEST, 60, (14, 0, 2))
    _key_rot(pb, NECK, 60, (0, 5, -3))
    _key_rot(pb, HEAD, 60, (15, 5, -5))

    # --- Frame 120 (4s): Right arm uncoils, raises into the dock light shaft ---
    scene.frame_set(120)
    _key_shoulder(pb, 120, R, flexext=95, abdadd=15, rot=-10)
    _key_arm_fk(pb, 120, R, elbow_deg=55, twist_deg=-15)
    _key_rot(pb, HEAD, 120, (5, 10, 0))

    # --- Frame 150 (5s): Fingers curl -- testing the joints ---
    scene.frame_set(150)
    for finger_bone, curl_deg in [
        ('CTRL_Index_MCP_Flex', 55), ('CTRL_Index_PIP', 60),
        ('CTRL_Middle_MCP_Flex', 58), ('CTRL_Middle_PIP', 62),
        ('CTRL_Ring_MCP_Flex', 50), ('CTRL_Ring_PIP', 55),
        ('CTRL_Little_MCP_Flex', 45), ('CTRL_Little_PIP', 50),
        ('CTRL_Thumb_MCP', 30), ('CTRL_Thumb_IP', 35),
    ]:
        _key_rot(pb, finger_bone, 150, (curl_deg, 0, 0))

    # --- Frame 180 (6s): Fingers release -- micro-flex test complete ---
    scene.frame_set(180)
    for finger_bone in [
        'CTRL_Index_MCP_Flex', 'CTRL_Index_PIP', 'CTRL_Middle_MCP_Flex', 'CTRL_Middle_PIP',
        'CTRL_Ring_MCP_Flex', 'CTRL_Ring_PIP', 'CTRL_Little_MCP_Flex', 'CTRL_Little_PIP',
        'CTRL_Thumb_MCP', 'CTRL_Thumb_IP',
    ]:
        _key_rot(pb, finger_bone, 180, (5, 0, 0))

    # --- Frame 300 (10s): Weight shift + floor push-off via hips/knees ---
    scene.frame_set(300)
    pelvis.location = (0, 0.03, 0.10)
    pelvis.keyframe_insert(data_path='location', frame=300)
    # hip_flex=-6 here, not the originally-intended -15: same diagnostic
    # sweep, this frame's pelvis offset (0, 0.03, 0.10). -15 clamped the LEFT
    # knee+ankle to their hard boundary; -6 is the best compromise -- left
    # fully clean (knee 0.9deg, dorsi -20.1deg), right's knee clamps mildly to
    # a straight 0deg (ankle clean at 16.7deg). Same real, minor asymmetry as
    # frame 1's note above, mirrored to the other leg this time.
    for side in (R, L):
        _key_hip(pb, 300, side, flexext=-6)
    _key_leg_grounded(pb, 300, R, solve_knee_r, solve_ankle_r)
    _key_leg_grounded(pb, 300, L, solve_knee_l, solve_ankle_l)
    _key_rot(pb, CHEST, 300, (-5, 0, 0))
    _key_shoulder(pb, 300, R, flexext=40, abdadd=10, rot=-10)  # arm lowering from the raised test pose
    _key_arm_fk(pb, 300, R, elbow_deg=30)

    # --- Frame 450 (15s): Full stand, stabilized, gaze locked up at its own hand ---
    scene.frame_set(450)
    pelvis.location = (0, 0, 0.0)
    pelvis.keyframe_insert(data_path='location', frame=450)
    for side in (R, L):
        _key_hip(pb, 450, side, flexext=0)
        _key_shoulder(pb, 450, side, flexext=0, abdadd=0, rot=0)
        _key_arm_fk(pb, 450, side, elbow_deg=0)
    _key_leg_grounded(pb, 450, R, solve_knee_r, solve_ankle_r)
    _key_leg_grounded(pb, 450, L, solve_knee_l, solve_ankle_l)
    _key_rot(pb, CHEST, 450, (0, 0, 0))
    _key_rot(pb, NECK, 450, (0, 0, 0))
    _key_rot(pb, HEAD, 450, (-10, 15, 5))  # looking up/aside at its own raised hand

    scene.frame_set(1)
    bpy.context.view_layer.update()

    fcurve_count = sum(len(cb.fcurves) for layer in action.layers for strip in layer.strips for cb in strip.channelbags)
    result = {'action': ACTION_NAME, 'fcurve_count': fcurve_count, 'frame_range': [1, 450]}
    if report:
        print("self_discovery_blocking_v1.build():", result)
    return result
