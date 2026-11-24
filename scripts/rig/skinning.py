"""
skinning.py -- v1.0
Rigid bone-skinning assignment for all 78 mesh objects, per
docs/phase_skinning.md.

Method: rigid parenting via Armature modifier + exactly ONE vertex group
at 100% weight per object -- no weight painting, no blending. This is a
hard-surface segmented robot; every piece is a rigid shell that follows
exactly one bone.

Mapping confirmed by project owner (2026-09) after a resolution pass that
cross-checked finger identity against actual DEF bone positions rather
than trusting the Finger1-4 naming (which turned out reversed -- Finger4
is adjacent to the thumb, corresponding to Index, not Finger1). See the
resolution-pass output in conversation history for the derivation.

Two gaps found and filled while building this mapping (flagged, not
silently folded in):
  - 411_Shoulder_L/R: present in the scene, not in the confirmed mapping
    list, but exactly the case the shoulder-socket precedent (used to
    resolve HipSocket -> DEF_Thigh) was named after. Assigned to
    DEF_UpperArm per that same precedent.
  - 411_N_ServicePort: sits in the neck Z-range but off-center (X=0.153),
    so it was excluded by the neck-region query's |center_x|<0.05 filter
    during the resolution pass. Not covered by any confirmed mapping
    line. Assigned to DEF_Neck, matching the other collar-area pieces --
    it isn't mirrored (no _L/_R pair), so it reads as an asymmetric
    neck/collar detail, not a limb piece.

Known pre-existing issue, per the phase doc -- not fixed here:
411_Thumb_L and 411_Thumb_R use different mesh datablocks and are
genuinely different sizes (~26% width, ~41% height). Both skin correctly
to their respective bones; they'll just be different sizes.

Safe to rerun: clears any pre-existing vertex groups on each object
before creating the single correct one, and reuses an existing Armature
modifier if present rather than duplicating it.
"""

import bpy

ARMATURE_NAME = 'RobotArmature'

MAPPING = {
    # --- Head cluster -> DEF_Head ---
    '411_Head': 'DEF_Head',
    '411_S_CyanPoint': 'DEF_Head',
    '411_S_Exhaust_0': 'DEF_Head',
    '411_S_Exhaust_1': 'DEF_Head',
    '411_S_Exhaust_2': 'DEF_Head',
    '411_S_Mic_0': 'DEF_Head',
    '411_S_Mic_1': 'DEF_Head',
    '411_S_Mic_2': 'DEF_Head',
    '411_S_Mic_3': 'DEF_Head',
    '411_S_PrimaryLens': 'DEF_Head',
    '411_S_PrimaryStrip': 'DEF_Head',
    '411_S_RearCam': 'DEF_Head',
    '411_S_ServiceSeam': 'DEF_Head',
    '411_S_SideCam_0': 'DEF_Head',
    '411_S_SideCam_1': 'DEF_Head',
    '411_S_Vent_0_0': 'DEF_Head',
    '411_S_Vent_0_1': 'DEF_Head',
    '411_S_Vent_0_2': 'DEF_Head',
    '411_S_Vent_1_0': 'DEF_Head',
    '411_S_Vent_1_1': 'DEF_Head',
    '411_S_Vent_1_2': 'DEF_Head',

    # --- Neck -> DEF_Neck ---
    '411_Neck': 'DEF_Neck',
    '411_N_CableGuard': 'DEF_Neck',
    '411_N_JointRing': 'DEF_Neck',
    '411_N_LED_Amber': 'DEF_Neck',
    '411_N_LED_Cyan': 'DEF_Neck',
    '411_N_LED_Red': 'DEF_Neck',
    '411_N_ServicePort': 'DEF_Neck',  # gap-fill, flagged above

    # --- Chest ---
    '411_Torso': 'DEF_Chest',

    # --- Pelvis ---
    '411_Pelvis': 'DEF_Pelvis',
    '411_WaistRing_New': 'DEF_Pelvis',
    '411_WaistRing_Backing': 'DEF_Pelvis',
    '411_PelvicShell': 'DEF_Pelvis',

    # --- Right leg ---
    '411_Thigh_R': 'DEF_Thigh',
    '411_HipSocket_R': 'DEF_Thigh',
    '411_Shin_R': 'DEF_Shin',
    '411_Knee_R': 'DEF_Shin',
    '411_Foot_R': 'DEF_Foot',
    '411_Ankle_R': 'DEF_Foot',

    # --- Left leg ---
    '411_Thigh_L': 'DEF_Thigh_L',
    '411_HipSocket_L': 'DEF_Thigh_L',
    '411_Shin_L': 'DEF_Shin_L',
    '411_Knee_L': 'DEF_Shin_L',
    '411_Foot_L': 'DEF_Foot_L',
    '411_Ankle_L': 'DEF_Foot_L',

    # --- Right arm ---
    '411_UpperArm_R': 'DEF_UpperArm',
    '411_Shoulder_R': 'DEF_UpperArm',  # gap-fill, flagged above
    '411_Elbow_R': 'DEF_Elbow',
    '411_Forearm_R': 'DEF_Forearm_Twist',
    '411_Wrist_R': 'DEF_Wrist',
    '411_Palm_R': 'DEF_Palm_Index',
    # fingers: identity resolved by thumb-adjacency cross-check, NOT by number
    '411_Finger4_R_A': 'DEF_Index_MCP',
    '411_Finger4_R_B': 'DEF_Index_PIP',
    '411_Finger3_R_A': 'DEF_Middle_MCP',
    '411_Finger3_R_B': 'DEF_Middle_PIP',
    '411_Finger2_R_A': 'DEF_Ring_MCP',
    '411_Finger2_R_B': 'DEF_Ring_PIP',
    '411_Finger1_R_A': 'DEF_Little_MCP',
    '411_Finger1_R_B': 'DEF_Little_PIP',
    '411_ThumbJoint_R': 'DEF_Thumb_CMC',
    '411_Thumb_R': 'DEF_Thumb_MCP',

    # --- Left arm ---
    '411_UpperArm_L': 'DEF_UpperArm_L',
    '411_Shoulder_L': 'DEF_UpperArm_L',  # gap-fill, flagged above
    '411_Elbow_L': 'DEF_Elbow_L',
    '411_Forearm_L': 'DEF_Forearm_Twist_L',
    '411_Wrist_L': 'DEF_Wrist_L',
    '411_Palm_L': 'DEF_Palm_Index_L',
    '411_Finger4_L_A': 'DEF_Index_MCP_L',
    '411_Finger4_L_B': 'DEF_Index_PIP_L',
    '411_Finger3_L_A': 'DEF_Middle_MCP_L',
    '411_Finger3_L_B': 'DEF_Middle_PIP_L',
    '411_Finger2_L_A': 'DEF_Ring_MCP_L',
    '411_Finger2_L_B': 'DEF_Ring_PIP_L',
    '411_Finger1_L_A': 'DEF_Little_MCP_L',
    '411_Finger1_L_B': 'DEF_Little_PIP_L',
    '411_ThumbJoint_L': 'DEF_Thumb_CMC_L',
    '411_Thumb_L': 'DEF_Thumb_MCP_L',
}


def skin_all(report=True):
    if ARMATURE_NAME not in bpy.data.objects:
        raise RuntimeError(f"skinning: armature '{ARMATURE_NAME}' not found -- aborting, nothing changed.")
    arm = bpy.data.objects[ARMATURE_NAME]
    db = arm.data.bones

    skinned = []
    skipped_no_object = []
    skipped_no_bone = []

    for obj_name, bone_name in MAPPING.items():
        obj = bpy.data.objects.get(obj_name)
        if obj is None:
            skipped_no_object.append(obj_name)
            continue
        if bone_name not in db:
            skipped_no_bone.append((obj_name, bone_name))
            continue

        # clear any pre-existing vertex groups -- exactly one group, no blending
        for vg in list(obj.vertex_groups):
            obj.vertex_groups.remove(vg)

        vg = obj.vertex_groups.new(name=bone_name)
        vg.add(list(range(len(obj.data.vertices))), 1.0, 'REPLACE')

        arm_mod = None
        for m in obj.modifiers:
            if m.type == 'ARMATURE':
                arm_mod = m
                break
        if arm_mod is None:
            arm_mod = obj.modifiers.new(name='Armature', type='ARMATURE')
        arm_mod.object = arm
        arm_mod.use_vertex_groups = True

        skinned.append((obj_name, bone_name))
        if report:
            print(f"Skinned {obj_name} -> {bone_name}")

    print(f"\n=== Skinning summary ===")
    print(f"Total mapping entries: {len(MAPPING)}")
    print(f"Successfully skinned: {len(skinned)}")
    print(f"Skipped (object not found): {len(skipped_no_object)} {skipped_no_object}")
    print(f"Skipped (bone not found): {len(skipped_no_bone)} {skipped_no_bone}")

    return {
        'skinned': skinned,
        'skipped_no_object': skipped_no_object,
        'skipped_no_bone': skipped_no_bone,
    }
