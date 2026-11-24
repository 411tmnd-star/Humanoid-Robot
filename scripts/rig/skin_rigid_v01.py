"""
skin_rigid_v01.py

Purpose: Rigid-parent all 79 body mesh objects to RobotArmature. Each mesh
object gets exactly one Armature modifier + one vertex group (named after
its target DEF bone) at weight 1.0 on all vertices. No gradient weight
painting -- this is a hard-surface segmented robot, every piece is a rigid
shell driven by exactly one bone.

Mapping source: docs/phase_skinning.md resolution pass, confirmed live
against robot_armature_v10.blend (post arm-bone-placement fix), plus
user-confirmed answers for the ambiguous cases (neck ServicePort, Yoke,
Visor_Patch_backup, pelvis extras, palm-to-wrist binding, side-dependent
Finger1-4 -> Index/Middle/Ring/Little numbering).

Safe to rerun: if a mesh object already has an Armature modifier targeting
RobotArmature, it is reused rather than duplicated; if the required vertex
group already exists, its weights are reassigned rather than duplicated.
"""

import bpy

ARMATURE_NAME = "RobotArmature"

MAPPING = {}

# Head cluster: computed by Z cutoff at build time, not hardcoded here --
# see build_mapping() below. Everything else is explicit.

FINGER_MAP = {
    "R": {4: "Index", 3: "Middle", 2: "Ring", 1: "Little"},
    "L": {1: "Index", 2: "Middle", 3: "Ring", 4: "Little"},
}


def build_mapping():
    mapping = {}

    for o in bpy.data.objects:
        if o.type != "MESH":
            continue
        verts = [o.matrix_world @ v.co for v in o.data.vertices]
        if not verts:
            continue
        cz = (min(v.z for v in verts) + max(v.z for v in verts)) / 2
        if cz > 1.8:
            mapping[o.name] = "DEF_Head"
    mapping["411_Visor_Patch_backup"] = "DEF_Head"

    for n in [
        "411_Neck", "411_N_JointRing", "411_N_CableGuard", "411_N_LED_Amber",
        "411_N_LED_Red", "411_N_LED_Cyan", "411_N_ServicePort",
    ]:
        mapping[n] = "DEF_Neck"

    mapping["411_Torso"] = "DEF_Chest"
    mapping["411_Yoke"] = "DEF_Chest"

    for n in [
        "411_Pelvis", "411_WaistRing_New", "411_WaistRing_Backing",
        "411_PelvicShell", "411_HipSocket_L", "411_HipSocket_R",
    ]:
        mapping[n] = "DEF_Pelvis"

    for side in ["R", "L"]:
        suf = "" if side == "R" else "_L"
        mapping[f"411_Shoulder_{side}"] = f"DEF_UpperArm{suf}"
        mapping[f"411_UpperArm_{side}"] = f"DEF_UpperArm{suf}"
        mapping[f"411_Elbow_{side}"] = f"DEF_Elbow{suf}"
        mapping[f"411_Forearm_{side}"] = f"DEF_Forearm_Twist{suf}"
        mapping[f"411_Wrist_{side}"] = f"DEF_Wrist{suf}"
        mapping[f"411_Palm_{side}"] = f"DEF_Wrist{suf}"
        mapping[f"411_ThumbJoint_{side}"] = f"DEF_Thumb_CMC{suf}"
        mapping[f"411_Thumb_{side}"] = f"DEF_Thumb_MCP{suf}"
        for num, fname in FINGER_MAP[side].items():
            mapping[f"411_Finger{num}_{side}_A"] = f"DEF_{fname}_MCP{suf}"
            mapping[f"411_Finger{num}_{side}_B"] = f"DEF_{fname}_PIP{suf}"

    for side in ["R", "L"]:
        suf = "" if side == "R" else "_L"
        mapping[f"411_Thigh_{side}"] = f"DEF_Thigh{suf}"
        mapping[f"411_Knee_{side}"] = f"DEF_Shin{suf}"
        mapping[f"411_Shin_{side}"] = f"DEF_Shin{suf}"
        mapping[f"411_Ankle_{side}"] = f"DEF_Foot{suf}"
        mapping[f"411_Foot_{side}"] = f"DEF_Foot{suf}"

    return mapping


def skin_object(obj, bone_name, armature_obj):
    # Armature modifier: reuse if present, else create
    mod = None
    for m in obj.modifiers:
        if m.type == "ARMATURE" and m.object == armature_obj:
            mod = m
            break
    if mod is None:
        mod = obj.modifiers.new(name="Armature", type="ARMATURE")
        mod.object = armature_obj

    # Remove any other vertex groups -- rigid single-bone binding only
    for vg in list(obj.vertex_groups):
        if vg.name != bone_name:
            obj.vertex_groups.remove(vg)

    vg = obj.vertex_groups.get(bone_name)
    if vg is None:
        vg = obj.vertex_groups.new(name=bone_name)

    all_idx = [v.index for v in obj.data.vertices]
    vg.add(all_idx, 1.0, "REPLACE")


def main():
    armature_obj = bpy.data.objects.get(ARMATURE_NAME)
    if armature_obj is None:
        print(f"FAIL: armature '{ARMATURE_NAME}' not found")
        return

    mapping = build_mapping()
    skinned = 0
    skipped = []

    for obj_name, bone_name in sorted(mapping.items()):
        obj = bpy.data.objects.get(obj_name)
        if obj is None:
            skipped.append((obj_name, "object not found"))
            print(f"SKIP {obj_name}: object not found")
            continue
        if bone_name not in armature_obj.data.bones:
            skipped.append((obj_name, f"bone {bone_name} not found"))
            print(f"SKIP {obj_name}: bone {bone_name} not found")
            continue
        skin_object(obj, bone_name, armature_obj)
        skinned += 1
        print(f"Skinned {obj_name} -> {bone_name}")

    all_meshes = set(o.name for o in bpy.data.objects if o.type == "MESH")
    unmapped = sorted(all_meshes - set(mapping.keys()))
    for name in unmapped:
        skipped.append((name, "not in mapping table"))
        print(f"SKIP {name}: not in mapping table")

    print("--- SUMMARY ---")
    print(f"total mesh objects: {len(all_meshes)}")
    print(f"skinned: {skinned}")
    print(f"skipped: {len(skipped)}")
    for name, reason in skipped:
        print(f"  {name}: {reason}")


if __name__ == "__main__":
    main()
