"""export_glb_def_only.py -- SCRIPT_VERSION 1.0

Purpose: export the 411 robot (visible 411_* meshes + RobotArmature) to a DEF-only .glb
(docs/ENGINEERING_RULES.md section 5: the runtime sees DEF bones and baked DEF keyframes only).

What it does, in order, against the live scene:
  1. Requires a saved, unmodified .blend, the armature, and every exported mesh bound to it.
  2. Refuses to run if any non-DEF bone is still flagged use_deform (that leaks CTRL/MECH into the skin).
  3. In memory only: unhides the armature, asserts it is really selected, bakes the active action to
     DEF-only keys (clearing the DEF constraints, which would otherwise flatten the animation at export),
     exports with deform bones only.
  4. Parses the resulting .glb and raises (and deletes the file) if any skin joint or animated node
     is not a DEF bone, or if a mesh lost its skin.
  5. Always reloads the .blend from disk afterwards, so the working file is never modified.

Run inside Blender (MCP execute_blender_code or the Text Editor):
    ns = {}; exec(compile(open(PATH).read(), PATH, "exec"), ns)
    print(ns["export_def_only"](r"<output>.glb"))
"""
import json
import os
import struct

import bpy

SCRIPT_VERSION = "1.1"
CLIP_NAME = "hello_wave"  # glTF animation name required by my-portfolio-next HERO-3D-CONTRACT.md
ARMATURE = "RobotArmature"
MESH_PREFIX = "411_"
EXCLUDE_MESHES = ("411_Visor_Patch_backup",)  # hidden backup, not part of the deliverable


def read_glb_json(path):
    with open(path, "rb") as f:
        data = f.read()
    magic, _version, _length = struct.unpack("<4sII", data[:12])
    if magic != b"glTF":
        raise RuntimeError(f"{path} is not a GLB")
    off = 12
    while off < len(data):
        chunk_len, chunk_type = struct.unpack("<I4s", data[off:off + 8])
        if chunk_type == b"JSON":
            return json.loads(data[off + 8:off + 8 + chunk_len])
        off += 8 + chunk_len
    raise RuntimeError("GLB has no JSON chunk")


def validate_glb(path, expected_meshes):
    """Raise if the export is not DEF-only or lost skinning. Returns a summary dict."""
    js = read_glb_json(path)
    nodes = js["nodes"]
    skins, anims = js.get("skins", []), js.get("animations", [])
    problems = []
    if len(skins) != 1:
        problems.append(f"expected 1 skin, found {len(skins)}")
    joints = [nodes[j]["name"] for s in skins for j in s["joints"]]
    bad_joints = [n for n in joints if not n.startswith("DEF_")]
    if bad_joints:
        problems.append(f"{len(bad_joints)} non-DEF skin joints, e.g. {bad_joints[:5]}")
    skinned = sum(1 for n in nodes if "mesh" in n and "skin" in n)
    if skinned != expected_meshes:
        problems.append(f"{skinned} skinned mesh nodes, expected {expected_meshes}")
    if len(anims) != 1:
        problems.append(f"expected 1 animation, found {len(anims)}")
    animated = sorted({nodes[c["target"]["node"]]["name"] for a in anims for c in a["channels"]})
    bad_anim = [n for n in animated if not n.startswith("DEF_")]
    if bad_anim:
        problems.append(f"{len(bad_anim)} non-DEF animated nodes, e.g. {bad_anim[:5]}")
    if not animated:
        problems.append("animation has no channels")
    if problems:
        raise RuntimeError("GLB validation failed: " + "; ".join(problems))
    return {"skin_joints": len(joints), "skinned_meshes": skinned,
            "animation_channels": sum(len(a["channels"]) for a in anims), "animated_nodes": len(animated)}


def _bake_def_only(arm, src_action):
    view_layer = bpy.context.view_layer
    for o in bpy.data.objects:
        o.select_set(False)
    arm.select_set(True)
    view_layer.objects.active = arm
    bpy.ops.object.mode_set(mode="POSE")
    for pb in arm.pose.bones:
        pb.select = pb.name.startswith("DEF_")
    f0, f1 = int(src_action.frame_range[0]), int(src_action.frame_range[1])
    # clear_constraints=True (in memory only): the glTF exporter re-samples with constraints active, so DEF
    # copy-rotation constraints would otherwise overwrite the baked keys with stale MECH/CTRL values
    # and flatten every animation to one pose.
    bpy.ops.nla.bake(frame_start=f0, frame_end=f1, step=1, only_selected=True, visual_keying=True,
                     clear_constraints=True, clear_parents=False, use_current_action=False,
                     bake_types={"POSE"})
    bpy.ops.object.mode_set(mode="OBJECT")
    baked = arm.animation_data.action
    if baked is None or baked == src_action:
        raise RuntimeError("bake did not produce a new action")
    baked.name = f"{src_action.name}_DEFbake"
    curves = [fc for layer in baked.layers for strip in layer.strips for cb in strip.channelbags for fc in cb.fcurves]
    keyed = {fc.data_path.split('"')[1] for fc in curves if fc.data_path.startswith("pose.bones")}
    stray = sorted(b for b in keyed if not b.startswith("DEF_"))
    if not curves or stray:
        raise RuntimeError(f"baked action invalid: {len(curves)} fcurves, non-DEF bones keyed: {stray[:5]}")
    return baked, len(curves), len(keyed)


def export_def_only(out_path, prepare=None, reload=True):
    """Export DEF-only .glb to out_path. `prepare(arm)` may modify the scene in memory first
    (e.g. author a test action); it is discarded by the final reload."""
    src = bpy.data.filepath
    if not src:
        raise RuntimeError("save the .blend first")
    if bpy.data.is_dirty:
        raise RuntimeError("the .blend has unsaved changes; save or discard them first")
    out_path = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    try:
        arm = bpy.data.objects.get(ARMATURE)
        if arm is None or arm.type != "ARMATURE":
            raise RuntimeError(f"armature '{ARMATURE}' not found")
        meshes = sorted(o.name for o in bpy.data.objects
                        if o.type == "MESH" and o.name.startswith(MESH_PREFIX) and o.name not in EXCLUDE_MESHES)
        if not meshes:
            raise RuntimeError("no meshes to export")
        unbound = [n for n in meshes
                   if not any(m.type == "ARMATURE" and m.object == arm for m in bpy.data.objects[n].modifiers)]
        if unbound:
            raise RuntimeError(f"meshes not bound to the armature: {unbound[:5]}")
        leaking = sorted(b.name for b in arm.data.bones if not b.name.startswith("DEF_") and b.use_deform)
        if leaking:
            raise RuntimeError(f"{len(leaking)} non-DEF bones still have use_deform=True, e.g. {leaking[:5]}")

        arm.hide_set(False)  # a hidden object silently ignores select_set() and is then not exported
        if not arm.visible_get():
            raise RuntimeError("armature is still not visible (check its collection)")
        if prepare is not None:
            prepare(arm)
        if arm.animation_data is None or arm.animation_data.action is None:
            raise RuntimeError("armature has no active action to export")
        src_action = arm.animation_data.action
        _baked, n_curves, n_bones = _bake_def_only(arm, src_action)

        for o in bpy.data.objects:
            o.select_set(False)
        for n in meshes + [ARMATURE]:
            bpy.data.objects[n].select_set(True)
        bpy.context.view_layer.objects.active = arm
        unselected = [n for n in meshes + [ARMATURE] if not bpy.data.objects[n].select_get()]
        if unselected:
            raise RuntimeError(f"could not select for export: {unselected[:5]}")

        # export_apply=True: BEVEL/SUBSURF on six body meshes must be baked into the glTF geometry
        bpy.ops.export_scene.gltf(
            filepath=out_path, export_format="GLB", use_selection=True, export_yup=True, export_apply=True,
            export_skins=True, export_def_bones=True, export_animations=True,
            export_animation_mode="ACTIVE_ACTIONS", export_nla_strips_merged_animation_name=CLIP_NAME,
            export_force_sampling=True, export_normals=True, export_cameras=False, export_lights=False)
        if not os.path.exists(out_path):
            raise RuntimeError("exporter produced no file")
        try:
            summary = validate_glb(out_path, expected_meshes=len(meshes))
        except RuntimeError:
            os.remove(out_path)
            raise
        clip = read_glb_json(out_path)["animations"][0].get("name")
        if clip != CLIP_NAME:
            os.remove(out_path)
            raise RuntimeError(f"exported clip is named '{clip}', expected '{CLIP_NAME}'")
        summary.update(path=out_path, bytes=os.path.getsize(out_path), source_action=src_action.name,
                       clip_name=clip,
                       baked_fcurves=n_curves, baked_bones=n_bones, script_version=SCRIPT_VERSION)
        return summary
    finally:
        if reload:
            bpy.ops.wm.open_mainfile(filepath=src, load_ui=False)
            if bpy.data.is_dirty:
                raise RuntimeError("reload left the file dirty")
