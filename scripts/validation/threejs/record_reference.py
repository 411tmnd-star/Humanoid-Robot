"""record_reference.py -- SCRIPT_VERSION 1.0

Purpose: Blender-side ground truth for the Three.js viewer test (scripts/validation/threejs/run_viewer_test.mjs).
Headless:  python3 -I scripts/validation/threejs/record_reference.py BLEND OUT.json

Records, for the action assigned in the .blend (hello_sequence), in Blender world coordinates:
  * the head of every DEF bone every 2nd frame (the same data glb_roundtrip_check.py uses)
  * the evaluated, skinned bounding box (modifier stack incl. grooves + Armature) of selected meshes at KEY_FRAMES
Nothing in the .blend is modified on disk.
"""
import json
import sys

import bpy

SCRIPT_VERSION = "1.0"
KEY_FRAMES = (0, 10, 20, 40)
MESHES = ("411_Torso", "411_Head", "411_Palm_R", "411_Palm_L", "411_Index_Knuckle2", "411_IndexL_Knuckle2", "411_Thumb_R",
          "411_Forearm_R", "411_Foot_R", "411_Sole_R", "411_Visor_Glass", "411_Neck",
          "411_Thigh_R", "411_Shin_R", "411_PelvicShell")          # legs and pelvis: needed to see a leg-thickness change
WAIST_BAND = (1.05, 1.17)        # torso vertices in this z band: the waist; whole-mesh boxes are dominated by the unchanged chest


def main(blend, out):
    bpy.ops.wm.open_mainfile(filepath=blend)
    sc = bpy.context.scene
    arm = bpy.data.objects["RobotArmature"]
    act = arm.animation_data.action
    lo, hi = act.frame_range
    ref = {"script_version": SCRIPT_VERSION, "fps": sc.render.fps, "action": act.name, "frame_range": [lo, hi],
           "bones": {}, "meshes": {}}
    names = [b.name for b in arm.data.bones if b.name.startswith("DEF_")]
    # bone heads: geometry modifiers are irrelevant to bone motion and slow, so switch the booleans off for this pass only
    for o in bpy.data.objects:
        for m in o.modifiers:
            if m.name == "V18_Groove":
                m.show_viewport = False
    for f in range(int(lo), int(hi) + 1, 2):
        sc.frame_set(f)
        w = arm.matrix_world
        ref["bones"][str(f)] = {n: list(w @ arm.pose.bones[n].head) for n in names}
    for o in bpy.data.objects:
        for m in o.modifiers:
            if m.name == "V18_Groove":
                m.show_viewport = True
    for f in KEY_FRAMES:
        sc.frame_set(f)
        bpy.context.view_layer.update()
        dg = bpy.context.evaluated_depsgraph_get()
        row = {}
        for n in MESHES:
            ev = bpy.data.objects[n].evaluated_get(dg)
            me = ev.to_mesh()
            pts = [ev.matrix_world @ v.co for v in me.vertices]
            row[n] = {"min": [min(p[i] for p in pts) for i in range(3)], "max": [max(p[i] for p in pts) for i in range(3)], "verts": len(pts)}
            ev.to_mesh_clear()
        ref["meshes"][str(f)] = row
    sc.frame_set(0)
    bpy.context.view_layer.update()
    ev = bpy.data.objects["411_Torso"].evaluated_get(bpy.context.evaluated_depsgraph_get())
    me = ev.to_mesh()
    pts = [p for p in (ev.matrix_world @ v.co for v in me.vertices) if WAIST_BAND[0] <= p.z <= WAIST_BAND[1]]
    ref["waist_band"] = {"mesh": "411_Torso", "z": list(WAIST_BAND), "n": len(pts), "min": [min(p[i] for p in pts) for i in range(3)], "max": [max(p[i] for p in pts) for i in range(3)]}
    ev.to_mesh_clear()
    json.dump(ref, open(out, "w"))
    print("REF", len(ref["bones"]), "frames,", len(MESHES), "meshes x", len(KEY_FRAMES), "frames, fps", ref["fps"])


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.endswith(".py")]
    main(a[0], a[1])
