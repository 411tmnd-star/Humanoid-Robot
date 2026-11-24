"""thumb_curl_export_test.py -- SCRIPT_VERSION 1.0

Purpose: prove the left thumb curls as the X-mirror of the right thumb AFTER a DEF-only glTF
round trip (docs/ENGINEERING_RULES.md section 5). QA-003 close-out test.

Two modes, one file:

 1) In Blender (fresh call, saved clean .blend loaded):
        ns = {}; exec(compile(open(PATH).read(), PATH, "exec"), ns)
        ns["run_export_side"](EXPORT_SCRIPT, GLB_OUT, REF_JSON, abd_left_sign=-1)
    In memory it keys a 0 -> 30 degree curl at frames 0/10/20/30 on CTRL_Thumb_CMC_Flex, _MCP, _IP (local X)
    and CTRL_Thumb_CMC_Abd (local Z) for both hands, records wrist-pose-removed reference points,
    exports through export_glb_def_only.export_def_only, then reloads the .blend from disk.
    abd_left_sign=-1 is the mirrored (correct) left Abd key; +1 is a deliberate wrong-sign control.

 2) Headless clean reimport (exit code 0 = pass, 1 = fail):
        blender -b --factory-startup --python PATH -- check GLB_OUT REF_JSON

Method: every point is expressed with each wrist's own pose displacement removed
(pose matrix * inverse rest matrix), because the two arm chains are not mirror images at identity pose.
Left points are then compared with the X-reflected right points. glTF stores no bone tails, so the
IP tail is rebuilt as (bone pose delta) * (rest tail from the source rig).
"""
import json
import math
import sys

import bpy
from mathutils import Vector

SCRIPT_VERSION = "1.0"
KEY_FRAMES = (0, 10, 20, 30)
CURL_DEG = 30.0
TOL = 1e-4          # metres, mirror error and import-vs-source error
MIN_MOTION = 0.01   # metres, the curl must actually move the right thumb or the test is vacuous
THUMB = ("DEF_Thumb_CMC", "DEF_Thumb_MCP", "DEF_Thumb_IP")
IP = "DEF_Thumb_IP"
MESHES = {"Thumb": ("411_Thumb_R", "411_Thumb_L"), "ThumbJoint": ("411_ThumbJoint_R", "411_ThumbJoint_L")}
CURL_BONES = (("CTRL_Thumb_CMC_Flex", 0), ("CTRL_Thumb_MCP", 0), ("CTRL_Thumb_IP", 0), ("CTRL_Thumb_CMC_Abd", 2))


def _refl(v):
    return Vector((-v.x, v.y, v.z))


def _delta_world(arm, bone_name):
    pb, b, w = arm.pose.bones[bone_name], arm.data.bones[bone_name], arm.matrix_world
    return w @ (pb.matrix @ b.matrix_local.inverted()) @ w.inverted()


def _find_mesh(name):
    for o in bpy.data.objects:
        if o.type == "MESH" and (o.name == name or o.name.startswith(name + ".")):
            return o
    raise RuntimeError(f"mesh {name} not found")


def _mesh_centre(name):
    eo = _find_mesh(name).evaluated_get(bpy.context.evaluated_depsgraph_get())
    me = eo.to_mesh()
    uniq = {(round(p.x, 6), round(p.y, 6), round(p.z, 6)) for p in (eo.matrix_world @ v.co for v in me.vertices)}
    eo.to_mesh_clear()
    return sum((Vector(u) for u in uniq), Vector()) / len(uniq)


def measure(arm, rest_tail_world):
    """{'R': {key: Vector}, 'L': {...}} with each wrist's pose displacement removed."""
    out = {}
    for side, suf in (("R", ""), ("L", "_L")):
        t_inv = _delta_world(arm, "DEF_Wrist" + suf).inverted()
        pts = {}
        for n in THUMB:
            pts[n + ".head"] = t_inv @ (arm.matrix_world @ arm.pose.bones[n + suf].head)
        pts[IP + ".tail"] = t_inv @ (_delta_world(arm, IP + suf) @ Vector(rest_tail_world[IP + suf]))
        for key, (r_name, l_name) in MESHES.items():
            pts["mesh." + key] = t_inv @ _mesh_centre(r_name if side == "R" else l_name)
        out[side] = pts
    return out


def mirror_error(m):
    return max((m["L"][k] - _refl(m["R"][k])).length for k in m["R"])


def run_export_side(export_script, glb_out, ref_json, abd_left_sign=-1):
    ns = {}
    exec(compile(open(export_script).read(), export_script, "exec"), ns)
    ref = {"script_version": SCRIPT_VERSION, "abd_left_sign": abd_left_sign, "frames": {}, "blender_mirror_error": {}}

    def prepare(arm):
        sc = bpy.context.scene
        arm.animation_data_create()
        arm.animation_data.action = None
        for pb in arm.pose.bones:
            pb.location = (0, 0, 0); pb.rotation_quaternion = (1, 0, 0, 0)
            pb.rotation_euler = (0, 0, 0); pb.scale = (1, 1, 1)
        for f in KEY_FRAMES:
            a = math.radians(CURL_DEG * f / KEY_FRAMES[-1])
            for name, axis in CURL_BONES:
                for suf in ("", "_L"):
                    pb = arm.pose.bones[name + suf]
                    if pb.rotation_mode != "XYZ":
                        raise RuntimeError(f"{pb.name} rotation_mode {pb.rotation_mode}, expected XYZ")
                    sign = 1 if (axis == 0 or suf == "") else abd_left_sign
                    e = [0.0, 0.0, 0.0]; e[axis] = sign * a
                    pb.rotation_euler = e
                    pb.keyframe_insert("rotation_euler", frame=f)
        arm.animation_data.action.name = "thumb_curl_test"
        w = arm.matrix_world
        ref["fps"] = sc.render.fps
        ref["rest_tail_world"] = {IP + s: list(w @ arm.data.bones[IP + s].tail_local) for s in ("", "_L")}
        for f in KEY_FRAMES:
            sc.frame_set(f)
            m = measure(arm, ref["rest_tail_world"])
            ref["frames"][str(f)] = {s: {k: list(v) for k, v in pts.items()} for s, pts in m.items()}
            ref["blender_mirror_error"][str(f)] = mirror_error(m)
        json.dump(ref, open(ref_json, "w"), indent=1)

    summary = ns["export_def_only"](glb_out, prepare=prepare)
    return {"export": summary, "blender_mirror_error_m": ref["blender_mirror_error"], "abd_left_sign": abd_left_sign}


def check(glb, ref_json):
    ref = json.load(open(ref_json))
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=glb)
    arm = next(o for o in bpy.data.objects if o.type == "ARMATURE")
    sc = bpy.context.scene
    ratio = sc.render.fps / ref["fps"]
    print(f"IMPORT: bones {len(arm.data.bones)} (non-DEF: {[b.name for b in arm.data.bones if not b.name.startswith('DEF_')]}) | "
          f"fps {sc.render.fps} | action range {tuple(bpy.data.actions[0].frame_range)} | abd_left_sign {ref['abd_left_sign']}")
    ok, rest = True, None
    print("v16 frame | import frame | mirror err L vs X-reflected R | import vs source err | right thumb motion")
    for f in KEY_FRAMES:
        sc.frame_set(round(f * ratio))
        m = measure(arm, ref["rest_tail_world"])
        mir = mirror_error(m)
        src = max((m[s][k] - Vector(ref["frames"][str(f)][s][k])).length for s in ("R", "L") for k in m[s])
        if rest is None:
            rest = {k: v.copy() for k, v in m["R"].items()}
        motion = max((m["R"][k] - rest[k]).length for k in m["R"])
        print(f"{f:9d} | {round(f * ratio):12d} | {mir:.3e} m | {src:.3e} m | {motion:.4f} m")
        ok = ok and mir < TOL and src < TOL
    ok = ok and motion > MIN_MOTION
    print("RESULT:", "PASS" if ok else "FAIL", f"(tolerance {TOL} m, minimum motion {MIN_MOTION} m)")
    return ok


if __name__ == "__main__" and "--" in sys.argv:
    args = sys.argv[sys.argv.index("--") + 1:]
    if len(args) == 3 and args[0] == "check":
        sys.exit(0 if check(args[1], args[2]) else 1)
    raise SystemExit("usage: -- check <glb> <ref.json>")
