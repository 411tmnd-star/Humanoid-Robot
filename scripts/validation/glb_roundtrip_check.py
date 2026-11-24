"""glb_roundtrip_check.py -- SCRIPT_VERSION 1.0

Purpose: confirm that a DEF-only .glb reproduces the source action after a clean reimport, for ALL DEF bones
(head and tail), and that the check is not vacuous (a minimum number of bones must actually move).

 1) In Blender, saved .blend loaded, active action = the one that was exported (read-only; frame is restored):
        ns = {"__name__": "glb_roundtrip_check"}; exec(compile(open(PATH).read(), PATH, "exec"), ns)
        ns["record_reference"](REF_JSON)   # every 2nd frame over the action range
 2) Headless (exit code 0 = pass, 1 = fail):
        blender -b --factory-startup --python PATH -- check GLB REF_JSON

glTF stores no bone tails, so tails are rebuilt as (bone pose delta) * (rest tail from the source rig).
Frames are mapped by fps (the importer keeps the scene at 24 fps; the source is 30 fps).
"""
import json
import sys

import bpy
from mathutils import Vector

SCRIPT_VERSION = "1.0"
TOL = 1e-4        # metres
MOVE_EPS = 1e-3   # metres: a bone counts as moving if head or tail shifts more than this
MIN_MOVING = 3


def _points(arm, rest_tail):
    w = arm.matrix_world
    out = {}
    for pb in arm.pose.bones:
        if not pb.name.startswith("DEF_"):
            continue
        delta = w @ (pb.matrix @ arm.data.bones[pb.name].matrix_local.inverted()) @ w.inverted()
        out[pb.name] = (w @ pb.head, delta @ Vector(rest_tail[pb.name]))
    return out


def record_reference(ref_json, frames=None, step=2):
    """frames=None samples every `step` frames over the active action's range. Sparse samples can all land
    on rest poses and miss real motion, so keep this dense."""
    arm = bpy.data.objects["RobotArmature"]
    sc = bpy.context.scene
    keep = sc.frame_current
    if frames is None:
        lo, hi = arm.animation_data.action.frame_range
        frames = tuple(range(int(lo), int(hi) + 1, step))
    rest_tail = {b.name: list(arm.matrix_world @ b.tail_local) for b in arm.data.bones if b.name.startswith("DEF_")}
    ref = {"script_version": SCRIPT_VERSION, "fps": sc.render.fps, "rest_tail": rest_tail, "frames": {},
           "action": arm.animation_data.action.name if arm.animation_data and arm.animation_data.action else None}
    for f in frames:
        sc.frame_set(f)
        ref["frames"][str(f)] = {n: [list(h), list(t)] for n, (h, t) in _points(arm, rest_tail).items()}
    sc.frame_set(keep)
    json.dump(ref, open(ref_json, "w"))
    return {"action": ref["action"], "bones": len(rest_tail), "frames": list(frames)}


def check(glb, ref_json):
    ref = json.load(open(ref_json))
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=glb)
    arm = next(o for o in bpy.data.objects if o.type == "ARMATURE")
    sc = bpy.context.scene
    ratio = sc.render.fps / ref["fps"]
    missing = [n for n in ref["rest_tail"] if n not in arm.data.bones]
    extra = [b.name for b in arm.data.bones if not b.name.startswith("DEF_")]
    print(f"IMPORT: {len(arm.data.bones)} bones, missing DEF bones {missing[:5]}, non-DEF bones {extra[:5]} | action {ref['action']}")
    first, moving, worst, ok = None, set(), 0.0, not missing and not extra
    print("source frame | import frame | max head/tail error over all DEF bones")
    for f, row in ref["frames"].items():
        target = int(f) * ratio  # 30 fps source frame -> 24 fps import frame; usually fractional
        sc.frame_set(int(target), subframe=target - int(target))
        got = _points(arm, ref["rest_tail"])
        err = 0.0
        for n, (h, t) in row.items():
            err = max(err, (got[n][0] - Vector(h)).length, (got[n][1] - Vector(t)).length)
            if first is None:
                continue
            if max((Vector(h) - Vector(first[n][0])).length, (Vector(t) - Vector(first[n][1])).length) > MOVE_EPS:
                moving.add(n)
        if first is None:
            first = row
        worst = max(worst, err)
        print(f"{int(f):12d} | {int(f) * ratio:12.2f} | {err:.3e} m")
    ok = ok and worst < TOL and len(moving) >= MIN_MOVING
    print(f"bones that move in the source action: {len(moving)} {sorted(moving)[:6]}")
    print("RESULT:", "PASS" if ok else "FAIL", f"(tolerance {TOL} m, at least {MIN_MOVING} moving bones, max error {worst:.3e} m)")
    return ok


if __name__ == "__main__" and "--" in sys.argv:
    args = sys.argv[sys.argv.index("--") + 1:]
    if len(args) == 3 and args[0] == "check":
        sys.exit(0 if check(args[1], args[2]) else 1)
    raise SystemExit("usage: -- check <glb> <ref.json>")
