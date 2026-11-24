"""build_v19.py -- SCRIPT_VERSION 1.0

Purpose: derive blender/robot_armature_v19.blend from robot_armature_v18.blend (v18 is never modified).
Headless:  python3 -I scripts/build/build_v19.py [SRC.blend] [DST.blend]

Hand-only refinement (feedback: knuckles read as oversized black discs):
  * the 28 finger / thumb knuckle cylinders: radius x KNUCKLE_SCALE (diameter -37.5 %), length 29 -> 26 mm, 24 sides
  * the 2 thumb CMC spheres: scaled about their centre by KNUCKLE_SCALE (Subdivision modifier kept)
  * the 28 white phalanx shells: radius x SHELL_SCALE (-15 %), same neutral curl and palm-inward pose
Bones, constraints, limits, frames, torso / pelvis / feet / studio are untouched. Geometry is regenerated on the existing objects
(names, vertex groups, modifiers, materials kept); every vertex is re-added to its single DEF group.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import bpy
from mathutils import Vector

import v18_hand_mesh as HM

SCRIPT_VERSION = "1.0"
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SRC_DEFAULT = os.path.join(ROOT, "blender", "robot_armature_v18.blend")
DST_DEFAULT = os.path.join(ROOT, "blender", "robot_armature_v19.blend")

KNUCKLE_SCALE = 0.625          # diameter -37.5 % (brief: 35-40 %)
SHELL_SCALE = 0.85             # white shells slimmed 15 %
W_KNUCKLE = 0.026              # axial length (was 0.029): still a hinge pin slightly proud of the 23 mm shell
SEGMENTS = 24
FINGERS = ("Index", "Middle", "Ring", "Little")


def require(c, m):
    if not c:
        raise RuntimeError("v19 build: " + m)


def main(src, dst):
    require(os.path.exists(src), f"source not found: {src}")
    bpy.ops.wm.open_mainfile(filepath=src)
    sc = bpy.context.scene
    require(sc.get("v18_build_version") and not sc.get("v19_build_version"), "source must be a v18 file")
    arm = bpy.data.objects["RobotArmature"]
    require(arm.data.pose_position == "POSE", "armature must be in POSE mode")
    white = bpy.data.materials["BotWhite"]
    graphite = bpy.data.materials["Graphite_Brushed"]

    by = {"shell": {}, "knuckle": {}}
    for o in bpy.data.objects:
        if o.type == "MESH" and o.name.startswith("411_") and o.vertex_groups:
            g = o.vertex_groups[0].name
            mat = o.material_slots[0].material.name if o.material_slots and o.material_slots[0].material else None
            if "Knuckle" in o.name and mat == "Graphite_Brushed":
                by["knuckle"].setdefault(g, []).append(o)
            elif mat == "BotWhite" and (o.name.startswith(("411_Finger", "411_Thumb_")) or "Phalanx" in o.name):
                by["shell"].setdefault(g, []).append(o)

    old = {}
    done = 0
    for s in ("", "_L"):
        side = "L" if s else "R"
        chains = [(f, [f"DEF_{f}_MCP{s}", f"DEF_{f}_PIP{s}", f"DEF_{f}_Tip{s}"], 0.92 if f == "Little" else 1.0) for f in FINGERS]
        chains.append(("Thumb", [f"DEF_Thumb_MCP{s}", f"DEF_Thumb_IP{s}"], 1.08))
        for label, bones, scale in chains:
            for i, bn in enumerate(bones):
                shells, knuckles = by["shell"].get(bn, []), by["knuckle"].get(bn, [])
                require(len(shells) == 1 and len(knuckles) == 1, f"{bn}: expected 1 shell + 1 knuckle, found {len(shells)} + {len(knuckles)}")
                head, tail, x, y = HM.bone_axes(arm, bn)
                last = i == len(bones) - 1
                r = HM.R_PHAL[min(i, 2)] * scale * SHELL_SCALE
                gap = HM.GAP * scale
                a = head + y * gap
                b = tail - (Vector((0, 0, 0)) if last else y * gap)
                sh, kn = shells[0], knuckles[0]
                old[kn.name] = max(sh_ for sh_ in kn.dimensions)
                HM.cylinder(sh, (a + b) / 2, y, x, r * (1.08 if i == 0 else 1.04), r * (0.92 if last else 0.97), (b - a).length)
                HM._finish(sh, arm, bn, white, 0.0035 if not last else 0.006)
                rk = HM.R_KNUCKLE * scale * KNUCKLE_SCALE
                HM.cylinder(kn, head, x, y, rk, rk, W_KNUCKLE * scale, segs=SEGMENTS)
                HM._finish(kn, arm, bn, graphite, 0.0015)
                done += 2
        # thumb CMC sphere: scale about its own centre (Subdivision modifier is kept)
        tj = bpy.data.objects[f"411_ThumbJoint_{side}"]
        M = tj.matrix_world
        Mi = M.inverted()
        W = [M @ v.co for v in tj.data.vertices]
        c = sum(W, Vector()) / len(W)
        for v, p in zip(tj.data.vertices, W):
            v.co = Mi @ (c + (p - c) * KNUCKLE_SCALE)
        tj.data.update()
        require(any(m.type == "SUBSURF" for m in tj.modifiers), f"{tj.name}: Subdivision modifier missing")
        done += 1
    require(done == 28 * 2 + 2, f"unexpected object count {done}")
    sc["v19_build_version"] = SCRIPT_VERSION
    bpy.ops.wm.save_as_mainfile(filepath=dst)
    print("rebuilt hand objects:", done, "knuckle radius (R scale 1):", HM.R_KNUCKLE * KNUCKLE_SCALE, "shell radii:", [round(v * SHELL_SCALE, 5) for v in HM.R_PHAL])
    print("saved", dst)


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if x.endswith(".blend")]
    main(a[0] if a else SRC_DEFAULT, a[1] if len(a) > 1 else DST_DEFAULT)
