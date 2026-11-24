"""extract_rig_hierarchy.py -- read the bone hierarchy of a robot .blend and write it as JSON. Read-only. Needs bpy.

  python -I scripts/showcase/extract_rig_hierarchy.py robot_armature_v20.blend proof/rig_hierarchy_v20.json
"""
import hashlib
import json
import math
import os
import sys

import bpy

ARMATURE = "RobotArmature"


def main(blend, out):
    bpy.ops.wm.open_mainfile(filepath=os.path.abspath(blend))
    arm = bpy.data.objects[ARMATURE]
    world = arm.matrix_world  # positions are written in world space: the armature object itself is rotated
    bones = []
    for b in arm.data.bones:
        pb = arm.pose.bones[b.name]
        limits = {}
        for c in pb.constraints:
            if c.type == "LIMIT_ROTATION" and not c.mute:
                for ax in "xyz":
                    lo, hi = math.degrees(getattr(c, f"min_{ax}")), math.degrees(getattr(c, f"max_{ax}"))
                    if abs(hi - lo) > 1e-6:
                        limits[ax] = [round(lo, 2), round(hi, 2)]
        bones.append({
            "name": b.name, "layer": b.name.split("_")[0], "parent": b.parent.name if b.parent else None,
            "head": [round(v, 5) for v in world @ b.head_local], "tail": [round(v, 5) for v in world @ b.tail_local],
            "length_m": round(b.length, 5), "deform": b.use_deform, "limits_deg": limits,
            "constraints": [{"type": c.type, "target": getattr(c, "subtarget", "") or None} for c in pb.constraints],
        })
    layers = {p: sum(x["layer"] == p for x in bones) for p in ("DEF", "MECH", "CTRL")}
    with open(blend, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    doc = {"source_blend": os.path.basename(blend), "source_sha256": digest, "blender": bpy.app.version_string,
           "bone_count": len(bones), "layers": layers, "deform_bones": sum(x["deform"] for x in bones), "bones": bones}
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print("RIG", doc["bone_count"], layers, "deform", doc["deform_bones"])


if __name__ == "__main__":
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    main(args[0], args[1])
