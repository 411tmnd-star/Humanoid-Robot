"""export_v18.py -- SCRIPT_VERSION 1.0

Purpose: export robot_armature_v18.blend to blender/export/robot_v18.glb through export_glb_def_only.export_def_only.
Headless:  python3 -I scripts/build/export_v18.py [BLEND] [GLB]

Why a wrapper: v18 puts exact BOOLEAN groove / pocket modifiers on six meshes. The DEF-only exporter bakes the action first and then
samples it again, and both passes re-evaluate the whole scene on every frame, which re-runs every exact boolean (tens of minutes).
The exporter's `prepare` hook runs in memory only (the .blend is reloaded from disk afterwards), so here each grooved mesh is evaluated
ONCE with the Armature modifier off and replaced by the result; every per-frame evaluation afterwards is cheap. The geometry that reaches
the .glb is identical to what `export_apply=True` would have produced from the modifier stack.
"""
import json
import os
import sys

import bpy

SCRIPT_VERSION = "1.0"
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
EXPORTER = os.path.join(ROOT, "scripts", "build", "export_glb_def_only.py")


def bake_grooved_meshes(arm):
    baked = []
    for o in sorted(bpy.data.objects, key=lambda x: x.name):
        if o.type != "MESH" or not o.name.startswith("411_") or not any(m.name == "V18_Groove" for m in o.modifiers):
            continue
        arms = [m for m in o.modifiers if m.type == "ARMATURE"]
        keep = [(m, m.show_viewport) for m in arms]
        for m in arms:
            m.show_viewport = False
        bpy.context.view_layer.update()
        dg = bpy.context.evaluated_depsgraph_get()
        ev = o.evaluated_get(dg)
        new = bpy.data.meshes.new_from_object(ev, preserve_all_data_layers=True, depsgraph=dg)
        new.name = o.data.name + "_exportbake"
        old_count = len(o.data.vertices)
        o.data = new
        for m in list(o.modifiers):
            if m.type != "ARMATURE":
                o.modifiers.remove(m)
        for m, v in keep:
            m.show_viewport = v
        unweighted = sum(1 for v in new.vertices if len(v.groups) == 0)
        if unweighted:
            raise RuntimeError(f"{o.name}: {unweighted} unweighted vertices after the in-memory bake")
        baked.append((o.name, old_count, len(new.vertices)))
    if len(baked) != 6:
        raise RuntimeError(f"expected 6 grooved meshes, baked {len(baked)}: {baked}")
    return baked


def main(blend, glb):
    bpy.ops.wm.open_mainfile(filepath=blend)
    ns = {"__name__": "export_glb_def_only"}
    exec(compile(open(EXPORTER).read(), EXPORTER, "exec"), ns)
    log = []
    summary = ns["export_def_only"](glb, prepare=lambda arm: log.extend(bake_grooved_meshes(arm)))
    summary["baked_in_memory"] = log
    print("EXPORT", json.dumps(summary))


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.endswith(".py")]
    main(a[0] if a else os.path.join(ROOT, "blender", "robot_armature_v18.blend"),
         a[1] if len(a) > 1 else os.path.join(ROOT, "blender", "export", "robot_v18.glb"))
