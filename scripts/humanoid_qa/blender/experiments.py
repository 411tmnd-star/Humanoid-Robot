"""Isolated pose and export experiments (bpy).

SAFETY MODEL
  * open_isolated_copy() copies the source into a temp directory and opens
    the COPY. The source path is never opened for writing and nothing here
    calls save. The copy's hash is re-checked after the experiments to prove
    it was never saved either.
  * Every pose test restores the pose and verifies the evaluated mesh
    returned to its rest snapshot within a stated tolerance.
  * The copy is disposable: the action is unassigned so animation cannot
    overwrite the test pose (recorded in the output).
"""

import math
import os
import re
import shutil
import tempfile

import bpy
from mathutils import Euler

from ..core import evidence, glb


def open_isolated_copy(src):
    tmp = tempfile.mkdtemp(prefix="hqa_exp_")
    dst = os.path.join(tmp, "isolated_copy.blend")
    shutil.copy2(src, dst)
    src_hash, copy_hash = evidence.sha256_file(src), evidence.sha256_file(dst)
    if src_hash != copy_hash:
        raise RuntimeError("isolated copy differs from source")
    bpy.ops.wm.open_mainfile(filepath=dst)
    return {"copy_path": dst, "copy_sha256": copy_hash, "source_sha256": src_hash}


def prepare_for_posing(arm_obj):
    """Detach animation so the test pose is not overwritten. Copy only."""
    notes = []
    ad = arm_obj.animation_data
    if ad and ad.action:
        notes.append(f"unassigned action {ad.action.name} from {arm_obj.name} (isolated copy only)")
        ad.action = None
    bpy.context.scene.frame_set(1)
    arm_obj.data.pose_position = "POSE"
    return notes


def snapshot_eval(objs):
    dg = bpy.context.evaluated_depsgraph_get()
    out = {}
    for o in objs:
        ev = o.evaluated_get(dg)
        me = ev.to_mesh()
        try:
            mw = ev.matrix_world
            out[o.name] = [tuple(mw @ v.co) for v in me.vertices]
        finally:
            ev.to_mesh_clear()
    return out


def displacement(a, b):
    """Per-object max vertex displacement between two snapshots."""
    res = {}
    for name, pa in a.items():
        pb = b[name]
        if len(pa) != len(pb):
            res[name] = None
            continue
        res[name] = max((math.dist(x, y) for x, y in zip(pa, pb)), default=0.0)
    return res


def pose_matrices(arm_obj):
    return {pb.name: [list(r) for r in pb.matrix] for pb in arm_obj.pose.bones}


def moved_bones(before, after, tol=1e-6):
    out = []
    for n, m0 in before.items():
        m1 = after[n]
        if max(abs(m0[i][j] - m1[i][j]) for i in range(4) for j in range(4)) > tol:
            out.append(n)
    return sorted(out)


def pose_test(arm_obj, bone_name, euler_xyz_rad, objs, rest_snap, rest_mats,
              move_threshold=1e-4, restore_tolerance=1e-6):
    pb = arm_obj.pose.bones[bone_name]
    saved = (pb.rotation_mode, pb.rotation_euler.copy(), pb.rotation_quaternion.copy())
    try:
        pb.rotation_mode = "XYZ"
        pb.rotation_euler = Euler(euler_xyz_rad, "XYZ")
        bpy.context.view_layer.update()
        snap = snapshot_eval(objs)
        mats = pose_matrices(arm_obj)
        disp = displacement(rest_snap, snap)
        moved_b = moved_bones(rest_mats, mats)
    finally:
        pb.rotation_mode, pb.rotation_euler, pb.rotation_quaternion = saved[0], saved[1], saved[2]
        bpy.context.view_layer.update()
    after = snapshot_eval(objs)
    restore_err = max((v for v in displacement(rest_snap, after).values() if v is not None), default=0.0)
    return {
        "bone": bone_name, "euler_xyz_deg": [round(math.degrees(a), 6) for a in euler_xyz_rad],
        "moved_bones": moved_b,
        "moved_def_bones": [b for b in moved_b if b.startswith("DEF_")],
        "moved_meshes": {n: d for n, d in sorted(disp.items()) if d is not None and d > move_threshold},
        "vertex_count_changed": sorted(n for n, d in disp.items() if d is None),
        "move_threshold_m": move_threshold,
        "restore_max_error_m": restore_err,
        "restore_verified": restore_err <= restore_tolerance,
    }


FLEX_RE = re.compile(r"^CTRL_.*(Flex|PIP|MCP|_IP|Elbow|Knee|Dorsi)")
EXCLUDE_RE = re.compile(r"(Abd|Rot|Twist|Dev|Inversion|FlexExt)")


def default_test_bones(arm_obj):
    return sorted(pb.name for pb in arm_obj.pose.bones
                  if FLEX_RE.match(pb.name) and not EXCLUDE_RE.search(pb.name))


def run_pose_experiments(arm_obj, body_objs, bone_names, angle_deg=30.0):
    notes = prepare_for_posing(arm_obj)
    bpy.context.view_layer.update()
    rest_snap = snapshot_eval(body_objs)
    rest_mats = pose_matrices(arm_obj)
    results = [pose_test(arm_obj, b, (math.radians(angle_deg), 0.0, 0.0), body_objs, rest_snap, rest_mats)
               for b in bone_names]
    return {"notes": notes, "angle_deg": angle_deg, "axis": "local X (rotation_euler.x)",
            "rest_state": "all pose bones as stored in the copy with the action unassigned",
            "results": results}


def export_probe(arm_obj, body_objs, out_dir):
    """Export the isolated copy twice (all bones / deform bones only) and read
    the joint lists from the resulting GLBs. Failures are recorded, never
    papered over."""
    os.makedirs(out_dir, exist_ok=True)
    notes = []
    # select_set() silently does nothing on objects hidden in the view layer,
    # which would export meshes without their skin. Unhide in the COPY only.
    for o in list(body_objs) + [arm_obj]:
        if o.hide_get():
            o.hide_set(False)
            notes.append(f"unhid {o.name} in the isolated copy so it can be selected for export")
    for o in bpy.data.objects:
        o.select_set(False)
    for o in list(body_objs) + [arm_obj]:
        o.select_set(True)
    bpy.context.view_layer.objects.active = arm_obj
    not_selected = [o.name for o in list(body_objs) + [arm_obj] if not o.select_get()]
    if not_selected:
        return {"error": f"objects could not be selected for export: {not_selected[:5]}", "notes": notes}
    out = {"_notes": notes}
    for label, def_only in (("all_bones", False), ("deform_bones_only", True)):
        path = os.path.join(out_dir, f"probe_{label}.glb")
        rec = {"path": os.path.basename(path), "export_def_bones": def_only}
        try:
            bpy.ops.export_scene.gltf(filepath=path, export_format="GLB", use_selection=True,
                                      export_def_bones=def_only, export_animations=False,
                                      export_apply=False)
            with open(path, "rb") as f:
                data = f.read()
            rec["bytes"] = len(data)
            rec["summary"] = glb.skin_summary(glb.read_glb_json(data))
            for sk in rec["summary"]["skins"]:
                sk["joints"] = sk["joints"][:0] if len(sk["joints"]) > 400 else sk["joints"]
        except Exception as e:  # noqa: BLE001 - recorded as a result
            rec["error"] = f"{type(e).__name__}: {e}"
        out[label] = rec
    return out
