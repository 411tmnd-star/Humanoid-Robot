"""
humanoid_inspect_v01.py  (script version: humanoid_inspect_v01.0)

Purpose: READ-ONLY baseline inspection of a 411 Humanoid .blend. Measures the
scene, armature (bones, hierarchy, rolls, deform flags), body meshes (geometry,
topology, symmetry, scale), vertex-group weights, modifiers, shape keys,
constraints, drivers, actions and collections, then writes structured JSON +
a markdown summary, a manifest with SHA-256 hashes, and an evidence-ledger
line. Measurements are kept separate from interpretation: every issue record
carries a "measured" block and an "interpretation" string.

No spec file named "mesh.humanoid.inspect_body" exists in this repository
(searched docs/, specifications/, scripts/ and blender/ at implementation
time). This script is the first implementation of that intent.

Run headless (never open the GUI session for this):

  blender --background --factory-startup --disable-autoexec ^
      <file.blend> --python-exit-code 1 ^
      --python scripts/validation/humanoid_inspect_v01.py -- ^
      --out-dir blender/inspection/<run_id> --run-id <run_id> ^
      --ledger blender/inspection/evidence_ledger.jsonl

Read-only guarantees:
  - never calls save_mainfile / save_as_mainfile / any bpy.ops write;
  - never adds, deletes, renames, reparents bones; never edits weights,
    modifiers, shape keys, transforms, rest/pose state or topology;
  - bmesh is used only on temporary copies (bm.from_mesh, then bm.free());
  - SHA-256 of the source .blend is taken before and after; any change fails
    loudly (exit code 1).

Safe to rerun: refuses to write into an existing run directory (use a new
--run-id), so no prior evidence is overwritten.
"""

import argparse
import bmesh
import bpy
import datetime
import hashlib
import json
import os
import re
import sys
from collections import Counter, defaultdict

from mathutils import Vector
from mathutils.kdtree import KDTree

SCRIPT_VERSION = "humanoid_inspect_v01.0"
LAYER_RE = re.compile(r"^(DEF|MECH|CTRL)_[A-Za-z0-9]+(_[A-Za-z0-9]+)*$")
SUFFIX_DUP_RE = re.compile(r"\.\d{3}$")
SYM_TOL = 1e-3           # Blender units; scale is verified separately
WEIGHT_SUM_TOL = 1e-3
GLTF_MAX_JOINTS_PER_VERTEX = 4   # glTF 2.0: JOINTS_n/WEIGHTS_n hold 4 per set
BODY_PREFIX = "411_"             # body meshes; everything else is environment/set dressing
SPINE_MIDLINE = ("DEF_Pelvis", "DEF_Chest", "DEF_Neck", "DEF_Head")  # unsided midline bones


def is_body(name):
    return name.startswith(BODY_PREFIX)


def scope_of(name):
    return "body" if is_body(name) else "environment"


SIDE_INFIX_RE = {"L": re.compile(r"(^|_)L(_|$)"), "R": re.compile(r"(^|_)R(_|$)")}


def side_of(name):
    """Classify the side marker: suffix (_L / _R), infix (_L_ / _R_ mid-name),
    dot form (.L / .R, the docs/ENGINEERING_RULES.md §6 wording), or unsided."""
    if name.endswith("_L"):
        return "suffix_L"
    if name.endswith("_R"):
        return "suffix_R"
    if name.endswith(".L") or name.endswith(".R"):
        return "dot_LR"
    if SIDE_INFIX_RE["L"].search(name):
        return "infix_L"
    if SIDE_INFIX_RE["R"].search(name):
        return "infix_R"
    return "unsided"


def side_position(p, midline_x):
    if p.x > midline_x + SYM_TOL:
        return "+X"
    if p.x < midline_x - SYM_TOL:
        return "-X"
    return "midline"


# ---------------------------------------------------------------- helpers

def fail(msg):
    print("HUMANOID_INSPECT FAILED:", msg)
    raise SystemExit(1)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def rnd(x):
    """Round floats recursively; convert mathutils types to plain lists."""
    if isinstance(x, bool) or x is None or isinstance(x, (int, str)):
        return x
    if isinstance(x, float):
        return round(x, 6)
    if isinstance(x, dict):
        return {str(k): rnd(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [rnd(v) for v in x]
    try:
        return [rnd(v) for v in x]
    except TypeError:
        return str(x)


def id_name(v):
    return v.name if v is not None and hasattr(v, "name") else None


def simple_props(obj, skip=()):
    """Generic read of simple RNA properties (bool/int/float/str/enum/ID)."""
    out = {}
    for p in obj.bl_rna.properties:
        ident = p.identifier
        if ident in skip or ident in ("rna_type", "name", "type") or p.is_readonly and p.type == "POINTER":
            continue
        if p.type in ("BOOLEAN", "INT", "FLOAT", "STRING", "ENUM"):
            if getattr(p, "is_array", False) or (p.type != "STRING" and getattr(p, "array_length", 0)):
                try:
                    out[ident] = rnd(list(getattr(obj, ident)))
                except TypeError:
                    pass
                continue
            out[ident] = rnd(getattr(obj, ident))
        elif p.type == "POINTER":
            val = getattr(obj, ident, None)
            if isinstance(val, bpy.types.ID) or val is None:
                out[ident] = id_name(val)
    return out


def prefix_of(name):
    return name.split("_", 1)[0] if "_" in name else None


# ---------------------------------------------------------------- prereqs

def check_prerequisites(args):
    if not bpy.data.filepath:
        fail("no source .blend loaded (bpy.data.filepath is empty)")
    arms = [o for o in bpy.data.objects if o.type == "ARMATURE"]
    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    if not arms:
        fail("source contains no ARMATURE object; nothing to inspect for bones")
    run_dir = args.out_dir
    if os.path.exists(run_dir) and os.listdir(run_dir):
        fail(f"run directory already exists and is non-empty: {run_dir} (choose a new --run-id)")
    print(f"prereq ok: {len(arms)} armature(s), {len(meshes)} mesh object(s)")


# ---------------------------------------------------------------- scene

def inspect_scene():
    s = bpy.context.scene
    us = s.unit_settings
    objs = []
    for o in sorted(bpy.data.objects, key=lambda x: x.name):
        objs.append({
            "name": o.name,
            "type": o.type,
            "data": id_name(o.data),
            "parent": id_name(o.parent),
            "parent_type": o.parent_type,
            "collections": sorted(c.name for c in o.users_collection),
            "hide_viewport": o.hide_viewport,
            "hide_render": o.hide_render,
            "location": rnd(o.location),
            "rotation_euler": rnd(o.rotation_euler),
            "scale": rnd(o.scale),
            "delta_scale": rnd(o.delta_scale),
            "dimensions": rnd(o.dimensions),
        })
    return {
        "scene": s.name,
        "scene_count": len(bpy.data.scenes),
        "unit_system": us.system,
        "unit_length": us.length_unit,
        "unit_scale_length": rnd(us.scale_length),
        "frame_range": [s.frame_start, s.frame_end],
        "object_count": len(bpy.data.objects),
        "objects": objs,
        "counts_by_type": dict(sorted(Counter(o.type for o in bpy.data.objects).items())),
    }


# ---------------------------------------------------------------- armature

def inspect_armature(arm_obj):
    arm = arm_obj.data
    bones = list(arm.bones)
    names = [b.name for b in bones]
    by_name = {b.name: b for b in bones}
    pose_pos = arm.pose_position
    mw = arm_obj.matrix_world
    # Midline plane: mean world X of the unsided spine bones (head + tail).
    spine = [b for b in bones if b.name in SPINE_MIDLINE]
    midline_x = (sum((mw @ p).x for b in spine for p in (b.head_local, b.tail_local))
                 / (2 * len(spine))) if spine else 0.0

    bone_recs = []
    for b in bones:
        axes = b.matrix_local.to_3x3()
        pb = arm_obj.pose.bones.get(b.name)
        pose_state = None
        if pb is not None:
            pose_state = {
                "rotation_mode": pb.rotation_mode,
                "location": rnd(pb.location),
                "rotation_quaternion": rnd(pb.rotation_quaternion),
                "rotation_euler": rnd(pb.rotation_euler),
                "scale": rnd(pb.scale),
                "is_identity_pose": (
                    pb.location.length < 1e-9
                    and abs(pb.rotation_quaternion.angle) < 1e-9
                    and (pb.scale - Vector((1, 1, 1))).length < 1e-9
                ) if pb.rotation_mode == "QUATERNION" else None,
            }
        bone_recs.append({
            "name": b.name,
            "layer_prefix": prefix_of(b.name),
            "conforms_to_layer_naming": bool(LAYER_RE.match(b.name)),
            "parent": b.parent.name if b.parent else None,
            "children": [c.name for c in b.children],
            "depth": None,  # filled by graph pass
            "head_local": rnd(b.head_local),
            "tail_local": rnd(b.tail_local),
            "head_world": rnd(mw @ b.head_local),
            "tail_world": rnd(mw @ b.tail_local),
            "side_suffix": side_of(b.name),
            "side_position": side_position(mw @ b.head_local, midline_x),
            "length": rnd(b.length),
            # Bone.roll is not exposed on Bone in the Python API; the rest
            # frame is recorded as its three local axes instead.
            "rest_axis_x": rnd(axes.col[0]),
            "rest_axis_y": rnd(axes.col[1]),
            "rest_axis_z": rnd(axes.col[2]),
            "use_deform": b.use_deform,
            "use_connect": b.use_connect,
            "use_inherit_rotation": b.use_inherit_rotation,
            "use_local_location": b.use_local_location,
            "inherit_scale": b.inherit_scale,
            "hide": b.hide,
            "bbone_segments": b.bbone_segments,
            "bone_collections": sorted(c.name for c in getattr(b, "collections", [])),
            "pose": pose_state,
        })

    # Deform-flag and layer consistency (measured; interpretation separate)
    deform_on = [r["name"] for r in bone_recs if r["use_deform"]]
    deform_mismatch_def_off = [r["name"] for r in bone_recs
                               if r["layer_prefix"] == "DEF" and not r["use_deform"]]
    deform_mismatch_nondef_on = [r["name"] for r in bone_recs
                                 if r["layer_prefix"] in ("MECH", "CTRL") and r["use_deform"]]

    layer_counts = Counter(r["layer_prefix"] or "UNPREFIXED" for r in bone_recs)
    nonconforming = [r["name"] for r in bone_recs if not r["conforms_to_layer_naming"]]
    suffix_dups = [n for n in names if SUFFIX_DUP_RE.search(n)]
    side_counts = Counter(side_of(n) for n in names)
    side_by_position = Counter(f"{r['side_suffix']}|{r['side_position']}" for r in bone_recs)

    posed = [r["name"] for r in bone_recs
             if r["pose"] and r["pose"].get("is_identity_pose") is False]

    return {
        "object": arm_obj.name,
        "data": arm.name,
        "bone_count": len(bones),
        "pose_position": pose_pos,
        "object_scale": rnd(arm_obj.scale),
        "object_rotation_euler": rnd(arm_obj.rotation_euler),
        "object_location": rnd(arm_obj.location),
        "bone_collections": [c.name for c in getattr(arm, "collections_all", [])],
        "layer_prefix_counts": dict(layer_counts),
        "nonconforming_names": nonconforming,
        "suffix_dup_names": suffix_dups,
        "midline_x_world": rnd(midline_x),
        "side_suffix_counts": dict(side_counts),
        "side_by_position": dict(side_by_position),
        "deform_on_count": len(deform_on),
        "deform_flag_def_off": deform_mismatch_def_off,
        "deform_flag_nondef_on": deform_mismatch_nondef_on,
        "posed_bones_not_at_identity": posed,
        "bones": bone_recs,
    }


def facing_cross_check(arm_rec, mesh_info):
    """Infer the figure's forward (+Y or -Y) from two independent foot cues and
    compare the anatomical left (facing +Y: left = -X; facing -Y: left = +X)
    with the side the _L markers sit on. Two cues only; not a full proof."""
    by = {b["name"]: b for b in arm_rec["bones"]}
    cues = {}
    if "DEF_Foot" in by:
        h, t = by["DEF_Foot"]["head_world"], by["DEF_Foot"]["tail_world"]
        cues["foot_bone_tail_minus_head_y"] = rnd(t[1] - h[1])
        ankle_y = h[1]
    else:
        ankle_y = None
    feet = [m for m in mesh_info["meshes"] if m["scope"] == "body" and m["name"].startswith("411_Foot_")]
    if feet and ankle_y is not None:
        centre_y = [(m["world_bbox_min"][1] + m["world_bbox_max"][1]) / 2 for m in feet]
        cues["foot_mesh_centre_minus_ankle_y"] = rnd(sum(centre_y) / len(centre_y) - ankle_y)
    signs = {k: (1 if v > 0 else -1) for k, v in cues.items() if abs(v) > SYM_TOL}
    forward_y_sign = None
    if signs and len(set(signs.values())) == 1:
        forward_y_sign = list(signs.values())[0]
    marked = [b for b in arm_rec["bones"] if b["side_suffix"] in ("suffix_L", "infix_L")]
    marked_x = sorted(b["head_world"][0] for b in marked)
    marked_x_sign = None
    if marked_x:
        med = marked_x[len(marked_x) // 2]
        marked_x_sign = 1 if med > 0 else -1
    anatomical_left_x_sign = -forward_y_sign if forward_y_sign is not None else None
    return {
        "cues": cues,
        "cue_signs": signs,
        "forward_y_sign": forward_y_sign,
        "marked_L_bones_median_x_sign": marked_x_sign,
        "anatomical_left_x_sign_if_forward_is_this": anatomical_left_x_sign,
        "marked_L_matches_anatomical_left": (
            marked_x_sign == anatomical_left_x_sign
            if marked_x_sign is not None and anatomical_left_x_sign is not None else None),
        "limitation": "Two foot cues only; no visor/face cue used (the visor mesh is hidden and "
                      "named *_backup). Owner should confirm the front axis.",
    }


def build_bone_graph(arm_rec):
    bones = arm_rec["bones"]
    by = {b["name"]: b for b in bones}
    roots = [b["name"] for b in bones if b["parent"] is None]

    # Cycle check: walk parent chains with a visited guard.
    cycles = []
    for b in bones:
        seen, cur, steps = set(), b["name"], 0
        while cur is not None:
            if cur in seen:
                cycles.append(cur)
                break
            seen.add(cur)
            cur = by[cur]["parent"]
            steps += 1
            if steps > len(bones) + 1:
                cycles.append(cur)
                break

    # Depth + subtree size
    def depth_of(n):
        d, cur = 0, by[n]["parent"]
        while cur is not None:
            d += 1
            cur = by[cur]["parent"]
        return d
    for b in bones:
        b["depth"] = depth_of(b["name"])

    # Connected components via union-find on parent edges
    parent_uf = {n: n for n in by}

    def find(x):
        while parent_uf[x] != x:
            parent_uf[x] = parent_uf[parent_uf[x]]
            x = parent_uf[x]
        return x
    for b in bones:
        if b["parent"] is not None:
            parent_uf[find(b["name"])] = find(b["parent"])
    comps = defaultdict(list)
    for n in by:
        comps[find(n)].append(n)
    components = sorted((sorted(v) for v in comps.values()), key=lambda c: -len(c))

    def subtree_size(n):
        return 1 + sum(subtree_size(c) for c in by[n]["children"])

    root_info = [{"root": r, "subtree_size": subtree_size(r)} for r in roots]
    edges = [{"parent": b["parent"], "child": b["name"]} for b in bones if b["parent"]]
    return {
        "object": arm_rec["object"],
        "node_count": len(bones),
        "edge_count": len(edges),
        "root_candidates": roots,
        "root_count": len(roots),
        "root_subtrees": root_info,
        "component_count": len(components),
        "component_sizes": [len(c) for c in components],
        "components": components,
        "cycle_nodes": sorted(set(cycles)),
        "max_depth": max((b["depth"] for b in bones), default=0),
        "leaf_count": sum(1 for b in bones if not b["children"]),
        "edges": edges,
    }


def bone_mirror_residuals(arm_obj, midline_x):
    """Mirror each bone endpoint (world space) across two candidate planes:
    the world X=0 plane and the spine-midline plane. For each bone, keep the
    worst endpoint's distance to the nearest endpoint of the mirrored skeleton."""
    mw = arm_obj.matrix_world
    pts = []
    for b in arm_obj.data.bones:
        pts.append((b.name, mw @ b.head_local))
        pts.append((b.name, mw @ b.tail_local))
    if not pts:
        return {"measured": False}
    kd = KDTree(len(pts))
    for i, (_, p) in enumerate(pts):
        kd.insert(p, i)
    kd.balance()
    planes = {}
    for label, cx in (("world_x0", 0.0), ("spine_midline", midline_x)):
        per_bone = {}
        for name, p in pts:
            m = Vector((2.0 * cx - p.x, p.y, p.z))
            _, _, dist = kd.find(m)
            per_bone[name] = max(per_bone.get(name, 0.0), dist)
        vals = sorted(per_bone.values())
        outliers = sorted(((v, n) for n, v in per_bone.items() if v > SYM_TOL), reverse=True)
        planes[label] = {
            "plane_x_world": rnd(cx),
            "bones_checked": len(per_bone),
            "median_residual": rnd(vals[len(vals) // 2]),
            "max_residual": rnd(vals[-1]),
            "tolerance": SYM_TOL,
            "outlier_count": len(outliers),
            "outliers": [{"bone": n, "residual": rnd(v)} for v, n in outliers[:50]],
        }
    return {"measured": True, "planes": planes}


# ---------------------------------------------------------------- meshes

def mesh_world_verts(o):
    mw = o.matrix_world
    return [mw @ v.co for v in o.data.vertices]


def inspect_meshes(midline_x):
    meshes = [o for o in sorted(bpy.data.objects, key=lambda x: x.name) if o.type == "MESH"]
    recs = []
    all_world = []
    for o in meshes:
        me = o.data
        verts_w = mesh_world_verts(o)
        if is_body(o.name):
            all_world.extend(verts_w)
        if verts_w:
            mins = Vector((min(v.x for v in verts_w), min(v.y for v in verts_w), min(v.z for v in verts_w)))
            maxs = Vector((max(v.x for v in verts_w), max(v.y for v in verts_w), max(v.z for v in verts_w)))
        else:
            mins = maxs = Vector((0, 0, 0))
        tri_count = sum(max(len(p.vertices) - 2, 0) for p in me.polygons)
        ngon_count = sum(1 for p in me.polygons if len(p.vertices) > 4)

        # Topology on a temporary bmesh copy (source is never written)
        bm = bmesh.new()
        bm.from_mesh(me)
        boundary = sum(1 for e in bm.edges if e.is_boundary)
        nonmanifold = sum(1 for e in bm.edges if not e.is_manifold and not e.is_boundary and not e.is_wire)
        wire_edges = sum(1 for e in bm.edges if e.is_wire)
        loose_verts = sum(1 for v in bm.verts if len(v.link_edges) == 0)
        zero_area = sum(1 for f in bm.faces if f.calc_area() < 1e-12)
        bm.free()

        kd = KDTree(len(me.vertices))
        for i, v in enumerate(me.vertices):
            kd.insert(v.co, i)
        kd.balance()
        dup_pairs = 0
        for i, v in enumerate(me.vertices):
            near = kd.find_range(v.co, 1e-6)
            dup_pairs += sum(1 for _, j, _ in near if j > i)

        recs.append({
            "name": o.name,
            "scope": scope_of(o.name),
            "data": me.name,
            "hide_viewport": o.hide_viewport,
            "hide_render": o.hide_render,
            "parent": id_name(o.parent),
            "scale": rnd(o.scale),
            "rotation_euler": rnd(o.rotation_euler),
            "delta_scale": rnd(o.delta_scale),
            "unit_scale_applied_check": all(abs(s - 1.0) < 1e-9 for s in o.scale),
            "vertices": len(me.vertices),
            "edges": len(me.edges),
            "faces": len(me.polygons),
            "triangles": tri_count,
            "ngons_gt4": ngon_count,
            "world_bbox_min": rnd(mins),
            "world_bbox_max": rnd(maxs),
            "world_dimensions": rnd(maxs - mins),
            "materials": [slot.material.name if slot.material else None for slot in o.material_slots],
            "topology": {
                "boundary_edges": boundary,
                "nonmanifold_edges": nonmanifold,
                "wire_edges": wire_edges,
                "loose_vertices": loose_verts,
                "zero_area_faces": zero_area,
                "duplicate_vertex_pairs_1e-6": dup_pairs,
            },
            "armature_modifier_targets": [m.object.name if m.object else None
                                          for m in o.modifiers if m.type == "ARMATURE"],
        })

    # Body-level dimensions and mirror symmetry over all mesh vertices
    body = {"measured": False}
    if all_world:
        xs = [v.x for v in all_world]
        ys = [v.y for v in all_world]
        zs = [v.z for v in all_world]
        body = {
            "measured": True,
            "min": rnd(Vector((min(xs), min(ys), min(zs)))),
            "max": rnd(Vector((max(xs), max(ys), max(zs)))),
            "height_z": rnd(max(zs) - min(zs)),
            "width_x": rnd(max(xs) - min(xs)),
            "depth_y": rnd(max(ys) - min(ys)),
            "mean_x": rnd(sum(xs) / len(xs)),
        }
        kd = KDTree(len(all_world))
        for i, p in enumerate(all_world):
            kd.insert(p, i)
        kd.balance()
        body["mirror_residual"] = {}
        for label, cx in (("world_x0", 0.0), ("spine_midline", midline_x)):
            resid = []
            for p in all_world:
                _, _, d = kd.find(Vector((2.0 * cx - p.x, p.y, p.z)))
                resid.append(d)
            resid.sort()
            n = len(resid)
            body["mirror_residual"][label] = {
                "plane_x_world": rnd(cx),
                "median": rnd(resid[n // 2]),
                "p99": rnd(resid[min(n - 1, int(n * 0.99))]),
                "max": rnd(resid[-1]),
                "tolerance": SYM_TOL,
                "fraction_above_tolerance": rnd(sum(1 for d in resid if d > SYM_TOL) / n),
            }
        body["mirror_residual"]["note"] = (
            "Nearest vertex of the mirrored body-mesh set (411_* only) at each candidate plane. "
            "Measures approximate symmetry; not a defect test by itself.")

    # Sided pairs (*_L / *_R) dimension comparison
    body_recs = [r for r in recs if r["scope"] == "body"]
    by_name = {r["name"]: r for r in body_recs}
    pairs = []
    for r in body_recs:
        if r["name"].endswith("_L"):
            twin = r["name"][:-2] + "_R"
            if twin in by_name:
                a, b = r["world_dimensions"], by_name[twin]["world_dimensions"]
                pairs.append({
                    "left": r["name"], "right": twin,
                    "dim_delta": rnd([a[i] - b[i] for i in range(3)]),
                    "max_abs_dim_delta": rnd(max(abs(a[i] - b[i]) for i in range(3))),
                })
    unpaired_sided = sorted(r["name"] for r in body_recs
                            if (r["name"].endswith("_L") and r["name"][:-2] + "_R" not in by_name)
                            or (r["name"].endswith("_R") and r["name"][:-2] + "_L" not in by_name))

    return {"mesh_count": len(recs), "body_mesh_count": len(body_recs),
            "environment_mesh_count": len(recs) - len(body_recs),
            "meshes": recs, "body": body,
            "sided_pairs": pairs, "unpaired_sided_meshes": unpaired_sided}


# ---------------------------------------------------------------- weights

def inspect_weights(mesh_objs, bone_names):
    bone_set = set(bone_names)
    per_mesh = []
    bone_binding = Counter()
    unmatched_all = set()
    unweighted_total = 0
    over_limit = []
    for o in mesh_objs:
        me = o.data
        vg_names = [g.name for g in o.vertex_groups]
        n = len(me.vertices)
        weighted, unweighted, bad_sum, multi, max_inf = 0, 0, 0, 0, 0
        per_group_verts = Counter()
        over_count = 0
        for v in me.vertices:
            inf = [(g.group, g.weight) for g in v.groups if g.weight > 0]
            k = len(inf)
            max_inf = max(max_inf, k)
            if k == 0:
                unweighted += 1
                continue
            weighted += 1
            if k > 1:
                multi += 1
            if k > GLTF_MAX_JOINTS_PER_VERTEX:
                over_count += 1
            total = sum(w for _, w in inf)
            if abs(total - 1.0) > WEIGHT_SUM_TOL:
                bad_sum += 1
            for gi, _ in inf:
                per_group_verts[vg_names[gi] if gi < len(vg_names) else f"<bad index {gi}>"] += 1
        unweighted_total += unweighted
        if over_count:
            over_limit.append({"mesh": o.name, "vertices_over_4_influences": over_count})
        unmatched = sorted(g for g in vg_names if g not in bone_set)
        unmatched_all.update(unmatched)
        for g, c in per_group_verts.items():
            if g in bone_set:
                bone_binding[g] += c
        per_mesh.append({
            "mesh": o.name,
            "vertex_count": n,
            "vertex_groups": len(vg_names),
            "weighted_vertices": weighted,
            "unweighted_vertices": unweighted,
            "coverage_fraction": rnd(weighted / n) if n else None,
            "multi_influence_vertices": multi,
            "max_influences_per_vertex": max_inf,
            "vertices_with_weight_sum_off_1": bad_sum,
            "unmatched_vertex_groups": unmatched,
            "groups_with_no_vertices": sorted(g for g in vg_names if per_group_verts.get(g, 0) == 0),
            "armature_modifier": [m.object.name if m.object else None
                                  for m in o.modifiers if m.type == "ARMATURE"],
        })
    bones_unbound = sorted(b for b in bone_names if bone_binding.get(b, 0) == 0)
    return {
        "mesh_count": len(mesh_objs),
        "meshes": per_mesh,
        "total_unweighted_vertices": unweighted_total,
        "unmatched_group_names_all_meshes": sorted(unmatched_all),
        "bones_with_zero_vertex_binding": bones_unbound,
        "bones_with_zero_vertex_binding_count": len(bones_unbound),
        "bones_bound_count": len(bone_names) - len(bones_unbound),
        "vertices_over_gltf_4_influence_limit": over_limit,
        "bone_vertex_binding_counts": dict(sorted(bone_binding.items())),
    }


# ---------------------------------------------------------------- modifiers / keys / constraints / drivers

def inspect_modifiers():
    out = []
    for o in sorted(bpy.data.objects, key=lambda x: x.name):
        for idx, m in enumerate(o.modifiers):
            rec = {
                "object": o.name,
                "index": idx,
                "name": m.name,
                "type": m.type,
                "show_viewport": m.show_viewport,
                "show_render": m.show_render,
                "scope": scope_of(o.name),
                "target_object": id_name(getattr(m, "object", None)),
                "settings": simple_props(m),
            }
            out.append(rec)
    return {
        "modifier_count": len(out),
        "by_type": dict(Counter(m["type"] for m in out)),
        "modifiers": out,
    }


def inspect_shape_keys():
    out = []
    for me in sorted(bpy.data.meshes, key=lambda x: x.name):
        sk = me.shape_keys
        if sk is None:
            continue
        blocks = []
        for kb in sk.key_blocks:
            blocks.append({
                "name": kb.name,
                "value": rnd(kb.value),
                "slider_min": rnd(kb.slider_min),
                "slider_max": rnd(kb.slider_max),
                "mute": kb.mute,
                "relative_key": kb.relative_key.name,
                "vertex_group": kb.vertex_group,
                "interpolation": kb.interpolation,
            })
        out.append({
            "mesh_data": me.name,
            "key_name": sk.name,
            "use_relative": sk.use_relative,
            "reference_key": sk.reference_key.name,
            "block_count": len(blocks),
            "blocks": blocks,
        })
    return {"shape_key_datablocks": len(out), "keys": out}


def inspect_constraints(arm_objs):
    obj_level = []
    for o in sorted(bpy.data.objects, key=lambda x: x.name):
        for c in o.constraints:
            obj_level.append({"owner": o.name, "constraint": c.name, "type": c.type,
                              "target": id_name(getattr(c, "target", None)),
                              "subtarget": getattr(c, "subtarget", None),
                              "influence": rnd(c.influence), "mute": c.mute,
                              "enabled": c.enabled if hasattr(c, "enabled") else None})
    bone_level = []
    for arm_obj in arm_objs:
        for pb in arm_obj.pose.bones:
            for c in pb.constraints:
                bone_level.append({
                    "owner": f"{arm_obj.name}:{pb.name}", "constraint": c.name, "type": c.type,
                    "target": id_name(getattr(c, "target", None)),
                    "subtarget": getattr(c, "subtarget", None),
                    "pole_target": id_name(getattr(c, "pole_target", None)),
                    "chain_count": getattr(c, "chain_count", None),
                    "influence": rnd(c.influence), "mute": c.mute,
                    "owner_space": getattr(c, "owner_space", None),
                    "target_space": getattr(c, "target_space", None),
                })
    return {
        "object_constraints": obj_level,
        "pose_bone_constraints": bone_level,
        "total": len(obj_level) + len(bone_level),
    }


SELECTION_TOKENS = ("select", "active", "context.", "bpy.context")


def action_fcurve_count(act):
    """Blender 4.4+ layered actions keep F-curves in layers/strips/channelbags;
    older builds expose act.fcurves directly. Count whichever is present."""
    direct = getattr(act, "fcurves", None)
    if direct is not None:
        return len(direct)
    n = 0
    for layer in getattr(act, "layers", []):
        for strip in layer.strips:
            for cb in getattr(strip, "channelbags", []):
                n += len(cb.fcurves)
    return n


def inspect_drivers_and_actions():
    drivers = []
    id_sets = [("OBJECT", bpy.data.objects), ("MESH", bpy.data.meshes),
               ("ARMATURE", bpy.data.armatures), ("KEY", bpy.data.shape_keys),
               ("MATERIAL", bpy.data.materials), ("SCENE", bpy.data.scenes),
               ("WORLD", bpy.data.worlds), ("CAMERA", bpy.data.cameras),
               ("LIGHT", bpy.data.lights)]
    for kind, coll in id_sets:
        for idb in coll:
            ad = getattr(idb, "animation_data", None)
            if ad is None:
                continue
            for fc in ad.drivers:
                drv = fc.driver
                variables = []
                for v in drv.variables:
                    variables.append({
                        "name": v.name, "type": v.type,
                        "targets": [{"id": id_name(t.id), "id_type": t.id_type,
                                     "data_path": t.data_path, "bone_target": t.bone_target,
                                     "transform_type": t.transform_type,
                                     "transform_space": t.transform_space} for t in v.targets],
                    })
                expr = drv.expression or ""
                drivers.append({
                    "owner_kind": kind, "owner": idb.name,
                    "data_path": fc.data_path, "array_index": fc.array_index,
                    "driver_type": drv.type, "expression": expr,
                    "variables": variables,
                    "references_selection_or_context": any(t in expr for t in SELECTION_TOKENS),
                    "modifiers": len(fc.modifiers),
                })
    actions = []
    for act in bpy.data.actions:
        actions.append({"name": act.name, "fcurve_count": action_fcurve_count(act),
                        "frame_range": rnd(list(act.frame_range)),
                        "users": act.users})
    return {
        "driver_count": len(drivers),
        "drivers": drivers,
        "drivers_referencing_selection_or_context": sum(
            1 for d in drivers if d["references_selection_or_context"]),
        "action_count": len(actions),
        "actions": actions,
    }


def inspect_collections():
    cols = []
    for c in sorted(bpy.data.collections, key=lambda x: x.name):
        cols.append({"name": c.name, "objects": sorted(o.name for o in c.objects),
                     "children": sorted(ch.name for ch in c.children),
                     "hide_viewport": c.hide_viewport, "hide_render": c.hide_render})
    scene = bpy.context.scene
    scene_objs = set(o.name for o in scene.collection.all_objects)
    all_objs = set(o.name for o in bpy.data.objects)
    return {
        "collection_count": len(cols),
        "collections": cols,
        "scene_master_object_count": len(scene_objs),
        "objects_not_in_scene": sorted(all_objs - scene_objs),
        "orphan_objects_no_collection": sorted(o.name for o in bpy.data.objects if not o.users_collection),
    }


def inspect_materials():
    return {
        "material_count": len(bpy.data.materials),
        "materials": sorted(m.name for m in bpy.data.materials),
        "note": "Node-tree contents are not inspected by v01.",
    }


# ---------------------------------------------------------------- issues

ISSUES = []


def add_issue(category, target, measured, status, severity, interpretation,
              next_test, remedy, risks):
    ISSUES.append({
        "id": f"HI-{len(ISSUES) + 1:03d}",
        "category": category,
        "target": target,
        "measured": rnd(measured),
        "verification_status": status,   # MEASURED | CANDIDATE | CONFIRMED | UNRESOLVED
        "severity": severity,            # info | low | medium | high
        "interpretation": interpretation,
        "proposed_next_test": next_test,
        "potential_remedy": remedy,
        "risks_and_dependencies": risks,
    })


def derive_issues(scene, arm_recs, graphs, mesh_info, weights, mods, keys, cons, drv, cols, mats,
                  mirror_b, facing):
    arm = arm_recs[0]
    g = graphs[0]

    # --- Scale / units
    scale_ok = scene["unit_scale_length"] == 1.0 and scene["unit_system"] != "NONE"
    if not scale_ok:
        add_issue("unit_scale", "scene", {"unit_system": scene["unit_system"],
                                          "scale_length": scene["unit_scale_length"]},
                  "UNRESOLVED", "medium",
                  "Scene unit scale is not the metric 1.0 baseline; any measurement in "
                  "metres (height, widths) is only valid after converting.",
                  "Confirm intended unit convention with owner; compare against design brief in metres.",
                  "Set scene scale deliberately or document the conversion factor.",
                  "Changing scene scale alters all dimensions, so do this only with owner approval.")

    body_meshes = [m for m in mesh_info["meshes"] if m["scope"] == "body"]
    odd_scale = [m["name"] for m in body_meshes if not m["unit_scale_applied_check"]]
    arm_scale_ok = all(abs(s - 1.0) < 1e-9 for s in arm["object_scale"])
    if odd_scale or not arm_scale_ok:
        add_issue("object_scale", "meshes/armature",
                  {"meshes_with_non_unit_object_scale": odd_scale,
                   "armature_scale": arm["object_scale"]},
                  "CANDIDATE", "low",
                  "Non-unit object scale is legitimate when deliberate, but unapplied scale "
                  "affects glTF export and bone-length readings.",
                  "Compare object scale against mesh-data bounds for each affected object.",
                  "Apply transforms only after owner review; never automatic.",
                  "Applying transforms rewrites mesh/bone data and is not performed by v01.")

    # --- Naming
    sc = arm["side_suffix_counts"]
    if sc.get("suffix_R", 0) + sc.get("infix_R", 0) == 0 and sc.get("suffix_L", 0) + sc.get("infix_L", 0) > 0:
        add_issue("side_convention", "armature bones", {
            "side_suffix_counts": sc,
            "side_by_position": arm["side_by_position"],
            "midline_x_world": arm["midline_x_world"]},
            "CONFIRMED", "medium",
            "docs/ENGINEERING_RULES.md §6 says bilateral limbs use the .L/.R suffix. Measured: no bone carries an "
            "R marker, left-side bones are marked with _L (suffix) or _L_ (infix), and the remaining "
            "bones are unsided. Which physical side the unsided bones represent is UNRESOLVED until "
            "checked against the world positions in side_by_position.",
            "Compare each unsided limb bone's world head X against its _L twin's sign and "
            "against the spine midline (midline_x_world).",
            "Owner decides one convention (suffix form and which side is which), then renames "
            "with a constraint/driver/vertex-group reference update.",
            "Renaming without updating vertex groups and constraints silently unbinds limbs.")
    if facing["marked_L_matches_anatomical_left"] is False:
        add_issue("side_facing_mismatch", "armature bones + body meshes", {
            "forward_y_sign": facing["forward_y_sign"],
            "cues": facing["cues"],
            "marked_L_bones_median_x_sign": facing["marked_L_bones_median_x_sign"],
            "anatomical_left_x_sign": facing["anatomical_left_x_sign_if_forward_is_this"]},
            "CANDIDATE", "high",
            "The _L-marked bones and 411_*_L meshes sit on the side that is anatomical RIGHT if the "
            "figure faces the -Y direction the foot cues indicate. If confirmed, every _L marker, "
            "and anything keyed to it (hello/wave arm choice, mirror pairs, skinning map), is "
            "inverted. Facing is inferred from two foot cues only.",
            "Owner confirms the front axis of the figure; then check the hello_sequence arm and "
            "the skin mapping against it.",
            "Relabel L/R only with a reference-update plan; do not swap meshes or bones silently.",
            "Relabeling touches vertex-group names, constraints and every animation keyed to a side.")
    if arm["nonconforming_names"]:
        add_issue("naming_convention", "armature bones", {
            "count": len(arm["nonconforming_names"]),
            "names": arm["nonconforming_names"]},
            "CONFIRMED", "medium",
            "Names do not match the docs/ENGINEERING_RULES.md §6 layer-prefix convention "
            "({LAYER}_{Segment}_{Joint}[_{Axis}]). Measured fact; whether they need renaming is an owner decision.",
            "Re-run inspector after any rename; confirm no constraint or driver targets these names.",
            "Rename to the convention only with a reference-update plan (constraints/drivers/vgroups).",
            "Renaming breaks vertex-group and constraint name references if done without updating them.")
    if arm["suffix_dup_names"]:
        add_issue("duplicate_suffix", "armature bones", arm["suffix_dup_names"], "CANDIDATE", "low",
                  "Blender .001-style suffixes on bone names usually indicate duplicated bones.",
                  "Compare the suffixed bone's transform and weights against its base name.",
                  "Remove only after confirming it is unused (owner approval).",
                  "Deleting a bone that carries weights changes deformation.")

    # --- Deform flags
    if arm["deform_flag_def_off"]:
        add_issue("deform_flag", "DEF bones", arm["deform_flag_def_off"], "CANDIDATE", "medium",
                  "DEF-prefixed bones with use_deform off will not deform the mesh even if weighted.",
                  "Cross-check each listed bone against the vertex-weight binding table in weights.json.",
                  "Enable use_deform on DEF bones only after weight check.",
                  "Turning deform on for a helper bone can unexpectedly pull vertices.")
    if arm["deform_flag_nondef_on"]:
        add_issue("deform_flag", "MECH/CTRL bones", arm["deform_flag_nondef_on"], "CANDIDATE", "medium",
                  "MECH/CTRL bones with use_deform on would deform the body from mechanism or control "
                  "bones, contrary to docs/ENGINEERING_RULES.md §4 (DEF is the deformation layer).",
                  "Check whether any vertex weights reference these bones.",
                  "Disable use_deform on MECH/CTRL bones after confirming no weights depend on them.",
                  "Some mechanism bones may be intentionally deform-enabled; needs owner review.")

    # --- Graph
    if g["root_count"] > 1:
        add_issue("skeleton_roots", "armature", {"roots": g["root_candidates"],
                                                 "components": g["component_count"],
                                                 "component_sizes": g["component_sizes"]},
                  "CANDIDATE", "medium" if g["component_count"] > 1 else "low",
                  "Multiple root bones. The brief notes multi-root structures are not necessarily "
                  "incorrect; this is a candidate until the intended root (e.g. DEF_Root/hips) is decided.",
                  "Verify which root each MECH/CTRL/DEF island should attach to per specs/ and RIGGING.md.",
                  "Parent detached roots only after owner decides hierarchy.",
                  "Re-parenting changes rest-space and can alter inherited transforms.")
    if g["cycle_nodes"]:
        add_issue("skeleton_cycle", "armature", g["cycle_nodes"], "CONFIRMED", "high",
                  "Parent-chain cycle detected; Blender should not allow this, so the file state needs checking.",
                  "Re-open file read-only and recount parent chains.", "Reconstruct the parent link.",
                  "Cycle breaks evaluation.")

    # --- Mesh topology
    topo_bad = []
    for m in body_meshes:
        t = m["topology"]
        flags = {k: v for k, v in t.items() if v}
        if flags:
            topo_bad.append({"mesh": m["name"], **flags})
    if topo_bad:
        add_issue("mesh_topology", "meshes", {"meshes_with_flags": len(topo_bad), "sample": topo_bad[:20]},
                  "CANDIDATE", "low",
                  "Boundary (open) edges are normal on hard-surface shells; non-manifold, loose, "
                  "duplicate or zero-area geometry is a candidate for review, not a confirmed defect.",
                  "Inspect flagged meshes in a viewport at high zoom; decide whether shells are intended open.",
                  "Repair topology only after owner review; v01 never repairs.",
                  "Repair can change silhouette and shading.")

    # --- Armature binding
    unskinned = [m["name"] for m in body_meshes if not m["armature_modifier_targets"]]
    if unskinned:
        add_issue("skinning_coverage", "meshes", {"meshes_without_armature_modifier": len(unskinned),
                                                  "names": unskinned[:40]},
                  "CANDIDATE", "medium",
                  "Meshes without an Armature modifier will not follow the rig. The project's "
                  "skin_rigid_v01.py expects one per body mesh, so this is a deviation from that script's "
                  "intent unless the body was re-parented on purpose.",
                  "Compare against docs/phase_skinning.md mapping.",
                  "Re-run skinning only with owner approval.",
                  "Re-skinning can change deformation on every body part.")

    wrong_target = [m["name"] for m in body_meshes
                    if m["armature_modifier_targets"] and arm["object"] not in m["armature_modifier_targets"]]
    if wrong_target:
        add_issue("armature_modifier_target", "meshes", wrong_target, "CANDIDATE", "medium",
                  "Armature modifier points at a different object than the inspected armature.",
                  "Confirm target object for each listed mesh.", "Retarget after owner review.",
                  "Retargeting moves deformation to another skeleton.")

    if weights["total_unweighted_vertices"]:
        add_issue("weight_coverage", "meshes",
                  {"total_unweighted_vertices": weights["total_unweighted_vertices"],
                   "per_mesh": [m["mesh"] for m in weights["meshes"] if m["unweighted_vertices"]][:40]},
                  "CANDIDATE", "medium",
                  "Vertices with no positive weight stay at rest while the rest of the bone moves.",
                  "Check if they lie on rigid shells that intentionally have no bone (e.g. unweighted decals).",
                  "Assign weights only after owner review.", "Weight edits change deformation.")

    if weights["unmatched_group_names_all_meshes"]:
        add_issue("vertex_group_orphans", "meshes", weights["unmatched_group_names_all_meshes"],
                  "CANDIDATE", "low",
                  "Vertex groups whose names match no bone cannot drive deformation.",
                  "Check whether group names are stale after a bone rename.", "Delete or rename after review.",
                  "Deleting a group that has weights removes that influence.")

    if weights["vertices_over_gltf_4_influence_limit"]:
        add_issue("gltf_influence_limit", "meshes", weights["vertices_over_gltf_4_influence_limit"],
                  "CONFIRMED", "high",
                  "glTF 2.0 stores at most 4 joint influences per vertex per set; vertices above that "
                  "will be truncated or rejected at export.",
                  "Count per-vertex influences after proposed weight changes.",
                  "Limit influences per vertex to 4 before export (owner-approved weight edit).",
                  "Limiting influences changes deformation slightly.")

    # Bones with no binding (expected for MECH/CTRL; candidate for DEF)
    def_unbound = [b for b in weights["bones_with_zero_vertex_binding"] if b.startswith("DEF_")]
    if def_unbound:
        add_issue("bone_binding", "DEF bones", def_unbound, "CANDIDATE", "medium",
                  "DEF bones with no vertex binding will not deform any vertex.",
                  "Check against palm/finger/forearm specs before any change.",
                  "Bind only after owner review.", "Binding changes deformation.")

    # --- Mirror / symmetry
    if mirror_b.get("measured"):
        best = min(mirror_b["planes"].values(), key=lambda p: p["outlier_count"])
        add_issue("bone_symmetry", "armature bones",
                  {"planes": {k: {kk: vv for kk, vv in v.items() if kk != "outliers"}
                              for k, v in mirror_b["planes"].items()},
                   "best_plane_top_outliers": best["outliers"][:15]},
                  "CANDIDATE", "low",
                  "Bones whose X-mirrored endpoint has no counterpart within tolerance. Asymmetric or "
                  "one-sided bones (e.g. arm on one side only) are not necessarily wrong.",
                  "Confirm which bones are intentionally one-sided per specs.",
                  "Adjust only after spec check.", "Moving bones alters weights and rest pose.")
    body = mesh_info["body"]
    if body.get("measured") and min(p["fraction_above_tolerance"]
                                    for p in body["mirror_residual"].values()
                                    if isinstance(p, dict)) > 0.01:
        add_issue("mesh_symmetry", "body meshes", body["mirror_residual"], "CANDIDATE", "low",
                  "More than 1% of body vertices lack a mirror counterpart within tolerance even at "
                  "the better of the two candidate planes.",
                  "Compare left/right mesh pairs and centreline placement.",
                  "Fix only after identifying the cause.", "Mesh edits change the look.")
    if mesh_info["sided_pairs"]:
        worst = max(mesh_info["sided_pairs"], key=lambda p: p["max_abs_dim_delta"])
        if worst["max_abs_dim_delta"] > SYM_TOL:
            add_issue("sided_dimension_delta", "sided mesh pairs",
                      {"worst_pair": worst, "pairs_over_tol": sum(
                          1 for p in mesh_info["sided_pairs"] if p["max_abs_dim_delta"] > SYM_TOL)},
                      "CANDIDATE", "low",
                      "Left/right mesh pairs differ in world dimensions above tolerance.",
                      "Check whether the difference is intentional (e.g. port/asymmetric detail).",
                      "Review per part.", "None until checked.")
    if mesh_info["unpaired_sided_meshes"]:
        add_issue("sided_pairing", "meshes", mesh_info["unpaired_sided_meshes"], "CANDIDATE", "low",
                  "Sided mesh names without a matching twin.", "Confirm intended unpaired parts.",
                  "Rename or add twins only after review.", "None until checked.")

    # --- Pose / rest state
    if arm["pose_position"] != "REST" or arm["posed_bones_not_at_identity"]:
        add_issue("rest_pose_state", "armature",
                  {"pose_position": arm["pose_position"],
                   "posed_bone_count": len(arm["posed_bones_not_at_identity"]),
                   "sample": arm["posed_bones_not_at_identity"][:20]},
                  "MEASURED", "info",
                  "The file was saved with a pose applied or pose_position set to POSE. Measurements "
                  "of bone rest positions use bone head/tail (rest data), not pose transforms.",
                  "Confirm owner intent for the saved pose.", "None; record only.", "None.")

    # --- Modifiers / constraints / drivers
    if drv["drivers_referencing_selection_or_context"]:
        add_issue("driver_selection_dependency", "drivers",
                  [d["owner"] + ":" + d["data_path"] for d in drv["drivers"]
                   if d["references_selection_or_context"]],
                  "CONFIRMED", "high",
                  "docs/ENGINEERING_RULES.md §10: no driver expression may depend on selection or viewport state.",
                  "Re-read each listed expression.", "Replace with SINGLE_PROP on a custom property.",
                  "Changing a driver changes the rig's live behaviour.")
    if cons["total"]:
        add_issue("constraints_present", "armature/objects", {
            "object_constraints": len(cons["object_constraints"]),
            "pose_bone_constraints": len(cons["pose_bone_constraints"])},
            "MEASURED", "info",
            "Constraints exist. Each needs the docs/ENGINEERING_RULES.md §9 review (target, owner, space, axes, "
            "influence, order). Not evaluated by v01.",
            "Audit constraints listed in constraints.json against specs.", "None until audited.",
            "Constraints can conflict with glTF export (docs/ENGINEERING_RULES.md §5).")
    if mods["modifier_count"]:
        add_issue("modifier_inventory", "objects", mods["by_type"], "MEASURED", "info",
                  "Modifier inventory. Evaluated (post-modifier) geometry was not measured.",
                  "Evaluate depsgraph in a copy in a later version.", "None.", "None.")

    # --- Environment scope (recorded so it is not mistaken for body geometry)
    env = [m for m in mesh_info["meshes"] if m["scope"] == "environment"]
    if env:
        add_issue("environment_scope", "environment meshes", {
            "environment_mesh_count": len(env),
            "with_armature_modifier": sum(1 for m in env if m["armature_modifier_targets"]),
            "non_unit_scale": sum(1 for m in env if not m["unit_scale_applied_check"])},
            "MEASURED", "info",
            "Non-411_ meshes (set dressing: warehouse, cable, crate, studio, cinematic camera volume). "
            "They are excluded from all body measurements; they are not humanoid defects.",
            "None for the humanoid; confirm owner wants them kept in the body file.",
            "None.", "None.")

    # --- Hidden body meshes
    hidden = [m["name"] for m in body_meshes if m["hide_viewport"] or m["hide_render"]]
    if hidden:
        add_issue("hidden_meshes", "meshes", {"count": len(hidden), "names": hidden[:40]},
                  "MEASURED", "info",
                  "Hidden in viewport/render. May be intentional (helpers, backups).",
                  "Confirm each hidden mesh's purpose.", "None.", "None.")

    return ISSUES


# ---------------------------------------------------------------- outputs

def write_json(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(rnd(data), f, indent=2, sort_keys=True)
        f.write("\n")


def write_markdown(path, ctx):
    scene, arm, g, mesh_info, weights, issues = (
        ctx["scene"], ctx["arm"], ctx["graph"], ctx["mesh_info"], ctx["weights"], ctx["issues"])
    L = []
    L.append(f"# Body structure baseline — {ctx['run_id']}")
    L.append("")
    L.append("Read-only inspection. Section A is measured fact. Section B lists interpretations, "
             "each tagged with its verification status. Nothing here has been modified in the source.")
    L.append("")
    L.append(f"- Source: `{ctx['source']}`")
    L.append(f"- Source SHA-256 (unchanged before and after the run): `{ctx['source_sha']}`")
    L.append(f"- Blender {ctx['blender_version']}, script `{SCRIPT_VERSION}`")
    L.append(f"- Units: {scene['unit_system']} / {scene['unit_length']}, scale_length = {scene['unit_scale_length']}")
    L.append("")
    L.append("## A. Measured")
    L.append("")
    L.append(f"- Objects: {scene['object_count']} total — {scene['counts_by_type']}")
    L.append(f"- Armature `{arm['object']}`: **{arm['bone_count']} bones**, "
             f"pose_position = {arm['pose_position']}, roots = {g['root_count']}, "
             f"components = {g['component_count']} (sizes {g['component_sizes']}), "
             f"max depth = {g['max_depth']}, leaves = {g['leaf_count']}, cycles = {len(g['cycle_nodes'])}")
    L.append(f"- Layer prefixes: {arm['layer_prefix_counts']}")
    L.append(f"- Side suffixes: {arm['side_suffix_counts']} (by world position vs spine midline "
             f"x = {arm['midline_x_world']}: {arm['side_by_position']})")
    L.append(f"- Bones with use_deform on: {arm['deform_on_count']}")
    body_m = [m for m in mesh_info["meshes"] if m["scope"] == "body"]
    L.append(f"- Meshes: **{mesh_info['mesh_count']}** total = {mesh_info['body_mesh_count']} body (`411_*`) "
             f"+ {mesh_info['environment_mesh_count']} environment. Body: vertices "
             f"{sum(m['vertices'] for m in body_m)}, faces {sum(m['faces'] for m in body_m)}, "
             f"triangles {sum(m['triangles'] for m in body_m)}")
    b = mesh_info["body"]
    if b.get("measured"):
        L.append(f"- Body extents (world, Blender units): height_z {b['height_z']}, "
                 f"width_x {b['width_x']}, depth_y {b['depth_y']}, mean_x {b['mean_x']}")
    L.append(f"- Weights: {weights['mesh_count']} meshes, unweighted vertices "
             f"{weights['total_unweighted_vertices']}, bones bound "
             f"{weights['bones_bound_count']}, bones unbound {weights['bones_with_zero_vertex_binding_count']}")
    L.append(f"- Modifiers: {ctx['mods']['modifier_count']} — {ctx['mods']['by_type']}")
    L.append(f"- Shape-key datablocks: {ctx['keys']['shape_key_datablocks']}")
    L.append(f"- Constraints: {ctx['cons']['total']} "
             f"(object {len(ctx['cons']['object_constraints'])}, "
             f"pose-bone {len(ctx['cons']['pose_bone_constraints'])})")
    L.append(f"- Drivers: {ctx['drv']['driver_count']} "
             f"(referencing selection/context: {ctx['drv']['drivers_referencing_selection_or_context']}); "
             f"actions: {ctx['drv']['action_count']}")
    L.append(f"- Collections: {ctx['cols']['collection_count']}; objects not in scene: "
             f"{len(ctx['cols']['objects_not_in_scene'])}; orphan objects: "
             f"{len(ctx['cols']['orphan_objects_no_collection'])}")
    L.append(f"- Materials: {ctx['mats']['material_count']}")
    L.append("")
    L.append("### Bone hierarchy (roots and components)")
    L.append("")
    for r in g["root_subtrees"]:
        L.append(f"- root `{r['root']}` — subtree {r['subtree_size']} bones")
    L.append("")
    L.append("### Bone names outside the layer convention")
    L.append("")
    L.append(f"{len(arm['nonconforming_names'])} of {arm['bone_count']}: "
             + (", ".join(f"`{n}`" for n in arm["nonconforming_names"][:60]) or "none"))
    L.append("")
    L.append("## B. Interpretation and issues (status-tagged)")
    L.append("")
    if not issues:
        L.append("No issues recorded by v01 checks.")
    for it in issues:
        L.append(f"### {it['id']} — {it['category']} ({it['verification_status']}, severity {it['severity']})")
        L.append("")
        L.append(f"- Target: {it['target']}")
        L.append(f"- Interpretation: {it['interpretation']}")
        L.append(f"- Next test: {it['proposed_next_test']}")
        L.append(f"- Potential remedy (not applied): {it['potential_remedy']}")
        L.append(f"- Risks: {it['risks_and_dependencies']}")
        L.append("")
    L.append("## C. Not measured by v01")
    L.append("")
    for u in ctx["unavailable"]:
        L.append(f"- {u}")
    L.append("")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


# ---------------------------------------------------------------- main

def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--ledger", default=None)
    args = ap.parse_args(argv)

    check_prerequisites(args)
    source = bpy.data.filepath
    source_sha_before = sha256_file(source)
    started = datetime.datetime.now(datetime.timezone.utc).isoformat()

    os.makedirs(args.out_dir, exist_ok=True)

    scene = inspect_scene()
    arm_objs = sorted((o for o in bpy.data.objects if o.type == "ARMATURE"), key=lambda o: o.name)
    arm_recs = [inspect_armature(a) for a in arm_objs]
    graphs = [build_bone_graph(a) for a in arm_recs]
    mirror_b = bone_mirror_residuals(arm_objs[0], arm_recs[0]["midline_x_world"])
    mesh_info = inspect_meshes(arm_recs[0]["midline_x_world"])

    mesh_objs = [o for o in sorted(bpy.data.objects, key=lambda x: x.name) if o.type == "MESH"]
    body_objs = [o for o in mesh_objs if is_body(o.name)]
    bone_names = [b["name"] for b in arm_recs[0]["bones"]]
    weights = inspect_weights(body_objs, bone_names)
    mods = inspect_modifiers()
    keys = inspect_shape_keys()
    cons = inspect_constraints(arm_objs)
    drv = inspect_drivers_and_actions()
    cols = inspect_collections()
    mats = inspect_materials()

    facing = facing_cross_check(arm_recs[0], mesh_info)
    issues = derive_issues(scene, arm_recs, graphs, mesh_info, weights, mods, keys, cons, drv,
                           cols, mats, mirror_b, facing)

    d = args.out_dir
    write_json(os.path.join(d, "raw_body.json"), {
        "scene": scene, "collections": cols, "materials": mats,
        "mesh_summary": {"count": mesh_info["mesh_count"], "body": mesh_info["body"]},
    })
    write_json(os.path.join(d, "bones.json"), {
        "armatures": [{k: v for k, v in a.items() if k != "bones"} for a in arm_recs],
        "bones": {a["object"]: a["bones"] for a in arm_recs},
        "bone_mirror_residuals": mirror_b,
    })
    write_json(os.path.join(d, "bone_graph.json"), {"graphs": graphs, "facing_cross_check": facing})
    write_json(os.path.join(d, "mesh.json"), mesh_info)
    write_json(os.path.join(d, "weights.json"), weights)
    write_json(os.path.join(d, "modifiers.json"), mods)
    write_json(os.path.join(d, "shape_keys.json"), keys)
    write_json(os.path.join(d, "constraints.json"), cons)
    write_json(os.path.join(d, "drivers.json"), {**drv, "materials": mats})

    unavailable = [
        "Evaluated (post-modifier, depsgraph) geometry: not measured; only original mesh data.",
        "Face-normal orientation consistency: not measured in v01.",
        "Skinning deformation under pose: not measured; posing would change pose state, out of scope.",
        "Rest-pose bone roll as a single angle: Bone.roll is not exposed; rest axes recorded instead.",
        "Material node-tree contents: not inspected.",
        "Anatomical proportions vs design brief: not measured; the brief is a reference, not evidence.",
        "Bone-to-mesh dimensional comparison against specs/*.md numbers: not performed in v01.",
    ]
    if not arm_recs[0]["bones"]:
        unavailable.append("Armature has zero bones.")

    write_json(os.path.join(d, "body_issues.json"), {
        "issue_count": len(issues),
        "status_counts": dict(Counter(i["verification_status"] for i in issues)),
        "issues": issues,
        "unavailable_measurements": unavailable,
    })

    write_markdown(os.path.join(d, "body_structure.md"), {
        "run_id": args.run_id, "source": source, "source_sha": source_sha_before,
        "blender_version": bpy.app.version_string, "scene": scene, "arm": arm_recs[0],
        "graph": graphs[0], "mesh_info": mesh_info, "weights": weights, "mods": mods,
        "keys": keys, "cons": cons, "drv": drv, "cols": cols, "mats": mats,
        "issues": issues, "unavailable": unavailable,
    })

    # Read-only guarantee: source must be byte-identical after the run.
    source_sha_after = sha256_file(source)
    if source_sha_after != source_sha_before:
        fail("source .blend hash changed during inspection; outputs are invalid")

    outputs = {}
    for name in sorted(os.listdir(d)):
        p = os.path.join(d, name)
        outputs[name] = {"sha256": sha256_file(p), "bytes": os.path.getsize(p)}

    finished = datetime.datetime.now(datetime.timezone.utc).isoformat()
    manifest = {
        "run_id": args.run_id,
        "script": "scripts/validation/humanoid_inspect_v01.py",
        "script_version": SCRIPT_VERSION,
        "script_sha256": sha256_file(os.path.realpath(__file__)) if "__file__" in globals() else None,
        "source": {"path": source, "sha256_before": source_sha_before,
                   "sha256_after": source_sha_after, "unchanged": True},
        "blender_version": bpy.app.version_string,
        "started_utc": started,
        "finished_utc": finished,
        "counts": {
            "objects": scene["object_count"], "armature_objects": len(arm_objs),
            "bones": arm_recs[0]["bone_count"], "meshes": mesh_info["mesh_count"],
            "body_meshes": mesh_info["body_mesh_count"],
            "vertices": sum(m["vertices"] for m in mesh_info["meshes"]),
            "faces": sum(m["faces"] for m in mesh_info["meshes"]),
            "modifiers": mods["modifier_count"], "shape_key_datablocks": keys["shape_key_datablocks"],
            "constraints": cons["total"], "drivers": drv["driver_count"],
            "actions": drv["action_count"], "issues": len(issues),
        },
        "outputs": outputs,
    }
    write_json(os.path.join(d, "body_manifest.json"), manifest)

    if args.ledger:
        ledger_dir = os.path.dirname(os.path.abspath(args.ledger))
        os.makedirs(ledger_dir, exist_ok=True)
        entry = {"run_id": args.run_id, "finished_utc": finished, "status": "MEASURED",
                 "source_sha256": source_sha_before, "script_version": SCRIPT_VERSION,
                 "manifest_sha256": sha256_file(os.path.join(d, "body_manifest.json")),
                 "out_dir": os.path.relpath(d, ledger_dir).replace("\\", "/")}
        with open(args.ledger, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry, sort_keys=True) + "\n")

    print("HUMANOID_INSPECT OK run_id=", args.run_id)
    print("bones=", arm_recs[0]["bone_count"], "meshes=", mesh_info["mesh_count"],
          "issues=", len(issues), "source_unchanged=", source_sha_before == source_sha_after)


main()
