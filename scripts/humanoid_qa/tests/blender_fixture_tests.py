"""Blender synthetic-fixture tests. Never touches a project .blend.

Run:
  blender --background --factory-startup --python-exit-code 1 \
      --python scripts/humanoid_qa/tests/blender_fixture_tests.py -- <results.json>

Every fixture is built in memory with analytically known geometry. B-RO-01
saves a FIXTURE (not the production asset) to a temp file to prove the
extraction path does not write to the file it reads.
"""

import json
import math
import os
import sys
import tempfile
import traceback

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))  # scripts/

from humanoid_qa.blender import experiments, extract, volume as bvol  # noqa: E402
from humanoid_qa.core import classify, evidence, geom, glb, rig, rules  # noqa: E402
from humanoid_qa.core import intersect as ix  # noqa: E402
from humanoid_qa.core import solid  # noqa: E402

CFG = rules.load_config(os.path.join(os.path.dirname(HERE), "config", "quality_rules.json"))
TOL = 1e-6
RESULTS = []

CUBE_V = [(x, y, z) for x in (0, 1) for y in (0, 1) for z in (0, 1)]
CUBE_F = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)


def close(a, b, tol=TOL):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def test(test_id, purpose):
    def deco(fn):
        def run():
            try:
                fn()
                RESULTS.append({"id": test_id, "purpose": purpose, "result": "PASS", "detail": ""})
            except AssertionError as e:
                RESULTS.append({"id": test_id, "purpose": purpose, "result": "FAIL", "detail": str(e)})
            except Exception:
                RESULTS.append({"id": test_id, "purpose": purpose, "result": "ERROR",
                                "detail": traceback.format_exc()})
        run.__name__ = fn.__name__
        return run
    return deco


def reset_scene():
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)


def mesh_obj(name, verts=CUBE_V, faces=CUBE_F, loc=(0, 0, 0), rot=(0, 0, 0), scl=(1, 1, 1)):
    me = bpy.data.meshes.new(name + "_me")
    me.from_pydata(verts, [], faces)
    me.update()
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    o.location, o.rotation_euler, o.scale = loc, rot, scl
    return o


def make_armature(name, loc=(0, 0, 0), rot=(0, 0, 0)):
    arm = bpy.data.armatures.new(name + "_data")
    o = bpy.data.objects.new(name, arm)
    bpy.context.scene.collection.objects.link(o)
    o.location, o.rotation_euler = loc, rot
    bpy.context.view_layer.objects.active = o
    bpy.ops.object.mode_set(mode="EDIT")
    r = arm.edit_bones.new("Root")
    r.head, r.tail = (0, 0, 0), (0, 0, 1)
    c = arm.edit_bones.new("Child")
    c.head, c.tail = (0, 0, 1), (1, 0, 1)
    c.parent = r
    bpy.ops.object.mode_set(mode="OBJECT")
    return o


def world_bbox(o):
    bpy.context.view_layer.update()
    return geom.bbox(extract.mesh_data(o)["world"])


@test("B-XF-01", "Extraction world vs local: unit cube under T(1,2,3) R_z(90) S(2) -> world x[-1,1] y[2,4] z[3,5]; local stays [0,1]^3")
def t_xf01():
    reset_scene()
    o = mesh_obj("Box", loc=(1, 2, 3), rot=(0, 0, math.pi / 2), scl=(2, 2, 2))
    lo, hi = world_bbox(o)
    check(close(lo, (-1, 2, 3)) and close(hi, (1, 4, 5)), f"world bbox {lo} {hi}")
    llo, lhi = geom.bbox(extract.mesh_data(o)["local"])
    check(close(llo, (0, 0, 0)) and close(lhi, (1, 1, 1)), f"local bbox {llo} {lhi}")


@test("B-XF-02", "Non-unit scale: cube scaled (1,2,3) reports object scale (1,2,3) and world dims (1,2,3); scene units read as METRIC/1.0 in factory startup")
def t_xf02():
    reset_scene()
    o = mesh_obj("Scaled", scl=(1, 2, 3))
    dims = geom.bbox_dims(world_bbox(o))
    check(close(dims, (1, 2, 3)), f"dims {dims}")
    rec = [r for r in extract.objects_info() if r["name"] == "Scaled"][0]
    check(close(rec["scale"], (1, 2, 3)), f"scale {rec['scale']}")
    s = extract.scene_info()
    check(s["unit_system"] == "METRIC" and abs(s["scale_length"] - 1.0) < TOL, f"units {s}")


@test("B-XF-03", "Bone world space: armature at (0,0,1) rotated 90 deg about Z; Child tail local (1,0,1) -> world (0,1,2); lengths 1")
def t_xf03():
    reset_scene()
    a = make_armature("Arm", loc=(0, 0, 1), rot=(0, 0, math.pi / 2))
    bpy.context.view_layer.update()
    info = extract.armature_info(a)
    b = {x["name"]: x for x in info["bones"]}
    check(close(b["Root"]["head_w"], (0, 0, 1)) and close(b["Root"]["tail_w"], (0, 0, 2)), f"root {b['Root']}")
    check(close(b["Child"]["tail_w"], (0, 1, 2)), f"child tail {b['Child']['tail_w']}")
    check(abs(b["Child"]["length"] - 1) < TOL and abs(b["Root"]["length"] - 1) < TOL, "lengths")
    check(b["Child"]["parent"] == "Root", "parent")
    # pure-core pair metric on the extracted data: a mirrored copy gives 0
    mirrored = {**b["Child"], "head_w": geom.mirror_x(b["Child"]["head_w"]),
                "tail_w": geom.mirror_x(b["Child"]["tail_w"])}
    check(rig.pair_metrics(b["Child"], mirrored, 0.0)["max_endpoint_delta"] < TOL, "pair metric")


@test("B-EVAL-01", "Base vs evaluated: unit cube + Array(count 2, rel offset X 1) -> base x-extent 1, evaluated x-extent 2")
def t_eval01():
    reset_scene()
    o = mesh_obj("Arr")
    m = o.modifiers.new("Array", "ARRAY")
    m.count = 2
    m.relative_offset_displace = (1, 0, 0)
    bpy.context.view_layer.update()
    base = geom.bbox_dims(world_bbox(o))
    ev = extract.evaluated_bounds([o])
    ev_dx = ev["max"][0] - ev["min"][0]
    check(abs(base[0] - 1) < TOL, f"base {base}")
    check(abs(ev_dx - 2) < TOL, f"evaluated {ev_dx}")
    check(len(o.data.vertices) == 8, "base mesh untouched by evaluation")


@test("B-WGT-01", "Weights: groups A (verts 0,1 at 1.0) and B (vert 2 at 0.0) -> nonzero A=2, B=0, unweighted=2, max influences 1")
def t_wgt01():
    reset_scene()
    a = make_armature("Arm")
    o = mesh_obj("Plane", verts=[(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)], faces=[(0, 1, 2, 3)])
    ga, gb = o.vertex_groups.new(name="Root"), o.vertex_groups.new(name="Child")
    ga.add([0, 1], 1.0, "REPLACE")
    gb.add([2], 0.0, "REPLACE")
    md = o.modifiers.new("Armature", "ARMATURE")
    md.object = a
    d = extract.mesh_data(o)
    check(d["vgroup_nonzero"] == {"Root": 2, "Child": 0}, f"nonzero {d['vgroup_nonzero']}")
    check(d["unweighted_vertices"] == 2 and d["max_influences"] == 1, f"{d['unweighted_vertices']} {d['max_influences']}")
    s = rig.skinning_layers(extract.armature_info(a)["bones"], [d], "Arm")
    row = {r["bone"]: r for r in s["bones"]}
    check(row["Root"]["5_eligible_to_deform"] and not row["Child"]["5_eligible_to_deform"], "eligibility")


@test("B-WGT-02", "Modifier integrity: Armature modifier with no object reports target None; mesh is then not bound")
def t_wgt02():
    reset_scene()
    o = mesh_obj("Unbound")
    o.modifiers.new("Armature", "ARMATURE")
    d = extract.mesh_data(o)
    check(d["armature_mods"][0]["target"] is None, f"{d['armature_mods']}")
    rec = [r for r in extract.objects_info() if r["name"] == "Unbound"][0]
    check(rec["modifiers"][0]["type"] == "ARMATURE" and rec["modifiers"][0]["target"] is None, "objects_info")


@test("B-CON-01", "Constraints: COPY_ROTATION to a missing subtarget and COPY_LOCATION with no target are both reported; a valid one is not")
def t_con01():
    reset_scene()
    a = make_armature("Arm")
    pb = a.pose.bones["Child"]
    c1 = pb.constraints.new("COPY_ROTATION")
    c1.target, c1.subtarget = a, "Missing"
    pb.constraints.new("COPY_LOCATION")
    c3 = a.pose.bones["Root"].constraints.new("COPY_ROTATION")
    c3.target, c3.subtarget = a, "Child"
    info = extract.armature_info(a)
    objs = {o["name"]: o["type"] for o in extract.objects_info()}
    probs = rig.constraint_reference_check(info["pose_constraints"], objs,
                                           {"Arm": {b["name"] for b in info["bones"]}})
    got = sorted((p["owner"], p["type"], p["problem"]) for p in probs)
    exp = sorted([("Child", "COPY_ROTATION", "subtarget bone does not exist"),
                  ("Child", "COPY_LOCATION", "missing target")])
    check(got == exp, f"{got}")


@test("B-CLS-01", "Classification: bound 411_ -> body; ENV_/Studio_ -> environment; CINE_ -> presentation; hidden 411_ -> excluded_backup; unbound 411_ -> unresolved")
def t_cls01():
    reset_scene()
    a = make_armature(CFG["classification"]["armature_object"])
    body = mesh_obj("411_Body")
    body.modifiers.new("Armature", "ARMATURE").object = a
    mesh_obj("ENV_Crate")
    mesh_obj("Studio_Floor")
    mesh_obj("CINE_Vol")
    bk = mesh_obj("411_Visor_Patch_backup")
    bk.hide_render = True
    mesh_obj("411_Loose")
    got = {k: v["class"] for k, v in classify.classify_all(extract.objects_info(), CFG).items()}
    exp = {"411_Body": "body", "ENV_Crate": "environment", "Studio_Floor": "environment",
           "CINE_Vol": "presentation", "411_Visor_Patch_backup": "excluded_backup",
           "411_Loose": "unresolved"}
    check(got == exp, f"{got}")


@test("B-RO-01", "Read-only: a saved FIXTURE .blend keeps the same SHA-256 after reopening and running every extraction function")
def t_ro01():
    reset_scene()
    a = make_armature("Arm")
    o = mesh_obj("411_Body")
    o.modifiers.new("Armature", "ARMATURE").object = a
    d = tempfile.mkdtemp(prefix="hqa_fixture_")
    path = os.path.join(d, "fixture.blend")
    bpy.ops.wm.save_as_mainfile(filepath=path, copy=True)
    before = evidence.sha256_file(path)
    bpy.ops.wm.open_mainfile(filepath=path)
    extract.scene_info()
    objs = extract.objects_info()
    for ob in bpy.data.objects:
        if ob.type == "MESH":
            extract.mesh_data(ob)
    extract.evaluated_bounds([ob for ob in bpy.data.objects if ob.type == "MESH"])
    extract.armature_info(bpy.data.objects["Arm"])
    after = evidence.sha256_file(path)
    check(before == after, f"{before} != {after}")
    check(len(objs) == 2, "reopened fixture content")


def skinned_fixture():
    """Armature Root (0,0,0)->(0,0,1) deform; plane in XZ weighted 1.0 to Root."""
    reset_scene()
    a = make_armature("Arm")
    v = [(1, 0, 0), (1, 0, 1), (2, 0, 1), (2, 0, 0)]
    o = mesh_obj("411_Plane", verts=v, faces=[(0, 1, 2, 3)])
    g = o.vertex_groups.new(name="Root")
    g.add([0, 1, 2, 3], 1.0, "REPLACE")
    o.modifiers.new("Armature", "ARMATURE").object = a
    bpy.context.view_layer.update()
    return a, o


@test("B-DEF-01", "Pose harness analytic: rotating a Z-pointing bone 90 deg about its own axis (local Y) moves plane vertex (1,0,0.5)-> (0,1,0.5); every vertex lands at (-y,x,z)")
def t_def01():
    a, o = skinned_fixture()
    rest = experiments.snapshot_eval([o])
    mats = experiments.pose_matrices(a)
    r = experiments.pose_test(a, "Root", (0.0, math.pi / 2, 0.0), [o], rest, mats)
    # pose_test restores before returning; re-pose manually to read positions
    pb = a.pose.bones["Root"]
    pb.rotation_mode = "XYZ"
    pb.rotation_euler = (0.0, math.pi / 2, 0.0)
    bpy.context.view_layer.update()
    got = experiments.snapshot_eval([o])["411_Plane"]
    exp = [(-y, x, z) for x, y, z in rest["411_Plane"]]
    check(all(close(g, e, 1e-5) for g, e in zip(got, exp)), f"{got} vs {exp}")
    check("411_Plane" in r["moved_meshes"] and abs(r["moved_meshes"]["411_Plane"] - math.dist((2, 0, 1), (0, 2, 1))) < 1e-5,
          f"max displacement {r['moved_meshes']}")
    check("Root" in r["moved_bones"] and "Child" in r["moved_bones"], f"moved bones {r['moved_bones']}")


@test("B-DEF-02", "Restore: after the pose test the evaluated mesh equals the rest snapshot (error <= 1e-6) and pose_test reports restore_verified; an unbound (unweighted) mesh does not move")
def t_def02():
    a, o = skinned_fixture()
    free = mesh_obj("411_Free")                       # no vertex group, no modifier
    bpy.context.view_layer.update()
    rest = experiments.snapshot_eval([o, free])
    mats = experiments.pose_matrices(a)
    r = experiments.pose_test(a, "Root", (0.0, math.pi / 2, 0.0), [o, free], rest, mats)
    check(r["restore_verified"] and r["restore_max_error_m"] <= 1e-6, f"restore {r['restore_max_error_m']}")
    check("411_Free" not in r["moved_meshes"], "unbound mesh moved")
    after = experiments.snapshot_eval([o, free])
    check(experiments.displacement(rest, after)["411_Plane"] <= 1e-6, "plane not restored")


@test("B-EXP-01", "Export probe: fixture skeleton Root(deform), Child(deform), Helper(non-deform, child of Root): all-bones GLB joints = {Root,Child,Helper}; deform-only GLB joints = {Root,Child}")
def t_exp01():
    a, o = skinned_fixture()
    bpy.context.view_layer.objects.active = a
    bpy.ops.object.mode_set(mode="EDIT")
    h = a.data.edit_bones.new("Helper")
    h.head, h.tail = (0, 0, 1), (0, 0, 2)
    h.parent = a.data.edit_bones["Root"]
    h.use_deform = False
    bpy.ops.object.mode_set(mode="OBJECT")
    out = experiments.export_probe(a, [o], tempfile.mkdtemp(prefix="hqa_exp_fix_"))
    for k in ("all_bones", "deform_bones_only"):
        check("error" not in out[k], f"{k}: {out[k].get('error')}")
    allj = set(out["all_bones"]["summary"]["skins"][0]["joints"])
    defj = set(out["deform_bones_only"]["summary"]["skins"][0]["joints"])
    check(allj == {"Root", "Child", "Helper"}, f"all {allj}")
    check(defj == {"Root", "Child"}, f"deform-only {defj}")


@test("B-EXP-02", "Export probe with a HIDDEN armature (select_set would silently fail): probe unhides it in the copy, records the note, and still exports a skin with joints {Root,Child}")
def t_exp02():
    a, o = skinned_fixture()
    a.hide_set(True)
    check(a.hide_get(), "fixture armature should be hidden")
    out = experiments.export_probe(a, [o], tempfile.mkdtemp(prefix="hqa_exp_fix_"))
    check("error" not in out, f"{out.get('error')}")
    check(any("unhid" in n for n in out["_notes"]), f"notes {out['_notes']}")
    js = set(out["all_bones"]["summary"]["skins"][0]["joints"])
    check(js == {"Root", "Child"}, f"joints {js}")


@test("B-NRM-01", "Blender recalc cross-check: outward cube flips 0 faces; one reversed face flips 1; fully reversed cube flips 6")
def t_nrm01():
    reset_scene()
    good = mesh_obj("Good")
    one = mesh_obj("One", faces=[tuple(reversed(CUBE_F[0]))] + list(CUBE_F[1:]))
    allr = mesh_obj("AllRev", faces=[tuple(reversed(f)) for f in CUBE_F])
    got = (extract.recalc_flip_count(good), extract.recalc_flip_count(one), extract.recalc_flip_count(allr))
    check(got == (0, 1, 6), f"{got}")


ITOL = ix.Tol(plane=1e-6, contact=1e-6, area=1e-12)      # explicit, pinned by U-INT-CFG in the pure tests
F32 = 1e-6                                               # float32 world coordinates at ~1 m


def tri_mesh_from(o):
    d = extract.mesh_tris(o)
    return ix.TriMesh(o.name, d["world"], d["tris"], d["poly"], 1e-6, ITOL), d


@test("B-INT-01", "Tessellation + transform: a quad-faced cube T(3,0,0) S(2) tessellates to 12 triangles (2 per polygon) in world space, and its exact distance to the unit cube at the origin is 2.0 (x 1 -> 3)")
def t_int01():
    reset_scene()
    a = mesh_obj("A")
    b = mesh_obj("B", loc=(3, 0, 0), scl=(2, 2, 2))
    bpy.context.view_layer.update()
    ma, da = tri_mesh_from(a)
    mb, db = tri_mesh_from(b)
    check(len(db["tris"]) == 12 and sorted(db["poly"]) == sorted(list(range(6)) * 2), f"tris {len(db['tris'])} poly {db['poly']}")
    lo, hi = geom.bbox(db["world"])
    check(close(lo, (3, 0, 0)) and close(hi, (5, 2, 2)), f"world bbox {lo} {hi}")
    pm = ix.mesh_proximity(ma, mb, math.inf)
    check(abs(pm["distance"] - 2.0) < F32, f"distance {pm['distance']}")
    check(ix.pair_status(ix.cross_intersections(ma, mb, ITOL, 10**6), len(ma.active), len(mb.active)) == "NONE_DETECTED", "status")


@test("B-INT-02", "Independent implementation cross-check (two meshes): cubes offset (0.5,0.4,0.3): the core proper-crossing pair set equals the set Blender's BVHTree.overlap reports, and the exact rational oracle confirms every pair")
def t_int02():
    reset_scene()
    a = mesh_obj("A")
    b = mesh_obj("B", loc=(0.5, 0.4, 0.3))
    bpy.context.view_layer.update()
    ma, da = tri_mesh_from(a)
    mb, db = tri_mesh_from(b)
    pure = ix.cross_intersections(ma, mb, ITOL, 10**6)
    pure_set = {(d["a"]["tri"], d["b"]["tri"]) for d in pure["definite"]}
    check(len(pure_set) > 0 and pure["counts"]["intersect"] > 0, f"pure found {pure['counts']}")
    bl = extract.blender_bvh_overlap(da["world"], da["tris"], db["world"], db["tris"])
    check(set(pure["intersect_pairs_all"]) == set(bl), f"core {len(pure['intersect_pairs_all'])} vs blender {len(bl)}")
    check(all(ix.exact_proper_crossing(ma.corners[i], mb.corners[j]) for i, j in pure["intersect_pairs_all"]), "exact oracle disagrees")


@test("B-INT-03", "Independent implementation cross-check (self): cube + spike triangle built in Blender: exactly 2 definite self-intersections, both involving the spike triangle, both reported by Blender's BVHTree self-overlap; a plain cube has 0")
def t_int03():
    reset_scene()
    cube_o = mesh_obj("Plain")
    v = list(CUBE_V) + [(0.3, 0.6, -0.5), (0.3, 0.6, 1.5), (0.7, 0.6, 0.5)]
    f = list(CUBE_F) + [(8, 9, 10)]
    spiked = mesh_obj("Spiked", verts=v, faces=f)
    bpy.context.view_layer.update()
    mp, _ = tri_mesh_from(cube_o)
    check(ix.self_intersections(mp, ITOL, 10**6)["definite_count"] == 0, "plain cube reported intersections")
    ms, ds = tri_mesh_from(spiked)
    r = ix.self_intersections(ms, ITOL, 10**6)
    spike_tri = len(ds["tris"]) - 1
    check(r["definite_count"] == 2, f"definite {r['definite_count']} counts {r['counts']}")
    check(all(spike_tri in (d["a"]["tri"], d["b"]["tri"]) for d in r["definite"]), "pairs must involve the spike triangle")
    bl = set(extract.blender_bvh_self_overlap(ds["world"], ds["tris"]))
    got = {(d["a"]["tri"], d["b"]["tri"]) for d in r["definite"]}
    check(got <= bl, f"missing from Blender self candidates: {sorted(got - bl)}")


@test("B-INT-04", "Blender BVHTree.overlap semantics (documented assumption pinned): two triangles whose bounding boxes overlap but whose surfaces are apart are NOT reported; a genuinely crossing pair is")
def t_int04():
    from mathutils.bvhtree import BVHTree
    A = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
    apart = [(0.9, 0.9, -0.1), (1.0, 1.0, 0.1), (1.0, 0.95, 0.0)]            # boxes overlap, triangles apart (x+y>1 side)
    cross = [(0.2, 0.2, -0.5), (0.2, 0.2, 0.5), (0.6, 0.2, 0.0)]
    check(ix.box_overlap(ix.box_of(A), ix.box_of(apart)), "fixture must have overlapping boxes")
    check(ix.classify_triangle_pair(tuple(A), tuple(apart), ITOL)["kind"] == "disjoint", "fixture must be disjoint")
    ta = BVHTree.FromPolygons(A, [(0, 1, 2)], all_triangles=True, epsilon=0.0)
    got_apart = ta.overlap(BVHTree.FromPolygons(apart, [(0, 1, 2)], all_triangles=True, epsilon=0.0))
    got_cross = ta.overlap(BVHTree.FromPolygons(cross, [(0, 1, 2)], all_triangles=True, epsilon=0.0))
    check(list(got_apart) == [], f"apart reported {got_apart}")
    check(list(got_cross) == [(0, 0)], f"cross reported {got_cross}")


# ---------------------------------------------------------------- B-VOL: solid volume + axial depth
# Analytic ground truth for every case is derived from the fixture geometry in each docstring.
VOL_PARAMS = {"null_volume": 1e-12, "degenerate_area": 1e-12, "iters": 18, "agree_rel": 1e-4, "identity_rel": 1e-4,
              "axial_scan_samples": 32, "tri_contact": 1e-6, "tri_plane_coplanar": 1e-6,
              "coincident_vertex": 1e-6, "refine_edge_tol": 1e-6}
VOL_PARAMS_LEGACY = {k: v for k, v in VOL_PARAMS.items()
                     if k not in ("axial_scan_samples", "tri_contact", "tri_plane_coplanar", "coincident_vertex")}
VOL_TOL = 1e-9


def box_mesh(lo, hi, reverse=False):
    """Closed box with welded vertex indices; outward wound unless reverse=True."""
    verts, index, tris = [], {}, []

    def v(p):
        k = tuple(round(x, 12) for x in p)
        if k not in index:
            index[k] = len(verts)
            verts.append(tuple(float(x) for x in p))
        return index[k]
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    quads = [((x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)),
             ((x1, y0, z0), (x1, y0, z1), (x1, y1, z1), (x1, y1, z0)),
             ((x0, y0, z0), (x0, y0, z1), (x1, y0, z1), (x1, y0, z0)),
             ((x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)),
             ((x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)),
             ((x0, y0, z1), (x0, y1, z1), (x1, y1, z1), (x1, y0, z1))]
    for q in quads:
        if not reverse:
            q = tuple(reversed(q))
        a, b, c, d = (v(p) for p in q)
        tris += [(a, b, c), (a, c, d)]
    return verts, tris


def run_pair(ca, ta, cb, tb, params=None):
    return solid.measure_pair(ca, ta, cb, tb, bvol.BooleanBackend.factory("fixture"), params or VOL_PARAMS)


def merge_meshes(*meshes):
    """Concatenate (coords, tris) meshes into one multi-component mesh."""
    coords, tris = [], []
    for c, t in meshes:
        off = len(coords)
        coords += list(c)
        tris += [(a + off, b + off, c_ + off) for a, b, c_ in t]
    return coords, tris


# Float32 kernel: vertex positions near 1 m carry ~1.2e-7 m error; over ~1 m^2 of boundary this is ~1e-7 m^3.
# Exactly representable fixtures keep the strict VOL_TOL; rotated (inexact) fixtures use F32_VOL_TOL.
F32_VOL_TOL = 1e-6


def check_volume(r, expect, label, tol=VOL_TOL):
    check(r["status"] in ("VOLUME_VALIDATED", "VOLUME_MEASURED"), f"{label}: status {r['status']} {r.get('reasons')}")
    check(abs(r["overlap_volume_m3"] - expect) <= tol, f"{label}: volume {r['overlap_volume_m3']} expected {expect}")


@test("B-VOL-01", "Boolean overlap volume of two cubes with known overlap: A=[0,1]^3, B=A+(0.5,0.25,0.125): V = 0.328125 m^3 with both kernels; solver agreement and the identity V(AnB)+V(A-B)=V(A) validated")
def t_vol01():
    ca, ta = box_mesh((0, 0, 0), (1, 1, 1))
    cb, tb = box_mesh((0.5, 0.25, 0.125), (1.5, 1.25, 1.125))
    r = run_pair(ca, ta, cb, tb)
    check(r["status"] == "VOLUME_VALIDATED", f"status {r['status']} {r.get('solver_agreement')} {r.get('identity')}")
    check_volume(r, 0.328125, "B-VOL-01")
    for s_, v in r["volumes"]["AB"].items():
        check(abs(v - 0.328125) <= VOL_TOL, f"kernel {s_} volume {v}")
    check(r["identity"]["ok"], f"identity {r['identity']}")


@test("B-VOL-02", "Zero overlap, four cases: separated (gap 0.5), face contact, edge contact, vertex contact: every kernel reports |V| <= 1e-12 m^3 and status NULL_OVERLAP (a verdict, not UNABLE)")
def t_vol02():
    ca, ta = box_mesh((0, 0, 0), (1, 1, 1))
    for s_ in ((1.5, 0, 0), (1, 0, 0), (1, 1, 0), (1, 1, 1)):
        cb, tb = box_mesh(s_, (s_[0] + 1, s_[1] + 1, s_[2] + 1))
        r = run_pair(ca, ta, cb, tb)
        check(r["status"] == "NULL_OVERLAP", f"shift {s_}: status {r['status']} {r.get('volumes')}")
        for k, v in r["volumes"]["AB"].items():
            check(abs(v) <= 1e-12, f"shift {s_} kernel {k} residual volume {v}")


@test("B-VOL-03", "Contained solid: B=[0.25,0.75]^3 inside A=[0,1]^3: V = 0.125 m^3 (validated)")
def t_vol03():
    ca, ta = box_mesh((0, 0, 0), (1, 1, 1))
    cb, tb = box_mesh((0.25, 0.25, 0.25), (0.75, 0.75, 0.75))
    r = run_pair(ca, ta, cb, tb)
    check(r["status"] == "VOLUME_VALIDATED", f"status {r['status']}")
    check_volume(r, 0.125, "B-VOL-03")


@test("B-VOL-04", "Rotated overlapping boxes: unit cube and the same cube rotated 45 deg about z through (0.5,0.5): V = 2*sqrt(2)-2 = 0.8284271247 m^3 with both kernels")
def t_vol04():
    ca, ta = box_mesh((0, 0, 0), (1, 1, 1))
    cb, tb = box_mesh((0, 0, 0), (1, 1, 1))
    c, s_ = math.cos(math.radians(45)), math.sin(math.radians(45))
    cb = [(0.5 + c * (x - 0.5) - s_ * (y - 0.5), 0.5 + s_ * (x - 0.5) + c * (y - 0.5), z) for x, y, z in cb]
    r = run_pair(ca, ta, cb, tb)
    check(r["status"] == "VOLUME_VALIDATED", f"status {r['status']} {r.get('solver_agreement')} {r.get('identity')}")
    check_volume(r, 2 * math.sqrt(2) - 2, "B-VOL-04", tol=F32_VOL_TOL)


@test("B-VOL-05", "Reversed shell: cube A wound inward (volume -1) is reversed on a copy and measured; B=A+(0.5,0.5,0.5): V = 0.125 m^3; the reversal is reported and the input is not modified")
def t_vol05():
    ca, ta = box_mesh((0, 0, 0), (1, 1, 1), reverse=True)
    cb, tb = box_mesh((0.5, 0.5, 0.5), (1.5, 1.5, 1.5))
    before = list(ta)
    r = run_pair(ca, ta, cb, tb)
    check(r["status"] == "VOLUME_VALIDATED", f"status {r['status']} {r.get('reasons')}")
    check_volume(r, 0.125, "B-VOL-05")
    check(r["orientation"]["A"]["reversed_components"] == [0], f"orientation {r['orientation']['A']}")
    check(list(ta) == before, "input triangles modified")


@test("B-VOL-06", "Non-convex operand: L-prism (footprint area 3, height 1, volume 3 m^3) against cube [0.5,1.5]x[0.5,1.5]x[0.25,1.25]: footprint overlap 0.75 m^2 x height overlap 0.75 m = 0.5625 m^3 (validated)")
def t_vol06():
    P = [(0, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2)]
    verts, index, tris = [], {}, []

    def v(p):
        k = tuple(round(x, 12) for x in p)
        if k not in index:
            index[k] = len(verts)
            verts.append(tuple(float(x) for x in p))
        return index[k]
    bot = [v((x, y, 0.0)) for x, y in P]
    top = [v((x, y, 1.0)) for x, y in P]
    for a, b, c in ((0, 1, 2), (0, 2, 3), (0, 3, 5), (3, 4, 5)):
        tris += [(bot[a], bot[c], bot[b]), (top[a], top[b], top[c])]
    for i in range(6):
        j = (i + 1) % 6
        tris += [(bot[i], bot[j], top[j]), (bot[i], top[j], top[i])]
    check(abs(geom.signed_volume(verts, tris) - 3.0) < 1e-12, f"L volume {geom.signed_volume(verts, tris)}")
    cb, tb = box_mesh((0.5, 0.5, 0.25), (1.5, 1.5, 1.25))
    r = run_pair(verts, tris, cb, tb)
    check(r["status"] == "VOLUME_VALIDATED", f"status {r['status']} {r.get('solver_agreement')} {r.get('identity')}")
    check_volume(r, 0.5625, "B-VOL-06")


@test("B-VOL-07", "Multi-component operand: A = unit cube plus a disjoint unit cube at x=3 wound inward (second component reversed); B=A+(0.5,0.5,0.5) cube: V = 0.125 m^3; only the inward component is reversed")
def t_vol07():
    c1, t1 = box_mesh((0, 0, 0), (1, 1, 1))
    c2, t2 = box_mesh((3, 0, 0), (4, 1, 1), reverse=True)
    coords = c1 + c2
    tris = t1 + [(a + len(c1), b + len(c1), c + len(c1)) for a, b, c in t2]
    cb, tb = box_mesh((0.5, 0.5, 0.5), (1.5, 1.5, 1.5))
    r = run_pair(coords, tris, cb, tb)
    check(r["status"] == "VOLUME_VALIDATED", f"status {r['status']} {r.get('reasons')}")
    check_volume(r, 0.125, "B-VOL-07")
    check(sorted(r["orientation"]["A"]["reversed_components"]) == [1], f"orientation {r['orientation']['A']}")


@test("B-VOL-08", "Axial separation bound on cubes A=[0,1]^3, B=A+(0.5,0.25,0.125): verified t* = +X 1.5, -X 0.5, +Y 1.25, -Y 0.75, +Z 1.125, -Z 0.875 m (each within 1e-5 m); axial separation bound = 0.5 m, direction -X")
def t_vol08():
    ca, ta = box_mesh((0, 0, 0), (1, 1, 1))
    cb, tb = box_mesh((0.5, 0.25, 0.125), (1.5, 1.25, 1.125))
    r = run_pair(ca, ta, cb, tb)
    check(r["status"] == "VOLUME_VALIDATED", f"status {r['status']}")
    expect = {"+X": 1.5, "-X": 0.5, "+Y": 1.25, "-Y": 0.75, "+Z": 1.125, "-Z": 0.875}
    for k, v in expect.items():
        ax = r["axial"][k]
        check(ax["status"] == "VERIFIED" and abs(ax["t"] - v) <= 1e-5, f"{k}: {ax}")
    check(r["separation_direction"] == "-X" and abs(r["separation_bound_m"] - 0.5) <= 1e-5,
          f"separation {r['separation_bound_m']} {r['separation_direction']}")
    check(r.get("separation_cross_check_ok") is True, f"cross-check {r.get('separation_cross_check')}")


@test("B-VOL-09", "Refusal and cleanup: an open cube (one face missing) is refused before any kernel runs; after the measurements above no __hqa object, mesh or collection remains in bpy.data")
def t_vol09():
    ca, ta = box_mesh((0, 0, 0), (1, 1, 1))
    cb, tb = box_mesh((0.5, 0.5, 0.5), (1.5, 1.5, 1.5))
    r = run_pair(ca, ta[2:], cb, tb)
    check(r["status"] == "UNABLE_TO_DETERMINE" and "OPEN_BOUNDARY" in r["reasons"]["A"],
          f"open cube: {r['status']} {r['reasons']}")
    check(r["volumes"] == {}, "a refused pair must not record any volume")
    for label, coll in (("objects", bpy.data.objects), ("meshes", bpy.data.meshes), ("collections", bpy.data.collections)):
        left = [x.name for x in coll if x.name.startswith("__hqa")]
        check(not left, f"temporary {label} left behind: {left}")


@test("B-VOL-10", "Disconnected target, non-monotone overlap: A=[0,1]^3; B = box [0.5,1.5]x[0,1]x[0,1] plus a separate box [3,6]x[0,1]x[0,1]. Moving A along +X the overlap ends at t=1.5, is absent for 1.5<=t<=2, and returns for 2<t<6. The scanned search returns 1.5 (within 2e-6) and flags the reappearance; the legacy single bisection returns the last transition 6.0 (within 1e-4), a valid but non-minimal separation. Overlap volume 0.5 m^3 (validated); the six-axis minimum is -X 0.5 m")
def t_vol10():
    ca, ta = box_mesh((0, 0, 0), (1, 1, 1))
    cb, tb = merge_meshes(box_mesh((0.5, 0, 0), (1.5, 1, 1)), box_mesh((3, 0, 0), (6, 1, 1)))
    r = run_pair(ca, ta, cb, tb)
    check_volume(r, 0.5, "B-VOL-10")
    ax = r["axial"]["+X"]
    check(ax["status"] == "VERIFIED" and abs(ax["t"] - 1.5) <= 2e-6, f"scanned +X {ax}")
    check(ax["scan"]["overlap_reappears_after_first_null"] is True, f"reappearance not flagged: {ax['scan']}")
    check("+X" in r["axial_nonmonotone_directions"], f"nonmonotone list {r['axial_nonmonotone_directions']}")
    old = run_pair(ca, ta, cb, tb, VOL_PARAMS_LEGACY)["axial"]["+X"]
    check(old["status"] == "VERIFIED" and abs(old["t"] - 6.0) <= 1e-4, f"legacy +X {old}")
    check(ax["t"] < old["t"], "scanned value must not exceed the legacy value on this fixture")
    check(abs(r["separation_bound_m"] - 0.5) <= 1e-5 and r["separation_direction"] == "-X",
          f"bound {r['separation_bound_m']} {r['separation_direction']}")


def pyramid_mesh():
    """Square pyramid, apex (0,0,-0.5), base corners (+-1,+-1,0.5) (45 degree faces), outward winding."""
    from humanoid_qa.tests.test_refine import _orient
    coords = [(0.0, 0.0, -0.5), (-1.0, -1.0, 0.5), (1.0, -1.0, 0.5), (1.0, 1.0, 0.5), (-1.0, 1.0, 0.5)]
    polys = [(1, 2, 3, 4), (0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 1)]
    return coords, _orient(coords, polys, centre=(0.0, 0.0, 0.0))


@test("B-VOL-11", "Sharp-tip penetration: pyramid (apex down, 45 degree faces, V(h)=4h^3/3 for tip depth h) against slab [-2,2]x[-2,2]x[-1,0]; true separating translation along +Z is 0.5 m. A null-volume test alone (1e-12 m^3) stops 9.09e-5 m short (at most 0.5-3e-5); with the surface-crossing certificate the returned +Z value is in [0.5-3e-6, 0.5+2e-6] and has zero crossing pairs")
def t_vol11():
    ca, ta = pyramid_mesh()
    cb, tb = box_mesh((-2, -2, -1), (2, 2, 0))
    r0 = run_pair(ca, ta, cb, tb, VOL_PARAMS_LEGACY)
    t0 = r0["axial"]["+Z"]["t"]
    check(t0 <= 0.5 - 3e-5, f"volume-only +Z {t0} should fall short of 0.5 by ~9e-5")
    r1 = run_pair(ca, ta, cb, tb)
    a = r1["axial"]["+Z"]
    check(a["status"] == "VERIFIED" and 0.5 - 3e-6 <= a["t"] <= 0.5 + 2e-6, f"certified +Z {a}")
    check(a["certificate"]["crossing_pairs"] == 0 and a["certificate"]["adjustments"] > 0, f"certificate {a['certificate']}")


@test("B-VOL-12", "Axial bound is not the minimum translation distance: unit cube against a slab with face x+y = 2-sqrt(2)/4 (corner sunk 0.25 m along the normal, MTD = 0.25 m). Kernel-measured bound = sqrt(2)/4 = 0.353553 m within 3e-6 (direction -X or -Y), i.e. sqrt(2) times MTD within 1e-4")
def t_vol12():
    from humanoid_qa.tests.test_refine import plain_cube
    s2 = math.sqrt(2.0)
    c0 = 2.0 - s2 / 4.0
    ub, tb = plain_cube((c0 / s2, -3.0, -2.0), (c0 / s2 + 3.0, 3.0, 3.0))
    cb = [((u - v) / s2, (u + v) / s2, z) for u, v, z in ub]
    ca, ta = box_mesh((0, 0, 0), (1, 1, 1))
    r = run_pair(ca, ta, cb, tb)
    check(r["status"] in ("VOLUME_VALIDATED", "VOLUME_MEASURED"), f"status {r['status']} {r.get('reasons')}")
    b = r["separation_bound_m"]
    check(abs(b - s2 / 4.0) <= 3e-6 and r["separation_direction"] in ("-X", "-Y"), f"bound {b} {r['separation_direction']}")
    check(b >= 0.25 and abs(b / 0.25 - s2) <= 1e-4, f"bound/MTD {b / 0.25}")


@test("B-REF-01", "Refined copy, kernel-backed: the sliver cube (unit cube whose bottom-front edge is doubled by 1e-13 m; 4 degenerate triangles; gate reason DEGENERATE_FACE only) against B=[0.5,1.5]x[0.25,1.25]x[0.125,1.125]: overlap 0.328125 m^3 with both kernels, VOLUME_VALIDATED, geometry_basis REFINED_COPY:A=NUMERICALLY_EQUIVALENT; the unrefined shell through the same kernels gives the same volume (refinement_effect ok); the six axial separations equal the box face distances within 1e-5 m; without refinement enabled the pair stays UNABLE_TO_DETERMINE")
def t_ref01():
    from humanoid_qa.tests.test_refine import plain_cube, sliver_cube
    ca, ta = sliver_cube()
    cb, tb = plain_cube((0.5, 0.25, 0.125), (1.5, 1.25, 1.125))
    r = run_pair(ca, ta, cb, tb)
    check_volume(r, 0.328125, "B-REF-01")
    check(r["status"] == "VOLUME_VALIDATED", f"status {r['status']}")
    check(r["geometry_basis"] == "REFINED_COPY:A=NUMERICALLY_EQUIVALENT", r["geometry_basis"])
    eff = r["refinement_effect"]
    check(eff["status"] == "COMPARED" and eff["ok"], f"effect {eff}")
    for k, v in {"+X": 1.5, "-X": 0.5, "+Y": 1.25, "-Y": 0.75, "+Z": 1.125, "-Z": 0.875}.items():
        check(abs(r["axial"][k]["t"] - v) <= 1e-5, f"{k}: {r['axial'][k]['t']}")
    legacy = run_pair(ca, ta, cb, tb, {k: v for k, v in VOL_PARAMS.items() if k != "refine_edge_tol"})
    check(legacy["status"] == "UNABLE_TO_DETERMINE" and legacy["reasons"]["A"] == ["DEGENERATE_FACE"],
          f"without refinement the pair stays refused: {legacy['status']} {legacy['reasons']}")


@test("B-REF-02", "Refinement leaves no residue: after measuring a refined pair (which opens a second, unrefined backend for the concordance check) no __hqa object, mesh or collection remains, and the sliver-cube input lists are unchanged")
def t_ref02():
    from humanoid_qa.tests.test_refine import plain_cube, sliver_cube
    ca, ta = sliver_cube()
    c0, t0 = list(ca), list(ta)
    cb, tb = plain_cube((0.5, 0.25, 0.125), (1.5, 1.25, 1.125))
    before = (len(bpy.data.objects), len(bpy.data.meshes), len(bpy.data.collections))
    run_pair(ca, ta, cb, tb)
    check((ca, ta) == (c0, t0), "inputs were modified")
    left = [x.name for coll in (bpy.data.objects, bpy.data.meshes, bpy.data.collections) for x in coll if x.name.startswith("__hqa")]
    check(not left, f"temporary data left behind: {left}")
    check((len(bpy.data.objects), len(bpy.data.meshes), len(bpy.data.collections)) == before, "datablock counts changed")


def main():
    out = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else None
    for t in (t_xf01, t_xf02, t_xf03, t_eval01, t_wgt01, t_wgt02, t_con01, t_cls01, t_ro01,
              t_def01, t_def02, t_exp01, t_exp02, t_nrm01, t_int01, t_int02, t_int03, t_int04,
              t_vol01, t_vol02, t_vol03, t_vol04, t_vol05, t_vol06, t_vol07, t_vol08, t_vol09,
              t_vol10, t_vol11, t_vol12, t_ref01, t_ref02):
        t()
    for r in RESULTS:
        print(r["id"], r["result"], r["detail"][:300])
    doc = {"suite": "blender_fixture", "blender": bpy.app.version_string,
           "python": sys.version.split()[0], "tests": RESULTS}
    if out:
        with open(out, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=2)
    if any(r["result"] != "PASS" for r in RESULTS):
        raise SystemExit(1)


main()
