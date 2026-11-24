"""
run_qa.py  (humanoid_qa_v01.0)

Purpose: read-only QA run over a 411 Humanoid .blend. Stages:
  0  provenance + integrity (full SHA-256 before/after, fail loudly on change)
  1  facing direction + side-mapping table (decision package if unresolved)
  2  measurement-method qualification from the test-result files
  3  geometry: inventory/classification, dimensions (base vs evaluated),
     topology, duplicates, symmetry
  4  rig: graph, naming, side pairs, mirror metrics, MECH/DEF co-location,
     skinning conditions 1-4, constraint/modifier references
  5  findings with evidence class, rule category, severity
  6  optional diagnostic renders (in memory, after all measurement)

Run (see run_qa.ps1 for the full pipeline incl. tests):
  blender --background --factory-startup --disable-autoexec <file.blend> ^
    --python-exit-code 1 --python scripts/humanoid_qa/run_qa.py -- ^
    --out-dir <run_dir> --run-id <id> --ledger blender/inspection/evidence_ledger.jsonl ^
    --unit-results <u.json> --fixture-results <b.json> [--no-render]

Never saves. Refuses an existing non-empty run directory. Detected anomalies
do not change the exit code; integrity or stage failures exit 1.
"""

import argparse
import json
import os
import sys
import traceback
from collections import Counter

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # scripts/

from humanoid_qa import QA_VERSION, SCHEMA_VERSION  # noqa: E402
from humanoid_qa.blender import diagnostics, extract, volume as bvol  # noqa: E402
from humanoid_qa.core import classify, evidence, gate, geom, intersect, refine, rig, rules, solid  # noqa: E402

FINGERS = ("Index", "Middle", "Ring", "Little", "Thumb")


def fail(msg):
    print("HUMANOID_QA FAILED:", msg)
    raise SystemExit(1)


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--ledger", default=None)
    ap.add_argument("--config", default=os.path.join(HERE, "config", "quality_rules.json"))
    ap.add_argument("--unit-results", default=None)
    ap.add_argument("--fixture-results", default=None)
    ap.add_argument("--baseline-run", default=None,
                    help="previous run directory this run builds on (recorded, never modified)")
    ap.add_argument("--no-render", action="store_true")
    return ap.parse_args(argv)


# ===================================================================== stage 1

def facing_cues(objs_by_name, meshes, bones_by_name, body_centre_y, band):
    """Independent cues; each implies a forward sign along Y (-1 or +1)."""
    cues = []

    def centre(n):
        return geom.bbox_centre(geom.bbox(meshes[n]["world"])) if n in meshes else None

    def add(cue_id, kind, desc, value, implies_forward_sign, basis):
        cues.append({"cue": cue_id, "kind": kind, "description": desc, "measured_value_m": value,
                     "implied_forward_y_sign": implies_forward_sign, "basis": basis})

    named = [("C1", "411_S_PrimaryLens", "front", "Primary optical sensor (Mat_Lens); a primary lens faces forward"),
             ("C2", "411_S_RearCam", "rear", "Object named RearCam; manifest calls it a rear-facing camera"),
             ("C3", "411_N_LED_Amber", "front", "Status LED on the neck front (weak: LEDs could face any way)")]
    for cid, name, role, basis in named:
        c = centre(name)
        if c is None:
            continue
        dy = c[1] - body_centre_y
        if abs(dy) <= band:
            continue
        sign = (1 if dy > 0 else -1) * (1 if role == "front" else -1)
        add(cid, "named_object", f"{name} centre y relative to body centre y ({role} cue)", dy, sign, basis)

    exh = [n for n in meshes if n.startswith("411_S_Exhaust_")]
    if exh:
        dy = sum(centre(n)[1] for n in exh) / len(exh) - body_centre_y
        if abs(dy) > band:
            add("C4", "named_object", "mean centre y of 411_S_Exhaust_* relative to body centre (rear cue)",
                dy, -1 if dy > 0 else 1, "Exhaust ports conventionally rear-facing (HYPOTHESIS-grade cue)")

    foot = bones_by_name.get("DEF_Foot")
    if foot:
        dy = foot["tail_w"][1] - foot["head_w"][1]
        if abs(dy) > band:
            add("C5", "bone_geometry", "DEF_Foot tail-minus-head y (ankle -> toe direction)", dy,
                1 if dy > 0 else -1, "Foot bone runs ankle to toe; toes point forward")
        feet = [n for n in meshes if n.startswith("411_Foot_")]
        if feet:
            dy2 = sum(centre(n)[1] for n in feet) / len(feet) - foot["head_w"][1]
            if abs(dy2) > band:
                add("C6", "mesh_geometry", "411_Foot_* mean centre y minus ankle (DEF_Foot head) y", dy2,
                    1 if dy2 > 0 else -1, "Foot mass extends forward of the ankle")
    return cues


def side_mapping(bones, meshes_meta, plane_x, band, forward_sign, cfg):
    """Machine-readable side table. Anatomical candidate assumes up=+Z and
    forward = forward_sign*Y; right = forward x up."""
    right = geom.right_axis((0, forward_sign, 0), (0, 0, 1)) if forward_sign else None
    right_sign_x = right[0] if right else None
    rows = []

    def row(name, kind, x):
        marker, base = rig.parse_side(name)
        phys = rig.physical_side(x, plane_x, band)
        if phys == "midline":
            anat = "midline"
        elif right_sign_x is None:
            anat = "UNRESOLVED"
        else:
            anat = "RIGHT" if (x - plane_x) * right_sign_x > 0 else "LEFT"
        ms = rig.marker_side(marker)
        if ms:
            label = ms
            label_basis = f"explicit marker {marker}"
        elif phys == "midline":
            label, label_basis = None, "unsided, on midline"
        else:
            label, label_basis = "R", "unsided off-midline; docs treat unsided as right side (phase_mirror.md, manifest D2)"
        agree = None
        if label and anat in ("LEFT", "RIGHT"):
            agree = (label == anat[0])
        rows.append({"name": name, "kind": kind, "side_marker": marker, "label_side": label,
                     "label_basis": label_basis, "world_x": x, "physical_side": phys,
                     "anatomical_candidate": anat,
                     "anatomical_side_adjudicated": gate.anatomical_side(cfg, ms, phys, right_sign_x),
                     "label_matches_anatomical_candidate": agree,
                     "status": "MIDLINE" if phys == "midline" else (
                         "STRONG_CANDIDATE_OWNER_CONFIRMATION_REQUIRED" if agree is not None else "UNRESOLVED")})

    for b in bones:
        row(b["name"], "bone", b["head_w"][0])
    for n, m in meshes_meta.items():
        row(n, "mesh", m["centre"][0])
    return rows


# ===================================================================== stage 3e helpers

GROUP_RULES = (
    ("hand", ("Finger", "Palm", "Thumb", "Wrist")),
    ("joint_housing", ("Shoulder", "Elbow", "Knee", "Ankle", "HipSocket")),
    ("limb_shell", ("UpperArm", "Forearm", "Thigh", "Shin", "Foot")),
    ("torso_pelvis", ("Torso", "Pelvis", "PelvicShell", "WaistRing", "Yoke")),
    ("head_neck_sensor", ("Head", "Neck", "N_", "S_")),
)


def component_group(name):
    """Coarse component label from the mesh name (reporting only; first matching rule wins)."""
    base = name[len("411_"):] if name.startswith("411_") else name
    for label, keys in GROUP_RULES:
        if any(base.startswith(k) for k in keys):
            return label
    return "other"


def overlap_class(status):
    """Evidence class of a pair's volumetric result (reporting categories)."""
    return {"VOLUME_VALIDATED": "MEASURED_VOLUME_AND_DEPTH_BOUND",
            "SOLVER_DISAGREEMENT": "APPROXIMATION_ONLY",
            "IDENTITY_FAILED": "APPROXIMATION_ONLY",
            "NULL_OVERLAP": "GEOMETRIC_INTERSECTION_NO_VOLUME",
            "REFINEMENT_AFFECTS_RESULT": "APPROXIMATION_ONLY"}.get(status, "INCONCLUSIVE")


def refinement_summary(ref):
    """Compact per-operand refinement record for penetration.json (full reports: refinement_preservation.json)."""
    out = {}
    for k, v in (ref or {}).items():
        p = v.get("preservation") or {}
        out[k] = {"status": v["status"], "classification": v.get("classification"), "failures": v.get("failures"),
                  "steps": v.get("steps"),
                  "vertices_merged": sum(s["vertices_merged"] for s in v.get("steps", [])),
                  "triangles_removed": len(p.get("removed_triangles", [])),
                  "max_vertex_displacement_m": p.get("max_vertex_displacement_m"),
                  "surface_distance_bound_m": p.get("surface_distance_bound_m"),
                  "signed_volume_change_m3": (p.get("deltas") or {}).get("signed_volume_m3"),
                  "swept_volume_bound_m3": p.get("swept_volume_bound_m3")}
    return out


def penetration_record(na, nb, tri_row, res, err):
    """One reportable row: triangle-level facts from stage 3d plus the solid measurement."""
    ax = res.get("axial") or {}
    orient = res.get("orientation") or {}
    status = res.get("status")
    return {
        "a": na, "b": nb, "group": f"{component_group(na)}|{component_group(nb)}",
        "status": status, "overlap_class": overlap_class(status), "reasons": res.get("reasons", {}),
        "triangle_level": {"intersect_triangle_pairs": tri_row["definite_count"],
                           "contact_triangle_pairs": tri_row["contact_count"],
                           "counts": tri_row["counts"]},
        "box_gap_m": tri_row["box_gap_m"], "reliability_notes": tri_row["reliability_notes"],
        "overlap_volume_m3": res.get("overlap_volume_m3"), "volumes_by_kind_and_kernel": res.get("volumes"),
        "solver_agreement": res.get("solver_agreement"), "symmetry_rel_diff": res.get("symmetry_rel_diff"),
        "identity": res.get("identity"),
        "axial_separation_m": {k: v.get("t") for k, v in ax.items()},
        "axial_status": {k: v.get("status") for k, v in ax.items()},
        "axial_separation_bound_m": res.get("separation_bound_m"), "axial_separation_direction": res.get("separation_direction"),
        "separation_cross_check_ok": res.get("separation_cross_check_ok"), "separation_status": res.get("separation_status"),
        "separation_residual_volume_m3": res.get("separation_residual_volume_m3"),
        "axial_nonmonotone_directions": res.get("axial_nonmonotone_directions"),
        "axial_certified": {k: (v.get("certificate") or {}).get("ok") for k, v in ax.items()},
        "axial_certificate_added_m": {k: (v.get("certificate") or {}).get("added_m") for k, v in ax.items()},
        "axial_bracket_width_m": {k: v.get("bracket_width") for k, v in ax.items()},
        "geometry_basis": res.get("geometry_basis", "ORIGINAL"),
        "refinement": refinement_summary(res.get("refinement")),
        "refinement_effect": res.get("refinement_effect"),
        "reversed_components": {k: (orient.get(k) or {}).get("reversed_components", []) for k in ("A", "B")},
        "method": "solid_v02 Boolean backend: Boolean INTERSECT/DIFFERENCE on in-memory copies; axial scan+bisection (MANIFOLD) with surface-crossing certificate, verified with EXACT",
        "units": "m^3 / m", "error": err,
    }


def write_penetration_summary(out, run_id, doc):
    """Concise human-readable companion to penetration.json."""
    L = [f"# Solid overlap measurements — {run_id}", "",
         f"Method {doc['solid_version']} (volume backend {doc['volume_backend_version']}). Units {doc['units']}.",
         f"Methods: M-SOL-VOLUME {doc['methods']['M-SOL-VOLUME']}, M-SOL-DEPTH-AXIAL {doc['methods']['M-SOL-DEPTH-AXIAL']}, M-SOL-REFINE {doc['methods']['M-SOL-REFINE']}.",
         "", "The axial separation bound is an UPPER BOUND on the minimum translation distance (MTD), not the MTD and not an exact penetration depth.",
         "", "## Status counts", ""]
    L += [f"- {k}: {v}" for k, v in doc["status_counts"].items()]
    L += ["", "## Largest validated overlap volumes", "", "| pair | volume m^3 | axial separation bound m | direction |",
          "|---|---|---|---|"]
    by_key = {(r["a"], r["b"]): r for r in doc["pairs"]}
    fmt = lambda x: "—" if x is None else f"{x:.4g}"  # noqa: E731
    for a, b, v in doc["ranked_by_volume"][:15]:
        r = by_key[(a, b)]
        L.append(f"| {a} / {b} | {fmt(v)} | {fmt(r['axial_separation_bound_m'])} | {r['axial_separation_direction'] or '—'} |")
    L += ["", "## Largest validated axial separation bounds", "", "| pair | axial separation bound m | volume m^3 |", "|---|---|---|"]
    for a, b, d in doc["ranked_by_axial_separation_bound"][:15]:
        L.append(f"| {a} / {b} | {fmt(d)} | {fmt(by_key[(a, b)]['overlap_volume_m3'])} |")
    L += ["", f"Separation status: {doc['separation_status_counts']} (CONFIRMED_BY_ALL_KERNELS = both kernels confirm the minimising translation).",
          f"Pairs measured on a refined in-memory copy: {doc['refined_pairs']} (basis: {doc['geometry_basis_counts']}).",
          f"Pairs with sampled non-monotone overlap along an axis: {doc['nonmonotone_pairs']}.", "", "## Not validated or inconclusive", ""]
    odd = [r for r in doc["pairs"] if r["status"] != "VOLUME_VALIDATED"]
    refined = [r for r in doc["pairs"] if r["geometry_basis"] != "ORIGINAL"]
    L += [f"- {r['a']} / {r['b']}: {r['status']} {r['reasons'] or ''}" for r in odd] or ["- none"]
    L += ["", "## Validated on a refined in-memory copy (original .blend untouched)", ""]
    for r in refined:
        eff = r.get("refinement_effect") or {}
        worst = max((v["rel_diff"] for v in (eff.get("by_solver") or {}).values()), default=None)
        L.append(f"- {r['a']} / {r['b']}: {r['status']}, V = {fmt(r['overlap_volume_m3'])} m^3, basis {r['geometry_basis']}, "
                 f"refined-vs-original volume rel diff {('n/a' if worst is None else f'{worst:.2e}')}")
    if not refined:
        L.append("- none")
    L += ["", "## Near-contact reference (no intersection)", ""]
    L += [f"- {r['a']} / {r['b']}: {r['distance_m']:.6g} m" for r in doc["near_contact_reference"][:10]]
    L += ["", "## Groups", "", "| group | pairs | validated | max volume m^3 |", "|---|---|---|---|"]
    for g, s in doc["group_summary"].items():
        mv = f"{s['max_volume_m3']:.4g}" if s["max_volume_m3"] is not None else "—"
        L.append(f"| {g} | {s['pairs']} | {s['validated']} | {mv} |")
    with open(os.path.join(out, "penetration_summary.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")


# ===================================================================== main

def main():
    args = parse_args()
    src = bpy.data.filepath
    if not src:
        fail("no source .blend loaded")
    ident_before = evidence.file_identity(src)
    started = evidence.utc_now()
    out = args.out_dir
    if os.path.exists(out) and os.listdir(out):
        fail(f"run directory exists and is not empty: {out}")
    os.makedirs(out, exist_ok=True)

    cfg = rules.load_config(args.config)
    tr = rules.load_test_results([args.unit_results, args.fixture_results])
    methods = rules.qualify_methods(cfg, tr)

    def ms(mid):
        return methods[mid]["status"]

    T = {k: rules.tol(cfg, k) for k in cfg["tolerances"]}
    F = rules.Findings("QA")
    stage_status = {}

    # ----------------------------------------------------------- extraction
    scene = extract.scene_info()
    objs = extract.objects_info()
    objs_by_name = {o["name"]: o for o in objs}
    arm_name = cfg["classification"]["armature_object"]
    if arm_name not in bpy.data.objects:
        fail(f"armature {arm_name} not found")
    arm = extract.armature_info(bpy.data.objects[arm_name])
    bones = arm["bones"]
    bones_by_name = {b["name"]: b for b in bones}
    classes = classify.classify_all(objs, cfg)
    body_names = sorted(n for n, c in classes.items() if c["class"] == "body")
    meshes = {n: extract.mesh_data(bpy.data.objects[n]) for n in sorted(classes)}
    body = {n: meshes[n] for n in body_names}
    evaluated = extract.evaluated_bounds([bpy.data.objects[n] for n in body_names])

    units_ok = scene["unit_system"] == "METRIC" and abs(scene["scale_length"] - 1.0) <= T["float_compare"]
    unit = "m" if units_ok else "blender_units"

    # ----------------------------------------------------------- stage 3a: inventory + dims
    inventory = []
    for n in sorted(classes):
        o, md = objs_by_name[n], meshes[n]
        wb = geom.bbox(md["world"])
        lb = geom.bbox(md["local"])
        inventory.append({
            "name": n, "data": o["data"], "class": classes[n]["class"], "reason": classes[n]["reason"],
            "collections": o["collections"], "location": o["location"], "rotation_euler": o["rotation_euler"],
            "scale": o["scale"], "non_unit_scale": any(abs(s - 1) > T["float_compare"] for s in o["scale"]),
            "world_bbox": wb, "local_bbox": lb, "vertices": len(md["local"]), "edges": len(md["edges"]),
            "faces": len(md["faces"]), "modifiers": [(m["type"], m["target"]) for m in o["modifiers"]],
            "armature_targets": [m["target"] for m in md["armature_mods"]],
        })
    class_counts = Counter(c["class"] for c in classes.values())
    body_pts = [p for n in body_names for p in body[n]["world"]]
    bb = geom.bbox(body_pts)
    dims = geom.bbox_dims(bb)
    ev_dims = [evaluated["max"][i] - evaluated["min"][i] for i in range(3)] if evaluated["min"] else None

    def metric(name, value, method, included, frame="world", tol_key="float_compare",
               evidence_class="MEASURED_FACT", limits=""):
        return {"metric": name, "value": value, "unit": unit if name != "body_vertex_count" else "count",
                "frame": frame, "included": included, "excluded": "all non-body classes (see inventory)",
                "method": method, "method_status": ms(method), "tolerance": cfg["tolerances"][tol_key],
                "evidence_class": evidence_class, "limitations": limits, "kind": "observation"}

    inc = f"{len(body_names)} objects classified body"
    metrics = [
        metric("body_height_z_base", dims[2], "M-BBOX-WORLD", inc,
               limits="Rest/base mesh data; floor-to-top including head sensors."),
        metric("body_width_x_base", dims[0], "M-BBOX-WORLD", inc, limits="Includes arm span at rest pose."),
        metric("body_depth_y_base", dims[1], "M-BBOX-WORLD", inc),
        metric("body_min_z_base", bb[0][2], "M-BBOX-WORLD", inc, limits="Floor contact check."),
        metric("body_height_z_evaluated", ev_dims[2] if ev_dims else None, "M-BBOX-EVAL", inc,
               limits=f"Depsgraph at frame {scene['frame_current']}, active action {arm['active_action']}."),
        metric("body_width_x_evaluated", ev_dims[0] if ev_dims else None, "M-BBOX-EVAL", inc,
               limits=f"Depsgraph at frame {scene['frame_current']}."),
        metric("body_vertex_count", len(body_pts), "M-COUNTS", inc, frame="n/a"),
    ]

    spec = scene["custom_properties"].get("SPEC_JSON")
    spec_cmp = None
    if isinstance(spec, dict) and "total_height" in spec:
        spec_cmp = {"spec_key": "total_height", "spec_value": spec["total_height"],
                    "measured_key": "body_height_z_base", "measured_value": dims[2],
                    "difference": dims[2] - float(spec["total_height"]),
                    "method_status": ms("M-SPEC-COMPARE"),
                    "landmark_definition": "floor (min z) to highest body vertex; SPEC_JSON's own definition is undocumented",
                    "other_spec_keys_not_compared": sorted(k for k in spec if k != "total_height")}

    # ----------------------------------------------------------- stage 1: facing
    plane_x = cfg["coordinate_convention"]["mirror_plane"]["value"]
    band = T["midline_band"]
    body_centre_y = geom.bbox_centre(bb)[1]
    spine = [bones_by_name[n] for n in ("DEF_Pelvis", "DEF_Chest", "DEF_Neck", "DEF_Head") if n in bones_by_name]
    spine_x = [p[0] for b in spine for p in (b["head_w"], b["tail_w"])]
    cues = facing_cues(objs_by_name, body, bones_by_name, body_centre_y, band)
    signs = Counter(c["implied_forward_y_sign"] for c in cues)
    forward_sign = None
    if cues and len(signs) == 1:
        forward_sign = cues[0]["implied_forward_y_sign"]
    meshes_meta = {n: {"centre": geom.bbox_centre(geom.bbox(body[n]["world"]))} for n in body_names}
    smap = side_mapping(bones, meshes_meta, plane_x, band, forward_sign, cfg)
    smap_counts = Counter((r["kind"], r["label_side"], r["physical_side"], r["anatomical_candidate"])
                          for r in smap)
    disagree = [r for r in smap if r["label_matches_anatomical_candidate"] is False]
    agree = [r for r in smap if r["label_matches_anatomical_candidate"] is True]
    convention = {
        "declaration": cfg["coordinate_convention"],
        "frames": {
            "world": "Blender world, right-handed, Z up; units " + unit,
            "object_local": "per object; meshes read with matrix_world for world values",
            "armature_object": f"{arm_name} matrix_world rotation_euler={arm['rotation_euler']} (non-zero, manifest D1)",
            "bone_rest": "bone.head_local/tail_local in armature space -> world via armature matrix_world",
            "pose": f"pose_position={arm['pose_position']}, active_action={arm['active_action']}; not used for rest measurements",
        },
        "spine_midline_x": {"mean": sum(spine_x) / len(spine_x) if spine_x else None,
                            "min": min(spine_x) if spine_x else None, "max": max(spine_x) if spine_x else None,
                            "bones": [b["name"] for b in spine]},
        "geometric_cues": cues,
        "geometric_cue_agreement": {"forward_y_sign_votes": dict(signs), "unanimous_forward_y_sign": forward_sign},
        "documentary_evidence": [
            {"source": "specifications/legs/knee.md:33-40", "class": "SOURCE_FACT",
             "says": "Empirical DEF_Shin +15 deg moved foot tail +0.1048 world Y = backward; forward = -Y (via front lens position)"},
            {"source": "docs/411_ROBOT_MASTER_VISUAL_ENGINEERING_MANIFEST.md D2 (l.652-656)", "class": "SOURCE_FACT",
             "says": "Rig uses no suffix for the right side and _L for the left"},
            {"source": "docs/phase_mirror.md", "class": "SOURCE_FACT",
             "says": "Right side built first, duplicated to _L by negating X"},
            {"source": "docs/anim_hello.md:35-46,94", "class": "SOURCE_FACT",
             "says": "'Right arm' wave moves IK_Target.arm.R to x=+0.45; camera at (0,-2.8,1.1) front-facing"},
            {"source": "docs/ARCHITECTURE.md:17-24", "class": "SOURCE_FACT",
             "says": "'X: Forward/Back, Y: Left/Right' then 'X points along the bone' - describes bone-local axes, self-contradictory, states no world facing (manifest D4)"},
            {"source": "docs/NAMING.md:26-35", "class": "SOURCE_FACT",
             "says": "Bilateral bones use .L/.R; never create them by hand"},
        ],
        "derived": {
            "anatomical_right_axis_if_forward_minus_Y": geom.right_axis((0, -1, 0), (0, 0, 1)),
            "documented_right_side_physical_x": "+X (unsided limb bones, *_R meshes)",
            "conflict": "With forward=-Y and up=+Z, anatomical right is -X; the documented 'right' side is +X. "
                        "Both cannot be anatomical. The docs' 'right' matches viewer-right in the front camera.",
        },
        "outcome": cfg["coordinate_convention"]["outcome"],
        "decision_package": {
            "decision": "Do side labels (unsided/_R vs _L, and future .L/.R) name the ROBOT'S OWN sides (anatomical) or the sides as SEEN FROM THE FRONT CAMERA (viewer-relative)?",
            "candidate_A_anatomical": {
                "meaning": "+X is the robot's LEFT; every current side label is inverted",
                "for": ["Rigify/Blender convention is anatomical (.L = character's left, +X when facing -Y)",
                        "NAMING.md mandates Rigify .L/.R", "glTF consumers and mirror tools assume anatomical sides"],
                "against": ["Every existing doc, script (arm_ik_solver.py vs _L), action and IK target name uses the current labels",
                            "Relabelling touches bones, vertex groups, constraints, actions, solver scripts"]},
            "candidate_V_viewer_relative": {
                "meaning": "Keep labels as-is and document them as viewer-relative",
                "for": ["Zero rename risk", "Matches all existing docs and animation notes"],
                "against": ["Contradicts Rigify/.L.R semantics in NAMING.md", "Blender's X-mirror tools and future Rigify generation would treat _L/.L as anatomical left"]},
            "safe_before_decision": ["label-independent mirror-pair measurements", "topology, dimensions, skinning conditions, reference checks"],
            "blocked_until_decision": ["declaring which side of a pair is 'correct'", "any rename", "anatomical deformation tests", "side-dependent spec adjudication"],
            "diagnostic_views": "diag_*.png in this run directory; see diagnostic_views.json for measured image-right axes",
        },
    }
    stage_status["1_facing"] = "OWNER_DECISION_REQUIRED"

    # ----------------------------------------------------------- stage 3b: topology / duplicates
    topo = {}
    for n in body_names:
        md = body[n]
        t = geom.topology(len(md["local"]), md["edges"], md["faces"], md["local"],
                          T["degenerate_face_area"], T["zero_length_edge"])
        t["coincident_vertex_pairs"] = len(geom.coincident_pairs(md["local"], T["coincident_vertex"]))
        t["near_duplicate_vertex_pairs"] = len(geom.coincident_pairs(md["local"], T["near_duplicate_vertex"]))
        topo[n] = t
    topo_totals = {k: sum(t[k] for t in topo.values()) for k in
                   ("boundary_edges", "nonmanifold_edges", "wire_edges", "loose_vertices",
                    "degenerate_faces", "zero_length_edges", "coincident_vertex_pairs",
                    "near_duplicate_vertex_pairs")}

    # ----------------------------------------------------------- stage 3b2: orientation
    orient = {}
    for n in body_names:
        md, t = body[n], topo[n]
        m3 = [r[:3] for r in objs_by_name[n]["matrix_world"][:3]]
        det = (m3[0][0] * (m3[1][1] * m3[2][2] - m3[1][2] * m3[2][1])
               - m3[0][1] * (m3[1][0] * m3[2][2] - m3[1][2] * m3[2][0])
               + m3[0][2] * (m3[1][0] * m3[2][1] - m3[1][1] * m3[2][0]))
        closed = t["boundary_edges"] == 0 and t["nonmanifold_edges"] == 0 and t["wire_edges"] == 0
        vol = geom.signed_volume(md["local"], md["faces"]) if closed else None
        orient[n] = {"winding_inconsistent_edges": geom.winding_inconsistencies(md["faces"]),
                     "closed_manifold": closed, "signed_volume_local": vol,
                     "matrix_determinant": det,
                     "inside_out_in_world": (vol * (1 if det > 0 else -1) < 0) if vol is not None else None,
                     "blender_recalc_would_flip_faces": extract.recalc_flip_count(bpy.data.objects[n])}
    wind_bad = {n: r for n, r in orient.items() if r["winding_inconsistent_edges"]}
    inside_out = {n: r for n, r in orient.items() if r["inside_out_in_world"]}
    blender_flips = {n: r["blender_recalc_would_flip_faces"] for n, r in orient.items()
                     if r["blender_recalc_would_flip_faces"]}

    # ----------------------------------------------------------- stage 3c: symmetry
    self_d = geom.self_mirror_distances(body_pts, plane_x)
    body_sym = geom.distance_summary(self_d, T["symmetry_match"])
    per_obj_over = []
    i = 0
    for n in body_names:
        k = len(body[n]["world"])
        seg = self_d[i:i + k]
        i += k
        over = sum(1 for d in seg if d > T["symmetry_match"])
        if over:
            per_obj_over.append({"object": n, "vertices": k, "over_tolerance": over,
                                 "fraction": over / k, "max": max(seg)})
    per_obj_over.sort(key=lambda r: -r["over_tolerance"])
    mesh_pairs = []
    fmap = {k: v for k, v in cfg["finger_mesh_map"].items() if k in ("L", "R")}
    canon = {classify.canonical_mesh_name(n, fmap): n for n in body_names}
    for p in rig.pair_by_name(sorted(canon))["pairs"]:
        p = {**p, "l_marked": canon[p["l_marked"]], "counterpart": canon[p["counterpart"]]}
        a, b = body[p["l_marked"]]["world"], body[p["counterpart"]]["world"]
        r = geom.mirror_compare(a, b, plane_x, T["symmetry_match"], None)
        mesh_pairs.append({"pair": [p["l_marked"], p["counterpart"]],
                           "vertices": [len(a), len(b)],
                           "forward_max": r["forward"]["max"], "backward_max": r["backward"]["max"],
                           "forward_p95": r["forward"]["p95"], "backward_p95": r["backward"]["p95"],
                           "forward_over_tol": r["forward"]["count_over_tolerance"],
                           "backward_over_tol": r["backward"]["count_over_tolerance"]})
    mesh_pairs.sort(key=lambda r: -max(r["forward_max"], r["backward_max"]))
    stage_status["3_geometry"] = "COMPLETE"

    # ----------------------------------------------------------- stage 3d: intersections & proximity
    itol = intersect.Tol(plane=T["tri_plane_coplanar"], contact=T["tri_contact"], area=T["degenerate_face_area"])
    max_pairs = int(T["max_candidate_pairs"])
    radius = T["proximity_search_radius"]
    tmesh, traw, mesh_notes = {}, {}, {}
    for n in body_names:
        d = extract.mesh_tris(bpy.data.objects[n])
        traw[n] = d
        tmesh[n] = intersect.TriMesh(n, d["world"], d["tris"], d["poly"], T["coincident_vertex"], itol)
        mesh_notes[n] = intersect.reliability_notes(topo[n])

    def _ex(items, k=3):
        return [{kk: vv for kk, vv in it.items()} for it in items[:k]]

    self_rows, self_undetermined = [], []
    for n in body_names:
        tm = tmesh[n]
        row = {"mesh": n, "triangles": len(tm.tris), "active_triangles": len(tm.active),
               "degenerate_triangles_excluded": len(tm.degenerate),
               "degenerate_triangle_indices": tm.degenerate[:20], "reliability_notes": mesh_notes[n]}
        try:
            r = intersect.self_intersections(tm, itol, max_pairs)
        except intersect.BudgetExceeded as e:
            row["undetermined"] = str(e)
            self_undetermined.append(n)
            self_rows.append(row)
            continue
        row.update({"candidate_pairs": r["candidate_pairs"], "adjacent_pairs_ordinary": r["adjacent_pairs_ordinary"],
                    "counts": r["counts"], "definite_count": r["definite_count"], "contact_count": r["contact_count"],
                    "examples_definite": _ex(r["definite"]), "examples_contact": _ex(r["contact"])})
        if r["definite_count"]:
            bl = set(extract.blender_bvh_self_overlap(traw[n]["world"], traw[n]["tris"]))
            row["blender_bvh_contains_all_listed_definite"] = all(
                (min(d["a"]["tri"], d["b"]["tri"]), max(d["a"]["tri"], d["b"]["tri"])) in bl for d in r["definite"])
        self_rows.append(row)
    self_def_total = sum(r.get("definite_count", 0) for r in self_rows)
    self_meshes_def = [r["mesh"] for r in self_rows if r.get("definite_count")]

    pair_rows, prox_rows, pair_undetermined = [], [], []
    status_counts = Counter()
    beyond = 0
    xcheck = {"pairs_checked": 0, "identical_sets": 0, "only_core": 0, "only_core_exact_confirms": 0,
              "only_core_exact_rejects": 0, "only_blender": 0, "only_blender_by_core_kind": {}, "details": []}
    for ai in range(len(body_names)):
        for bi in range(ai + 1, len(body_names)):
            na, nb = body_names[ai], body_names[bi]
            a, b = tmesh[na], tmesh[nb]
            if not a.active or not b.active:
                pair_undetermined.append([na, nb])
                status_counts["UNABLE_TO_DETERMINE"] += 1
                continue
            gap = intersect.box_gap(a.bvh.box, b.bvh.box)
            if gap > radius:
                beyond += 1
                status_counts["NONE_DETECTED"] += 1
                continue
            res, status = None, "NONE_DETECTED"
            if gap <= itol.contact:
                try:
                    res = intersect.cross_intersections(a, b, itol, max_pairs)
                    status = intersect.pair_status(res, len(a.active), len(b.active))
                except intersect.BudgetExceeded:
                    status = "UNABLE_TO_DETERMINE"
                    pair_undetermined.append([na, nb])
            status_counts[status] += 1
            if res is not None and res["candidate_pairs"]:
                # Blender's BVHTree.overlap applies its own float32 triangle test (verified by B-INT-04), so it is an
                # independent intersection implementation. Disagreements are adjudicated by exact rational arithmetic.
                bl = set(extract.blender_bvh_overlap(traw[na]["world"], traw[na]["tris"], traw[nb]["world"], traw[nb]["tris"]))
                core_set = set(res["intersect_pairs_all"])
                xcheck["pairs_checked"] += 1
                if bl == core_set:
                    xcheck["identical_sets"] += 1
                for (i, j) in sorted(core_set - bl):
                    ok = intersect.exact_proper_crossing(a.corners[i], b.corners[j])
                    xcheck["only_core"] += 1
                    xcheck["only_core_exact_confirms" if ok else "only_core_exact_rejects"] += 1
                    if len(xcheck["details"]) < 12:
                        xcheck["details"].append({"pair": [na, nb], "tri_a": i, "tri_b": j, "only_in": "core",
                                                  "exact_oracle_crossing": ok})
                for (i, j) in sorted(bl - core_set):
                    k = intersect.classify_triangle_pair(a.corners[i], b.corners[j], itol)["kind"]
                    xcheck["only_blender"] += 1
                    xcheck["only_blender_by_core_kind"][k] = xcheck["only_blender_by_core_kind"].get(k, 0) + 1
                    if len(xcheck["details"]) < 12:
                        xcheck["details"].append({"pair": [na, nb], "tri_a": i, "tri_b": j, "only_in": "blender",
                                                  "core_kind": k,
                                                  "exact_oracle_crossing": intersect.exact_proper_crossing(a.corners[i], b.corners[j])})
            if res is not None and (res["definite_count"] or res["contact_count"]) or status == "UNABLE_TO_DETERMINE":
                both_closed = not mesh_notes[na] and not mesh_notes[nb]
                pair_rows.append({
                    "a": na, "b": nb, "status": status, "box_gap_m": gap,
                    "counts": res["counts"] if res else None,
                    "definite_count": res["definite_count"] if res else None,
                    "contact_count": res["contact_count"] if res else None,
                    "candidate_pairs": res["candidate_pairs"] if res else None,
                    "volumetric_interpretation_reliable": both_closed,
                    "reliability_notes": {k: mesh_notes[k] for k in (na, nb) if mesh_notes[k]},
                    "examples_definite": _ex(res["definite"]) if res else [],
                    "examples_contact": _ex(res["contact"]) if res else []})
            if status != "DEFINITE_INTERSECTION":
                pm = intersect.mesh_proximity(a, b, radius)
                if pm is not None:
                    prox_rows.append({"a": na, "b": nb, "distance_m": pm["distance"], "tri_a": pm["a"], "tri_b": pm["b"],
                                      "point_a": pm["point_a"], "point_b": pm["point_b"],
                                      "touching_within_numerical_tolerance": pm["distance"] <= itol.contact})
    pair_rows.sort(key=lambda r: (r["status"], r["a"], r["b"]))
    prox_rows.sort(key=lambda r: (r["distance_m"], r["a"], r["b"]))
    pairs_total = len(body_names) * (len(body_names) - 1) // 2
    definite_pairs = [r for r in pair_rows if r["status"] == "DEFINITE_INTERSECTION"]
    possible_pairs = [r for r in pair_rows if r["status"] == "POSSIBLE_REQUIRES_INSPECTION"]
    touching = [r for r in prox_rows if r["touching_within_numerical_tolerance"]]
    mesh_analysis = {
        "schema_version": SCHEMA_VERSION,
        "basis": "BASE (unmodified) mesh data of body-classified objects, Blender loop-triangle tessellation, world space, metres. Evaluated (modifier) geometry is NOT analysed.",
        "units_verified_metric": units_ok,
        "tolerances": {k: cfg["tolerances"][k] for k in ("tri_plane_coplanar", "tri_contact", "degenerate_face_area",
                                                          "coincident_vertex", "max_candidate_pairs", "proximity_search_radius")},
        "methods": {k: methods[k]["status"] for k in ("M-TRI-PAIR", "M-SELFINT", "M-INTERMESH", "M-DISTANCE", "M-BVH", "M-XCHECK", "M-PROX-CLASS")},
        "scope": {"body_meshes": len(body_names), "pairs_total": pairs_total, "pairs_beyond_search_radius_unmeasured": beyond,
                  "excluded_classes": {k: v for k, v in sorted(class_counts.items()) if k != "body"}},
        "self_intersection": {"total_definite": self_def_total, "meshes_with_definite": self_meshes_def,
                              "undetermined_meshes": self_undetermined, "per_mesh": self_rows},
        "pair_status_counts": dict(sorted(status_counts.items())),
        "pair_unable_to_determine": pair_undetermined,
        "pairs_reported": pair_rows,
        "independent_crosscheck_vs_blender_overlap": xcheck,
        "proximity": {"search_radius_m": radius, "search_radius_status": "EXPERIMENTAL", "pairs_within_radius": len(prox_rows),
                      "touching_within_numerical_tolerance": len(touching), "closest": prox_rows},
        "known_limitations": [
            "A result with no findings is NOT proof of no intersection; it means none were detected by these methods and tolerances.",
            "Crossings exactly on a triangle boundary are classified 'contact' (POSSIBLE), not 'intersect'.",
            "Self-intersection adjacency uses vertices welded within coincident_vertex; deliberately separate coincident surfaces would be treated as connected.",
            "Open/non-manifold meshes: triangle intersections are facts but volumetric penetration is not interpretable (flagged per pair).",
            "Modifier-evaluated geometry (Bevel/Subsurf on some body meshes) is not analysed.",
            "Proximity beyond the search radius is not measured; the radius is an EXPERIMENTAL reporting scope, not a requirement."]}
    stage_status["3d_mesh_intersection_proximity"] = "COMPLETE" if not (self_undetermined or pair_undetermined) else "PARTIAL"
    for nm, val, meth in (("body_self_intersecting_meshes", len(self_meshes_def), "M-SELFINT"),
                          ("body_self_intersection_definite_pairs", self_def_total, "M-SELFINT"),
                          ("body_mesh_pairs_definite_intersection", len(definite_pairs), "M-INTERMESH"),
                          ("body_mesh_pairs_possible_contact", len(possible_pairs), "M-INTERMESH"),
                          ("body_mesh_pairs_within_search_radius", len(prox_rows), "M-DISTANCE")):
        metrics.append({"metric": nm, "value": val, "unit": "count", "frame": "world", "included": inc,
                        "excluded": "all non-body classes (see inventory)", "method": meth, "method_status": ms(meth),
                        "tolerance": cfg["tolerances"]["tri_contact"], "evidence_class": "MEASURED_FACT",
                        "limitations": "See mesh_analysis.json known_limitations.", "kind": "observation"})

    # ----------------------------------------------------------- stage 3e: solid overlap volume + axial separation bound + degenerate-face refinement (solid_v02)
    # Measured on in-memory copies only (blender/volume.py). Definite triangle-intersecting body pairs are the scope;
    # near-contact pairs (no intersection) are carried for reference with their separations from stage 3d.
    sol_params = {"null_volume": T["penetration_null_volume"], "degenerate_area": T["degenerate_face_area"],
                  "iters": int(T["penetration_bisection_iterations"]), "agree_rel": T["volume_solver_agreement_rel"],
                  "identity_rel": T["volume_identity_rel"],
                  "axial_scan_samples": int(T["penetration_axial_scan_samples"]),
                  "tri_contact": T["tri_contact"], "tri_plane_coplanar": T["tri_plane_coplanar"],
                  "coincident_vertex": T["coincident_vertex"], "max_candidate_pairs": int(T["max_candidate_pairs"]),
                  "refine_edge_tol": T["zero_length_edge"]}
    pen_rows, pen_errors, ref_full = [], [], []
    for r in definite_pairs:
        na, nb = r["a"], r["b"]
        if not units_ok:
            rec = penetration_record(na, nb, r, {"status": "UNIT_NOT_METRIC"}, None)
        else:
            try:
                res = solid.measure_pair(traw[na]["world"], traw[na]["tris"], traw[nb]["world"], traw[nb]["tris"],
                                         bvol.BooleanBackend.factory("pen"), sol_params)
                rec = penetration_record(na, nb, r, res, None)
                if res.get("refinement"):
                    ref_full.append({"a": na, "b": nb, "geometry_basis": res.get("geometry_basis"),
                                     "operands": res["refinement"], "refinement_effect": res.get("refinement_effect")})
            except Exception:
                pen_errors.append([na, nb])
                rec = penetration_record(na, nb, r, {"status": "BACKEND_ERROR"}, traceback.format_exc())
        pen_rows.append(rec)
    near_rows = []
    for p in prox_rows[:20]:
        near_rows.append({"a": p["a"], "b": p["b"], "status": "NEAR_CONTACT_NO_INTERSECTION",
                          "distance_m": p["distance_m"], "group": f"{component_group(p['a'])}|{component_group(p['b'])}",
                          "touching_within_numerical_tolerance": p["touching_within_numerical_tolerance"]})
    pen_status = Counter(r["status"] for r in pen_rows)
    pen_class = Counter(r["overlap_class"] for r in pen_rows)
    validated = [r for r in pen_rows if r["status"] == "VOLUME_VALIDATED"]
    ranked_volume = sorted(validated, key=lambda r: (-r["overlap_volume_m3"], r["a"], r["b"]))
    ranked_depth = sorted([r for r in validated if r["axial_separation_bound_m"] is not None],
                          key=lambda r: (-r["axial_separation_bound_m"], r["a"], r["b"]))
    group_counts = Counter(r["group"] for r in pen_rows)
    group_volume = {}
    for r in validated:
        group_volume.setdefault(r["group"], []).append(r["overlap_volume_m3"])
    group_summary = {g: {"pairs": group_counts[g], "validated": len(group_volume.get(g, [])),
                         "max_volume_m3": max(group_volume[g]) if g in group_volume else None}
                     for g in sorted(group_counts)}
    stage_status["3e_solid_overlap"] = "COMPLETE" if not pen_errors and units_ok else "PARTIAL"
    pen_method_status = ms("M-SOL-VOLUME")
    depth_method_status = ms("M-SOL-DEPTH-AXIAL")
    refine_method_status = ms("M-SOL-REFINE")
    qualified_pen = pen_method_status in ("QUALIFIED", "QUALIFIED_WITH_LIMITATIONS") and \
        depth_method_status in ("QUALIFIED", "QUALIFIED_WITH_LIMITATIONS")

    pen_doc = {
        "schema_version": SCHEMA_VERSION, "solid_version": solid.VERSION, "volume_backend_version": bvol.VERSION,
        "scope": "definite triangle-intersecting body pairs (stage 3d) plus near-contact reference pairs; environment and presentation meshes excluded",
        "parameters": sol_params, "units": "m (lengths), m^3 (volumes)" if units_ok else "blender_units",
        "methods": {"M-SOL-VOLUME": pen_method_status, "M-SOL-DEPTH-AXIAL": depth_method_status,
                    "M-SOL-REFINE": refine_method_status},
        "definitions": {
            "overlap_volume_m3": "Boolean INTERSECT volume V(A n B) of two closed body shells after per-component outward orientation on in-memory copies. Both kernels (MANIFOLD, EXACT) and both operand orders must agree (volume_solver_agreement_rel) and V(A n B)+V(A minus B)=V(A) must hold (volume_identity_rel).",
            "axial_separation_bound_m": "min over the six world axis directions of a verified separating translation t(d): A moved by t(d) along d has overlap volume <= penetration_null_volume and no proper surface crossing deeper than tri_contact. It is an UPPER BOUND on the minimum translation distance (MTD) of the pair, to within about tri_contact; it is not the MTD, not an exact penetration depth, and along an axis it need not be the first separation when the overlap is non-monotone (see axial_nonmonotone_directions).",
            "geometry_basis": "ORIGINAL, or REFINED_COPY:<operand>=<class>: the measurement used an in-memory copy of that operand whose collapsed (zero-area) triangles were removed by merging vertices <= zero_length_edge apart; the pair is validated only if the unrefined shells give the same volume in both kernels (refinement_effect). Full preservation reports: refinement_preservation.json.",
            "intersect_triangle_pairs": "count of exact triangle-pair intersections from stage 3d (a triangle count, NOT a depth or severity measure)",
            "near_contact": "closest non-intersecting body pairs from stage 3d; reported distances only"},
        "status_counts": dict(sorted(pen_status.items())), "class_counts": dict(sorted(pen_class.items())),
        "separation_status_counts": dict(sorted(Counter(r["separation_status"] for r in pen_rows).items())),
        "geometry_basis_counts": dict(sorted(Counter(r["geometry_basis"] for r in pen_rows).items())),
        "refined_pairs": sum(1 for r in pen_rows if r["geometry_basis"] != "ORIGINAL"),
        "nonmonotone_pairs": sum(1 for r in pen_rows if r.get("axial_nonmonotone_directions")),
        "pairs": pen_rows, "errors": pen_errors,
        "ranked_by_volume": [[r["a"], r["b"], r["overlap_volume_m3"]] for r in ranked_volume],
        "ranked_by_axial_separation_bound": [[r["a"], r["b"], r["axial_separation_bound_m"]] for r in ranked_depth],
        "group_summary": group_summary, "near_contact_reference": near_rows,
        "qualified_for_findings": qualified_pen,
    }

    F_pen_affected = sorted({x for r in validated for x in (r["a"], r["b"])})

    # ----------------------------------------------------------- stage 4: rig
    graph = rig.graph_summary(bones)
    layer_counts = Counter(rig.layer_of(b["name"]) or "NONE" for b in bones)
    marker_counts = Counter(rig.parse_side(b["name"])[0] for b in bones)
    bpairs = rig.pair_by_name([b["name"] for b in bones])
    paired = {p["l_marked"] for p in bpairs["pairs"]} | {p["counterpart"] for p in bpairs["pairs"]}
    unpaired_offmid = sorted(b["name"] for b in bones if b["name"] not in paired
                             and rig.physical_side(b["head_w"][0], plane_x, band) != "midline")
    pair_rows = []
    for p in bpairs["pairs"]:
        a, b = bones_by_name[p["l_marked"]], bones_by_name[p["counterpart"]]
        m = rig.pair_metrics(a, b, plane_x, T["bone_min_length_for_direction"])
        pair_rows.append({"pair": [p["l_marked"], p["counterpart"]], "layer": rig.layer_of(a["name"]),
                          "finger": next((f for f in FINGERS if f"_{f}_" in a["name"]), None), **m,
                          "position_over_tol": m["max_endpoint_delta"] > T["bone_pair_position"],
                          "direction_over_tol": (m["direction_angle_deg"] or 0) > T["bone_pair_direction"]})
    pair_rows.sort(key=lambda r: -r["max_endpoint_delta"])
    over_pairs = [r for r in pair_rows if r["position_over_tol"] or r["direction_over_tol"]]
    over_by_layer = Counter((r["layer"], bool(r["finger"])) for r in over_pairs)

    defs = [b for b in bones if rig.layer_of(b["name"]) == "DEF"]
    coloc = []
    for b in bones:
        if rig.layer_of(b["name"]) in ("MECH", "CTRL") and any(f"_{f}_" in b["name"] for f in FINGERS):
            side_defs = [d for d in defs if rig.physical_side(d["head_w"][0], plane_x, band)
                         == rig.physical_side(b["head_w"][0], plane_x, band)]
            r = rig.nearest_def_head(b, side_defs)
            coloc.append({"bone": b["name"], "physical_side": rig.physical_side(b["head_w"][0], plane_x, band),
                          "nearest_def": r["def_bone"], "distance": r["distance"]})
    coloc_by_side = {}
    for side in ("+X", "-X"):
        ds = sorted(c["distance"] for c in coloc if c["physical_side"] == side)
        if ds:
            coloc_by_side[side] = {"count": len(ds), "median": geom.percentile(ds, 50), "max": ds[-1]}

    thumbs = {"unsided_hand": rig.thumb_adjacency(bones_by_name, ""),
              "L_marked_hand": rig.thumb_adjacency(bones_by_name, "_L")}
    for k, v in thumbs.items():
        if v:
            v["hand_physical_side"] = rig.physical_side(v["thumb_head_w"][0], plane_x, band)
    shorts = rig.short_bones(bones, T["bone_min_length_for_direction"])
    skin = rig.skinning_layers(bones, list(body.values()), arm_name)
    srows = skin["bones"]
    skin_summary = Counter((r["layer"], r["1_use_deform"], r["2_vertex_group_exists"],
                            r["3_nonzero_weight_vertices"] > 0, bool(r["4_bound_meshes"])) for r in srows)
    mech_ctrl_deform = [r["bone"] for r in srows if r["layer"] in ("MECH", "CTRL") and r["1_use_deform"]]
    mech_ctrl_weighted = [r["bone"] for r in srows if r["layer"] in ("MECH", "CTRL") and r["3_nonzero_weight_vertices"]]
    def_unweighted = [r["bone"] for r in srows if r["layer"] == "DEF" and not r["3_nonzero_weight_vertices"]]
    weight_stats = {"unweighted_vertices": sum(m["unweighted_vertices"] for m in body.values()),
                    "max_influences": max(m["max_influences"] for m in body.values()),
                    "weight_sum_off_1": sum(m["weight_sum_off_1"] for m in body.values())}

    all_cons = arm["pose_constraints"] + [c for o in objs for c in o["constraints"]]
    obj_types = {o["name"]: o["type"] for o in objs}
    con_problems = rig.constraint_reference_check(all_cons, obj_types, {arm_name: set(bones_by_name)})
    con_invalid_flag = [c for c in all_cons if not c["is_valid"]]
    cons_by_owner = {}
    for c in arm["pose_constraints"]:
        cons_by_owner.setdefault(c["owner"], []).append(c)
    con_pair_diffs = rig.constraint_pair_diffs(cons_by_owner, bpairs["pairs"])
    mod_problems = []
    for n in body_names:
        for m in body[n]["armature_mods"]:
            if m["target"] != arm_name or not m["use_vertex_groups"] or not (m["show_viewport"] and m["show_render"]):
                mod_problems.append({"object": n, **m})
    stage_status["4_rig"] = "PARTIAL (deformation tests gated on Phase 1)"

    # ----------------------------------------------------------- stage 5: findings
    unresolved_q = []
    F.add(title="Facing is -Y by every geometric cue, but the side labels name +X as 'right'",
          rule_category="E_HUMAN_ACCEPTANCE", severity="HIGH", evidence_class="OWNER_DECISION_REQUIRED",
          affected=["all side-labelled bones and meshes"],
          measured={"cues": [(c["cue"], round(c["measured_value_m"], 4), c["implied_forward_y_sign"]) for c in cues],
                    "label_vs_anatomical_candidate": {"disagree": len(disagree), "agree": len(agree)}},
          method_ids=["M-FACING", "M-SIDE-PARSE"], method_status=ms("M-FACING"),
          threshold={"midline_band": cfg["tolerances"]["midline_band"]}, threshold_status="PROVISIONAL",
          interpretation="Geometry and knee.md agree the robot faces -Y, so its anatomical right is -X. "
                         "Project docs call the +X side 'right'. Either the labels are viewer-relative "
                         "(document it) or anatomically inverted (rename later). Not resolvable from data.",
          confidence_and_limits="Facing: strong (independent cues unanimous). Label intent: a design question.",
          next_action="Owner chooses candidate A (anatomical, relabel later) or V (viewer-relative, document).",
          owner_judgement_required=True)
    unresolved_q.append(convention["decision_package"]["decision"])

    F.add(title="Bone side naming does not follow NAMING.md .L/.R",
          rule_category="B_SPEC_CONSTRAINT", severity="MEDIUM", evidence_class="MEASURED_FACT",
          affected=[arm_name], measured={"side_marker_counts": dict(marker_counts)},
          method_ids=["M-SIDE-PARSE"], method_status=ms("M-SIDE-PARSE"),
          threshold="docs/NAMING.md:26-35 (.L/.R suffixes)", threshold_status="SOURCE_FACT",
          interpretation="Known and recorded as manifest D2; compliance requires the Phase 1 decision first.",
          confidence_and_limits="Name parsing only.",
          next_action="Decide convention (QA-001), then plan a reference-safe rename.",
          owner_judgement_required=True)

    if bpairs["l_marked_unpaired"] or unpaired_offmid:
        F.add(title="Bones without a mirror partner", rule_category="C_ENGINEERING_DIAGNOSTIC",
              severity="LOW", evidence_class="MEASURED_FACT", affected=bpairs["l_marked_unpaired"] + unpaired_offmid,
              measured={"l_marked_unpaired": bpairs["l_marked_unpaired"], "unsided_off_midline_unpaired": unpaired_offmid},
              method_ids=["M-SIDE-PARSE", "M-BONE-WORLD"], method_status=ms("M-SIDE-PARSE"),
              threshold={"midline_band": cfg["tolerances"]["midline_band"]}, threshold_status="PROVISIONAL",
              interpretation="May be intentional one-sided bones or missing mirrors.", confidence_and_limits="Name-based pairing.",
              next_action="Owner/spec check per bone.", owner_judgement_required=True)

    wrong_thumb = {k: v for k, v in thumbs.items() if v and v["thumb_beside"] != "Index"}
    if wrong_thumb:
        F.add(title="A thumb sits beside the Little finger instead of the Index (palm.md:180)",
              rule_category="B_SPEC_CONSTRAINT", severity="HIGH", evidence_class="MEASURED_FACT",
              affected=[v["thumb"] for v in wrong_thumb.values()],
              measured={"per_hand": thumbs},
              method_ids=["M-THUMB-ADJ"], method_status=ms("M-THUMB-ADJ"),
              threshold="specifications/hand/palm.md:179-181 'index nearest the thumb side, little furthest'",
              threshold_status="SOURCE_FACT",
              interpretation="Label-independent: each hand is compared with its own finger order. The four fingers mirror "
                             "exactly between hands, but one thumb (bones and its rigidly bound mesh) sits on the Little side, "
                             "so that hand is not the mirror of the other. Consistent with the thumb chain having been "
                             "translated rather than mirrored when the side was duplicated (HYPOTHESIS).",
              confidence_and_limits="Adjacency measured on DEF bones (qualified). Cause not established.",
              next_action="Owner confirms this is unintended; then plan a mirrored rebuild of that thumb chain (bones, mesh, "
                          "constraints) on a copy. No change made here.",
              owner_judgement_required=True)
        unresolved_q.append("Confirm the thumb on the hand listed in the thumb finding is misplaced (palm.md:180) and approve a rebuild plan.")
    if shorts:
        F.add(title="Near-zero-length bones", rule_category="C_ENGINEERING_DIAGNOSTIC", severity="MEDIUM",
              evidence_class="MEASURED_FACT", affected=[r["bone"] for r in shorts], measured={"bones": shorts},
              method_ids=["M-SHORT-BONE"], method_status=ms("M-SHORT-BONE"),
              threshold={"bone_min_length_for_direction": cfg["tolerances"]["bone_min_length_for_direction"]},
              threshold_status="JUSTIFIED",
              interpretation="A bone of ~2e-6 m has no numerically stable direction or roll, so its local axes are set by "
                             "float noise. These are parents of the palm/thumb chains. Whether this affects deformation or "
                             "glTF export is UNTESTED.",
              confidence_and_limits="Length measured exactly; effect is a HYPOTHESIS.",
              next_action="Check wrist.md for the intended DEF_Wrist geometry; export/pose test on a copy.",
              owner_judgement_required=True)
    fp = [r for r in over_pairs if r["finger"]]
    if over_pairs:
        F.add(title="Mirror-pair bone deviations (concentrated in finger MECH/CTRL bones)",
              rule_category="C_ENGINEERING_DIAGNOSTIC", severity="MEDIUM", evidence_class="MEASURED_FACT",
              affected=[r["pair"] for r in over_pairs[:20]],
              measured={"pairs_total": len(pair_rows), "pairs_over_tolerance": len(over_pairs),
                        "over_by_layer_and_finger": {f"{k[0]}|finger={k[1]}": v for k, v in over_by_layer.items()},
                        "worst": over_pairs[:8], "mech_ctrl_to_def_colocation_by_side": coloc_by_side},
              method_ids=["M-SYM-BONE", "M-COLOC"], method_status=ms("M-SYM-BONE"),
              threshold={"bone_pair_position": cfg["tolerances"]["bone_pair_position"],
                         "bone_pair_direction": cfg["tolerances"]["bone_pair_direction"]},
              threshold_status="PROVISIONAL",
              interpretation="fingers.md specifies chain topology, axes and DIP:PIP ratios but no bone placement "
                             "rule, so mirror symmetry is not a stated requirement. DEF finger bones mirror; the "
                             "deviations sit in MECH/CTRL. Co-location shows which side's mechanism bones sit on "
                             "their DEF fingers. Whether placement affects behaviour (local-space Copy Rotation) "
                             "needs a deformation test, which is gated.",
              confidence_and_limits="Measurement qualified; defect status UNRESOLVED (spec gap).",
              next_action="Owner/spec: state whether MECH/CTRL finger bones must co-locate with DEF; then a gated pose test.",
              owner_judgement_required=True)
        unresolved_q.append("Must MECH/CTRL finger bones co-locate with their DEF bones (fingers.md is silent on placement)?")

    if mech_ctrl_deform:
        F.add(title="MECH/CTRL bones flagged use_deform but carrying no weights",
              rule_category="C_ENGINEERING_DIAGNOSTIC", severity="MEDIUM", evidence_class="MEASURED_FACT",
              affected=mech_ctrl_deform[:30],
              measured={"mech_ctrl_use_deform": len(mech_ctrl_deform), "mech_ctrl_with_nonzero_weights": len(mech_ctrl_weighted),
                        "skinning_condition_table": {f"{k}": v for k, v in skin_summary.items()}},
              method_ids=["M-SKIN"], method_status=ms("M-SKIN"), threshold="exact (weight > 0)", threshold_status="JUSTIFIED",
              interpretation="Conditions 1 true, 2-4 false: they cannot deform the body today. Risk is export-side: "
                             "deform-flagged bones are treated as skin joints by 'deform bones only' glTF export "
                             "(docs/ENGINEERING_RULES.md §5: runtime sees DEF only). Condition 5 not tested.",
              confidence_and_limits="Conditions 1-4 measured; export effect is a HYPOTHESIS until an export test.",
              next_action="Export test with deform-only option on a copy; then owner decides to clear the flags.",
              owner_judgement_required=True)

    if def_unweighted:
        F.add(title="DEF bones with no weighted vertices",
              rule_category="C_ENGINEERING_DIAGNOSTIC", severity="LOW", evidence_class="MEASURED_FACT",
              affected=def_unweighted, measured={"count": len(def_unweighted)},
              method_ids=["M-SKIN"], method_status=ms("M-SKIN"), threshold="exact", threshold_status="JUSTIFIED",
              interpretation="Tips are leaf bones of rigid per-segment skinning (skin_rigid_v01.py binds each mesh to one bone); "
                             "palm DEF bones may be structural. HYPOTHESIS, not a defect.",
              confidence_and_limits="Design intent not recorded.", next_action="Owner/spec confirm purpose of tip and palm DEF bones.",
              owner_judgement_required=True)

    topo_flag = {n: {k: v for k, v in t.items() if k in topo_totals and v} for n, t in topo.items()}
    topo_flag = {n: v for n, v in topo_flag.items() if v}
    if topo_flag:
        F.add(title="Topology conditions present on body meshes",
              rule_category="C_ENGINEERING_DIAGNOSTIC", severity="LOW", evidence_class="MEASURED_FACT",
              affected=sorted(topo_flag), measured={"totals": topo_totals, "per_object": topo_flag},
              method_ids=["M-TOPO", "M-DUP", "M-DEGEN"], method_status=ms("M-TOPO"),
              threshold={k: cfg["tolerances"][k] for k in ("coincident_vertex", "near_duplicate_vertex", "degenerate_face_area")},
              threshold_status="JUSTIFIED (coincident, area) / PROVISIONAL (near-duplicate)",
              interpretation="Counts are facts; open boundaries on hard-surface shells may be intentional. Not defects by themselves.",
              confidence_and_limits="Normals/winding and self-intersection NOT measured (M-NORMALS, M-SELFINT not implemented).",
              next_action="Visual check of listed objects; owner decides intent.", owner_judgement_required=True)

    F.add(title="Face winding and shell orientation on body meshes",
          rule_category="C_ENGINEERING_DIAGNOSTIC", severity=("MEDIUM" if set(inside_out) & set(blender_flips)
                    else "LOW" if (wind_bad or inside_out or blender_flips) else "INFO"),
          evidence_class="MEASURED_FACT", affected=sorted(set(wind_bad) | set(inside_out) | set(blender_flips)),
          measured={"objects_checked": len(orient),
                    "closed_manifold_objects": sum(1 for r in orient.values() if r["closed_manifold"]),
                    "objects_with_inconsistent_winding": {n: r["winding_inconsistent_edges"] for n, r in wind_bad.items()},
                    "closed_shells_inside_out_in_world": sorted(inside_out),
                    "blender_recalc_would_flip_faces (independent algorithm)": blender_flips,
                    "both_methods_agree_on_inside_out": sorted(set(inside_out) & set(blender_flips))},
          method_ids=["M-NORMALS", "M-NORMALS-BLENDER"], method_status=ms("M-NORMALS"), threshold="exact", threshold_status="JUSTIFIED",
          interpretation="Winding is judged only on edges shared by exactly two faces; orientation only on closed shells. "
                         "Zero means no inconsistency was detected by these definitions. Where both independent methods agree a "
                         "closed shell is inside-out, a default FrontSide glTF/Three.js material would draw its inner side "
                         "(HYPOTHESIS, not yet tested in a renderer).",
          confidence_and_limits="Custom split normals, shading and self-intersection are not assessed.",
          next_action="None if zero; otherwise inspect the listed objects.", owner_judgement_required=False)

    F.add(title="Body mesh mirror symmetry distribution",
          rule_category="C_ENGINEERING_DIAGNOSTIC", severity="INFO" if body_sym["fraction_over_tolerance"] < 0.05 else "LOW",
          evidence_class="MEASURED_FACT", affected=[r["object"] for r in per_obj_over[:10]],
          measured={"whole_body": body_sym, "objects_with_most_vertices_over_tol": per_obj_over[:10],
                    "worst_L_R_mesh_pairs": mesh_pairs[:6]},
          method_ids=["M-SYM-MESH"], method_status=ms("M-SYM-MESH"),
          threshold={"symmetry_match": cfg["tolerances"]["symmetry_match"]}, threshold_status="PROVISIONAL",
          interpretation="Nearest-neighbour distances bound the surface mismatch; they are not vertex correspondence.",
          confidence_and_limits="Different vertex layouts inflate distances.", next_action="Review the listed objects only if asymmetry is unintended.",
          owner_judgement_required=False)

    F.add(title="Body height measured from base geometry",
          rule_category="C_ENGINEERING_DIAGNOSTIC", severity="INFO", evidence_class="MEASURED_FACT",
          affected=["body"], measured={"metrics": metrics[:5], "units_ok": units_ok,
                                       "spec_json_comparison": spec_cmp,
                                       "brief_target_1.75-1.85m": "NOT FOUND in repository (searched docs/, specifications/, docs/ENGINEERING_RULES.md)"},
          method_ids=["M-BBOX-WORLD", "M-BBOX-EVAL", "M-UNITS"], method_status=ms("M-BBOX-WORLD"),
          threshold=None, threshold_status="no approved target located",
          interpretation="Units are METRIC/1.0 and the inclusion set excludes environment and backups, so the base height is a "
                         "valid measurement of the asset. Whether it is the intended height is an owner question.",
          confidence_and_limits="Landmark = floor to highest body vertex (incl. head sensors).",
          next_action="Owner supplies the authoritative height spec if one exists; no scaling.",
          owner_judgement_required=True)
    unresolved_q.append("Where is the authoritative height target? The brief's 1.75-1.85 m was not found in the repo; "
                        + ("the scene has no SPEC_JSON property; " if spec_cmp is None else "SPEC_JSON total_height is compared in findings; ")
                        + "the manifest records 2.353 m as VERIFIED (a measurement, not a target).")

    if any(abs(r) > 1e-6 for r in arm["rotation_euler"]):
        F.add(title="Armature object carries a non-zero rotation",
              rule_category="C_ENGINEERING_DIAGNOSTIC", severity="LOW", evidence_class="MEASURED_FACT",
              affected=[arm_name], measured={"rotation_euler_rad": arm["rotation_euler"]},
              method_ids=["M-UNITS"], method_status=ms("M-UNITS"), threshold="float_compare", threshold_status="JUSTIFIED",
              interpretation="Already recorded as manifest D1. All rig measurements here use world space, so they are not distorted by it; "
                             "bone-local axes differ from world axes by this rotation.",
              confidence_and_limits="Measured.", next_action="Owner decision per phase_armature_rotation_cleanup.md.",
              owner_judgement_required=True)

    F.add(title="Constraint and modifier references",
          rule_category="A_HARD_INVARIANT", severity="BLOCKER" if (con_problems or mod_problems) else "INFO",
          evidence_class="MEASURED_FACT", affected=[c["owner"] for c in con_problems] + [m["object"] for m in mod_problems],
          measured={"constraints_checked": len(all_cons), "broken_references": con_problems,
                    "blender_is_valid_false": con_invalid_flag, "armature_modifier_problems": mod_problems,
                    "constraint_types": dict(Counter(c["type"] for c in all_cons)),
                    "pair_stack_differences": len(con_pair_diffs)},
          method_ids=["M-CONSTRAINT-REF", "M-MOD-REF"], method_status=ms("M-CONSTRAINT-REF"),
          threshold="exact", threshold_status="JUSTIFIED",
          interpretation="A missing target is a broken relationship; none found means references are intact, not that constraints are correct.",
          confidence_and_limits="Limits/axes not compared.", next_action="None if zero.",
          owner_judgement_required=False)
    if con_pair_diffs:
        F.add(title="Constraint stacks differ between mirror pairs",
              rule_category="C_ENGINEERING_DIAGNOSTIC", severity="LOW", evidence_class="MEASURED_FACT",
              affected=[d["pair"] for d in con_pair_diffs], measured={"diffs": con_pair_diffs[:10], "count": len(con_pair_diffs)},
              method_ids=["M-CONSTRAINT-PAIR"], method_status=ms("M-CONSTRAINT-PAIR"),
              threshold="influence to 1e-6", threshold_status="JUSTIFIED",
              interpretation="Configuration differs; which side is correct is a Phase 1/spec question.",
              confidence_and_limits="Type, subtarget, influence, spaces, mute only.", next_action="Review listed pairs.",
              owner_judgement_required=True)

    if class_counts.get("unresolved") or class_counts.get("excluded_backup"):
        F.add(title="Objects excluded from body measurements",
              rule_category="C_ENGINEERING_DIAGNOSTIC", severity="INFO", evidence_class="MEASURED_FACT",
              affected=sorted(n for n, c in classes.items() if c["class"] in ("unresolved", "excluded_backup")),
              measured={"class_counts": dict(class_counts),
                        "excluded": {n: c for n, c in classes.items() if c["class"] in ("unresolved", "excluded_backup")}},
              method_ids=["M-CLASSIFY"], method_status=ms("M-CLASSIFY"), threshold=None, threshold_status="PROPOSED_RULE",
              interpretation="Excluded by rule, not by judgement of their design.", confidence_and_limits="Rule-based.",
              next_action="Owner confirms exclusions.", owner_judgement_required=True)

    F.add(title="Controlled deformation tests not run",
          rule_category="E_HUMAN_ACCEPTANCE", severity="INFO", evidence_class="UNKNOWN_NOT_MEASURED",
          affected=["skinning condition 5"], measured={}, method_ids=["M-DEFORM-TEST"], method_status=ms("M-DEFORM-TEST"),
          threshold=None, threshold_status="n/a",
          interpretation="Phase 1 gate not passed (side convention needs owner decision).",
          confidence_and_limits="Actual deformation contribution unknown.",
          next_action="After the decision: pose tests on an in-memory copy, never saved.", owner_judgement_required=False)

    # ---- appended findings (mesh analysis); appended last so earlier finding IDs do not shift
    tolmap = {k: cfg["tolerances"][k] for k in ("tri_plane_coplanar", "tri_contact", "degenerate_face_area", "coincident_vertex")}
    deg_meshes = {r["mesh"]: r["degenerate_triangles_excluded"] for r in self_rows if r["degenerate_triangles_excluded"]}
    F.add(title="Body mesh self-intersection analysis",
          rule_category="C_ENGINEERING_DIAGNOSTIC", severity="LOW" if self_def_total else "INFO",
          evidence_class="MEASURED_FACT", affected=self_meshes_def,
          measured={"meshes_analysed": len(self_rows), "active_triangles": sum(r["active_triangles"] for r in self_rows),
                    "total_definite_pairs": self_def_total,
                    "meshes_with_definite": [{"mesh": r["mesh"], "definite": r["definite_count"], "counts": r["counts"],
                                              "examples": r["examples_definite"],
                                              "reliability_notes": r["reliability_notes"],
                                              "blender_bvh_confirms": r.get("blender_bvh_contains_all_listed_definite")}
                                             for r in self_rows if r.get("definite_count")][:25],
                    "meshes_with_contact_only": [{"mesh": r["mesh"], "contact": r["contact_count"], "examples": r["examples_contact"]}
                                                 for r in self_rows if r.get("contact_count") and not r.get("definite_count")][:25],
                    "degenerate_triangles_excluded": deg_meshes, "undetermined_meshes": self_undetermined},
          method_ids=["M-SELFINT", "M-BVH", "M-TRI-PAIR"], method_status=ms("M-SELFINT"),
          threshold=tolmap, threshold_status="JUSTIFIED (numerical tolerances); no project requirement on self-intersection found",
          interpretation=("Definite triangle intersections exist within the listed meshes; whether they are construction artefacts or "
                          "intentional is unknown, so severity stays LOW." if self_def_total else
                          "No self-intersection was detected by these methods and tolerances. This is not proof that none exist."),
          confidence_and_limits="Triangle relations are exact to tolerance; adjacency uses welded vertices; base geometry only; excluded "
                                "degenerate triangles are listed; see mesh_analysis.json known_limitations.",
          next_action="Inspect the listed triangle pairs (mesh, triangle, face index, world location) in Blender before deciding.",
          owner_judgement_required=bool(self_def_total))
    F.add(title="Body mesh pair intersections",
          rule_category="C_ENGINEERING_DIAGNOSTIC", severity="INFO", evidence_class="MEASURED_FACT",
          affected=sorted({x for r in definite_pairs for x in (r["a"], r["b"])}),
          measured={"pairs_total": pairs_total, "status_counts": dict(sorted(status_counts.items())),
                    "definite_pairs": [{"a": r["a"], "b": r["b"], "counts": r["counts"], "definite": r["definite_count"],
                                        "volumetric_interpretation_reliable": r["volumetric_interpretation_reliable"],
                                        "reliability_notes": r["reliability_notes"], "examples": r["examples_definite"]}
                                       for r in definite_pairs][:40],
                    "possible_pairs_contact_only": [{"a": r["a"], "b": r["b"], "contact": r["contact_count"],
                                                     "examples": r["examples_contact"]} for r in possible_pairs][:40],
                    "unable_to_determine": pair_undetermined, "independent_crosscheck_vs_blender_overlap": xcheck},
          method_ids=["M-INTERMESH", "M-BVH", "M-XCHECK", "M-TRI-PAIR"], method_status=ms("M-INTERMESH"),
          threshold=tolmap, threshold_status="JUSTIFIED (numerical tolerances); no project requirement on body-part overlap found",
          interpretation="Overlaps between adjacent body components are common in hard-surface construction (housings over limbs); "
                         "the requirement is unknown, so these are recorded as INFO facts, not defects.",
          confidence_and_limits="Triangle-level facts; penetration depth not measured; open meshes flagged per pair; base geometry only.",
          next_action="Owner reviews definite pairs against intended construction; confirm which overlaps are by design.",
          owner_judgement_required=bool(definite_pairs))
    F.add(title="Body mesh proximity (raw minimum separations)",
          rule_category="C_ENGINEERING_DIAGNOSTIC", severity="INFO", evidence_class="MEASURED_FACT",
          affected=sorted({x for r in prox_rows[:20] for x in (r["a"], r["b"])}),
          measured={"search_radius_m": radius, "search_radius_status": "EXPERIMENTAL",
                    "pairs_within_radius": len(prox_rows), "touching_within_numerical_tolerance": len(touching),
                    "closest_20": prox_rows[:20]},
          method_ids=["M-DISTANCE", "M-BVH", "M-PROX-CLASS"], method_status=ms("M-DISTANCE"),
          threshold={"proximity_search_radius": cfg["tolerances"]["proximity_search_radius"]},
          threshold_status="EXPERIMENTAL (reporting scope only; no approved proximity requirement exists)",
          interpretation="Raw minimum separations between non-intersecting body mesh pairs. A small distance is not evidence of a "
                         "defect; no threshold-based classification is made (M-PROX-CLASS is EXPERIMENTAL).",
          confidence_and_limits="Distances exact to float precision within the search radius; farther pairs are not measured.",
          next_action="Review only if a clearance requirement is defined.", owner_judgement_required=False)

    F.add(title="Solid overlap volumes and axial separation bounds for intersecting body pairs",
          rule_category="C_ENGINEERING_DIAGNOSTIC", severity="INFO",
          evidence_class="MEASURED_FACT" if qualified_pen else "UNKNOWN_NOT_MEASURED",
          affected=F_pen_affected,
          measured={"status_counts": pen_doc["status_counts"], "class_counts": pen_doc["class_counts"],
                    "separation_status_counts": pen_doc["separation_status_counts"],
                    "geometry_basis_counts": pen_doc["geometry_basis_counts"],
                    "top_by_volume_m3": [[a, b, f"{v:.6g}"] for a, b, v in pen_doc["ranked_by_volume"][:10]],
                    "top_by_axial_separation_bound_m": [[a, b, f"{v:.6g}"] for a, b, v in pen_doc["ranked_by_axial_separation_bound"][:10]],
                    "group_summary": {g: {**s, "max_volume_m3": None if s["max_volume_m3"] is None else f"{s['max_volume_m3']:.6g}"}
                                      for g, s in pen_doc["group_summary"].items()},
                    "errors": pen_errors, "parameters": sol_params},
          method_ids=["M-SOL-VOLUME", "M-SOL-DEPTH-AXIAL", "M-SOL-REFINE", "M-INTERMESH"], method_status=pen_method_status,
          threshold=None, threshold_status="no approved penetration limit located in the repository",
          interpretation="Volumes and axial separation bounds are geometric measurements of each overlap. A large value is "
                         "not a defect: no penetration limit is approved, and housing-over-shell overlaps may be intentional "
                         "construction. The two inverted shells (QA-009) were measured on reversed in-memory copies only.",
          confidence_and_limits="Volumes need both kernels to agree and the identity V(AnB)+V(A-B)=V(A) to hold; axial values are "
                                "upper bounds on the minimum translation distance (not exact depths) for non-box shapes. Pairs whose only "
                                "defect is collapsed (zero-area) triangles are measured on a refined in-memory copy and validated only if the "
                                "unrefined shells give the same volume. Open, non-manifold, winding-inconsistent or non-refinable pairs are refused.",
          next_action="Owner reviews the ranked overlaps against intended construction; no threshold is applied.",
          owner_judgement_required=bool(validated))

    # ----------------------------------------------------------- write outputs
    W = lambda name, data: evidence.write_json(os.path.join(out, name), data)  # noqa: E731
    W("coordinate_convention.json", convention)
    W("side_mapping.json", {"plane_x": plane_x, "midline_band": cfg["tolerances"]["midline_band"],
                            "forward_y_sign_candidate": forward_sign, "rows": smap,
                            "summary": [{"kind": k[0], "label_side": k[1], "physical_side": k[2],
                                         "anatomical_candidate": k[3], "count": v} for k, v in sorted(smap_counts.items(), key=str)]})
    W("measurement_methods.json", {"config_version": cfg["config_version"], "methods": methods,
                                   "tolerances": cfg["tolerances"],
                                   "rule": "Only QUALIFIED or QUALIFIED_WITH_LIMITATIONS methods may support MEASURED_FACT/VERIFIED_RESULT findings (enforced in rules.validate_finding)."})
    os.makedirs(os.path.join(out, "tests"), exist_ok=True)
    for src_res in (args.unit_results, args.fixture_results):
        if src_res and os.path.exists(src_res):
            with open(src_res, encoding="utf-8") as fh:
                W(os.path.join("tests", os.path.basename(src_res)), json.load(fh))
    W("test_results.json", {"tests": [{"id": k, "result": v} for k, v in sorted(tr.items())],
                            "all_pass": all(v == "PASS" for v in tr.values()) and bool(tr)})
    W("geometry.json", {"inventory": inventory, "class_counts": dict(class_counts), "metrics": metrics,
                        "spec_json_comparison": spec_cmp, "evaluated_bounds": {k: v for k, v in evaluated.items() if k != "per_object"},
                        "topology": topo, "topology_totals": topo_totals, "orientation": orient,
                        "symmetry": {"whole_body_self_mirror": body_sym, "per_object_over_tolerance": per_obj_over,
                                     "l_r_mesh_pairs": mesh_pairs}})
    W("rig.json", {"graph": graph, "layer_counts": dict(layer_counts), "side_marker_counts": dict(marker_counts),
                   "pairs": bpairs, "pair_metrics": pair_rows, "unsided_off_midline_unpaired": unpaired_offmid,
                   "thumb_adjacency": thumbs, "short_bones": shorts,
                   "finger_mech_ctrl_colocation": coloc, "colocation_by_side": coloc_by_side,
                   "skinning": skin, "weight_stats": weight_stats,
                   "constraints": {"problems": con_problems, "pair_diffs": con_pair_diffs},
                   "armature": {k: v for k, v in arm.items() if k not in ("bones", "pose_constraints")}})
    W("findings.json", {"schema_version": SCHEMA_VERSION, "severity_definitions": cfg["severity_definitions"],
                        "findings": F.items, "unresolved_questions": unresolved_q})
    W("unresolved_questions.json", {"questions": unresolved_q})
    W("mesh_analysis.json", mesh_analysis)
    evidence.write_json_sig(os.path.join(out, "penetration.json"), pen_doc)
    evidence.write_json_sig(os.path.join(out, "refinement_preservation.json"),
                            {"refine_version": refine.VERSION, "solid_version": solid.VERSION,
                             "note": "Full preservation reports for every pair measured on a refined in-memory copy. The source .blend is not modified.",
                             "pairs": ref_full})
    write_penetration_summary(out, args.run_id, pen_doc)

    diag = {"rendered": False, "reason": "--no-render"}
    if not args.no_render:
        try:
            diag = diagnostics.render_views(out, set(body_names))
        except Exception:
            diag = {"rendered": False, "error": traceback.format_exc()}
    W("diagnostic_views.json", diag)

    ident_after = evidence.file_identity(src)
    if ident_after["sha256"] != ident_before["sha256"]:
        fail("SOURCE HASH CHANGED DURING RUN - outputs invalid, source NOT restored; investigate")

    write_summary(out, args.run_id, ident_before, scene, metrics, convention, methods, tr, F.items,
                  class_counts, graph, stage_status, unresolved_q, diag, spec_cmp)
    manifest = {
        "run_id": args.run_id, "qa_version": QA_VERSION, "schema_version": SCHEMA_VERSION,
        "config_version": cfg["config_version"], "baseline_run": args.baseline_run,
        "source_before": ident_before, "source_after": ident_after, "source_unchanged": True,
        "blender_version": bpy.app.version_string, "python": sys.version.split()[0],
        "started_utc": started, "finished_utc": evidence.utc_now(),
        "stage_status": stage_status,
        "scripts": {os.path.relpath(p, os.path.dirname(HERE)).replace("\\", "/"): evidence.sha256_file(p)
                    for p in sorted(_py_files(HERE))},
        "outputs": evidence.hash_outputs(out, skip=("run_manifest.json",)),
    }
    W("run_manifest.json", manifest)
    if args.ledger:
        evidence.append_ledger(args.ledger, {
            "run_id": args.run_id, "finished_utc": manifest["finished_utc"], "status": "MEASURED",
            "kind": "humanoid_qa", "qa_version": QA_VERSION, "source_sha256": ident_before["sha256"],
            "manifest_sha256": evidence.sha256_file(os.path.join(out, "run_manifest.json")),
            "out_dir": os.path.relpath(out, os.path.dirname(os.path.abspath(args.ledger))).replace("\\", "/"),
            "builds_on": args.baseline_run})
    print("HUMANOID_QA OK", args.run_id, "findings=", len(F.items), "source_unchanged=True")


def _py_files(root):
    for d, _, fs in os.walk(root):
        if "__pycache__" in d:
            continue
        for f in fs:
            if f.endswith(".py") or f.endswith(".json") and "config" in d:
                yield os.path.join(d, f)


def write_summary(out, run_id, ident, scene, metrics, conv, methods, tr, findings, class_counts,
                  graph, stage_status, questions, diag, spec_cmp):
    L = [f"# 411 Humanoid QA — {run_id}", "",
         f"Source `{ident['path']}` — SHA-256 `{ident['sha256']}` ({ident['bytes']} bytes, mtime {ident['mtime_utc']}), "
         f"unchanged after the run. Blender {scene['blender_version']}. Units {scene['unit_system']} / scale {scene['scale_length']}.", "",
         "## Stage status", ""]
    L += [f"- {k}: {v}" for k, v in stage_status.items()]
    L += ["", "## Coordinate convention (Phase 1)", "",
          f"Outcome: **{conv['outcome']}**. Forward-Y sign votes from geometric cues: {conv['geometric_cue_agreement']['forward_y_sign_votes']}.", ""]
    L += [f"- {c['cue']}: {c['description']} = {c['measured_value_m']:+.4f} m → forward sign {c['implied_forward_y_sign']:+d}" for c in conv["geometric_cues"]]
    L += ["", f"Conflict: {conv['derived']['conflict']}", "", f"**Decision needed:** {conv['decision_package']['decision']}", "",
          "## Headline metrics", "", "| metric | value | unit | method (status) | limits |", "|---|---|---|---|---|"]
    for m in metrics:
        v = m["value"]
        L.append(f"| {m['metric']} | {v:.4f} | {m['unit']} | {m['method']} ({m['method_status']}) | {m['limitations']} |"
                 if isinstance(v, float) else f"| {m['metric']} | {v} | {m['unit']} | {m['method']} ({m['method_status']}) | {m['limitations']} |")
    if spec_cmp:
        L += ["", f"SPEC_JSON total_height = {spec_cmp['spec_value']} vs measured {spec_cmp['measured_value']:.4f} "
                  f"(difference {spec_cmp['difference']:+.4f}); comparison method {spec_cmp['method_status']}."]
    L += ["", f"Object classes: {dict(class_counts)}. Bones {graph['bone_count']}, roots {graph['roots']}, "
              f"components {graph['component_count']}, cycles {len(graph['cycle_nodes'])}.", "",
          "## Method qualification", "", "| method | status | tests |", "|---|---|---|"]
    for mid, m in methods.items():
        L.append(f"| {mid} | {m['status']} | {', '.join(f'{k}:{v}' for k, v in m['test_results'].items()) or '—'} |")
    L += ["", f"Tests: {sum(1 for v in tr.values() if v == 'PASS')}/{len(tr)} PASS.", "", "## Findings", ""]
    for f in findings:
        L += [f"### {f['id']} — {f['title']}", "",
              f"- {f['severity']} · {f['rule_category']} · {f['evidence_class']} · methods {f['method_ids']} ({f['method_status']}) · threshold {f['threshold_status']}",
              f"- Interpretation: {f['interpretation']}", f"- Limits: {f['confidence_and_limits']}",
              f"- Next: {f['next_action']} · owner judgement: {f['owner_judgement_required']}", ""]
    L += ["## Unresolved questions", ""] + [f"- {q}" for q in questions]
    L += ["", "## Diagnostic views", ""]
    if isinstance(diag, dict) and diag.get("rendered") is False:
        L.append(f"Not rendered: {diag.get('reason') or diag.get('error', '')[:300]}")
    else:
        for k, v in diag.items():
            L.append(f"- `{v['file']}` — looking {v['view_direction_world']}, image-right = world {v['image_right_world']}")
    with open(os.path.join(out, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")


main()
