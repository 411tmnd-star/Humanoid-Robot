"""
run_refine.py  (humanoid_refine_v01.0)

Purpose: resolve the pairs that solid_v01 left UNABLE_TO_DETERMINE because an operand
failed the DEGENERATE_FACE gate, by trying the least invasive in-memory refinements in
order and recording every experiment. Launch Blender with the source loaded (read-only):

  blender --background --factory-startup --disable-autoexec <source.blend> ^
    --python-exit-code 1 --python scripts/humanoid_qa/run_refine.py -- ^
    --out-dir <dir> --run-id <id> --ledger <ledger.jsonl> --baseline-run <qa run dir> ^
    [--unit-results u.json --fixture-results b.json]

Guarantees: the .blend is never saved (hash + mtime verified before and after); all
geometry is copied into plain lists and measured on temporary in-memory objects that are
removed afterwards; nothing is repaired in the asset; an existing output directory is
refused, never overwritten; one line is appended to the evidence ledger.

Per affected pair (taken from the baseline run's penetration.json, status
UNABLE_TO_DETERMINE):
  CAUSE   gate reasons per operand, degenerate triangles and zero-length edges with the
          vertex ids, positions, gaps and whether the gap is bit-exact
  E0      unrefined shells through both Boolean kernels with the gate bypassed (diagnostic:
          shows what the kernels do on the ORIGINAL geometry; not a validated measurement)
  E1      exact reindexing of bit-identical vertices
  E2      removal of degenerate faces alone
  E3      collapse of zero-length edges (cluster centroid) + preservation report
  E3s     sensitivity of the refined volume to the cluster representative (first/last/centroid)
  E4      kernel-free grid estimate of the overlap volume on the ORIGINAL meshes
  FINAL   solid.measure_pair with production parameters (the same call run_qa makes)
"""

import argparse
import json
import math
import os
import sys
import time

import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # scripts/

from humanoid_qa import QA_VERSION, SCHEMA_VERSION  # noqa: E402
from humanoid_qa.blender import extract, volume as bvol  # noqa: E402
from humanoid_qa.core import evidence, geom, refine, rules, solid  # noqa: E402

VERSION = "humanoid_refine_v01.0"


def fail(msg):
    print("HUMANOID_REFINE FAILED:", msg)
    raise SystemExit(1)


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--ledger", default=None)
    ap.add_argument("--config", default=os.path.join(HERE, "config", "quality_rules.json"))
    ap.add_argument("--baseline-run", required=True, help="QA run directory whose penetration.json defines the affected pairs")
    ap.add_argument("--unit-results", default=None)
    ap.add_argument("--fixture-results", default=None)
    return ap.parse_args(argv)


# ------------------------------------------------------------- helpers

def mesh_data(name):
    o = bpy.data.objects[name]
    d = extract.mesh_tris(o)
    me = o.data
    return {"world": [tuple(p) for p in d["world"]], "tris": [tuple(t) for t in d["tris"]],
            "polygons": len(me.polygons), "edges_bl": len(me.edges), "vertices_bl": len(me.vertices),
            "modifiers": [m.type for m in o.modifiers], "attributes": sorted(a.name for a in me.attributes if not a.name.startswith(".")),
            "matrix_is_identity": all(abs(o.matrix_world[i][j] - (1.0 if i == j else 0.0)) < 1e-9
                                      for i in range(4) for j in range(4))}


def defect_trace(coords, tris, area_tol, edge_tol):
    """Where the gate's degenerate faces and zero-length edges are, with exact gaps."""
    degenerate = []
    for k, t in enumerate(tris):
        a = refine._tri_area(coords, t)
        if a <= area_tol:
            degenerate.append({"triangle": k, "vertices": list(t), "area_m2": a,
                               "edge_lengths_m": [geom.dist(coords[t[i]], coords[t[(i + 1) % 3]]) for i in range(3)]})
    short = []
    for u, v in sorted(refine._edges_of(tris)):
        d = geom.dist(coords[u], coords[v])
        if d <= edge_tol:
            short.append({"vertices": [u, v], "gap_m": d, "bit_identical": tuple(coords[u]) == tuple(coords[v]),
                          "positions": [list(coords[u]), list(coords[v])],
                          "per_axis_gap_m": [abs(coords[u][k] - coords[v][k]) for k in range(3)]})
    return {"degenerate_triangles": degenerate, "zero_length_edges": short}


def kernel_volumes(coords_a, tris_a, coords_b, tris_b, null, tag):
    """Both kernels, kinds AB/BA/AMB, on oriented in-memory copies (gate NOT applied)."""
    oa, ia = solid.orient_components(coords_a, tris_a, null)
    ob, ib = solid.orient_components(coords_b, tris_b, null)
    be = bvol.BooleanBackend(coords_a, oa, coords_b, ob, tag)
    try:
        vols = {s: {k: be.volume(k, s) for k in ("AB", "BA", "AMB")} for s in be.solvers}
    finally:
        be.close()
    va = geom.signed_volume(coords_a, oa)
    p = be.solvers[0]
    v = vols[p]["AB"]
    sc = max(abs(v), 1e-12)
    spread = max(vols[s]["AB"] for s in be.solvers) - min(vols[s]["AB"] for s in be.solvers)
    return {"volumes": vols, "V_A_oriented_m3": va,
            "kernel_agreement_rel": spread / max(max(abs(vols[s]["AB"]) for s in be.solvers), 1e-12),
            "symmetry_rel": abs(vols[p]["AB"] - vols[p]["BA"]) / max(abs(vols[p]["AB"]), abs(vols[p]["BA"]), 1e-12),
            "identity_rel": abs(vols[p]["AB"] + vols[p]["AMB"] - va) / max(abs(va), 1e-12),
            "orientation": {"A_reversed": ia["reversed_components"], "B_reversed": ib["reversed_components"]}}


def inside(bvh, p, d=Vector((0.5773, 0.3137, 0.7531)).normalized()):
    """Ray-parity point-in-solid test (closed mesh). Direction is irrational-ish to avoid edge hits."""
    origin, n = Vector(p), 0
    for _ in range(64):
        hit = bvh.ray_cast(origin, d)
        if hit[0] is None:
            break
        n += 1
        origin = hit[0] + d * 1e-7
    return n % 2 == 1


def grid_overlap_volume(ca, ta, cb, tb, n_long):
    """Kernel-free estimate of V(A n B): cell-centred grid over the overlap of the two bounding boxes."""
    ba, bb = geom.bbox(ca), geom.bbox(cb)
    lo = [max(ba[0][k], bb[0][k]) for k in range(3)]
    hi = [min(ba[1][k], bb[1][k]) for k in range(3)]
    dims = [hi[k] - lo[k] for k in range(3)]
    if min(dims) <= 0:
        return {"n_long": n_long, "volume_m3": 0.0, "points": 0}
    h = max(dims) / n_long
    ns = [max(1, int(math.ceil(dims[k] / h))) for k in range(3)]
    A = BVHTree.FromPolygons([Vector(p) for p in ca], [tuple(t) for t in ta], all_triangles=True, epsilon=0.0)
    B = BVHTree.FromPolygons([Vector(p) for p in cb], [tuple(t) for t in tb], all_triangles=True, epsilon=0.0)
    cnt = 0
    for i in range(ns[0]):
        x = lo[0] + (i + 0.5) * h
        for j in range(ns[1]):
            y = lo[1] + (j + 0.5) * h
            for k in range(ns[2]):
                p = (x, y, lo[2] + (k + 0.5) * h)
                if inside(A, p) and inside(B, p):
                    cnt += 1
    return {"n_long": n_long, "cell_m": h, "grid": ns, "points": ns[0] * ns[1] * ns[2], "inside_both": cnt,
            "volume_m3": cnt * h ** 3}


def convex_oracle(coords_a, tris_a, coords_b, tris_b, null):
    try:
        oa, _ = solid.orient_components(coords_a, tris_a, null)
        ob, _ = solid.orient_components(coords_b, tris_b, null)
        be = solid.ConvexBackend(coords_a, oa, coords_b, ob)
        return {"status": "COMPUTED", "AB_m3": be.volume("AB", "PURE_CONVEX"), "BA_m3": be.volume("BA", "PURE_CONVEX")}
    except solid.NonConvexError as e:
        return {"status": "NOT_APPLICABLE", "reason": str(e)}


def prod_params(cfg):
    T = {k: rules.tol(cfg, k) for k in cfg["tolerances"]}
    return T, {"null_volume": T["penetration_null_volume"], "degenerate_area": T["degenerate_face_area"],
               "iters": int(T["penetration_bisection_iterations"]), "agree_rel": T["volume_solver_agreement_rel"],
               "identity_rel": T["volume_identity_rel"], "axial_scan_samples": int(T["penetration_axial_scan_samples"]),
               "tri_contact": T["tri_contact"], "tri_plane_coplanar": T["tri_plane_coplanar"],
               "coincident_vertex": T["coincident_vertex"], "max_candidate_pairs": int(T["max_candidate_pairs"]),
               "refine_edge_tol": T["zero_length_edge"]}


def main():
    args = parse_args()
    started = evidence.utc_now()
    src = os.path.abspath(bpy.data.filepath)
    if not src or not os.path.exists(src):
        fail("no source .blend loaded")
    out = args.out_dir
    if os.path.exists(out) and os.listdir(out):
        fail(f"output directory exists and is not empty: {out}")
    os.makedirs(out, exist_ok=True)
    ident_before = evidence.file_identity(src)

    cfg = rules.load_config(args.config)
    T, params = prod_params(cfg)
    null, area_tol, edge_tol = params["null_volume"], params["degenerate_area"], params["refine_edge_tol"]
    scene = bpy.context.scene
    if not (scene.unit_settings.system == "METRIC" and abs(scene.unit_settings.scale_length - 1.0) <= T["float_compare"]):
        fail("scene units are not metric with scale 1.0")

    base = json.load(open(os.path.join(args.baseline_run, "penetration.json"), encoding="utf-8"))
    affected = [(r["a"], r["b"], r["reasons"]) for r in base["pairs"] if r["status"] == "UNABLE_TO_DETERMINE"]
    baseline_validated = sum(1 for r in base["pairs"] if r["status"] == "VOLUME_VALIDATED")
    print(f"affected pairs: {len(affected)}; baseline validated: {baseline_validated}")

    names = sorted({n for a, b, _ in affected for n in (a, b)})
    data = {n: mesh_data(n) for n in names}
    gates = {n: solid.gate(data[n]["world"], data[n]["tris"], area_tol) for n in names}
    trace = {n: defect_trace(data[n]["world"], data[n]["tris"], area_tol, edge_tol) for n in names}
    meshes = {}
    for n in names:
        d = data[n]
        topo = geom.topology(len(d["world"]), [], d["tris"], d["world"], area_tol, edge_tol)
        meshes[n] = {"vertices": len(d["world"]), "triangles": len(d["tris"]), "polygons": d["polygons"],
                     "gate_reasons": gates[n], "topology": topo, "modifiers": d["modifiers"], "attributes": d["attributes"],
                     "matrix_world_is_identity": d["matrix_is_identity"],
                     "signed_volume_m3": geom.signed_volume(d["world"], d["tris"]),
                     "components": len(solid.components(d["tris"], len(d["world"]))),
                     "defects": trace[n]}

    # ---- operand-level experiments (the only defective mesh is refined; each distinct mesh once)
    operand = {}
    for n in names:
        if not gates[n]:
            continue
        d = data[n]
        c, t = d["world"], d["tris"]
        e1 = refine.exact_reindex(c, t)
        rep1 = refine.preservation_report(c, t, e1, area_tol, edge_tol)
        e2 = refine.remove_degenerate(c, t, area_tol)
        rep2 = refine.preservation_report(c, t, e2, area_tol, edge_tol)
        cls2, f2 = refine.accept(rep2, edge_tol)
        chain = refine.refine_for_gate(c, t, area_tol, edge_tol, lambda cc, tt: solid.gate(cc, tt, area_tol))
        variants = {}
        for rep_name in ("centroid", "first", "last"):
            r3 = refine.collapse_short_edges(c, t, edge_tol, rep_name)
            variants[rep_name] = {"result": r3, "gate_after": solid.gate(r3["coords"], r3["tris"], area_tol),
                                  "max_vertex_displacement_m": max(geom.dist(c[i], r3["coords"][r3["vmap"][i]]) for i in range(len(c)))}
        operand[n] = {
            "E1_exact_reindex": {"clusters": len(e1["clusters"]), "gate_after": solid.gate(e1["coords"], e1["tris"], area_tol),
                                 "outcome": "NO_EFFECT" if not e1["clusters"] else "APPLIED"},
            "E2_remove_degenerate_only": {"faces_removed": len(e2["removed"]), "gate_after": solid.gate(e2["coords"], e2["tris"], area_tol),
                                          "boundary_edges_after": rep2["after"]["boundary_edges"], "classification": cls2, "failures": f2},
            "E3_collapse": {"status": chain["status"], "classification": chain["class_"], "steps": chain["steps"],
                            "failures": chain["failures"], "preservation": chain["report"]},
            "_variants": variants, "_chain": chain,
        }

    pairs_out, preserve_full = [], {}
    for a, b, reasons in affected:
        t0 = time.time()
        A, B = data[a], data[b]
        rec = {"a": a, "b": b, "baseline_reasons": reasons,
               "cause": {"refused_by": "solid.gate before any Boolean kernel ran (stage 3e of solid_v01)",
                         "gate_reasons": {"A": gates[a], "B": gates[b]},
                         "kernel_failure": False}}
        # E0: the kernels on the unmodified shells
        try:
            rec["E0_unrefined_kernels_gate_bypassed"] = kernel_volumes(A["world"], A["tris"], B["world"], B["tris"], null, "e0")
        except Exception as e:  # noqa: BLE001
            rec["E0_unrefined_kernels_gate_bypassed"] = {"error": f"{type(e).__name__}: {e}"}
        # E3s sensitivity of the refined volume to the representative
        sens = {}
        defective = a if gates[a] else b
        for rep_name, v in operand[defective]["_variants"].items():
            r3 = v["result"]
            if defective == a:
                kv = kernel_volumes(r3["coords"], r3["tris"], B["world"], B["tris"], null, "e3s")
            else:
                kv = kernel_volumes(A["world"], A["tris"], r3["coords"], r3["tris"], null, "e3s")
            sens[rep_name] = {"gate_after": v["gate_after"], "max_vertex_displacement_m": v["max_vertex_displacement_m"],
                              "AB_m3": {s: kv["volumes"][s]["AB"] for s in kv["volumes"]},
                              "kernel_agreement_rel": kv["kernel_agreement_rel"], "identity_rel": kv["identity_rel"]}
        rec["E3s_representative_sensitivity"] = sens
        # E4 kernel-free grid estimate on the original meshes
        grid = [grid_overlap_volume(A["world"], A["tris"], B["world"], B["tris"], n) for n in (60, 120)]
        rec["E4_grid_estimate_original"] = grid
        # oracle
        chain = operand[defective]["_chain"]
        rc, rt = chain["result"]["coords"], chain["result"]["tris"]
        rec["E5_convex_oracle_on_refined"] = (convex_oracle(rc, rt, B["world"], B["tris"], null) if defective == a
                                              else convex_oracle(A["world"], A["tris"], rc, rt, null))
        # FINAL: production call
        res = solid.measure_pair(A["world"], A["tris"], B["world"], B["tris"], bvol.BooleanBackend.factory("final"), params)
        rec["FINAL"] = {"status": res["status"], "geometry_basis": res.get("geometry_basis"),
                        "overlap_volume_m3": res.get("overlap_volume_m3"), "volumes": res.get("volumes"),
                        "solver_agreement": res.get("solver_agreement"), "symmetry_rel_diff": res.get("symmetry_rel_diff"),
                        "identity": res.get("identity"), "refinement_effect": res.get("refinement_effect"),
                        "orientation_reversed": {k: (res.get("orientation") or {}).get(k, {}).get("reversed_components") for k in ("A", "B")},
                        "axial_separation_m": {k: v.get("t") for k, v in (res.get("axial") or {}).items()},
                        "axial_status": {k: v.get("status") for k, v in (res.get("axial") or {}).items()},
                        "axial_certificates": {k: (v.get("certificate") or {}) for k, v in (res.get("axial") or {}).items()},
                        "axial_nonmonotone_directions": res.get("axial_nonmonotone_directions"),
                        "separation_bound_m": res.get("separation_bound_m"), "separation_direction": res.get("separation_direction"),
                        "separation_status": res.get("separation_status"),
                        "separation_residual_volume_m3": res.get("separation_residual_volume_m3"),
                        "separation_cross_check": res.get("separation_cross_check")}
        # agreement of E0, FINAL and grid
        v_final = res.get("overlap_volume_m3")
        e0 = rec["E0_unrefined_kernels_gate_bypassed"]
        if v_final and "volumes" in e0:
            v0 = e0["volumes"]["MANIFOLD"]["AB"]
            rec["agreement"] = {"V_final_m3": v_final, "V_unrefined_MANIFOLD_m3": v0,
                                "final_vs_unrefined_rel": abs(v_final - v0) / max(abs(v_final), abs(v0)),
                                "grid_vs_final_rel": [abs(g["volume_m3"] - v_final) / v_final for g in grid],
                                "grid_resolution_change_rel": abs(grid[1]["volume_m3"] - grid[0]["volume_m3"]) / v_final}
        rec["seconds"] = round(time.time() - t0, 1)
        pairs_out.append(rec)
        print(f"{a} / {b}: {res['status']} V={res.get('overlap_volume_m3')} ({rec['seconds']} s)")

    for n, o in operand.items():
        o.pop("_variants")
        o.pop("_chain")
    ident_after = evidence.file_identity(src)
    if (ident_after["sha256"], ident_after["mtime_utc"], ident_after["bytes"]) != \
            (ident_before["sha256"], ident_before["mtime_utc"], ident_before["bytes"]):
        fail("source file identity changed during the run")

    status_counts = {}
    for p in pairs_out:
        status_counts[p["FINAL"]["status"]] = status_counts.get(p["FINAL"]["status"], 0) + 1
    doc = {"version": VERSION, "solid_version": solid.VERSION, "refine_version": refine.VERSION,
           "baseline_run": os.path.basename(os.path.normpath(args.baseline_run)),
           "baseline_validated": baseline_validated, "affected_pairs": len(affected),
           "parameters": params, "meshes": meshes, "operand_experiments": operand, "pairs": pairs_out,
           "final_status_counts": status_counts, "units": "m, m^3"}
    evidence.write_json_sig(os.path.join(out, "refinement_experiments.json"), doc)
    full = {n: o["E3_collapse"]["preservation"] for n, o in operand.items()}
    evidence.write_json_sig(os.path.join(out, "refinement_preservation_full.json"), full)

    L = [f"# Phase 7: inconclusive overlap pairs — {args.run_id}", "",
         f"Source `{ident_before['path']}` SHA-256 `{ident_before['sha256']}`, mtime {ident_before['mtime_utc']}: unchanged. "
         f"Baseline run `{doc['baseline_run']}` ({baseline_validated} validated, {len(affected)} UNABLE_TO_DETERMINE).", "",
         "## Cause", ""]
    for n in names:
        m = meshes[n]
        L.append(f"- `{n}`: gate {m['gate_reasons'] or 'clean'}; {m['topology']['degenerate_faces']} degenerate triangle(s), "
                 f"{m['topology']['zero_length_edges']} zero-length edge(s), boundary {m['topology']['boundary_edges']}, "
                 f"non-manifold {m['topology']['nonmanifold_edges']}, components {m['components']}, "
                 f"signed volume {m['signed_volume_m3']:.6g} m^3")
    L += ["", "The Boolean kernels were never run on these pairs by solid_v01: the gate refused them first.", "",
          "## Final status", ""]
    for p in pairs_out:
        f = p["FINAL"]
        ag = p.get("agreement", {})
        L.append(f"- {p['a']} / {p['b']}: **{f['status']}** ({f['geometry_basis']}), V = {f['overlap_volume_m3']:.6g} m^3, "
                 f"separation bound {f['separation_bound_m']:.5g} m ({f['separation_direction']}), "
                 f"refined-vs-unrefined rel diff {ag.get('final_vs_unrefined_rel', float('nan')):.2e}, "
                 f"grid vs final {[f'{x:.2%}' for x in ag.get('grid_vs_final_rel', [])]}")
    for n, o in operand.items():
        e3 = o["E3_collapse"]
        pr = e3["preservation"]
        L += ["", f"## Operand `{n}` experiments", "",
              f"- E1 exact reindex: {o['E1_exact_reindex']['clusters']} bit-identical clusters ({o['E1_exact_reindex']['outcome']}), gate after {o['E1_exact_reindex']['gate_after']}",
              f"- E2 remove degenerate faces only: {o['E2_remove_degenerate_only']['faces_removed']} removed, "
              f"{o['E2_remove_degenerate_only']['boundary_edges_after']} boundary edges after -> {o['E2_remove_degenerate_only']['classification']}",
              f"- E3 collapse: {e3['status']} / {e3['classification']}; vertices {pr['before']['vertices']}->{pr['after']['vertices']}, "
              f"triangles {pr['before']['triangles']}->{pr['after']['triangles']}, max displacement {pr['max_vertex_displacement_m']:.3g} m, "
              f"surface distance bound {pr['surface_distance_bound_m']:.3g} m, dV {pr['deltas']['signed_volume_m3']:.3g} m^3 "
              f"(swept-volume bound {pr['swept_volume_bound_m3']:.3g} m^3), dArea {pr['deltas']['area_m2']:.3g} m^2"]
    with open(os.path.join(out, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")

    manifest = {"run_id": args.run_id, "kind": "humanoid_refine", "version": VERSION, "qa_version": QA_VERSION,
                "schema_version": SCHEMA_VERSION, "config_version": cfg["config_version"],
                "source_before": ident_before, "source_after": ident_after, "source_unchanged": True,
                "blender_version": bpy.app.version_string, "started_utc": started, "finished_utc": evidence.utc_now(),
                "baseline_run": doc["baseline_run"],
                "script_sha256": {n: evidence.sha256_file(os.path.join(HERE, *n.split("/"))) for n in
                                  ("run_refine.py", "core/refine.py", "core/solid.py", "blender/volume.py", "core/intersect.py",
                                   "core/evidence.py", "config/quality_rules.json")},
                "outputs": evidence.hash_outputs(out, skip=("run_manifest.json",))}
    evidence.write_json(os.path.join(out, "run_manifest.json"), manifest)
    if args.ledger:
        evidence.append_ledger(args.ledger, {
            "run_id": args.run_id, "finished_utc": manifest["finished_utc"], "status": "MEASURED",
            "kind": "humanoid_refine", "qa_version": QA_VERSION, "source_sha256": ident_before["sha256"],
            "builds_on": doc["baseline_run"],
            "manifest_sha256": evidence.sha256_file(os.path.join(out, "run_manifest.json")),
            "out_dir": os.path.relpath(out, os.path.dirname(os.path.abspath(args.ledger))).replace("\\", "/")})
    print("HUMANOID_REFINE OK", args.run_id, status_counts)


main()
