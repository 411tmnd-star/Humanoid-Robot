"""
run_experiments.py  (humanoid_qa_v01.0)

Purpose: isolated pose-response and glTF-export experiments on a DISPOSABLE
COPY of the source. Launch Blender WITHOUT the source loaded:

  blender --background --factory-startup --disable-autoexec --python-exit-code 1 ^
    --python scripts/humanoid_qa/run_experiments.py -- --source <file.blend> ^
    --out-dir <run_dir> --run-id <id> --ledger <ledger.jsonl> ^
    --unit-results <u.json> --fixture-results <b.json> [--angle 30]

Guarantees: the source is only read (copied); the copy is opened, never saved;
source and copy hashes are verified before and after; any difference exits 1.
Responses are reported by PHYSICAL side only. Anatomical side is routed
through core.gate and stays UNRESOLVED until the owner resolves the convention.
"""

import argparse
import os
import sys
import tempfile

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from humanoid_qa import QA_VERSION, SCHEMA_VERSION  # noqa: E402
from humanoid_qa.blender import experiments, extract  # noqa: E402
from humanoid_qa.core import classify, evidence, gate, rig, rules  # noqa: E402


def fail(msg):
    print("HUMANOID_EXP FAILED:", msg)
    raise SystemExit(1)


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--ledger", default=None)
    ap.add_argument("--config", default=os.path.join(HERE, "config", "quality_rules.json"))
    ap.add_argument("--unit-results", default=None)
    ap.add_argument("--fixture-results", default=None)
    ap.add_argument("--angle", type=float, default=30.0)
    return ap.parse_args(argv)


def main():
    args = parse_args()
    src = os.path.abspath(args.source)
    if not os.path.exists(src):
        fail(f"source not found: {src}")
    out = args.out_dir
    if os.path.exists(out) and os.listdir(out):
        fail(f"run directory exists and is not empty: {out}")
    os.makedirs(out, exist_ok=True)
    ident_before = evidence.file_identity(src)
    started = evidence.utc_now()

    cfg = rules.load_config(args.config)
    tr = rules.load_test_results([args.unit_results, args.fixture_results])
    methods = rules.qualify_methods(cfg, tr)
    band = cfg["tolerances"]["midline_band"]["value"]
    plane_x = cfg["coordinate_convention"]["mirror_plane"]["value"]

    iso = experiments.open_isolated_copy(src)          # replaces the empty startup scene
    arm_name = cfg["classification"]["armature_object"]
    arm_obj = bpy.data.objects.get(arm_name)
    if arm_obj is None:
        fail(f"armature {arm_name} missing in copy")
    classes = classify.classify_all(extract.objects_info(), cfg)
    body_objs = [bpy.data.objects[n] for n, c in sorted(classes.items()) if c["class"] == "body"]

    bones = {b["name"]: b for b in extract.armature_info(arm_obj)["bones"]}
    test_bones = experiments.default_test_bones(arm_obj)
    pose = experiments.run_pose_experiments(arm_obj, body_objs, test_bones, args.angle)

    def side_of_bone(name):
        return rig.physical_side(bones[name]["head_w"][0], plane_x, band) if name in bones else None

    for r in pose["results"]:
        r["physical_side"] = side_of_bone(r["bone"])
        r["anatomical_side"] = gate.anatomical_side(cfg, None, r["physical_side"], None)
        r["max_mesh_displacement_m"] = max(r["moved_meshes"].values(), default=0.0)
        r["moved_mesh_count"] = len(r["moved_meshes"])

    by_name = {r["bone"]: r for r in pose["results"]}
    pairs = []
    for p in rig.pair_by_name([r["bone"] for r in pose["results"]])["pairs"]:
        a, b = by_name[p["l_marked"]], by_name[p["counterpart"]]
        base = lambda lst: sorted(rig.parse_side(n)[1] for n in lst)  # noqa: E731
        pairs.append({
            "pair": [p["l_marked"], p["counterpart"]],
            "physical_sides": [a["physical_side"], b["physical_side"]],
            "moved_def_bones_same_base_set": base(a["moved_def_bones"]) == base(b["moved_def_bones"]),
            "moved_def_bone_counts": [len(a["moved_def_bones"]), len(b["moved_def_bones"])],
            "moved_mesh_counts": [a["moved_mesh_count"], b["moved_mesh_count"]],
            "max_displacement_m": [a["max_mesh_displacement_m"], b["max_mesh_displacement_m"]],
            "displacement_ratio": (a["max_mesh_displacement_m"] / b["max_mesh_displacement_m"]
                                   if b["max_mesh_displacement_m"] > 0 else None),
        })
    pairs.sort(key=lambda r: (r["moved_def_bones_same_base_set"], r["pair"][0]))
    drive_nothing = sorted(r["bone"] for r in pose["results"] if not r["moved_meshes"])
    restore_all = all(r["restore_verified"] for r in pose["results"])

    probe_dir = tempfile.mkdtemp(prefix="hqa_probe_")
    export = experiments.export_probe(arm_obj, body_objs, probe_dir)
    export_notes = export.pop("_notes", []) if isinstance(export, dict) else []
    layer_joint_view = {}
    for k, v in export.items():
        if not isinstance(v, dict):
            continue
        sk = v.get("summary", {}).get("skins", [])
        layer_joint_view[k] = {"error": v.get("error"),
                               "joints": sk[0]["joint_count"] if sk else None,
                               "layers": sk[0]["layers"] if sk else None}

    # integrity: source untouched, copy never saved
    ident_after = evidence.file_identity(src)
    copy_after = evidence.sha256_file(iso["copy_path"])
    if ident_after["sha256"] != ident_before["sha256"]:
        fail("SOURCE HASH CHANGED - investigate; not restored")
    if copy_after != iso["copy_sha256"]:
        fail("isolated copy was modified on disk (a save occurred)")

    exp_findings = []
    dj, aj = layer_joint_view.get("deform_bones_only", {}), layer_joint_view.get("all_bones", {})
    if dj.get("layers") and aj.get("layers"):
        extra = {k: dj["layers"].get(k, 0) for k in ("MECH", "CTRL")}
        exp_findings.append({
            "title": "glTF 'deform bones only' export still writes MECH/CTRL bones as skin joints",
            "evidence_class": "MEASURED_FACT", "method_ids": ["M-EXPORT-PROBE"],
            "method_status": methods["M-EXPORT-PROBE"]["status"], "severity": "MEDIUM",
            "measured": {"all_bones_joints": aj["joints"], "deform_only_joints": dj["joints"],
                         "deform_only_layers": dj["layers"], "mech_ctrl_in_deform_only": extra,
                         "blender_version": bpy.app.version_string},
            "interpretation": "use_deform on MECH/CTRL bones makes the exporter keep them as joints even with the deform-only "
                              "option, contrary to the docs/ENGINEERING_RULES.md section 5 intent that the runtime sees the DEF layer only. "
                              "Whether the extra joints cause Three.js problems is UNTESTED.",
            "confidence_and_limits": "Single exporter version and option set, isolated copy, animations excluded.",
            "next_action": "Owner decides whether to clear use_deform on MECH/CTRL bones (no weights depend on them; see QA run).",
            "owner_judgement_required": True})
    result = {
        "schema_version": SCHEMA_VERSION, "findings": exp_findings, "isolated_copy": {**iso, "sha256_after": copy_after, "saved": False},
        "gate": gate.gated(cfg, "anatomical side labelling of responses", lambda: None),
        "pose_experiments": pose, "pair_comparison": pairs,
        "bones_driving_no_body_mesh": drive_nothing, "all_restores_verified": restore_all,
        "export_probe": {"notes": export_notes, "glb_files": {k: v.get("path") for k, v in export.items() if isinstance(v, dict)},
                         "probe_dir": probe_dir, "joint_layers": layer_joint_view, "raw": export},
        "method_status": {m: methods[m]["status"] for m in ("M-DEFORM-TEST", "M-EXPORT-PROBE") if m in methods},
    }
    evidence.write_json(os.path.join(out, "experiments.json"), result)

    L = [f"# Isolated experiments — {args.run_id}", "",
         f"Source `{ident_before['path']}` SHA-256 `{ident_before['sha256']}` unchanged. Experiments ran on a disposable copy "
         f"(`{iso['copy_sha256'][:12]}…`, never saved). Notes: {'; '.join(pose['notes']) or 'none'}.", "",
         f"Pose test: {args.angle}° about {pose['axis']} on each CTRL flex-type bone, evaluated mesh displacement of the "
         f"{len(body_objs)} body meshes, pose then restored. Restores verified for all {len(pose['results'])}: **{restore_all}**.",
         f"Method status: {result['method_status']}. Anatomical side of responses: {result['gate']['status']} (gate).", "",
         "## Mirror-pair response (label-independent, by physical side)", "",
         "| pair (L-marked, counterpart) | sides | same moved DEF set | moved meshes | max disp (m) |", "|---|---|---|---|---|"]
    for p in pairs:
        L.append(f"| {p['pair'][0]} / {p['pair'][1]} | {p['physical_sides']} | {p['moved_def_bones_same_base_set']} | "
                 f"{p['moved_mesh_counts']} | {[round(x, 4) for x in p['max_displacement_m']]} |")
    L += ["", f"## Bones that moved no body mesh ({len(drive_nothing)})", "", ", ".join(drive_nothing) or "none", "",
          "## glTF export probe (isolated copy, selected body + armature)", ""]
    for k, v in layer_joint_view.items():
        L.append(f"- {k}: " + (f"ERROR {v['error']}" if v["error"] else f"{v['joints']} joints, layers {v['layers']}"))
    L += [f"- note: {n}" for n in export_notes]
    if "error" in export:
        L.append(f"- probe error: {export['error']}")
    with open(os.path.join(out, "summary.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")

    manifest = {
        "run_id": args.run_id, "kind": "experiments", "qa_version": QA_VERSION, "schema_version": SCHEMA_VERSION,
        "config_version": cfg["config_version"], "source_before": ident_before, "source_after": ident_after,
        "source_unchanged": True, "isolated_copy": iso, "blender_version": bpy.app.version_string,
        "started_utc": started, "finished_utc": evidence.utc_now(),
        "outputs": evidence.hash_outputs(out, skip=("run_manifest.json",)),
    }
    evidence.write_json(os.path.join(out, "run_manifest.json"), manifest)
    if args.ledger:
        evidence.append_ledger(args.ledger, {
            "run_id": args.run_id, "finished_utc": manifest["finished_utc"], "status": "MEASURED",
            "kind": "humanoid_qa_experiments", "qa_version": QA_VERSION, "source_sha256": ident_before["sha256"],
            "manifest_sha256": evidence.sha256_file(os.path.join(out, "run_manifest.json")),
            "out_dir": os.path.relpath(out, os.path.dirname(os.path.abspath(args.ledger))).replace("\\", "/")})
    print("HUMANOID_EXP OK", args.run_id, "pose_tests=", len(pose["results"]), "restores_ok=", restore_all)


main()
