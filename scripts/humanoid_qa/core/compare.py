"""Regression comparison between two QA run directories (pure Python).

Reads run_manifest.json, findings.json, measurement_methods.json and
geometry.json from each run. Never modifies either run.
"""

import json
import os

STATUS_RANK = {"QUALIFIED": 0, "QUALIFIED_WITH_LIMITATIONS": 1, "EXPERIMENTAL": 2,
               "FAILED_VALIDATION": 3, "NOT_IMPLEMENTED": 4}
SEVERITY_RANK = {"INFO": 0, "LOW": 1, "MEDIUM": 2, "HIGH": 3, "BLOCKER": 4}


def _load(run_dir, name):
    p = os.path.join(run_dir, name)
    if not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def load_run(run_dir):
    man = _load(run_dir, "run_manifest.json")
    if man is None:
        raise ValueError(f"{run_dir}: no run_manifest.json (not a QA run)")
    return {
        "dir": run_dir, "manifest": man,
        "findings": (_load(run_dir, "findings.json") or {}).get("findings", []),
        "methods": (_load(run_dir, "measurement_methods.json") or {}).get("methods", {}),
        "metrics": {m["metric"]: m for m in (_load(run_dir, "geometry.json") or {}).get("metrics", [])},
        "tests": {t["id"]: t["result"] for t in (_load(run_dir, "test_results.json") or {}).get("tests", [])},
    }


def compare(a, b, metric_tol=1e-6):
    """a = earlier run, b = later run (dicts from load_run)."""
    same_source = a["manifest"]["source_after"]["sha256"] == b["manifest"]["source_after"]["sha256"]
    regressions, changes = [], []

    if not same_source:
        changes.append({"kind": "source_changed", "a": a["manifest"]["source_after"]["sha256"],
                        "b": b["manifest"]["source_after"]["sha256"]})

    # metrics
    mdiff = []
    for name in sorted(set(a["metrics"]) | set(b["metrics"])):
        ma, mb = a["metrics"].get(name), b["metrics"].get(name)
        if ma is None or mb is None:
            mdiff.append({"metric": name, "change": "added" if ma is None else "removed"})
            continue
        va, vb = ma["value"], mb["value"]
        if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
            d = vb - va
            if abs(d) > metric_tol:
                rec = {"metric": name, "a": va, "b": vb, "delta": d, "unit": mb.get("unit")}
                mdiff.append(rec)
                if same_source:
                    regressions.append({"kind": "NONDETERMINISM",
                                        "detail": f"{name} changed by {d:+.3g} with an identical source hash", **rec})
        elif va != vb:
            mdiff.append({"metric": name, "a": va, "b": vb})

    # method statuses
    msd = []
    for mid in sorted(set(a["methods"]) | set(b["methods"])):
        sa = a["methods"].get(mid, {}).get("status")
        sb = b["methods"].get(mid, {}).get("status")
        if sa != sb:
            rec = {"method": mid, "a": sa, "b": sb}
            msd.append(rec)
            if sa in STATUS_RANK and sb in STATUS_RANK and STATUS_RANK[sb] > STATUS_RANK[sa] and sb != "NOT_IMPLEMENTED":
                regressions.append({"kind": "METHOD_DOWNGRADE", **rec})

    # tests
    tdiff = []
    for tid in sorted(set(a["tests"]) | set(b["tests"])):
        ra, rb = a["tests"].get(tid), b["tests"].get(tid)
        if ra != rb:
            tdiff.append({"test": tid, "a": ra, "b": rb})
            if rb in ("FAIL", "ERROR") or (ra == "PASS" and rb is None):
                regressions.append({"kind": "TEST_REGRESSION", "test": tid, "a": ra, "b": rb})

    # findings keyed by title (ids shift between runs)
    fa = {f["title"]: f for f in a["findings"]}
    fb = {f["title"]: f for f in b["findings"]}
    fdiff = []
    for t in sorted(set(fa) | set(fb)):
        x, y = fa.get(t), fb.get(t)
        if x is None:
            fdiff.append({"title": t, "change": "added", "severity": y["severity"]})
            if SEVERITY_RANK[y["severity"]] >= SEVERITY_RANK["HIGH"]:
                regressions.append({"kind": "NEW_HIGH_FINDING", "title": t, "severity": y["severity"]})
        elif y is None:
            fdiff.append({"title": t, "change": "removed", "severity": x["severity"]})
        elif x["severity"] != y["severity"] or x["evidence_class"] != y["evidence_class"]:
            fdiff.append({"title": t, "change": "modified", "severity": [x["severity"], y["severity"]],
                          "evidence_class": [x["evidence_class"], y["evidence_class"]]})
            if SEVERITY_RANK[y["severity"]] > SEVERITY_RANK[x["severity"]]:
                regressions.append({"kind": "SEVERITY_INCREASE", "title": t,
                                    "from": x["severity"], "to": y["severity"]})

    return {
        "a": {"run_id": a["manifest"]["run_id"], "qa_version": a["manifest"].get("qa_version")},
        "b": {"run_id": b["manifest"]["run_id"], "qa_version": b["manifest"].get("qa_version")},
        "same_source_hash": same_source, "metric_tolerance": metric_tol,
        "metric_changes": mdiff, "method_status_changes": msd, "test_changes": tdiff,
        "finding_changes": fdiff, "changes": changes, "regressions": regressions,
        "verdict": "REGRESSIONS" if regressions else "NO_REGRESSIONS",
    }
