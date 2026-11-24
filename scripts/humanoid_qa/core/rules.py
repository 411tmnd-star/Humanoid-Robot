"""Quality-rule config, finding records and measurement-method qualification."""

import json
import os

FINDING_REQUIRED = (
    "id", "title", "rule_category", "severity", "evidence_class", "affected",
    "measured", "method_ids", "method_status", "threshold", "threshold_status",
    "interpretation", "confidence_and_limits", "next_action", "owner_judgement_required",
)
EVIDENCE_CLASSES = ("SOURCE_FACT", "MEASURED_FACT", "VERIFIED_RESULT", "HYPOTHESIS",
                    "PROPOSED_RULE", "OWNER_DECISION_REQUIRED", "UNKNOWN_NOT_MEASURED")
RULE_CATEGORIES = ("A_HARD_INVARIANT", "B_SPEC_CONSTRAINT", "C_ENGINEERING_DIAGNOSTIC",
                   "D_PROPOSED_DESIGN_RULE", "E_HUMAN_ACCEPTANCE")
SEVERITIES = ("BLOCKER", "HIGH", "MEDIUM", "LOW", "INFO")
METHOD_STATUSES = ("QUALIFIED", "QUALIFIED_WITH_LIMITATIONS", "EXPERIMENTAL",
                   "FAILED_VALIDATION", "NOT_IMPLEMENTED")


def load_config(path):
    with open(path, encoding="utf-8") as f:
        cfg = json.load(f)
    for k in ("config_version", "classification", "tolerances", "coordinate_convention",
              "severity_definitions", "methods"):
        if k not in cfg:
            raise ValueError(f"quality_rules.json missing key: {k}")
    return cfg


def tol(cfg, key):
    """Fetch a tolerance value; every tolerance carries unit + status + basis."""
    t = cfg["tolerances"][key]
    for k in ("value", "unit", "kind", "status", "basis"):
        if k not in t:
            raise ValueError(f"tolerance {key} missing {k}")
    return t["value"]


class Findings:
    def __init__(self, prefix):
        self.prefix = prefix
        self.items = []

    def add(self, **kw):
        kw["id"] = f"{self.prefix}-{len(self.items) + 1:03d}"
        validate_finding(kw)
        self.items.append(kw)
        return kw["id"]


def validate_finding(f):
    missing = [k for k in FINDING_REQUIRED if k not in f]
    if missing:
        raise ValueError(f"finding missing fields: {missing}")
    if f["severity"] not in SEVERITIES:
        raise ValueError(f"bad severity {f['severity']}")
    if f["evidence_class"] not in EVIDENCE_CLASSES:
        raise ValueError(f"bad evidence_class {f['evidence_class']}")
    if f["rule_category"] not in RULE_CATEGORIES:
        raise ValueError(f"bad rule_category {f['rule_category']}")
    # A definitive class may not rest on an unqualified method.
    if f["evidence_class"] in ("MEASURED_FACT", "VERIFIED_RESULT") and \
            f["method_status"] not in ("QUALIFIED", "QUALIFIED_WITH_LIMITATIONS", "N/A"):
        raise ValueError(f"{f['id']}: {f['evidence_class']} cannot rest on a "
                         f"{f['method_status']} method")


def qualify_methods(cfg, test_results):
    """Derive each method's status from the tests that actually ran.

    test_results: {test_id: "PASS" | "FAIL" | "ERROR" | "SKIP"}
    A method declared QUALIFIED/QUALIFIED_WITH_LIMITATIONS keeps that status
    only if every validating test exists and passed; otherwise it becomes
    FAILED_VALIDATION (a test failed) or EXPERIMENTAL (a test never ran).
    """
    out = {}
    for mid, m in cfg["methods"].items():
        declared = m["declared_status"]
        tests = m.get("validated_by", [])
        results = {t: test_results.get(t, "NOT_RUN") for t in tests}
        if declared in ("NOT_IMPLEMENTED", "EXPERIMENTAL"):
            status = declared
        elif any(r in ("FAIL", "ERROR") for r in results.values()):
            status = "FAILED_VALIDATION"
        elif not tests or any(r != "PASS" for r in results.values()):
            status = "EXPERIMENTAL"
        else:
            status = declared
        out[mid] = {**m, "status": status, "test_results": results}
    return out


def load_test_results(paths):
    merged = {}
    for p in paths:
        if p and os.path.exists(p):
            with open(p, encoding="utf-8") as f:
                for t in json.load(f)["tests"]:
                    merged[t["id"]] = t["result"]
    return merged
