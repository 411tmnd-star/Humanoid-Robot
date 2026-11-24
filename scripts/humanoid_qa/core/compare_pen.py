"""Regression comparison of two penetration.json documents (pure Python).

Version: compare_pen_v01.0. Used to check that a change to shared measurement code
(solid_v01 -> solid_v02) did not move results that were already validated, and to
explain every difference instead of assuming the old or the new value is right.

Per pair present in both documents:
  volume      V_new vs V_old (relative difference; bitwise-equal count)
  separation  bound_new vs bound_old and direction (old documents call the field
              axial_depth_bound_m / axial_depth_direction; new ones axial_separation_*)
  cause       how a separation difference is explained, from the NEW document's recorded
              evidence: certificate adjustment, sampled non-monotone overlap, or the
              change of bisection resolution (search bracket T/samples/2^iters versus
              T/2^iters). A difference with none of these recorded is UNEXPLAINED.
"""

VERSION = "compare_pen_v01.0"


def _sep(row):
    if "axial_separation_bound_m" in row:
        return row["axial_separation_bound_m"], row.get("axial_separation_direction")
    return row.get("axial_depth_bound_m"), row.get("axial_depth_direction")


def _explain(new_row, old_bound, new_bound, old_dir, new_dir, old_resolution_m):
    d = abs(new_bound - old_bound)
    cert = new_row.get("axial_certificate_added_m") or {}
    added = max((v or 0.0) for v in cert.values()) if cert else 0.0
    nm = new_row.get("axial_nonmonotone_directions") or []
    reasons = []
    if added > 0.0:
        reasons.append(f"surface-crossing certificate raised at least one direction by up to {added:.3g} m")
    if nm:
        reasons.append(f"sampled non-monotone overlap along {nm}")
    if d <= old_resolution_m:
        reasons.append(f"within the old bisection resolution ({old_resolution_m:.3g} m)")
    if old_dir != new_dir:
        reasons.append(f"minimising direction changed {old_dir} -> {new_dir}")
    return reasons


def compare(old, new, tol_volume_rel=1e-12, old_resolution_m=1e-5):
    old_rows = {(r["a"], r["b"]): r for r in old["pairs"]}
    new_rows = {(r["a"], r["b"]): r for r in new["pairs"]}
    out = {"version": VERSION, "pairs_old": len(old_rows), "pairs_new": len(new_rows),
           "only_in_old": sorted(set(old_rows) - set(new_rows)), "only_in_new": sorted(set(new_rows) - set(old_rows)),
           "status_transitions": {}, "baseline_validated": 0, "baseline_validated_still_validated": 0,
           "volume": {"compared": 0, "bitwise_equal": 0, "max_rel_diff": 0.0, "over_tol": []},
           "separation": {"compared": 0, "identical": 0, "changed": [], "max_abs_diff_m": 0.0, "unexplained": []}}
    for key in sorted(set(old_rows) & set(new_rows)):
        o, n = old_rows[key], new_rows[key]
        if o["status"] != n["status"]:
            out["status_transitions"].setdefault(f"{o['status']} -> {n['status']}", []).append(list(key))
        if o["status"] == "VOLUME_VALIDATED":
            out["baseline_validated"] += 1
            out["baseline_validated_still_validated"] += int(n["status"] == "VOLUME_VALIDATED")
            vo, vn = o["overlap_volume_m3"], n.get("overlap_volume_m3")
            if vn is None:
                out["volume"]["over_tol"].append({"pair": list(key), "old": vo, "new": None})
                continue
            rel = abs(vn - vo) / max(abs(vo), 1e-300)
            v = out["volume"]
            v["compared"] += 1
            v["bitwise_equal"] += int(vn == vo)
            v["max_rel_diff"] = max(v["max_rel_diff"], rel)
            if rel > tol_volume_rel:
                v["over_tol"].append({"pair": list(key), "old": vo, "new": vn, "rel_diff": rel})
            bo, do = _sep(o)
            bn, dn = _sep(n)
            if bo is not None and bn is not None:
                s = out["separation"]
                s["compared"] += 1
                diff = bn - bo
                s["max_abs_diff_m"] = max(s["max_abs_diff_m"], abs(diff))
                if diff == 0.0 and do == dn:
                    s["identical"] += 1
                else:
                    why = _explain(n, bo, bn, do, dn, old_resolution_m)
                    row = {"pair": list(key), "old_m": bo, "new_m": bn, "diff_m": diff, "old_dir": do, "new_dir": dn,
                           "explained_by": why}
                    s["changed"].append(row)
                    if not why:
                        s["unexplained"].append(row)
    return out


def to_markdown(c, old_name, new_name):
    L = [f"# Penetration regression: {old_name} -> {new_name}", "",
         f"Pairs: old {c['pairs_old']}, new {c['pairs_new']}. Baseline validated: {c['baseline_validated']}; "
         f"still validated: {c['baseline_validated_still_validated']}.", "",
         "## Status transitions", ""]
    L += [f"- {k}: {len(v)} pair(s) " + "; ".join(f"{a}/{b}" for a, b in v[:6]) for k, v in c["status_transitions"].items()] or ["- none"]
    v = c["volume"]
    L += ["", "## Overlap volume of previously validated pairs", "",
          f"- compared {v['compared']}, bitwise equal {v['bitwise_equal']}, max relative difference {v['max_rel_diff']:.3e}",
          f"- over tolerance: {len(v['over_tol'])}"]
    s = c["separation"]
    L += ["", "## Axial separation bound of previously validated pairs", "",
          f"- compared {s['compared']}, identical {s['identical']}, changed {len(s['changed'])}, max |diff| {s['max_abs_diff_m']:.3e} m, "
          f"unexplained {len(s['unexplained'])}"]
    for r in sorted(s["changed"], key=lambda r: -abs(r["diff_m"]))[:25]:
        L.append(f"  - {r['pair'][0]} / {r['pair'][1]}: {r['old_m']:.7g} ({r['old_dir']}) -> {r['new_m']:.7g} ({r['new_dir']}), "
                 f"diff {r['diff_m']:+.3e} m; {', '.join(r['explained_by']) or 'UNEXPLAINED'}")
    return "\n".join(L) + "\n"
