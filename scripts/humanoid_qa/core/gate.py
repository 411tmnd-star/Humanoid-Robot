"""Fail-safe gate for operations that depend on unresolved owner decisions.

The decision state lives in config/quality_rules.json under
coordinate_convention. Until `side_dependent_adjudication_allowed` is true,
every side-dependent answer is the string "UNRESOLVED" (never a guess), and
require_*() raises so a caller cannot silently proceed.
"""

UNRESOLVED = "UNRESOLVED"


class SideConventionUnresolved(RuntimeError):
    pass


def side_adjudication_allowed(cfg):
    cc = cfg["coordinate_convention"]
    return bool(cc.get("side_dependent_adjudication_allowed")) and cc.get("status") == "RESOLVED"


def require_side_convention(cfg, operation):
    if not side_adjudication_allowed(cfg):
        raise SideConventionUnresolved(
            f"{operation}: side convention is "
            f"{cfg['coordinate_convention'].get('status')} - refusing to decide which side is correct")


def gated(cfg, operation, fn, *args, **kw):
    """Run fn only when the convention is resolved; otherwise return a
    structured UNRESOLVED record (so reports can carry it)."""
    if not side_adjudication_allowed(cfg):
        return {"status": UNRESOLVED, "operation": operation,
                "reason": f"coordinate convention is {cfg['coordinate_convention'].get('status')}",
                "needs": cfg["coordinate_convention"].get("outcome")}
    return {"status": "OK", "operation": operation, "result": fn(*args, **kw)}


def anatomical_side(cfg, label_side, physical_side, right_sign_x):
    """Anatomical side of a bone/mesh. UNRESOLVED unless the owner has
    resolved the convention; `label_side` is accepted so a resolved
    convention can later map labels, but is not interpreted here."""
    if not side_adjudication_allowed(cfg):
        return UNRESOLVED
    if physical_side == "midline":
        return "midline"
    if right_sign_x is None:
        return UNRESOLVED
    return "RIGHT" if (physical_side == "+X") == (right_sign_x > 0) else "LEFT"
