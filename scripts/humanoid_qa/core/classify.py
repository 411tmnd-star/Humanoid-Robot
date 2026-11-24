"""Object classification (pure). Rules live in config/quality_rules.json."""


def classify_object(obj, cfg):
    """obj: {name, type, hide_viewport, hide_render, modifiers: [{type, target,
    show_viewport, show_render}]} -> (class, reason)."""
    c = cfg["classification"]
    n = obj["name"]
    for p in c["environment_prefixes"]:
        if n.startswith(p):
            return "environment", f"name prefix {p}"
    for p in c["presentation_prefixes"]:
        if n.startswith(p):
            return "presentation", f"name prefix {p}"
    if n.startswith(c["body_prefix"]):
        if obj["hide_viewport"] or obj["hide_render"]:
            return "excluded_backup", "body prefix but hidden"
        for m in c["backup_markers"]:
            if m in n:
                return "excluded_backup", f"body prefix but name contains {m}"
        bound = [md for md in obj["modifiers"]
                 if md["type"] == "ARMATURE" and md["target"] == c["armature_object"]
                 and md["show_viewport"] and md["show_render"]]
        if bound:
            return "body", f"body prefix + enabled Armature modifier -> {c['armature_object']}"
        return "unresolved", "body prefix but no enabled Armature modifier to the armature"
    return "unresolved", "no rule matched"


def classify_all(objects, cfg):
    out = {}
    for o in objects:
        if o["type"] != "MESH":
            continue
        cls, why = classify_object(o, cfg)
        out[o["name"]] = {"class": cls, "reason": why}
    return out


FINGER_RE = __import__("re").compile(r"^(411_Finger)(\d)_(L|R)(_.*)?$")


def canonical_mesh_name(name, finger_map):
    """411_Finger4_R_A -> 411_FingerIndex_R_A using the side-dependent map
    from scripts/rig/skin_rigid_v01.py, so L/R twins share a base name."""
    m = FINGER_RE.match(name)
    if not m:
        return name
    finger = finger_map.get(m.group(3), {}).get(m.group(2))
    if finger is None:
        return name
    return f"{m.group(1)}{finger}_{m.group(3)}{m.group(4) or ''}"
