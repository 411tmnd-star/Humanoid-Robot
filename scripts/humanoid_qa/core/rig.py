"""Pure rig analysis on plain dicts (no bpy).

Bone dict keys used here: name, parent, head_w, tail_w, length, use_deform.
All positions are world space (the adapter converts with the armature's
matrix_world), so pair comparisons are not distorted by armature rotation.
"""

from . import geom


# ------------------------------------------------------------- graph

def graph_summary(bones):
    names = [b["name"] for b in bones]
    seen, dupes = set(), []
    for n in names:
        if n in seen:
            dupes.append(n)
        seen.add(n)
    parent = {b["name"]: b.get("parent") for b in bones}
    missing_parent = sorted(n for n, p in parent.items() if p is not None and p not in parent)
    roots = sorted(n for n, p in parent.items() if p is None)
    children = {n: [] for n in parent}
    for n, p in parent.items():
        if p in children:
            children[p].append(n)

    cycle_nodes = set()
    depth = {}
    for n in parent:
        chain, cur = [], n
        visited = set()
        while cur is not None and cur in parent:
            if cur in visited:
                cycle_nodes.update(visited)
                break
            visited.add(cur)
            chain.append(cur)
            cur = parent[cur]
        else:
            depth[n] = len(chain) - 1

    # components over undirected parent edges
    uf = {n: n for n in parent}

    def find(x):
        while uf[x] != x:
            uf[x] = uf[uf[x]]
            x = uf[x]
        return x
    for n, p in parent.items():
        if p in uf:
            uf[find(n)] = find(p)
    comps = {}
    for n in parent:
        comps.setdefault(find(n), []).append(n)
    sizes = sorted((len(v) for v in comps.values()), reverse=True)
    return {
        "bone_count": len(names),
        "duplicate_names": dupes,
        "missing_parent_refs": missing_parent,
        "roots": roots,
        "component_count": len(comps),
        "component_sizes": sizes,
        "cycle_nodes": sorted(cycle_nodes),
        "max_depth": max(depth.values()) if depth else 0,
        "leaf_count": sum(1 for n in children if not children[n]),
        "edges": [{"parent": p, "child": n} for n, p in sorted(parent.items()) if p is not None],
    }


# ------------------------------------------------------------- naming / sides

LAYERS = ("DEF", "MECH", "CTRL")


def layer_of(name):
    head = name.split("_", 1)[0]
    return head if head in LAYERS else None


def parse_side(name):
    """Return (marker, base). marker in: suffix_L, suffix_R, dot_L, dot_R,
    infix_L, infix_R, none. base is the name with the marker removed, so
    paired names share a base."""
    if name.endswith(".L"):
        return "dot_L", name[:-2]
    if name.endswith(".R"):
        return "dot_R", name[:-2]
    if name.endswith("_L"):
        return "suffix_L", name[:-2]
    if name.endswith("_R"):
        return "suffix_R", name[:-2]
    parts = name.split("_")
    for side in ("L", "R"):
        if side in parts[1:-1]:
            i = parts.index(side, 1)
            return f"infix_{side}", "_".join(parts[:i] + parts[i + 1:])
    return "none", name


def marker_side(marker):
    return marker[-1] if marker != "none" else None


def physical_side(x, plane_x, tol):
    if x > plane_x + tol:
        return "+X"
    if x < plane_x - tol:
        return "-X"
    return "midline"


def pair_by_name(names):
    """Pair every L-marked name with its counterpart: an R-marked name with
    the same base, else the unsided base name. Pure name logic; says nothing
    about which side is anatomically left."""
    by_base = {}
    for n in names:
        m, base = parse_side(n)
        by_base.setdefault(base, {})[m] = n
    pairs, l_unpaired, r_unpaired = [], [], []
    for base, group in sorted(by_base.items()):
        l_names = [group[m] for m in ("suffix_L", "dot_L", "infix_L") if m in group]
        r_names = [group[m] for m in ("suffix_R", "dot_R", "infix_R") if m in group]
        unsided = group.get("none")
        for ln in l_names:
            cp = r_names[0] if r_names else unsided
            if cp is None:
                l_unpaired.append(ln)
            else:
                pairs.append({"base": base, "l_marked": ln, "counterpart": cp,
                              "counterpart_marker": parse_side(cp)[0]})
        if not l_names:
            r_unpaired.extend(r_names)
    return {"pairs": pairs, "l_marked_unpaired": l_unpaired, "r_marked_unpaired": r_unpaired}


def pair_metrics(a, b, plane_x, min_dir_length=0.0):
    """Mirror bone a across x = plane_x and compare with bone b (world).
    Direction is reported as None when either bone is shorter than
    min_dir_length: float32 endpoint error makes a near-zero bone's
    direction meaningless (seen on DEF_Wrist, 2e-6 m)."""
    ha = geom.mirror_x(a["head_w"], plane_x)
    ta = geom.mirror_x(a["tail_w"], plane_x)
    da = geom.sub(ta, ha)
    db = geom.sub(b["tail_w"], b["head_w"])
    out = {
        "head_delta": geom.dist(ha, b["head_w"]),
        "tail_delta": geom.dist(ta, b["tail_w"]),
        "length_delta": abs(a["length"] - b["length"]),
        "direction_angle_deg": None,
    }
    if geom.length(da) > max(min_dir_length, 0.0) and geom.length(db) > max(min_dir_length, 0.0):
        out["direction_angle_deg"] = geom.angle_deg(da, db)
    out["max_endpoint_delta"] = max(out["head_delta"], out["tail_delta"])
    return out


def nearest_def_head(bone, def_bones):
    """Distance from a bone's head to the nearest DEF bone head (world)."""
    best = (float("inf"), None)
    for d in def_bones:
        dd = geom.dist(bone["head_w"], d["head_w"])
        if dd < best[0]:
            best = (dd, d["name"])
    return {"distance": best[0], "def_bone": best[1]}


# ------------------------------------------------------------- skinning

def skinning_layers(bones, meshes, armature_name):
    """Separate the five conditions that are often conflated.

    meshes: [{name, vgroup_nonzero: {group: n_vertices_with_w>0},
              vgroup_names: [...], armature_mods: [{target, use_vertex_groups,
              show_viewport, show_render}]}]
    """
    per_bone = {}
    bone_names = {b["name"] for b in bones}
    unused_groups, empty_groups = [], []
    for m in meshes:
        bound = any(md["target"] == armature_name and md["use_vertex_groups"]
                    for md in m["armature_mods"])
        for g in m["vgroup_names"]:
            n = m["vgroup_nonzero"].get(g, 0)
            if g not in bone_names:
                unused_groups.append({"mesh": m["name"], "group": g, "nonzero_vertices": n})
                continue
            rec = per_bone.setdefault(g, {"vgroup_in_meshes": [], "nonzero_vertices": 0,
                                          "bound_meshes": []})
            rec["vgroup_in_meshes"].append(m["name"])
            rec["nonzero_vertices"] += n
            if n == 0:
                empty_groups.append({"mesh": m["name"], "group": g})
            if n > 0 and bound:
                rec["bound_meshes"].append(m["name"])
    rows = []
    for b in bones:
        r = per_bone.get(b["name"], {"vgroup_in_meshes": [], "nonzero_vertices": 0,
                                    "bound_meshes": []})
        rows.append({
            "bone": b["name"],
            "layer": layer_of(b["name"]),
            "1_use_deform": bool(b["use_deform"]),
            "2_vertex_group_exists": bool(r["vgroup_in_meshes"]),
            "3_nonzero_weight_vertices": r["nonzero_vertices"],
            "4_bound_meshes": sorted(r["bound_meshes"]),
            # 5 needs a pose test; this is only the necessary precondition
            "5_eligible_to_deform": bool(b["use_deform"]) and bool(r["bound_meshes"]),
        })
    return {"bones": rows, "vertex_groups_without_bone": unused_groups,
            "vertex_groups_with_zero_weights": empty_groups}


# ------------------------------------------------------------- constraints

NEEDS_TARGET = {"COPY_ROTATION", "COPY_LOCATION", "COPY_TRANSFORMS", "COPY_SCALE",
                "DAMPED_TRACK", "TRACK_TO", "LOCKED_TRACK", "IK", "STRETCH_TO",
                "CHILD_OF", "TRANSFORM", "ARMATURE"}


def constraint_reference_check(constraints, object_types, bones_by_armature):
    """constraints: [{owner, type, target, subtarget, influence, mute}]
    object_types: {object_name: type}; bones_by_armature: {arm: set(names)}"""
    problems = []
    for c in constraints:
        t = c.get("target")
        if c["type"] in NEEDS_TARGET and not t:
            problems.append({**c, "problem": "missing target"})
            continue
        if t and t not in object_types:
            problems.append({**c, "problem": "target object does not exist"})
            continue
        sub = c.get("subtarget")
        if t and sub and object_types.get(t) == "ARMATURE" and sub not in bones_by_armature.get(t, set()):
            problems.append({**c, "problem": "subtarget bone does not exist"})
    return problems


def constraint_signature(c):
    """Side-independent signature of a constraint for pair comparison."""
    sub = c.get("subtarget")
    return (c["type"], parse_side(sub)[1] if sub else None,
            round(float(c.get("influence", 1.0)), 6), c.get("owner_space"),
            c.get("target_space"), bool(c.get("mute")))


def constraint_pair_diffs(constraints_by_owner, pairs):
    diffs = []
    for p in pairs:
        a = [constraint_signature(c) for c in constraints_by_owner.get(p["l_marked"], [])]
        b = [constraint_signature(c) for c in constraints_by_owner.get(p["counterpart"], [])]
        if a != b:
            diffs.append({"pair": [p["l_marked"], p["counterpart"]],
                          "l_marked_stack": a, "counterpart_stack": b})
    return diffs


def thumb_adjacency(bones_by_name, suffix=""):
    """Which finger base the thumb sits beside (palm.md:180 requires index
    nearest the thumb side). Uses DEF_Thumb_CMC head vs DEF_{Index,Little}_MCP
    heads, world space. Label-independent: compares a hand with itself."""
    t = bones_by_name.get("DEF_Thumb_CMC" + suffix)
    i = bones_by_name.get("DEF_Index_MCP" + suffix)
    l = bones_by_name.get("DEF_Little_MCP" + suffix)
    if not (t and i and l):
        return None
    di = geom.dist(t["head_w"], i["head_w"])
    dl = geom.dist(t["head_w"], l["head_w"])
    return {"thumb": t["name"], "thumb_head_w": t["head_w"],
            "dist_to_index_mcp": di, "dist_to_little_mcp": dl,
            "thumb_beside": "Index" if di < dl else "Little"}


def short_bones(bones, min_length):
    return sorted(({"bone": b["name"], "length": b["length"]} for b in bones
                   if b["length"] < min_length), key=lambda r: r["bone"])
