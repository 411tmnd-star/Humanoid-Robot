"""Measurement-copy refinement of closed triangle shells (pure Python, stdlib only).

Version: refine_v01.0. Purpose: let a shell that is geometrically a closed solid but is
refused by the solid gate because of collapsed (zero-area) triangles be measured on a
DISPOSABLE IN-MEMORY COPY, with an explicit, checkable proof that the copy is the same
surface for the purposes of the overlap measurement. Nothing here touches a .blend; the
caller's lists are never mutated.

OPERATIONS (least invasive first; each returns a new copy plus a vertex map)
  exact_reindex          merge vertices whose coordinates are bit-identical (equivalence
                         is exact; no position changes). Faces that become degenerate
                         through the merge are dropped and counted.
  remove_degenerate      drop faces with area <= tol and nothing else. Reported so its
                         effect is on record; on a closed shell this usually opens the
                         surface (boundary edges) and is then NOT acceptable.
  collapse_short_edges   merge vertex clusters joined by edges <= edge_tol. The cluster
                         moves to a representative position (centroid by default), so
                         each vertex moves by at most the cluster radius (<= edge_tol/2
                         for a single edge with the centroid). Faces collapsed by the
                         merge are dropped.

PRESERVATION (`preservation_report`, `accept`)
  Counts, vertex displacement, area, signed volume (total and per component), component
  orientation, degenerate faces, zero-length edges, boundary / non-manifold / winding
  defects, and the geometry of every removed face. Classification:
    EXACT_EQUIVALENT         no position changed, no face changed (identical surface)
    NUMERICALLY_EQUIVALENT   all changes are bounded by `edge_tol` (the project's float32
                             coincident-vertex tolerance), the volume change is within the
                             swept-volume bound area*h, and topology is clean
    NOT_EQUIVALENT           anything else (the refinement must not be used)
  The swept-volume bound: if every point of a closed surface moves by at most h, the
  enclosed volume changes by at most (surface area) * h. A volume change larger than that
  cannot be explained by the displacement and fails acceptance. A volume that merely
  "looks similar" is never accepted on its own.
"""

import math

from . import geom, intersect

VERSION = "refine_v01.0"


# ------------------------------------------------------------- helpers

def _tri_area(coords, t):
    return intersect.tri_area((coords[t[0]], coords[t[1]], coords[t[2]]))


def _edges_of(tris):
    es = set()
    for a, b, c in tris:
        for u, v in ((a, b), (b, c), (c, a)):
            es.add((u, v) if u < v else (v, u))
    return es


def _components(tris, n):
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for t in tris:
        r0 = find(t[0])
        for v in t[1:]:
            rv = find(v)
            if rv != r0:
                parent[rv] = r0
    groups = {}
    for i, t in enumerate(tris):
        groups.setdefault(find(t[0]), []).append(i)
    return [groups[k] for k in sorted(groups)]


def _merge(coords, tris, clusters, positions):
    """Merge each cluster (list of old vertex ids) into its lowest member, placed at
    positions[k]. Returns (coords1, tris1, vmap, removed) where removed lists
    (original_triangle_index, original_triangle) for every dropped face."""
    rep = {}
    new_pos = {}
    for k, members in enumerate(clusters):
        m = sorted(members)
        for v in m:
            rep[v] = m[0]
        new_pos[m[0]] = positions[k]
    dead = {v for v, r in rep.items() if r != v}
    index, coords1 = {}, []
    for i, p in enumerate(coords):
        if i in dead:
            continue
        index[i] = len(coords1)
        coords1.append(tuple(new_pos.get(i, p)))
    vmap = [index[rep.get(i, i)] for i in range(len(coords))]
    tris1, removed = [], []
    for k, t in enumerate(tris):
        nt = (vmap[t[0]], vmap[t[1]], vmap[t[2]])
        if len(set(nt)) < 3:
            removed.append((k, tuple(t)))
        else:
            tris1.append(nt)
    return coords1, tris1, vmap, removed


def _result(coords1, tris1, vmap, removed, op, params, clusters):
    return {"op": op, "params": params, "coords": coords1, "tris": tris1, "vmap": vmap,
            "removed": removed, "clusters": [sorted(c) for c in clusters]}


# ------------------------------------------------------------- operations

def exact_reindex(coords, tris):
    """Merge vertices with bit-identical coordinates (exact equivalence)."""
    groups = {}
    for i, p in enumerate(coords):
        groups.setdefault(tuple(p), []).append(i)
    clusters = [g for g in groups.values() if len(g) > 1]
    positions = [tuple(coords[g[0]]) for g in clusters]
    c1, t1, vmap, removed = _merge(coords, tris, clusters, positions)
    return _result(c1, t1, vmap, removed, "EXACT_REINDEX", {}, clusters)


def remove_degenerate(coords, tris, area_tol):
    """Drop faces with area <= area_tol. No vertex is moved or merged."""
    keep, removed = [], []
    for k, t in enumerate(tris):
        if _tri_area(coords, t) <= area_tol:
            removed.append((k, tuple(t)))
        else:
            keep.append(tuple(t))
    return _result(list(coords), keep, list(range(len(coords))), removed,
                   "REMOVE_DEGENERATE_FACES", {"area_tol": area_tol}, [])


def short_edge_clusters(coords, tris, edge_tol):
    """Union-find clusters of vertices joined by edges of length <= edge_tol."""
    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for u, v in sorted(_edges_of(tris)):
        if geom.dist(coords[u], coords[v]) <= edge_tol:
            ru, rv = find(u), find(v)
            if ru != rv:
                parent[max(ru, rv)] = min(ru, rv)
    groups = {}
    for x in list(parent):
        groups.setdefault(find(x), []).append(x)
    return [sorted(g) for g in groups.values() if len(g) > 1]


def collapse_short_edges(coords, tris, edge_tol, representative="centroid"):
    """Merge clusters of vertices joined by edges <= edge_tol (see module docstring)."""
    clusters = short_edge_clusters(coords, tris, edge_tol)
    positions = []
    for g in clusters:
        pts = [coords[i] for i in g]
        if representative == "centroid":
            positions.append(tuple(sum(p[k] for p in pts) / len(pts) for k in range(3)))
        elif representative == "first":
            positions.append(tuple(pts[0]))
        elif representative == "last":
            positions.append(tuple(pts[-1]))
        else:
            raise ValueError(f"unknown representative {representative}")
    c1, t1, vmap, removed = _merge(coords, tris, clusters, positions)
    return _result(c1, t1, vmap, removed, "COLLAPSE_SHORT_EDGES",
                   {"edge_tol": edge_tol, "representative": representative}, clusters)


def compose(first, second):
    """Result of applying `second` to the output of `first`, expressed against the
    ORIGINAL mesh (vmap composed; removed faces mapped back to original indices)."""
    vmap = [second["vmap"][v] for v in first["vmap"]]
    # triangles of `first` output are in the order of the surviving originals
    survivors = [k for k in range(len(first["removed"]) + len(first["tris"]))
                 if k not in {r[0] for r in first["removed"]}]
    removed = list(first["removed"])
    for k, t in second["removed"]:
        removed.append((survivors[k], t))
    return {"op": first["op"] + "+" + second["op"],
            "params": {"first": first["params"], "second": second["params"]},
            "coords": second["coords"], "tris": second["tris"], "vmap": vmap,
            "removed": sorted(removed), "clusters": first["clusters"] + second["clusters"]}


# ------------------------------------------------------------- measurement of a shell

def shell_metrics(coords, tris, area_tol, edge_tol):
    """Everything the preservation comparison needs about one shell."""
    topo = geom.topology(len(coords), [], tris, coords, area_tol, edge_tol)
    comps = _components(tris, len(coords))
    comp_rows = []
    for comp in comps:
        faces = [tris[i] for i in comp]
        comp_rows.append({"triangles": len(comp),
                          "signed_volume_m3": geom.signed_volume(coords, faces),
                          "area_m2": sum(_tri_area(coords, f) for f in faces)})
    return {
        "vertices": len(coords), "triangles": len(tris), "edges": topo["edges"],
        "connected_components": len(comps),
        "boundary_edges": topo["boundary_edges"], "nonmanifold_edges": topo["nonmanifold_edges"],
        "degenerate_faces": topo["degenerate_faces"], "zero_length_edges": topo["zero_length_edges"],
        "winding_inconsistent_edges": geom.winding_inconsistencies(tris),
        "area_m2": sum(c["area_m2"] for c in comp_rows),
        "signed_volume_m3": geom.signed_volume(coords, tris),
        "components": comp_rows,
    }


def _sliver_height(coords, t):
    """Smallest altitude of the triangle: 2*area / longest edge (how far the triangle
    extends from the line through its longest edge)."""
    a, b, c = coords[t[0]], coords[t[1]], coords[t[2]]
    longest = max(geom.dist(a, b), geom.dist(b, c), geom.dist(c, a))
    return 0.0 if longest == 0.0 else 2.0 * _tri_area(coords, t) / longest


def _min_dist_to_mesh(p, coords, tris, limit=None):
    best = math.inf
    for t in tris:
        q = intersect.closest_point_on_triangle(p, coords[t[0]], coords[t[1]], coords[t[2]])
        d = geom.dist(p, q)
        if d < best:
            best = d
    return best


def _samples(coords, t):
    a, b, c = coords[t[0]], coords[t[1]], coords[t[2]]
    mid = lambda p, q: tuple((p[k] + q[k]) / 2.0 for k in range(3))  # noqa: E731
    cen = tuple((a[k] + b[k] + c[k]) / 3.0 for k in range(3))
    return [a, b, c, mid(a, b), mid(b, c), mid(c, a), cen]


def preservation_report(coords0, tris0, ref, area_tol, edge_tol):
    """Compare the original shell with the refined copy `ref` (a refine result).

    Returns a plain dict. Nothing is judged here except through `accept`."""
    c1, t1, vmap = ref["coords"], ref["tris"], ref["vmap"]
    m0 = shell_metrics(coords0, tris0, area_tol, edge_tol)
    m1 = shell_metrics(c1, t1, area_tol, edge_tol)

    # vertex displacement: original vertex i vs its image in the copy
    disp = [geom.dist(coords0[i], c1[vmap[i]]) for i in range(len(coords0))]
    moved = [i for i, d in enumerate(disp) if d > 0.0]
    max_disp = max(disp) if disp else 0.0

    # face bookkeeping
    removed_idx = {k for k, _ in ref["removed"]}
    removed_rows = []
    for k, t in ref["removed"]:
        removed_rows.append({"triangle": k, "vertices": list(t),
                             "area_m2": _tri_area(coords0, t),
                             "longest_edge_m": max(geom.dist(coords0[t[i]], coords0[t[(i + 1) % 3]]) for i in range(3)),
                             "min_altitude_m": _sliver_height(coords0, t)})
    survivors = [k for k in range(len(tris0)) if k not in removed_idx]
    remap_ok = len(survivors) == len(t1) and all(
        tuple(vmap[v] for v in tris0[k]) == tuple(t1[j]) for j, k in enumerate(survivors))
    surv_area_change = 0.0
    for j, k in enumerate(survivors):
        surv_area_change = max(surv_area_change, abs(_tri_area(coords0, tris0[k]) - _tri_area(c1, t1[j])))

    # surface distance on the affected region. Triangles whose three vertices did not move
    # and that were not removed are point-for-point identical in both shells (distance 0).
    moved_set = set(moved)
    affected0 = [k for k in range(len(tris0)) if k in removed_idx or moved_set & set(tris0[k])]
    affected1 = [j for j, k in enumerate(survivors) if moved_set & set(tris0[k])]
    d_removed = 0.0
    for k in affected0:
        for p in _samples(coords0, tris0[k]):
            d_removed = max(d_removed, _min_dist_to_mesh(p, c1, t1))
    d_new = 0.0
    for j in affected1:
        for p in _samples(c1, t1[j]):
            d_new = max(d_new, _min_dist_to_mesh(p, coords0, tris0))
    # a surviving triangle moves linearly with its vertices: every point moves <= max_disp
    hausdorff_bound = max(max_disp, d_removed, d_new)

    # duplicate (coincident) faces created by the merge
    seen, folds = {}, 0
    for t in t1:
        key = tuple(sorted(t))
        folds += 1 if key in seen else 0
        seen[key] = True

    return {
        "before": m0, "after": m1,
        "deltas": {"vertices": m1["vertices"] - m0["vertices"],
                   "triangles": m1["triangles"] - m0["triangles"],
                   "edges": m1["edges"] - m0["edges"],
                   "area_m2": m1["area_m2"] - m0["area_m2"],
                   "signed_volume_m3": m1["signed_volume_m3"] - m0["signed_volume_m3"],
                   "signed_volume_rel": (m1["signed_volume_m3"] - m0["signed_volume_m3"]) / m0["signed_volume_m3"]
                   if m0["signed_volume_m3"] else None},
        "vertices_moved": len(moved), "vertices_unchanged_bitwise": len(coords0) - len(moved),
        "max_vertex_displacement_m": max_disp,
        "removed_triangles": removed_rows,
        "removed_area_m2": sum(r["area_m2"] for r in removed_rows),
        "removed_max_altitude_m": max((r["min_altitude_m"] for r in removed_rows), default=0.0),
        "surviving_faces_remap_exact": remap_ok,
        "surviving_face_max_area_change_m2": surv_area_change,
        "sampled_distance_removed_to_copy_m": d_removed,
        "sampled_distance_new_to_original_m": d_new,
        "surface_distance_bound_m": hausdorff_bound,
        "swept_volume_bound_m3": m0["area_m2"] * hausdorff_bound,
        "coincident_face_pairs_after": folds,
        "component_signs_before": [c["signed_volume_m3"] > 0 for c in m0["components"]],
        "component_signs_after": [c["signed_volume_m3"] > 0 for c in m1["components"]],
        "affected_triangles_original": len(affected0),
    }


def accept(report, edge_tol, vol_slack=1e-15):
    """Decide whether the refined copy may be measured in place of the original.

    Returns (classification, failures). Every failure names the violated criterion."""
    b, a = report["before"], report["after"]
    fails = []
    if not report["surviving_faces_remap_exact"]:
        fails.append("surviving faces are not the original faces under the vertex map")
    if a["boundary_edges"] or a["nonmanifold_edges"] or a["winding_inconsistent_edges"]:
        fails.append(f"copy topology not clean (boundary {a['boundary_edges']}, non-manifold "
                     f"{a['nonmanifold_edges']}, winding {a['winding_inconsistent_edges']})")
    if a["connected_components"] != b["connected_components"]:
        fails.append("connected component count changed")
    if report["component_signs_before"] != report["component_signs_after"]:
        fails.append("component orientation changed")
    if a["degenerate_faces"] or a["zero_length_edges"]:
        fails.append(f"copy still has degenerate faces ({a['degenerate_faces']}) or zero-length edges "
                     f"({a['zero_length_edges']})")
    if report["coincident_face_pairs_after"]:
        fails.append("merge created coincident (folded) face pairs")
    if report["max_vertex_displacement_m"] > edge_tol:
        fails.append(f"vertex displacement {report['max_vertex_displacement_m']:.3g} m exceeds {edge_tol:g} m")
    if report["removed_max_altitude_m"] > edge_tol:
        fails.append(f"a removed face has altitude {report['removed_max_altitude_m']:.3g} m > {edge_tol:g} m "
                     "(not a collapsed sliver)")
    if report["surface_distance_bound_m"] > edge_tol:
        fails.append(f"surface distance bound {report['surface_distance_bound_m']:.3g} m exceeds {edge_tol:g} m")
    dv = abs(report["deltas"]["signed_volume_m3"])
    if dv > report["swept_volume_bound_m3"] + vol_slack:
        fails.append(f"volume change {dv:.3g} m^3 exceeds the swept-volume bound "
                     f"{report['swept_volume_bound_m3']:.3g} m^3")
    if fails:
        return "NOT_EQUIVALENT", fails
    exact = (report["vertices_moved"] == 0 and not report["removed_triangles"]
             and report["deltas"]["signed_volume_m3"] == 0.0)
    return ("EXACT_EQUIVALENT" if exact else "NUMERICALLY_EQUIVALENT"), []


def refine_for_gate(coords, tris, area_tol, edge_tol, gate_fn):
    """Apply the least invasive operations in order until `gate_fn(coords, tris)` returns
    no reasons, or give up. Returns a dict:
        status    NOT_NEEDED | REFINED | NOT_APPLICABLE | REJECTED
        steps     every attempted operation with the gate outcome after it
        result    the refine result of the accepted chain (REFINED) else None
        report    preservation report (REFINED / REJECTED) else None
        class_    EXACT_EQUIVALENT / NUMERICALLY_EQUIVALENT / NOT_EQUIVALENT (or None)
        failures  acceptance failures
    The caller's lists are not modified."""
    steps = []
    reasons0 = gate_fn(coords, tris)
    if not reasons0:
        return {"status": "NOT_NEEDED", "steps": steps, "result": None, "report": None,
                "class_": None, "failures": []}
    chain = None
    cur_c, cur_t = list(coords), [tuple(t) for t in tris]
    for name, fn in (("EXACT_REINDEX", lambda c, t: exact_reindex(c, t)),
                     ("COLLAPSE_SHORT_EDGES", lambda c, t: collapse_short_edges(c, t, edge_tol))):
        r = fn(cur_c, cur_t)
        changed = bool(r["clusters"])
        reasons = gate_fn(r["coords"], r["tris"]) if changed else reasons0
        steps.append({"op": name, "clusters": len(r["clusters"]), "vertices_merged": sum(len(c) - 1 for c in r["clusters"]),
                      "faces_removed": len(r["removed"]), "gate_after": reasons, "applied": changed})
        if not changed:
            continue
        chain = r if chain is None else compose(chain, r)
        cur_c, cur_t = r["coords"], r["tris"]
        if not reasons:
            break
    if chain is None:
        return {"status": "NOT_APPLICABLE", "steps": steps, "result": None, "report": None,
                "class_": None, "failures": ["no coincident vertices or short edges to merge"]}
    final_reasons = gate_fn(chain["coords"], chain["tris"])
    report = preservation_report(coords, [tuple(t) for t in tris], chain, area_tol, edge_tol)
    cls, fails = accept(report, edge_tol)
    if final_reasons:
        fails = fails + [f"copy still refused by the gate: {final_reasons}"]
        cls = "NOT_EQUIVALENT"
    return {"status": "REFINED" if not fails else "REJECTED", "steps": steps,
            "result": chain, "report": report, "class_": cls, "failures": fails}
