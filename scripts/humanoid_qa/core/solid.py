"""Solid-level overlap measurement for closed triangle meshes (pure Python).

Version: solid_v02.0. Stdlib only; the general (non-convex) volume backend lives in
blender/volume.py and is validated by the Blender fixture suite.

QUANTITIES (units: m for lengths, m^3 for volumes)
  overlap_volume(A, B)     V(A n B): volume of the solid intersection. Exact for the
                           pure convex backend; a Boolean-kernel value otherwise.
  axial_separation_bound   min over the six world axis directions d of t(d), where t(d)
                           is a VERIFIED SEPARATING TRANSLATION of A along d: A moved by
                           t(d)*d has overlap volume <= null_volume (primary kernel) and,
                           when certification is enabled, no proper triangle-surface
                           crossing deeper than tri_contact. That translation is itself a
                           feasible separation, so its length is >= MTD(A, B), the minimum
                           translation distance: the value is an UPPER BOUND on MTD (to
                           within tri_contact when certified). It is NOT the minimum:
                           (1) only the six axis directions are tried, so for general
                           shapes it exceeds MTD (a cube sunk 1 m into a 45-degree face
                           needs 1 m along the normal but sqrt(2) m along any axis);
                           (2) the search locates a transition of the overlap volume
                           along the ray; when that function is not monotone (non-convex
                           or disconnected shapes: overlap vanishes, then reappears) the
                           transition found need not be the first one, so even the
                           per-axis value is "a" verified separation, not the smallest.
                           A uniform scan reports reappearance at the sampled
                           translations only.
  null volume              |V| <= null_volume is reported as zero overlap (numerical).

INPUTS: closed, consistently wound triangle shells in world coordinates. Each operand
is GATED (open, non-manifold, degenerate, winding-inconsistent or zero-volume
components are refused as UNABLE_TO_DETERMINE, never measured). Components whose
signed volume is negative are reversed on an IN-MEMORY COPY; the source is never
modified. Orientation only decides which side is outside; the enclosed region of a
closed shell is unchanged by reversing it.

REFINEMENT (solid_v02.0, optional via params["refine_edge_tol"]): an operand refused for
DEGENERATE_FACE ALONE may be measured on an in-memory copy produced by core/refine.py
(exact reindex, then collapse of zero-length edges). The copy is used only if it passes
the gate and the preservation criteria, and the pair is validated only if the unrefined
shells give the same volume in both kernels (`refinement_effect`). Any other refusal
reason is never refined.
"""

import math

from . import geom, intersect, refine

VERSION = "solid_v02.0"
AXES = (("+X", (1.0, 0.0, 0.0)), ("-X", (-1.0, 0.0, 0.0)),
        ("+Y", (0.0, 1.0, 0.0)), ("-Y", (0.0, -1.0, 0.0)),
        ("+Z", (0.0, 0.0, 1.0)), ("-Z", (0.0, 0.0, -1.0)))


class NonConvexError(ValueError):
    """Raised by the pure convex backend when an operand is not convex."""


# ------------------------------------------------------------- topology / gating

def components(tris, n_verts):
    """Connected components of a triangle list, as lists of triangle indices."""
    parent = list(range(n_verts))

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


def gate(coords, tris, degenerate_area):
    """Refusal reasons for measuring the solid bounded by `tris`. Empty list = accepted."""
    reasons = []
    if not tris:
        return ["EMPTY"]
    topo = geom.topology(len(coords), [], tris, coords, degenerate_area, 0.0)
    if topo["boundary_edges"]:
        reasons.append("OPEN_BOUNDARY")
    if topo["nonmanifold_edges"]:
        reasons.append("NON_MANIFOLD")
    if topo["degenerate_faces"]:
        reasons.append("DEGENERATE_FACE")
    if geom.winding_inconsistencies(tris):
        reasons.append("WINDING_INCONSISTENT")
    return reasons


def orient_components(coords, tris, null_volume):
    """Reverse each connected component whose signed volume is negative.

    Returns (oriented_tris, info). info lists components by index, the number of
    triangles, the signed volume before orientation, and whether it was reversed.
    A component with |V| <= null_volume is reported in info["zero_volume"] (not
    orientable, not measurable).
    """
    out = [tuple(t) for t in tris]
    parts = []
    flipped, zero = [], []
    for k, comp in enumerate(components(tris, len(coords))):
        faces = [tuple(tris[i]) for i in comp]
        v = geom.signed_volume(coords, faces)
        rec = {"component": k, "triangles": len(comp), "signed_volume_m3": v, "reversed": False}
        if abs(v) <= null_volume:
            zero.append(k)
        elif v < 0.0:
            for i in comp:
                a, b, c = out[i]
                out[i] = (a, c, b)
            rec["reversed"] = True
            flipped.append(k)
        parts.append(rec)
    return out, {"components": len(parts), "reversed_components": flipped,
                 "zero_volume_components": zero, "detail": parts}


# ------------------------------------------------------------- convex geometry

def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _shift(p, s):
    return (p[0] + s[0], p[1] + s[1], p[2] + s[2])


def _polygon_volume(polys):
    """Signed volume of a closed set of outward-oriented planar polygons (divergence
    theorem, fan from each polygon's first vertex)."""
    v = 0.0
    for poly in polys:
        a = poly[0]
        for i in range(1, len(poly) - 1):
            v += geom.dot(a, geom.cross(poly[i], poly[i + 1])) / 6.0
    return v


def _supporting_planes(coords, tris, tol):
    """Outward supporting planes (n, d) of a convex closed triangle shell.

    Raises NonConvexError if some vertex lies beyond a facet plane by more than tol.
    """
    planes = {}
    for a, b, c in tris:
        pa, pb, pc = coords[a], coords[b], coords[c]
        n = geom.cross(_sub(pb, pa), _sub(pc, pa))
        ln = geom.length(n)
        if ln == 0.0:
            continue
        n = (n[0] / ln, n[1] / ln, n[2] / ln)
        d = geom.dot(n, pa)
        key = tuple(round(x, 9) for x in n) + (round(d, 9),)
        planes[key] = (n, d)
    out = list(planes.values())
    for n, d in out:
        for p in coords:
            if geom.dot(n, p) - d > tol:
                raise NonConvexError("operand is not convex (vertex beyond a facet plane)")
    return out


def _newell(poly):
    """Newell normal (not normalised) of a planar polygon, outward for a CCW loop."""
    nx = ny = nz = 0.0
    m = len(poly)
    for i in range(m):
        x1, y1, z1 = poly[i]
        x2, y2, z2 = poly[(i + 1) % m]
        nx += (y1 - y2) * (z1 + z2)
        ny += (z1 - z2) * (x1 + x2)
        nz += (x1 - x2) * (y1 + y2)
    return (nx, ny, nz)


def _clip(polys, n, d):
    """Keep the part of the polyhedron (given as polygons) with n.x <= d.

    Sutherland-Hodgman per polygon; the cut points form the cap polygon, which is
    ordered counter-clockwise about +n (outward normal of the kept side at the cap)."""
    if not any(geom.dot(n, p) - d < 0.0 for poly in polys for p in poly):
        return []                    # no vertex strictly on the kept side: empty or touching only
    kept, cut = [], []
    for poly in polys:
        s = [geom.dot(n, p) - d for p in poly]
        if all(v == 0.0 for v in s):
            # Face lying in the clipping plane. Its cross-section is the cap, so it is
            # never kept as a face (keeping it AND capping would count it twice).
            if geom.dot(_newell(poly), n) > 0.0:
                cut.extend(poly)
            continue
        out = []
        m = len(poly)
        for i in range(m):
            p, q = poly[i], poly[(i + 1) % m]
            sp, sq = s[i], s[(i + 1) % m]
            if sp <= 0.0:
                out.append(p)
            if sp == 0.0:                          # vertex on the plane: part of the cap
                cut.append(p)
            if (sp < 0.0 < sq) or (sq < 0.0 < sp):
                t = sp / (sp - sq)
                x = (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t, p[2] + (q[2] - p[2]) * t)
                out.append(x)
                cut.append(x)
        if len(out) >= 3:
            kept.append(out)
    # cap: dedupe cut points, order them around their centroid in the plane
    uniq = []
    for x in cut:
        if all(geom.dist(x, y) > 1e-12 for y in uniq):
            uniq.append(x)
    if len(uniq) >= 3:
        cx = sum(p[0] for p in uniq) / len(uniq)
        cy = sum(p[1] for p in uniq) / len(uniq)
        cz = sum(p[2] for p in uniq) / len(uniq)
        c = (cx, cy, cz)
        helper = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
        u = geom.normalize(geom.cross(n, helper))
        v = geom.cross(n, u)
        uniq.sort(key=lambda p: math.atan2(geom.dot(_sub(p, c), v), geom.dot(_sub(p, c), u)))
        kept.append(uniq)
    return kept


class ConvexBackend:
    """Exact overlap volumes for two convex, outward-oriented closed shells.

    volume(kind, solver, shift): kind "AB" = V(A+shift n B), "BA" = V(B n (A+shift)).
    kind "AMB" (A minus B) is not supported here: the identity check is only meaningful
    for an independent kernel, so the pipeline records it as NOT_AVAILABLE.
    """

    solvers = ("PURE_CONVEX",)

    def __init__(self, coords_a, tris_a, coords_b, tris_b):
        self.coords_a, self.tris_a = list(coords_a), [tuple(t) for t in tris_a]
        self.coords_b, self.tris_b = list(coords_b), [tuple(t) for t in tris_b]
        scale = max(geom.length(geom.bbox_dims(geom.bbox(self.coords_a + self.coords_b))), 1e-12)
        self._tol = 1e-9 * scale
        self._pa = _supporting_planes(self.coords_a, self.tris_a, self._tol)
        self._pb = _supporting_planes(self.coords_b, self.tris_b, self._tol)

    def close(self):
        return None

    def volume(self, kind, solver, shift=(0.0, 0.0, 0.0)):
        if kind == "AB":
            polys = [[_shift(p, shift) for p in (self.coords_a[a], self.coords_a[b], self.coords_a[c])]
                     for a, b, c in self.tris_a]
            planes = self._pb
        elif kind == "BA":
            polys = [[self.coords_b[a], self.coords_b[b], self.coords_b[c]] for a, b, c in self.tris_b]
            planes = [(n, d + geom.dot(n, shift)) for n, d in self._pa]
        else:
            return None
        for n, d in planes:
            polys = _clip(polys, n, d)
            if not polys:
                return 0.0
        return _polygon_volume(polys)


# ------------------------------------------------------------- separation bisection

def bisect_separation(volume_at, direction, T, iters, null_volume):
    """Smallest translation t in [0, T] along unit `direction` at which the overlap
    volume is null, by bisection; the returned t is always a VERIFIED separating
    value (its volume was evaluated and is <= null_volume).

    volume_at(t) returns the overlap volume with A shifted by t*direction.
    T must be a separating translation (volume_at(T) <= null_volume); it is checked.
    """
    v0 = volume_at(0.0)
    if v0 <= null_volume:
        return {"status": "NO_OVERLAP_AT_ZERO", "t": None, "volume_at_t": v0}
    lo, hi = 0.0, T
    v_hi = volume_at(hi)
    if v_hi > null_volume:
        return {"status": "BRACKET_NOT_SEPARATING", "t": None, "volume_at_t": v_hi}
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        v_mid = volume_at(mid)
        if v_mid > null_volume:
            lo = mid
        else:
            hi, v_hi = mid, v_mid
    return {"status": "VERIFIED", "t": hi, "lo": lo, "bracket_width": hi - lo,
            "volume_at_t": v_hi, "iterations": iters}


def search_separation(volume_at, T, iters, null_volume, samples):
    """Separating translation along one ray with a monotonicity scan.

    Evaluates the overlap volume at `samples` equally spaced translations in (0, T], takes
    the FIRST sampled translation whose volume is null, and bisects between it and the
    previous sample. The returned t is always VERIFIED (its volume was evaluated and is
    <= null_volume). It is an upper bound on the distance needed along this ray, not a
    proven minimum: overlap that vanishes and reappears BETWEEN samples is not seen.
    `overlap_reappears_after_first_null` reports sampled evidence that the volume function
    is not monotone on this ray (then the single bisection of the legacy search would have
    converged to an arbitrary one of several transitions).

    T must be a separating translation (volume_at(T) <= null_volume); it is checked."""
    v0 = volume_at(0.0)
    if v0 <= null_volume:
        return {"status": "NO_OVERLAP_AT_ZERO", "t": None, "volume_at_t": v0}
    v_T = volume_at(T)
    if v_T > null_volume:
        return {"status": "BRACKET_NOT_SEPARATING", "t": None, "volume_at_t": v_T}
    n = max(int(samples), 1)
    ts = [T * k / n for k in range(n + 1)]
    vs = [v0] + [volume_at(t) for t in ts[1:-1]] + [v_T]
    first = next(k for k in range(1, n + 1) if vs[k] <= null_volume)
    reappear = [ts[k] for k in range(first + 1, n + 1) if vs[k] > null_volume]
    lo, hi, v_hi = ts[first - 1], ts[first], vs[first]
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        v_mid = volume_at(mid)
        if v_mid > null_volume:
            lo = mid
        else:
            hi, v_hi = mid, v_mid
    return {"status": "VERIFIED", "t": hi, "lo": lo, "bracket_width": hi - lo,
            "volume_at_t": v_hi, "iterations": iters,
            "scan": {"samples": n, "first_null_sample_index": first,
                     "overlap_reappears_after_first_null": bool(reappear),
                     "reappearing_sample_t": reappear[:8]},
            "minimal_along_axis": "NOT_ESTABLISHED"}


def make_crossing_counter(coords_a, tris_a, coords_b, tris_b, params):
    """crossings(d, t) -> number of triangle pairs with a PROPER surface crossing deeper
    than tri_contact when A is translated by t*d. None when the parameters are absent."""
    if not params.get("tri_contact"):
        return None
    itol = intersect.Tol(plane=params["tri_plane_coplanar"], contact=params["tri_contact"],
                         area=params["degenerate_area"])
    weld = params.get("coincident_vertex", params["tri_contact"])
    maxp = int(params.get("max_candidate_pairs", 2000000))
    mb = intersect.TriMesh("B", coords_b, tris_b, None, weld, itol)

    def crossings(d, t):
        sh = [(p[0] + d[0] * t, p[1] + d[1] * t, p[2] + d[2] * t) for p in coords_a]
        ma = intersect.TriMesh("A", sh, tris_a, None, weld, itol)
        return intersect.cross_intersections(ma, mb, itol, maxp)["counts"]["intersect"]
    return crossings


def certify_translation(volume_at, crossings, d, t0, step0, t_max, null_volume, max_adjust=24):
    """Certify that A moved by t*d is separated, adjusting t upward if needed.

    Certificate = null overlap volume AND no proper surface crossing deeper than
    tri_contact. A volume threshold alone can pass while a sharp tip (volume ~ h^3)
    still penetrates by up to ~1e-4 m; the crossing test cannot. With no surface
    crossing, a non-null volume would require one solid to be contained in the other,
    which the volume test excludes, so the interiors are disjoint (touching allowed) to
    within tri_contact.
    Returns {ok, t, adjustments, added_m, crossing_pairs, volume_at_t}."""
    t, step, n = t0, step0, 0
    t_fail = None
    while True:
        c = crossings(d, t)
        v = volume_at(t)
        if c == 0 and v <= null_volume:
            added_search = 0
            if t_fail is not None:          # tighten: bisect between the last failure and this pass
                lo, hi, v_hi = t_fail, t, v
                for _ in range(12):
                    mid = 0.5 * (lo + hi)
                    if crossings(d, mid) == 0 and volume_at(mid) <= null_volume:
                        hi = mid
                    else:
                        lo = mid
                    added_search += 1
                t, v = hi, volume_at(hi)
            return {"ok": True, "t": t, "adjustments": n, "added_m": t - t0,
                    "crossing_pairs": 0, "volume_at_t": v, "tightening_steps": added_search}
        if n >= max_adjust or t >= t_max:
            return {"ok": False, "t": t, "adjustments": n, "added_m": t - t0,
                    "crossing_pairs": c, "volume_at_t": v}
        t_fail = t
        t = min(t + step, t_max)
        step *= 2.0
        n += 1


def axial_separations(coords_a, coords_b, volume_at_dir, null_volume, iters, samples=0, crossings=None):
    """Separating translations of A along the six world axes. volume_at_dir(direction, t) -> volume.

    T_d = max_B(d.x) - min_A(d.x): translating A by T_d along d puts A entirely beyond B,
    a separating translation for any pair of shapes. samples > 1 selects the scanned search
    (search_separation); 0 selects the single bisection (legacy, kept for regression).
    `crossings` (make_crossing_counter) enables the surface-crossing certificate."""
    out = {}
    for label, d in AXES:
        pa = [geom.dot(d, p) for p in coords_a]
        pb = [geom.dot(d, p) for p in coords_b]
        T = max(pb) - min(pa)
        va = lambda t, d=d: volume_at_dir(d, t)  # noqa: E731
        if samples and samples > 1:
            r = search_separation(va, T, iters, null_volume, samples)
        else:
            r = bisect_separation(va, d, T, iters, null_volume)
        r["T_bracket"] = T
        if crossings is not None and r["status"] == "VERIFIED":
            step0 = max(r["bracket_width"], 1e-6)
            c = certify_translation(va, lambda dd, tt: crossings(dd, tt), d, r["t"], step0, T, null_volume)
            r["certificate"] = c
            if c["ok"]:
                r["t_before_certificate"] = r["t"]
                r["t"], r["volume_at_t"] = c["t"], c["volume_at_t"]
            else:
                r["status"] = "UNCERTIFIED"
        out[label] = r
    return out


# ------------------------------------------------------------- refinement hook

def _refine_operand(coords, tris, params, reasons):
    """Refinement attempt for one operand refused ONLY for DEGENERATE_FACE. Returns
    (coords, tris, reasons, record) describing what to measure; record is None when no
    refinement was attempted."""
    if reasons != ["DEGENERATE_FACE"] or not params.get("refine_edge_tol"):
        return coords, tris, reasons, None
    out = refine.refine_for_gate(coords, tris, params["degenerate_area"], params["refine_edge_tol"],
                                 lambda c, t: gate(c, t, params["degenerate_area"]))
    rec = {"status": out["status"], "classification": out["class_"], "steps": out["steps"],
           "failures": out["failures"]}
    if out["report"] is not None:
        rec["preservation"] = out["report"]
    if out["status"] == "REFINED":
        res = out["result"]
        return res["coords"], res["tris"], [], rec
    return coords, tris, reasons + ["REFINEMENT_" + out["status"]], rec


# ------------------------------------------------------------- pipeline

def measure_pair(coords_a, tris_a, coords_b, tris_b, open_backend, params):
    """Gate -> (refine) -> orient (in-memory copies) -> volumes per solver -> consistency ->
    axial separation bounds. Returns a plain dict; never raises for ordinary refusals.

    open_backend(coords_a, tris_a, coords_b, tris_b) -> backend with
        volume(kind, solver, shift) and close(); its `solvers` tuple lists kernels.
    params: null_volume, degenerate_area, iters, agree_rel, identity_rel; optional:
        refine_edge_tol (enables the DEGENERATE_FACE refinement, core/refine.py),
        axial_scan_samples (scanned search), tri_contact + tri_plane_coplanar
        (surface-crossing certificate on every returned translation).
    """
    null = params["null_volume"]
    rec = {"status": None, "reasons": {}, "orientation": {}, "solvers": [],
           "volumes": {}, "solver_agreement": None, "symmetry_rel_diff": None,
           "identity": None, "axial": None, "separation_bound_m": None, "separation_direction": None,
           "separation_status": "NOT_COMPUTED", "geometry_basis": "ORIGINAL", "refinement": {}}

    orig = {"A": (coords_a, tris_a), "B": (coords_b, tris_b)}
    cur = {}
    for key in ("A", "B"):
        c, t = orig[key]
        reasons = gate(c, t, params["degenerate_area"])
        c, t, reasons, rrec = _refine_operand(c, t, params, reasons)
        if rrec is not None:
            rec["refinement"][key] = rrec
        rec["reasons"][key] = reasons
        cur[key] = (c, t)
    if any(rec["reasons"].values()):
        rec["status"] = "UNABLE_TO_DETERMINE"
        return rec
    refined = sorted(k for k, v in rec["refinement"].items() if v["status"] == "REFINED")
    if refined:
        rec["geometry_basis"] = "REFINED_COPY:" + ",".join(
            f"{k}={rec['refinement'][k]['classification']}" for k in refined)
    coords_a, tris_a = cur["A"]
    coords_b, tris_b = cur["B"]

    oa, ia = orient_components(coords_a, tris_a, null)
    ob, ib = orient_components(coords_b, tris_b, null)
    rec["orientation"] = {"A": ia, "B": ib}
    zero = [k for k, info in (("A", ia), ("B", ib)) if info["zero_volume_components"]]
    if zero:
        rec["status"] = "UNABLE_TO_DETERMINE"
        rec["reasons"]["zero_volume_component_in"] = zero
        return rec

    try:
        backend = open_backend(coords_a, oa, coords_b, ob)
    except NonConvexError as e:
        rec["status"] = "UNSUPPORTED_BY_BACKEND"
        rec["reasons"]["backend"] = str(e)
        return rec
    try:
        solvers = list(backend.solvers)
        rec["solvers"] = solvers
        primary = solvers[0]
        vol = {}
        for s in solvers:
            vol[("AB", s)] = backend.volume("AB", s, (0.0, 0.0, 0.0))
        rec["volumes"]["AB"] = {s: vol[("AB", s)] for s in solvers}

        vals = [vol[("AB", s)] for s in solvers]
        spread = max(vals) - min(vals)
        scale = max(max(abs(v) for v in vals), null)
        rec["solver_agreement"] = {"max_abs_diff_m3": spread, "rel_diff": spread / scale,
                                   "ok": spread <= params["agree_rel"] * scale}

        v_primary = vol[("AB", primary)]
        if v_primary <= null:
            rec["status"] = "NULL_OVERLAP"
            rec["reasons"]["note"] = "no volumetric overlap; any triangle crossing is surface-level"
            return rec

        ba = backend.volume("BA", primary, (0.0, 0.0, 0.0))
        rec["volumes"]["BA"] = {primary: ba}
        scale_ab = max(abs(v_primary), abs(ba), null)
        rec["symmetry_rel_diff"] = abs(v_primary - ba) / scale_ab

        v_a = geom.signed_volume(coords_a, oa)
        amb = backend.volume("AMB", primary, (0.0, 0.0, 0.0))
        if amb is None:
            rec["identity"] = "NOT_AVAILABLE"
        else:
            rec["volumes"]["AMB"] = {primary: amb}
            err = abs(v_primary + amb - v_a) / max(abs(v_a), null)
            rec["identity"] = {"V_A_m3": v_a, "V_AB_plus_AMB_m3": v_primary + amb,
                               "rel_error": err, "ok": err <= params["identity_rel"]}

        # A refined copy is only trusted if the ORIGINAL (unrefined) shells, run through the same
        # kernels, give the same volume: the refinement must not move the measurement.
        if refined:
            rec["refinement_effect"] = _refinement_effect(open_backend, orig, params, null, vol, solvers)

        def vol_at(d, t, solver=primary):
            return backend.volume("AB", solver, (d[0] * t, d[1] * t, d[2] * t))

        crossings = make_crossing_counter(coords_a, tris_a, coords_b, tris_b, params)
        ax = axial_separations(coords_a, coords_b, lambda d, t: vol_at(d, t), null, params["iters"],
                               int(params.get("axial_scan_samples", 0)), crossings)
        rec["axial"] = ax
        ok = [(ax[k]["t"], k) for k, _ in AXES if ax[k]["status"] == "VERIFIED"]
        if ok:
            depth, label = min(ok)
            rec["separation_bound_m"], rec["separation_direction"] = depth, label
            rec["separation_residual_volume_m3"] = ax[label]["volume_at_t"]
            # cross-check the minimising translation with every other kernel
            d = dict(AXES)[label]
            rec["separation_cross_check"] = {s: backend.volume("AB", s, (d[0] * depth, d[1] * depth, d[2] * depth))
                                             for s in solvers[1:]}
            rec["separation_cross_check_ok"] = all(v <= null for v in rec["separation_cross_check"].values())
        rec["axial_nonmonotone_directions"] = [
            k for k, _ in AXES if (ax[k].get("scan") or {}).get("overlap_reappears_after_first_null")]

        bad = [k for k in ax if ax[k]["status"] != "VERIFIED"]
        identity_ok = rec["identity"] in ("NOT_AVAILABLE",) or (isinstance(rec["identity"], dict) and rec["identity"]["ok"])
        # Volume verdict: kernel agreement, operand symmetry, the identity and (refined copies) concordance.
        agree_ok = rec["solver_agreement"]["ok"] and rec["symmetry_rel_diff"] <= params["agree_rel"]
        eff = rec.get("refinement_effect")
        if not agree_ok:
            rec["status"] = "SOLVER_DISAGREEMENT"
        elif not identity_ok:
            rec["status"] = "IDENTITY_FAILED"
        elif eff is not None and eff["status"] == "NOT_AVAILABLE":
            rec["status"] = "REFINEMENT_UNCONFIRMED"
        elif eff is not None and not eff["ok"]:
            rec["status"] = "REFINEMENT_AFFECTS_RESULT"
        else:
            rec["status"] = "VOLUME_VALIDATED" if rec["identity"] != "NOT_AVAILABLE" else "VOLUME_MEASURED"
        # Separation verdict is separate: CONFIRMED only if every kernel confirms the minimising translation.
        if ok:
            rec["separation_status"] = ("CONFIRMED_BY_ALL_KERNELS" if rec.get("separation_cross_check_ok", True)
                                        else "PRIMARY_KERNEL_ONLY")
        else:
            rec["separation_status"] = "NOT_COMPUTED"
        rec["axial_unverified_directions"] = bad
        rec["overlap_volume_m3"] = v_primary
        return rec
    finally:
        backend.close()


def _refinement_effect(open_backend, orig, params, null, refined_vol, solvers):
    """Overlap volume of the ORIGINAL shells (same kernels) against the refined copy's."""
    try:
        (ca, ta), (cb, tb) = orig["A"], orig["B"]
        oa, _ = orient_components(ca, ta, null)
        ob, _ = orient_components(cb, tb, null)
        be = open_backend(ca, oa, cb, ob)
        try:
            v0 = {s: be.volume("AB", s, (0.0, 0.0, 0.0)) for s in be.solvers}
        finally:
            be.close()
    except Exception as e:  # noqa: BLE001 - recorded, and it blocks validation
        return {"status": "NOT_AVAILABLE", "ok": False, "error": f"{type(e).__name__}: {e}"}
    rows = {}
    ok = True
    for s in solvers:
        if s not in v0:
            continue
        vr = refined_vol[("AB", s)]
        sc = max(abs(vr), abs(v0[s]), null)
        rel = abs(vr - v0[s]) / sc
        rows[s] = {"V_original_m3": v0[s], "V_refined_m3": vr, "abs_diff_m3": abs(vr - v0[s]), "rel_diff": rel}
        ok = ok and rel <= params["agree_rel"]
    return {"status": "COMPARED", "ok": ok, "by_solver": rows, "tolerance_rel": params["agree_rel"]}
