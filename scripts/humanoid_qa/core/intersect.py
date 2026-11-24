"""Triangle intersection, mesh self-intersection, inter-mesh intersection and
proximity (pure Python, stdlib only; no bpy).

METHOD SUMMARY
  classify_triangle_pair(t1, t2, tol)   exact predicates + explicit tolerances
      degenerate        either triangle has area <= tol.area or an edge <=
                        tol.contact (collapsed sliver; normal undefined; not judged)
      coplanar_overlap  one triangle lies within tol.plane of the other's plane
                        (a tolerance slab) AND the clipped overlap area > tol.area
      intersect         a PROPER (strict-interior) edge/face crossing exists AND
                        its depth (see crossing_depth) exceeds tol.contact
      contact           touching along an edge/vertex, min distance <= tol.contact,
                        or a proper crossing no deeper than tol.contact (that is
                        indistinguishable from touching at coordinate precision)
      disjoint          otherwise
  TriBVH                AABB hierarchy over triangles; candidate pairs are a
                        SUPERSET (box overlap); the exact classifier then
                        filters them. Never infer "no intersection" from "no
                        candidates" without the recorded method limits.
  self_intersections()  candidate pairs minus ordinary adjacency. Adjacency
                        uses WELDED vertex ids (positions within weld_tol are
                        one vertex) so split-vertex seams are not reported.
  cross_intersections() two meshes; no adjacency concept.
  min_distance()        exact branch-and-bound minimum separation.

GEOMETRY BASIS: whatever coordinates the caller provides (the adapter passes
BASE mesh data in world space, metres). Evaluated (post-modifier) geometry is
NOT analysed here.

All distances are in the units of the input coordinates.
"""

import heapq
import math
from collections import namedtuple

from . import geom

Tol = namedtuple("Tol", "plane contact area")

PARALLEL_REL = 1e-12      # relative |det| below which a segment is parallel to a triangle
BARY_EPS = 1e-9           # strict-interior margin (dimensionless) for proper crossings
CAP_PER_CATEGORY = 200    # stored examples per category (counts are always complete)


class BudgetExceeded(RuntimeError):
    pass


# ------------------------------------------------------------- primitives

def tri_normal(t):
    return geom.cross(geom.sub(t[1], t[0]), geom.sub(t[2], t[0]))


def tri_area(t):
    return 0.5 * geom.length(tri_normal(t))


def is_degenerate(t, tol):
    """Area <= tol.area OR any edge <= tol.contact. An edge no longer than the
    contact tolerance is below coordinate precision, so the triangle's normal
    is numerically undefined (a collapsed sliver)."""
    return (tri_area(t) <= tol.area or
            min(geom.dist(t[0], t[1]), geom.dist(t[1], t[2]), geom.dist(t[2], t[0])) <= tol.contact)


def box_of(points):
    return geom.bbox(points)


def box_overlap(a, b, margin=0.0):
    return (a[0][0] - margin <= b[1][0] and b[0][0] - margin <= a[1][0] and
            a[0][1] - margin <= b[1][1] and b[0][1] - margin <= a[1][1] and
            a[0][2] - margin <= b[1][2] and b[0][2] - margin <= a[1][2])


def box_gap(a, b):
    """Euclidean distance between two AABBs (0 when they overlap)."""
    s = 0.0
    for i in range(3):
        d = max(a[0][i] - b[1][i], b[0][i] - a[1][i], 0.0)
        s += d * d
    return math.sqrt(s)


def closest_point_on_triangle(p, a, b, c):
    """Closest point on triangle abc to p (Ericson, Real-Time Collision
    Detection 5.1.5). Degenerate triangles fall back to their three edges."""
    ab, ac, ap = geom.sub(b, a), geom.sub(c, a), geom.sub(p, a)
    d1, d2 = geom.dot(ab, ap), geom.dot(ac, ap)
    if d1 <= 0 and d2 <= 0:
        return a
    bp = geom.sub(p, b)
    d3, d4 = geom.dot(ab, bp), geom.dot(ac, bp)
    if d3 >= 0 and d4 <= d3:
        return b
    vc = d1 * d4 - d3 * d2
    if vc <= 0 and d1 >= 0 and d3 <= 0:
        return geom.add(a, geom.scale(ab, d1 / (d1 - d3)))
    cp = geom.sub(p, c)
    d5, d6 = geom.dot(ab, cp), geom.dot(ac, cp)
    if d6 >= 0 and d5 <= d6:
        return c
    vb = d5 * d2 - d1 * d6
    if vb <= 0 and d2 >= 0 and d6 <= 0:
        return geom.add(a, geom.scale(ac, d2 / (d2 - d6)))
    va = d3 * d6 - d5 * d4
    if va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0:
        return geom.add(b, geom.scale(geom.sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))))
    s = va + vb + vc
    if s == 0.0:                       # collinear triangle: nearest of its edges
        best = min((closest_points_segments(p, p, u, v) for u, v in ((a, b), (b, c), (c, a))),
                   key=lambda r: r[0])
        return best[2]
    v, w = vb / s, vc / s
    return geom.add(a, geom.add(geom.scale(ab, v), geom.scale(ac, w)))


def _clamp01(x):
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x


def closest_points_segments(p1, q1, p2, q2):
    """(distance, point_on_1, point_on_2) between segments p1q1 and p2q2
    (Ericson 5.1.9), including zero-length (point) segments."""
    d1, d2, r = geom.sub(q1, p1), geom.sub(q2, p2), geom.sub(p1, p2)
    a, e, f = geom.dot(d1, d1), geom.dot(d2, d2), geom.dot(d2, r)
    tiny = 1e-30
    if a <= tiny and e <= tiny:
        s = t = 0.0
    elif a <= tiny:
        s, t = 0.0, _clamp01(f / e)
    else:
        c = geom.dot(d1, r)
        if e <= tiny:
            t, s = 0.0, _clamp01(-c / a)
        else:
            b = geom.dot(d1, d2)
            denom = a * e - b * b
            s = _clamp01((b * f - c * e) / denom) if denom > 0.0 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, _clamp01(-c / a)
            elif t > 1.0:
                t, s = 1.0, _clamp01((b - c) / a)
    c1, c2 = geom.add(p1, geom.scale(d1, s)), geom.add(p2, geom.scale(d2, t))
    return geom.dist(c1, c2), c1, c2


def triangle_distance(t1, t2):
    """Exact minimum distance between two triangles that do NOT cross
    (vertex-face and edge-edge candidates). For crossing triangles this is
    >= 0 but not 0; callers test crossing first. Returns (d, p1, p2)."""
    best = (math.inf, None, None)
    for p in t1:
        q = closest_point_on_triangle(p, *t2)
        d = geom.dist(p, q)
        if d < best[0]:
            best = (d, p, q)
    for p in t2:
        q = closest_point_on_triangle(p, *t1)
        d = geom.dist(p, q)
        if d < best[0]:
            best = (d, q, p)
    for i in range(3):
        for j in range(3):
            d, c1, c2 = closest_points_segments(t1[i], t1[(i + 1) % 3], t2[j], t2[(j + 1) % 3])
            if d < best[0]:
                best = (d, c1, c2)
    return best


def segment_pierces_triangle(p, q, tri, strict=True):
    """Hit point if segment pq crosses the triangle's INTERIOR (strict) or
    interior-or-boundary (not strict); None if parallel or no hit."""
    a, b, c = tri
    d, e1, e2 = geom.sub(q, p), geom.sub(b, a), geom.sub(c, a)
    h = geom.cross(d, e2)
    det = geom.dot(e1, h)
    if abs(det) <= PARALLEL_REL * geom.length(d) * geom.length(e1) * geom.length(e2):
        return None
    f = 1.0 / det
    s = geom.sub(p, a)
    u = f * geom.dot(s, h)
    qv = geom.cross(s, e1)
    v = f * geom.dot(d, qv)
    t = f * geom.dot(e2, qv)
    m = BARY_EPS if strict else -BARY_EPS
    if u < m or v < m or u + v > 1.0 - m or t < m or t > 1.0 - m:
        return None
    return geom.add(p, geom.scale(d, t))


def _tri_edges(t):
    return ((t[0], t[1]), (t[1], t[2]), (t[2], t[0]))


def find_proper_crossing(t1, t2):
    """First proper (strict-interior) crossing point of an edge of one
    triangle through the other, else None."""
    for p, q in _tri_edges(t1):
        hit = segment_pierces_triangle(p, q, t2)
        if hit is not None:
            return hit
    for p, q in _tri_edges(t2):
        hit = segment_pierces_triangle(p, q, t1)
        if hit is not None:
            return hit
    return None


# ------------------------------------------------------------- coplanar overlap

def _project(t, k):
    i, j = [x for x in range(3) if x != k]
    return [(p[i], p[j]) for p in t]


def _signed_area2(poly):
    return 0.5 * sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
                     for i in range(len(poly)))


def _clip(subject, clip):
    """Sutherland-Hodgman: subject polygon clipped by a CCW convex polygon."""
    out = subject
    for i in range(len(clip)):
        a, b = clip[i], clip[(i + 1) % len(clip)]
        inp, out = out, []
        if not inp:
            break

        def side(p):
            return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])

        def cut(s, e):
            ds, de = side(s), side(e)
            tt = ds / (ds - de)
            return (s[0] + (e[0] - s[0]) * tt, s[1] + (e[1] - s[1]) * tt)
        s = inp[-1]
        for e in inp:
            if side(e) >= 0:
                if side(s) < 0:
                    out.append(cut(s, e))
                out.append(e)
            elif side(s) >= 0:
                out.append(cut(s, e))
            s = e
    return out


def coplanar_overlap_area(t1, t2):
    """Area (true 3-D units^2) of the overlap of two coplanar triangles."""
    n = tri_normal(t1)
    k = max(range(3), key=lambda i: abs(n[i]))
    nk = abs(n[k]) / geom.length(n)
    a, b = _project(t1, k), _project(t2, k)
    if _signed_area2(a) < 0:
        a.reverse()
    if _signed_area2(b) < 0:
        b.reverse()
    poly = _clip(b, a)
    return abs(_signed_area2(poly)) / nk if len(poly) >= 3 else 0.0


def overlap_in_plane(t1, t2, tol_plane):
    """(is_coplanar, overlap_area). Coplanar means ONE triangle lies entirely
    within tol_plane of the other's plane (a slab test, so a small triangle
    lying on a large slightly-tilted one is still coplanar); the overlap is
    then computed in the reference triangle's plane."""
    n1, n2 = geom.normalize(tri_normal(t1)), geom.normalize(tri_normal(t2))
    if all(abs(geom.dot(n1, geom.sub(p, t1[0]))) <= tol_plane for p in t2):
        return True, coplanar_overlap_area(t1, t2)
    if all(abs(geom.dot(n2, geom.sub(p, t2[0]))) <= tol_plane for p in t1):
        return True, coplanar_overlap_area(t2, t1)
    return False, 0.0


def _side_depth(ds):
    """How far the lesser side of a straddling triangle reaches past the
    other triangle's plane (0 if the triangle does not straddle it)."""
    lo, hi = min(ds), max(ds)
    return min(hi, -lo) if lo < 0.0 < hi else 0.0


def crossing_depth(t1, t2):
    """Penetration-depth proxy for two triangles that cross: the smaller of
    (t2 vs plane of t1) and (t1 vs plane of t2) lesser-side reaches."""
    n1, n2 = geom.normalize(tri_normal(t1)), geom.normalize(tri_normal(t2))
    d2 = [geom.dot(n1, geom.sub(p, t1[0])) for p in t2]
    d1 = [geom.dot(n2, geom.sub(p, t2[0])) for p in t1]
    return min(_side_depth(d2), _side_depth(d1))


def exact_proper_crossing(t1, t2):
    """EXACT (rational-arithmetic) strict-interior edge/face crossing test on
    the float coordinates, with no tolerances. Slow (stdlib fractions); used
    as an independent oracle for tests and for adjudicating disagreements."""
    from fractions import Fraction as Fr

    def sub(a, b):
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

    def cr(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    def dt(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    def pierce(p, q, tri):
        a, b, c = [tuple(Fr(x) for x in v) for v in tri]
        p, q = tuple(Fr(x) for x in p), tuple(Fr(x) for x in q)
        d, e1, e2 = sub(q, p), sub(b, a), sub(c, a)
        h = cr(d, e2)
        det = dt(e1, h)
        if det == 0:
            return False
        s_ = sub(p, a)
        u = dt(s_, h) / det
        if not 0 < u < 1:
            return False
        qv = cr(s_, e1)
        v = dt(d, qv) / det
        if not (v > 0 and u + v < 1):
            return False
        return 0 < dt(e2, qv) / det < 1
    for i in range(3):
        if pierce(t1[i], t1[(i + 1) % 3], t2) or pierce(t2[i], t2[(i + 1) % 3], t1):
            return True
    return False


# ------------------------------------------------------------- pair classifier

def classify_triangle_pair(t1, t2, tol):
    """See module docstring. Returns {"kind", "distance", "overlap_area",
    "location", "depth"} (keys present only when meaningful)."""
    if is_degenerate(t1, tol) or is_degenerate(t2, tol):
        return {"kind": "degenerate", "distance": triangle_distance(t1, t2)[0]}
    cop, area = overlap_in_plane(t1, t2, tol.plane)
    if cop:
        if area > tol.area:
            return {"kind": "coplanar_overlap", "overlap_area": area, "distance": 0.0}
        d, p, q = triangle_distance(t1, t2)
        return {"kind": "contact" if d <= tol.contact else "disjoint", "distance": d,
                "location": geom.scale(geom.add(p, q), 0.5)}
    n1, n2 = geom.normalize(tri_normal(t1)), geom.normalize(tri_normal(t2))
    d2 = [geom.dot(n1, geom.sub(p, t1[0])) for p in t2]
    d1 = [geom.dot(n2, geom.sub(p, t2[0])) for p in t1]
    if min(d2) > tol.contact or max(d2) < -tol.contact or min(d1) > tol.contact or max(d1) < -tol.contact:
        return {"kind": "disjoint", "distance": None}          # plane-separated beyond contact
    hit = find_proper_crossing(t1, t2)
    if hit is not None:
        depth = min(_side_depth(d2), _side_depth(d1))
        if depth > tol.contact:
            return {"kind": "intersect", "distance": 0.0, "location": hit, "depth": depth}
        return {"kind": "contact", "distance": 0.0, "location": hit, "shallow_crossing_depth": depth}
    d, p, q = triangle_distance(t1, t2)
    return {"kind": "contact" if d <= tol.contact else "disjoint", "distance": d,
            "location": geom.scale(geom.add(p, q), 0.5)}


# ------------------------------------------------------------- BVH

class TriBVH:
    """AABB hierarchy over triangles (median split on the widest centroid
    axis). Candidate queries return a SUPERSET of intersecting pairs."""

    LEAF = 4

    def __init__(self, boxes, ids):
        self.boxes = boxes
        self.root = self._build(sorted(ids)) if ids else None

    def _build(self, ids):
        lo = tuple(min(self.boxes[i][0][a] for i in ids) for a in range(3))
        hi = tuple(max(self.boxes[i][1][a] for i in ids) for a in range(3))
        if len(ids) <= self.LEAF:
            return (lo, hi, None, None, ids)

        def cen(i, a):
            return self.boxes[i][0][a] + self.boxes[i][1][a]
        axis = max(range(3), key=lambda a: max(cen(i, a) for i in ids) - min(cen(i, a) for i in ids))
        ids = sorted(ids, key=lambda i: (cen(i, axis), i))
        m = len(ids) // 2
        return (lo, hi, self._build(ids[:m]), self._build(ids[m:]), None)

    @property
    def box(self):
        return None if self.root is None else (self.root[0], self.root[1])

    @staticmethod
    def _vol(n):
        return (n[1][0] - n[0][0]) * (n[1][1] - n[0][1]) * (n[1][2] - n[0][2])

    def _cross(self, a, other, b, margin, out, limit, same):
        if not box_overlap((a[0], a[1]), (b[0], b[1]), margin):
            return
        if a[4] is not None and b[4] is not None:
            for i in a[4]:
                for j in b[4]:
                    if box_overlap(self.boxes[i], other.boxes[j], margin):
                        out.append((min(i, j), max(i, j)) if same else (i, j))
                        if len(out) > limit:
                            raise BudgetExceeded(f"more than {limit} candidate pairs")
            return
        if a[4] is not None:
            for ch in (b[2], b[3]):
                self._cross(a, other, ch, margin, out, limit, same)
        elif b[4] is not None:
            for ch in (a[2], a[3]):
                self._cross(ch, other, b, margin, out, limit, same)
        elif self._vol(a) >= self._vol(b):
            for ch in (a[2], a[3]):
                self._cross(ch, other, b, margin, out, limit, same)
        else:
            for ch in (b[2], b[3]):
                self._cross(a, other, ch, margin, out, limit, same)

    def self_pairs(self, margin, limit):
        """All unordered pairs (i < j) whose triangle boxes overlap."""
        out = []
        if self.root is None:
            return out

        def rec(n):
            if n[4] is not None:
                ids = n[4]
                for x in range(len(ids)):
                    for y in range(x + 1, len(ids)):
                        if box_overlap(self.boxes[ids[x]], self.boxes[ids[y]], margin):
                            out.append((min(ids[x], ids[y]), max(ids[x], ids[y])))
                return
            rec(n[2])
            rec(n[3])
            self._cross(n[2], self, n[3], margin, out, limit, True)
            if len(out) > limit:
                raise BudgetExceeded(f"more than {limit} candidate pairs")
        rec(self.root)
        return sorted(set(out))

    def cross_pairs(self, other, margin, limit):
        """All pairs (i in self, j in other) whose triangle boxes overlap."""
        out = []
        if self.root is None or other.root is None:
            return out
        self._cross(self.root, other, other.root, margin, out, limit, False)
        return sorted(set(out))

    def min_distance(self, other, tri_a, tri_b, radius=math.inf):
        """Exact branch-and-bound minimum triangle distance within `radius`.
        tri_a / tri_b map triangle id -> corner tuple. Returns
        (d, i, j, p, q) or None when nothing lies within radius."""
        if self.root is None or other.root is None:
            return None
        best_d, best = radius, None
        counter = 0
        heap = [(box_gap(self.box, other.box), counter, self.root, other.root)]
        while heap:
            lb, _, a, b = heapq.heappop(heap)
            if lb > best_d or (best is not None and lb >= best_d):
                break
            if a[4] is not None and b[4] is not None:
                for i in a[4]:
                    for j in b[4]:
                        if box_gap(self.boxes[i], other.boxes[j]) > best_d:
                            continue
                        d, p, q = triangle_distance(tri_a[i], tri_b[j])
                        if d < best_d or (best is None and d <= best_d):
                            best_d, best = d, (d, i, j, p, q)
                continue
            if a[4] is not None or (b[4] is None and self._vol(b) > self._vol(a)):
                kids = [(a, b[2]), (a, b[3])]
            else:
                kids = [(a[2], b), (a[3], b)]
            for x, y in kids:
                g = box_gap((x[0], x[1]), (y[0], y[1]))
                if g <= best_d:
                    counter += 1
                    heapq.heappush(heap, (g, counter, x, y))
        return best


# ------------------------------------------------------------- mesh level

class TriMesh:
    """Triangle soup with welded adjacency ids, degenerate exclusion and BVH.

    coords : list of (x, y, z); tris : list of vertex-index triples;
    polys  : source polygon index per triangle (for reporting; may be None).
    """

    def __init__(self, name, coords, tris, polys, weld_tol, tol):
        self.name, self.coords = name, coords
        self.tris = [tuple(t) for t in tris]
        self.polys = list(polys) if polys is not None else [None] * len(self.tris)
        parent = list(range(len(coords)))

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x
        for i, j in geom.coincident_pairs(coords, weld_tol):
            ri, rj = find(i), find(j)
            if ri != rj:
                parent[max(ri, rj)] = min(ri, rj)
        self.weld = [find(i) for i in range(len(coords))]
        self.welded = [tuple(self.weld[v] for v in t) for t in self.tris]
        self.corners = [(coords[a], coords[b], coords[c]) for a, b, c in self.tris]
        self.degenerate = [i for i, c in enumerate(self.corners)
                           if len(set(self.welded[i])) < 3 or is_degenerate(c, tol)]
        dset = set(self.degenerate)
        self.active = [i for i in range(len(self.tris)) if i not in dset]
        self.boxes = {i: box_of(self.corners[i]) for i in self.active}
        self.bvh = TriBVH(self.boxes, self.active)
        self.tol = tol

    def where(self, i):
        return {"tri": i, "face": self.polys[i], "vertices": list(self.tris[i])}


def _shared(ids1, ids2):
    return sorted(set(ids1) & set(ids2))


def _cap(lst):
    return lst[:CAP_PER_CATEGORY]


def self_intersections(mesh, tol, max_pairs):
    """Self-intersection of one mesh. Adjacent triangles are NOT reported as
    intersecting; only fold-overs and genuine crossings are.

    Adjacency rules (welded vertex ids):
      3 shared -> duplicate_triangle
      2 shared (edge)   -> only a coplanar overlap beyond the edge (fold-over)
      1 shared (vertex) -> only a proper crossing by the OPPOSITE edges, or a
                           coplanar overlap
      0 shared          -> full classifier (intersect/coplanar_overlap are
                           definite; contact is 'possible')
    """
    cand = mesh.bvh.self_pairs(tol.contact, max_pairs)
    definite, contact, adjacent_ordinary, dup = [], [], 0, []
    counts = {"intersect": 0, "coplanar_overlap": 0, "adjacent_fold_overlap": 0,
              "duplicate_triangle": 0, "contact": 0}
    for i, j in cand:
        ti, tj = mesh.corners[i], mesh.corners[j]
        sh = _shared(mesh.welded[i], mesh.welded[j])
        rec = None
        if len(sh) == 3:
            counts["duplicate_triangle"] += 1
            dup.append({"a": mesh.where(i), "b": mesh.where(j), "kind": "duplicate_triangle"})
            continue
        if len(sh) == 2:
            cop, area = overlap_in_plane(ti, tj, tol.plane)
            if cop and area > tol.area:
                rec = {"kind": "adjacent_fold_overlap", "overlap_area": area}
            if rec is None:
                adjacent_ordinary += 1
                continue
        elif len(sh) == 1:
            v = sh[0]
            ei = [(p, q) for (p, q), (a, b) in zip(_tri_edges(ti), ((mesh.welded[i][0], mesh.welded[i][1]),
                                                                    (mesh.welded[i][1], mesh.welded[i][2]),
                                                                    (mesh.welded[i][2], mesh.welded[i][0])))
                  if v not in (a, b)]
            ej = [(p, q) for (p, q), (a, b) in zip(_tri_edges(tj), ((mesh.welded[j][0], mesh.welded[j][1]),
                                                                    (mesh.welded[j][1], mesh.welded[j][2]),
                                                                    (mesh.welded[j][2], mesh.welded[j][0])))
                  if v not in (a, b)]
            hit = None
            for p, q in ei:
                hit = hit or segment_pierces_triangle(p, q, tj)
            for p, q in ej:
                hit = hit or segment_pierces_triangle(p, q, ti)
            cop, area = overlap_in_plane(ti, tj, tol.plane)
            if cop and area > tol.area:
                rec = {"kind": "coplanar_overlap", "overlap_area": area}
            elif hit is not None:
                depth = crossing_depth(ti, tj)
                rec = ({"kind": "intersect", "location": hit, "depth": depth} if depth > tol.contact
                       else {"kind": "contact", "location": hit, "shallow_crossing_depth": depth})
            else:
                adjacent_ordinary += 1
                continue
        else:
            rec = classify_triangle_pair(ti, tj, tol)
            if rec["kind"] == "disjoint":
                continue
            if rec["kind"] == "degenerate":
                continue
        kind = rec["kind"]
        counts[kind] = counts.get(kind, 0) + 1
        item = {"a": mesh.where(i), "b": mesh.where(j), **rec}
        (contact if kind == "contact" else definite).append(item)
    definite.sort(key=lambda r: (r["a"]["tri"], r["b"]["tri"]))
    contact.sort(key=lambda r: (r["a"]["tri"], r["b"]["tri"]))
    return {
        "mesh": mesh.name, "triangles": len(mesh.tris), "active_triangles": len(mesh.active),
        "degenerate_triangles_excluded": list(mesh.degenerate),
        "candidate_pairs": len(cand), "adjacent_pairs_ordinary": adjacent_ordinary,
        "counts": counts, "definite_count": counts["intersect"] + counts["coplanar_overlap"]
        + counts["adjacent_fold_overlap"], "definite": _cap(definite),
        "contact_count": counts["contact"], "contact": _cap(contact),
        "duplicate_triangles": _cap(dup),
        "method": "TriBVH box candidates (margin=tol.contact) -> welded-adjacency filter -> exact classifier",
    }


def cross_intersections(ma, mb, tol, max_pairs):
    """Intersections between two meshes (no adjacency)."""
    cand = ma.bvh.cross_pairs(mb.bvh, tol.contact, max_pairs)
    definite, contact, intersect_all = [], [], []
    counts = {"intersect": 0, "coplanar_overlap": 0, "contact": 0}
    for i, j in cand:
        rec = classify_triangle_pair(ma.corners[i], mb.corners[j], tol)
        k = rec["kind"]
        if k in ("disjoint", "degenerate"):
            continue
        counts[k] += 1
        if k == "intersect":
            intersect_all.append((i, j))
        item = {"a": ma.where(i), "b": mb.where(j), **rec}
        (contact if k == "contact" else definite).append(item)
    definite.sort(key=lambda r: (r["a"]["tri"], r["b"]["tri"]))
    contact.sort(key=lambda r: (r["a"]["tri"], r["b"]["tri"]))
    return {"a": ma.name, "b": mb.name, "candidate_pairs": len(cand), "counts": counts,
            "intersect_pairs_all": intersect_all,
            "definite_count": counts["intersect"] + counts["coplanar_overlap"],
            "definite": _cap(definite), "contact_count": counts["contact"], "contact": _cap(contact),
            "degenerate_excluded": [len(ma.degenerate), len(mb.degenerate)]}


def pair_status(result, a_active, b_active):
    """DEFINITE_INTERSECTION / POSSIBLE_REQUIRES_INSPECTION / NONE_DETECTED /
    UNABLE_TO_DETERMINE for a cross_intersections() result (or None)."""
    if result is None or a_active == 0 or b_active == 0:
        return "UNABLE_TO_DETERMINE"
    if result["definite_count"]:
        return "DEFINITE_INTERSECTION"
    if result["contact_count"]:
        return "POSSIBLE_REQUIRES_INSPECTION"
    return "NONE_DETECTED"


def mesh_proximity(ma, mb, radius):
    """Minimum separation between two meshes within `radius`, with the
    triangles and world points involved; None if farther than radius.
    Valid for NON-intersecting pairs (callers check intersection first)."""
    r = ma.bvh.min_distance(mb.bvh, {i: ma.corners[i] for i in ma.active},
                            {i: mb.corners[i] for i in mb.active}, radius)
    if r is None:
        return None
    d, i, j, p, q = r
    return {"distance": d, "a": ma.where(i), "b": mb.where(j), "point_a": p, "point_b": q}


def reliability_notes(topology):
    """Why a result on this mesh needs care (from geom.topology())."""
    notes = []
    if topology.get("boundary_edges"):
        notes.append("open mesh: volumetric (penetration) interpretation unreliable")
    if topology.get("nonmanifold_edges"):
        notes.append("non-manifold edges present")
    if topology.get("wire_edges"):
        notes.append("loose wire edges present")
    return notes
