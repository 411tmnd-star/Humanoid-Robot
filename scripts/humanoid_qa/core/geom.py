"""Pure geometry: vectors, transforms, bounds, nearest-neighbour, symmetry,
topology. Stdlib only. Points are 3-tuples of floats; matrices are 4x4
row-major nested lists (Blender's Matrix rows, converted by the adapter).

Units are whatever the caller supplies; the adapter supplies metres when the
scene is METRIC with scale_length 1.0 (checked in run_qa, not assumed here).
"""

import math


# ------------------------------------------------------------- vectors

def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def length(a):
    return math.sqrt(dot(a, a))


def dist(a, b):
    return length(sub(a, b))


def normalize(a):
    n = length(a)
    if n == 0.0:
        raise ValueError("cannot normalise a zero-length vector")
    return (a[0] / n, a[1] / n, a[2] / n)


def angle_deg(a, b):
    """Angle between two non-zero vectors, degrees, clamped for rounding."""
    c = dot(normalize(a), normalize(b))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


# ------------------------------------------------------------- transforms

def mat_apply(m, p):
    """Apply a 4x4 row-major affine matrix to a point."""
    x, y, z = p
    return (m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3],
            m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3],
            m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3])


def mirror_x(p, plane_x=0.0):
    """Reflect a point across the plane x = plane_x."""
    return (2.0 * plane_x - p[0], p[1], p[2])


def mirror_dir_x(v):
    """Reflect a direction vector across any plane normal to X."""
    return (-v[0], v[1], v[2])


# ------------------------------------------------------------- bounds

def bbox(points):
    """Axis-aligned bounds of an iterable of points; None when empty."""
    it = iter(points)
    try:
        first = next(it)
    except StopIteration:
        return None
    lo = list(first)
    hi = list(first)
    for p in it:
        for i in range(3):
            if p[i] < lo[i]:
                lo[i] = p[i]
            elif p[i] > hi[i]:
                hi[i] = p[i]
    return tuple(lo), tuple(hi)


def bbox_dims(bb):
    lo, hi = bb
    return (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])


def bbox_centre(bb):
    lo, hi = bb
    return ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2)


def bbox_union(boxes):
    boxes = [b for b in boxes if b is not None]
    if not boxes:
        return None
    return bbox([b[0] for b in boxes] + [b[1] for b in boxes])


# ------------------------------------------------------------- statistics

def percentile(sorted_vals, q):
    """Linear-interpolated percentile (q in [0, 100]) of a SORTED list."""
    if not sorted_vals:
        raise ValueError("percentile of empty list")
    if not 0.0 <= q <= 100.0:
        raise ValueError("q must be within [0, 100]")
    k = (len(sorted_vals) - 1) * q / 100.0
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return sorted_vals[int(k)]
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def distance_summary(dists, tol):
    """Distribution summary used by every symmetry measurement."""
    if not dists:
        return {"count": 0}
    s = sorted(dists)
    over = sum(1 for d in s if d > tol)
    return {
        "count": len(s),
        "min": s[0],
        "median": percentile(s, 50),
        "p90": percentile(s, 90),
        "p95": percentile(s, 95),
        "p99": percentile(s, 99),
        "max": s[-1],
        "mean": sum(s) / len(s),
        "tolerance": tol,
        "count_over_tolerance": over,
        "fraction_over_tolerance": over / len(s),
    }


# ------------------------------------------------------------- spatial index

class KDTree:
    """Exact nearest-neighbour k-d tree (pure Python, 3-D).

    Built by median splits on the widest axis; search descends to the leaf
    containing the query and backtracks into a sibling only when the
    splitting plane is closer than the best distance found so far, so the
    result equals brute force (verified in U-GEOM-04).
    """

    LEAF = 8

    def __init__(self, points):
        self.points = list(points)
        self.root = self._build(list(range(len(self.points)))) if self.points else None

    def _build(self, idx):
        if len(idx) <= self.LEAF:
            return ("leaf", idx)
        pts = self.points
        spans = []
        for a in range(3):
            vals = [pts[i][a] for i in idx]
            spans.append(max(vals) - min(vals))
        axis = spans.index(max(spans))
        if spans[axis] == 0.0:             # all identical: cannot split
            return ("leaf", idx)
        idx.sort(key=lambda i: pts[i][axis])
        mid = len(idx) // 2
        split = pts[idx[mid]][axis]
        return ("node", axis, split, self._build(idx[:mid]), self._build(idx[mid:]))

    def nearest(self, q, exclude=None):
        """(distance, index) of the nearest point; (inf, -1) when empty.

        Each stack entry carries a lower bound on the distance from q to any
        point in that subtree; subtrees whose bound cannot beat the best
        distance are skipped. Left holds coords <= split, right >= split, so
        |q[axis] - split| is a valid bound for the far side."""
        if self.root is None:
            return math.inf, -1
        best_d, best_i = math.inf, -1
        pts = self.points
        stack = [(self.root, 0.0)]
        while stack:
            node, bound = stack.pop()
            if bound >= best_d:
                continue
            if node[0] == "leaf":
                for i in node[1]:
                    if i == exclude:
                        continue
                    d = dist(q, pts[i])
                    if d < best_d:
                        best_d, best_i = d, i
                continue
            _, axis, split, left, right = node
            diff = q[axis] - split
            near, far = (left, right) if diff < 0 else (right, left)
            stack.append((far, max(bound, abs(diff))))
            stack.append((near, bound))
        return best_d, best_i


class GridIndex:
    """Uniform grid for fixed-radius queries with radius <= cell."""

    def __init__(self, points, cell):
        if cell <= 0:
            raise ValueError("cell size must be positive")
        self.cell = cell
        self.points = list(points)
        self.grid = {}
        for i, p in enumerate(self.points):
            self.grid.setdefault(self._key(p), []).append(i)

    def _key(self, p):
        c = self.cell
        return (math.floor(p[0] / c), math.floor(p[1] / c), math.floor(p[2] / c))

    def within(self, q, radius):
        if radius > self.cell:
            raise ValueError("radius must not exceed the cell size")
        kx, ky, kz = self._key(q)
        out = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for i in self.grid.get((kx + dx, ky + dy, kz + dz), ()):
                        if dist(q, self.points[i]) <= radius:
                            out.append(i)
        return out


def nearest_distances(queries, targets, cell=None):
    """For each query point, exact distance to its nearest target point.
    `cell` is accepted for backward compatibility and ignored."""
    tree = KDTree(targets)
    return [tree.nearest(q)[0] for q in queries]


def mirror_compare(a_pts, b_pts, plane_x, tol, cell=None):
    """Geometric (nearest-neighbour) mirror comparison of two point sets.

    forward:  mirror(a) -> nearest b   (is every a mirrored onto some b?)
    backward: mirror(b) -> nearest a   (is every b mirrored onto some a?)
    This is NOT a vertex correspondence: different vertex layouts on
    identical surfaces still give non-zero distances bounded by the
    vertex spacing.
    """
    fwd = nearest_distances([mirror_x(p, plane_x) for p in a_pts], b_pts, cell)
    bwd = nearest_distances([mirror_x(p, plane_x) for p in b_pts], a_pts, cell)
    return {"forward": distance_summary(fwd, tol),
            "backward": distance_summary(bwd, tol),
            "method": "bidirectional nearest-neighbour, exact grid search",
            "plane_x": plane_x}


def self_mirror_distances(pts, plane_x, cell=None):
    """Per-point distance from mirror(p) to the nearest point of the same set.
    Mirroring is an isometry, so one direction fully describes self-symmetry."""
    return nearest_distances([mirror_x(p, plane_x) for p in pts], pts)


def coincident_pairs(points, tol):
    """Unordered pairs (i < j) of points within tol of each other."""
    if tol <= 0:
        raise ValueError("tolerance must be positive")
    idx = GridIndex(points, tol)
    pairs = []
    for i, p in enumerate(idx.points):
        for j in idx.within(p, tol):
            if j > i:
                pairs.append((i, j))
    return pairs


# ------------------------------------------------------------- topology

def polygon_area(pts):
    """Area of a planar or near-planar polygon via the Newell normal."""
    nx = ny = nz = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1, z1 = pts[i]
        x2, y2, z2 = pts[(i + 1) % n]
        nx += (y1 - y2) * (z1 + z2)
        ny += (z1 - z2) * (x1 + x2)
        nz += (x1 - x2) * (y1 + y2)
    return 0.5 * math.sqrt(nx * nx + ny * ny + nz * nz)


def topology(n_verts, edges, faces, coords=None, area_tol=0.0, edge_tol=0.0):
    """Topology counts from raw connectivity.

    edges: iterable of (a, b) vertex-index pairs (the mesh's edge list)
    faces: iterable of vertex-index tuples
    Edge face-incidence is computed from the faces, so edges absent from the
    edge list but implied by faces are still counted.
    """
    faces = [tuple(f) for f in faces]
    incidence = {}
    for f in faces:
        k = len(f)
        for i in range(k):
            a, b = f[i], f[(i + 1) % k]
            key = (a, b) if a < b else (b, a)
            incidence[key] = incidence.get(key, 0) + 1
    all_edges = set(incidence)
    for a, b in edges:
        all_edges.add((a, b) if a < b else (b, a))

    boundary = sum(1 for e in all_edges if incidence.get(e, 0) == 1)
    nonmanifold = sum(1 for e in all_edges if incidence.get(e, 0) > 2)
    wire = sum(1 for e in all_edges if incidence.get(e, 0) == 0)

    used = set()
    for a, b in all_edges:
        used.add(a)
        used.add(b)
    loose_verts = n_verts - len(used)

    # connected components over vertices (edges + faces), isolated verts count
    parent = list(range(n_verts))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for a, b in all_edges:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    components = len({find(v) for v in range(n_verts)}) if n_verts else 0

    degenerate_faces = None
    zero_length_edges = None
    if coords is not None:
        degenerate_faces = sum(
            1 for f in faces if polygon_area([coords[i] for i in f]) <= area_tol)
        zero_length_edges = sum(
            1 for a, b in all_edges if dist(coords[a], coords[b]) <= edge_tol)

    return {
        "vertices": n_verts,
        "edges": len(all_edges),
        "faces": len(faces),
        "boundary_edges": boundary,
        "nonmanifold_edges": nonmanifold,
        "wire_edges": wire,
        "loose_vertices": loose_verts,
        "connected_components": components,
        "degenerate_faces": degenerate_faces,
        "zero_length_edges": zero_length_edges,
    }


# ------------------------------------------------------------- frames

def right_axis(forward, up):
    """Anatomical right for a figure looking along `forward` with head `up`:
    right = forward x up (right-handed). For forward=-Y, up=+Z this is -X."""
    return normalize(cross(forward, up))


# ------------------------------------------------------------- orientation

def winding_inconsistencies(faces):
    """Count manifold (exactly two-face) edges whose two faces traverse the
    edge in the SAME direction. Consistently wound neighbours traverse a
    shared edge in opposite directions. Edges with 1 or >2 faces are not
    judged here (they are counted by topology())."""
    directed = {}
    for f in faces:
        k = len(f)
        for i in range(k):
            e = (f[i], f[(i + 1) % k])
            directed[e] = directed.get(e, 0) + 1
    bad, seen = 0, set()
    for (a, b) in directed:
        key = (a, b) if a < b else (b, a)
        if key in seen:
            continue
        seen.add(key)
        fwd, rev = directed.get((a, b), 0), directed.get((b, a), 0)
        if fwd + rev == 2 and (fwd == 2 or rev == 2):
            bad += 1
    return bad


def signed_volume(coords, faces):
    """Signed volume by the divergence theorem (fan triangulation). Positive
    when faces wind counter-clockwise seen from outside (outward normals).
    Only meaningful for a closed, consistently wound shell."""
    v = 0.0
    for f in faces:
        a = coords[f[0]]
        for i in range(1, len(f) - 1):
            v += dot(a, cross(coords[f[i]], coords[f[i + 1]])) / 6.0
    return v
