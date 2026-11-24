"""Pure-Python fixtures for triangle/mesh intersection and proximity.

Every expected value is derived analytically from the fixture geometry.
Tolerances are passed EXPLICITLY (never read from config) so a config change
cannot silently weaken a test; U_INT_CFG pins the config values to the ones
these tests assume, so changing a threshold forces a conscious test edit.
Units: metres.
"""

import json
import math
import os
import random
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))  # scripts/

from humanoid_qa.core import geom, intersect as ix  # noqa: E402

TOL = ix.Tol(plane=1e-6, contact=1e-6, area=1e-12)
WELD = 1e-6
BUDGET = 2_000_000
EPS = 1e-9

T0 = ((0, 0, 0), (1, 0, 0), (0, 1, 0))


def shift(t, d):
    return tuple((p[0] + d[0], p[1] + d[1], p[2] + d[2]) for p in t)


def cube(origin=(0, 0, 0), size=1.0):
    ox, oy, oz = origin
    v = [(ox + size * x, oy + size * y, oz + size * z) for x in (0, 1) for y in (0, 1) for z in (0, 1)]
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, c, d in quads:
        tris += [(a, b, c), (a, c, d)]
    return v, tris


def mesh(name, coords, tris):
    return ix.TriMesh(name, coords, tris, list(range(len(tris))), WELD, TOL)


class Primitives(unittest.TestCase):
    def test_U_INT_01(self):
        """Two separated triangles: T0 vs T0 lifted 0.5 -> disjoint (plane-separated, distance not needed); exact distance via triangle_distance = 0.5."""
        r = ix.classify_triangle_pair(T0, shift(T0, (0, 0, 0.5)), TOL)
        self.assertEqual(r["kind"], "disjoint")
        d, p, q = ix.triangle_distance(T0, shift(T0, (0, 0, 0.5)))
        self.assertAlmostEqual(d, 0.5, delta=EPS)
        self.assertAlmostEqual(abs(p[2] - q[2]), 0.5, delta=EPS)
        far = ix.classify_triangle_pair(T0, shift(T0, (10, 10, 10)), TOL)
        self.assertEqual(far["kind"], "disjoint")

    def test_U_INT_02(self):
        """Intersecting triangles: a triangle in plane y=0.2 spanning z[-0.5,0.5] crosses T0 along y=0.2, x in [0.2,0.6]; kind=intersect, location on that segment (z=0, y=0.2)."""
        t2 = ((0.2, 0.2, -0.5), (0.2, 0.2, 0.5), (0.6, 0.2, 0.0))
        for a, b in ((T0, t2), (t2, T0)):                       # symmetric in its arguments
            r = ix.classify_triangle_pair(a, b, TOL)
            self.assertEqual(r["kind"], "intersect")
            x, y, z = r["location"]
            self.assertAlmostEqual(z, 0.0, delta=EPS)
            self.assertAlmostEqual(y, 0.2, delta=EPS)
            self.assertTrue(0.2 - EPS <= x <= 0.6 + EPS)

    def test_U_INT_03(self):
        """Coplanar overlap: T0 and T0 shifted (0.25,0.25,0) overlap in the triangle x,y>=0.25, x+y<=1 -> area 0.125 exactly; shifted (2,0,0) -> disjoint."""
        r = ix.classify_triangle_pair(T0, shift(T0, (0.25, 0.25, 0)), TOL)
        self.assertEqual(r["kind"], "coplanar_overlap")
        self.assertAlmostEqual(r["overlap_area"], 0.125, delta=1e-12)
        self.assertEqual(ix.classify_triangle_pair(T0, shift(T0, (2, 0, 0)), TOL)["kind"], "disjoint")
        # overlap area is invariant to the plane's orientation: same configuration rotated into the XZ plane
        rot = lambda t: tuple((p[0], 0.0, p[1]) for p in t)       # noqa: E731  (x,y,0)->(x,0,y)
        r2 = ix.classify_triangle_pair(rot(T0), rot(shift(T0, (0.25, 0.25, 0))), TOL)
        self.assertAlmostEqual(r2["overlap_area"], 0.125, delta=1e-12)

    def test_U_INT_04(self):
        """Coplanar edge contact: T0 and its reflection across the hypotenuse share only the edge (1,0,0)-(0,1,0) -> area 0, distance 0 -> 'contact', not an overlap."""
        refl = ((1, 0, 0), (0, 1, 0), (1, 1, 0))
        r = ix.classify_triangle_pair(T0, refl, TOL)
        self.assertEqual(r["kind"], "contact")
        self.assertLessEqual(r["distance"], 1e-12)

    def test_U_INT_05(self):
        """Vertex contact (non-coplanar): triangle whose apex touches T0's interior point (0.25,0.25,0) from above -> 'contact' (no proper crossing); lifted 2e-6 m -> disjoint at distance 2e-6."""
        apex = ((0.25, 0.25, 0.0), (0.5, 0.25, 1.0), (0.25, 0.5, 1.0))
        self.assertEqual(ix.classify_triangle_pair(T0, apex, TOL)["kind"], "contact")
        lifted = shift(apex, (0, 0, 2e-6))
        r = ix.classify_triangle_pair(T0, lifted, TOL)
        self.assertEqual(r["kind"], "disjoint")
        self.assertAlmostEqual(ix.triangle_distance(T0, lifted)[0], 2e-6, delta=1e-12)

    def test_U_INT_06(self):
        """Tolerance edges are explicit: offset 0.9e-6 (inside plane tol 1e-6) coplanar -> coplanar_overlap; offset 1.1e-6 (outside) -> disjoint. Changing TOL.plane/contact changes this result, so the test cannot pass silently under a weaker filter."""
        a = ix.classify_triangle_pair(T0, shift(T0, (0, 0, 0.9e-6)), TOL)
        b = ix.classify_triangle_pair(T0, shift(T0, (0, 0, 1.1e-6)), TOL)
        self.assertEqual((a["kind"], b["kind"]), ("coplanar_overlap", "disjoint"))
        wide = ix.Tol(plane=1e-5, contact=1e-5, area=1e-12)
        self.assertEqual(ix.classify_triangle_pair(T0, shift(T0, (0, 0, 1.1e-6)), wide)["kind"], "coplanar_overlap")

    def test_U_INT_07(self):
        """Degenerate geometry: a collinear 'triangle' is kind=degenerate (never intersect) even when it passes through T0's interior; closest-point and segment distances survive zero-length inputs."""
        line = ((0.2, 0.2, -1.0), (0.2, 0.2, 1.0), (0.2, 0.2, 0.0))
        self.assertEqual(ix.classify_triangle_pair(T0, line, TOL)["kind"], "degenerate")
        pt = ((0.2, 0.2, 0.3),) * 3
        self.assertEqual(ix.classify_triangle_pair(T0, pt, TOL)["kind"], "degenerate")
        self.assertAlmostEqual(ix.triangle_distance(T0, pt)[0], 0.3, delta=EPS)
        d, c1, c2 = ix.closest_points_segments((0, 0, 0), (0, 0, 0), (1, 0, 0), (1, 0, 0))
        self.assertAlmostEqual(d, 1.0, delta=EPS)

    def test_U_INT_08(self):
        """Distance primitives: point-triangle in face/edge/vertex regions (0.5,0.25,2)->2; (2,-1,0)->vertex (1,0,0) at sqrt2; segment pairs: crossing 0, parallel 1, skew 1."""
        self.assertAlmostEqual(geom.dist((0.25, 0.25, 2), ix.closest_point_on_triangle((0.25, 0.25, 2), *T0)), 2.0, delta=EPS)
        q = ix.closest_point_on_triangle((2, -1, 0), *T0)
        self.assertEqual(tuple(round(c, 12) for c in q), (1.0, 0.0, 0.0))
        e = ix.closest_point_on_triangle((0.5, -1, 0), *T0)       # nearest to edge y=0
        self.assertEqual(tuple(round(c, 12) for c in e), (0.5, 0.0, 0.0))
        self.assertAlmostEqual(ix.closest_points_segments((-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0))[0], 0.0, delta=EPS)
        self.assertAlmostEqual(ix.closest_points_segments((0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0))[0], 1.0, delta=EPS)
        self.assertAlmostEqual(ix.closest_points_segments((0, 0, 0), (1, 0, 0), (0.5, -0.5, 1), (0.5, 0.5, 1))[0], 1.0, delta=EPS)

    def test_U_INT_09(self):
        """Edge-edge triangle distance: triangle whose bottom edge floats 1.0 above T0's edge y=0 -> distance 1.0; diagonal cube-edge style offset sqrt(0.5^2+0.5^2)."""
        t2 = ((0.5, -0.5, 1.0), (0.5, 0.5, 1.0), (0.5, 0.0, 2.0))
        self.assertAlmostEqual(ix.triangle_distance(T0, t2)[0], 1.0, delta=EPS)
        a = ((0, 0, 0), (1, 0, 0), (0, 0, 1))          # in XZ plane, edge along x at y=0
        b = ((2.0, 0.5, -1), (2.0, 0.5, 1), (3.0, 0.5, 0))
        self.assertAlmostEqual(ix.triangle_distance(a, b)[0], math.hypot(1.0, 0.5), delta=EPS)


def sat_overlap(t1, t2):
    """INDEPENDENT oracle: separating-axis test (11 axes). For generic
    (non-coplanar, non-touching) triangles True <=> the triangles intersect."""
    def ax_edges(t):
        return [geom.sub(t[1], t[0]), geom.sub(t[2], t[1]), geom.sub(t[0], t[2])]
    e1, e2 = ax_edges(t1), ax_edges(t2)
    axes = [geom.cross(e1[0], e1[1]), geom.cross(e2[0], e2[1])] + [geom.cross(a, b) for a in e1 for b in e2]
    for ax in axes:
        if geom.length(ax) < 1e-12:
            continue
        p1 = [geom.dot(ax, p) for p in t1]
        p2 = [geom.dot(ax, p) for p in t2]
        if max(p1) < min(p2) or max(p2) < min(p1):
            return False
    return True


def rand_tri(rnd, size, centre_box=1.0):
    c = [rnd.uniform(0, centre_box) for _ in range(3)]
    return tuple(tuple(c[k] + rnd.uniform(-size, size) for k in range(3)) for _ in range(3))


class DegenerateRules(unittest.TestCase):
    def test_U_INT_23(self):
        """Degenerate definition is consistent at triangle and mesh level: an edge of 5e-7 m (<= contact tol) makes a sliver degenerate even though its area (2.5e-7 m^2) exceeds the area tolerance; the same triangle with a 5e-6 m edge is NOT degenerate; a collapsed sliver crossing a cube is excluded (listed) and produces no intersection."""
        sliver = ((0.2, 0.2, -0.5), (0.2, 0.2 + 5e-7, -0.5), (0.2, 0.2, 0.5))
        self.assertGreater(ix.tri_area(sliver), TOL.area)
        self.assertTrue(ix.is_degenerate(sliver, TOL))
        self.assertEqual(ix.classify_triangle_pair(T0, sliver, TOL)["kind"], "degenerate")
        ok = ((0.2, 0.2, -0.5), (0.2, 0.2 + 5e-6, -0.5), (0.2, 0.2, 0.5))
        self.assertFalse(ix.is_degenerate(ok, TOL))
        self.assertEqual(ix.classify_triangle_pair(T0, ok, TOL)["kind"], "intersect")
        v, t = cube()
        v += [(0.3, 0.6, -0.5), (0.3, 0.6 + 5e-7, -0.5), (0.3, 0.6, 1.5)]
        r = ix.self_intersections(mesh("collapsed", v, t + [(8, 9, 10)]), TOL, BUDGET)
        self.assertEqual((r["definite_count"], r["degenerate_triangles_excluded"]), (0, [12]))


class Oracles(unittest.TestCase):
    def test_U_INT_18(self):
        """Classifier vs an independent separating-axis oracle on 3000 random generic triangle pairs: kind=='intersect' exactly when SAT says overlap; every other pair is 'disjoint' (no spurious contact/coplanar in generic position)."""
        rnd = random.Random(2026)
        n_int = 0
        for _ in range(3000):
            a, b = rand_tri(rnd, 0.35, 0.7), rand_tri(rnd, 0.35, 0.7)     # tighter box -> more overlaps
            if ix.tri_area(a) < 1e-4 or ix.tri_area(b) < 1e-4:
                continue
            k = ix.classify_triangle_pair(a, b, TOL)["kind"]
            sat = sat_overlap(a, b)
            self.assertEqual(k == "intersect", sat, (a, b, k, sat))
            self.assertIn(k, ("intersect", "disjoint"))
            n_int += sat
        # sample-coverage guard (not a correctness threshold): the seeded sample yields 201 intersecting pairs of ~3000
        self.assertGreater(n_int, 150)
        self.assertLess(n_int, 2700)

    def test_U_INT_19(self):
        """triangle_distance vs dense sampling: for 150 SAT-disjoint random pairs the exact distance never exceeds any sampled point-pair distance (rigorous lower bound) and is within 0.05 of the sampled minimum (not wildly under-estimated)."""
        rnd = random.Random(7)

        def samples(t, n=12):
            pts = []
            for i in range(n + 1):
                for j in range(n + 1 - i):
                    u, v = i / n, j / n
                    pts.append(tuple(t[0][k] + u * (t[1][k] - t[0][k]) + v * (t[2][k] - t[0][k]) for k in range(3)))
            return pts
        done = 0
        while done < 150:
            a, b = rand_tri(rnd, 0.25, 1.5), rand_tri(rnd, 0.25, 1.5)
            if sat_overlap(a, b):
                continue
            exact = ix.triangle_distance(a, b)[0]
            sm = min(geom.dist(p, q) for p in samples(a) for q in samples(b))
            self.assertLessEqual(exact, sm + 1e-12)
            self.assertLess(sm - exact, 0.05)
            done += 1

    def test_U_INT_20(self):
        """Depth guard: a triangle poking through T0 by 5e-7 m (<= contact tol 1e-6) is 'contact' (indistinguishable from touching); poking 2e-6 m is 'intersect' with depth ~2e-6; the exact oracle says both genuinely cross, so the guard is a tolerance decision, not a geometry error."""
        shallow = ((0.2, 0.2, -5e-7), (0.2, 0.2, 0.5), (0.6, 0.2, 0.3))
        deep = ((0.2, 0.2, -2e-6), (0.2, 0.2, 0.5), (0.6, 0.2, 0.3))
        rs, rd = ix.classify_triangle_pair(T0, shallow, TOL), ix.classify_triangle_pair(T0, deep, TOL)
        self.assertEqual(rs["kind"], "contact")
        self.assertAlmostEqual(rs["shallow_crossing_depth"], 5e-7, delta=1e-12)
        self.assertEqual(rd["kind"], "intersect")
        self.assertAlmostEqual(rd["depth"], 2e-6, delta=1e-12)
        self.assertTrue(ix.exact_proper_crossing(T0, shallow))
        self.assertTrue(ix.exact_proper_crossing(T0, deep))
        # the guard follows the tolerance: with a 1e-7 contact tolerance the shallow case becomes a real intersection
        tight = ix.Tol(plane=1e-7, contact=1e-7, area=1e-12)
        self.assertEqual(ix.classify_triangle_pair(T0, shallow, tight)["kind"], "intersect")

    def test_U_INT_21(self):
        """Slab coplanarity: a small triangle lying within 5e-7 m of a large tilted triangle's plane (but the large one is >1e-6 from the small one's plane) is coplanar_overlap with area ~0.5 - not hidden as shallow contact; a genuine perpendicular crossing is unaffected."""
        big = ((0, 0, 0), (10, 0, 0), (0, 10, 0))
        small = ((1, 1, 5e-7), (2, 1, -5e-7), (1, 2, 0))
        r = ix.classify_triangle_pair(big, small, TOL)
        self.assertEqual(r["kind"], "coplanar_overlap")
        self.assertAlmostEqual(r["overlap_area"], 0.5, delta=1e-5)
        n2 = geom.normalize(ix.tri_normal(small))
        self.assertGreater(max(abs(geom.dot(n2, geom.sub(p, small[0]))) for p in big), 1e-6)   # two-way test would have failed
        perp = ((2, 2, -1), (2, 2, 1), (3, 2, 0))
        self.assertEqual(ix.classify_triangle_pair(big, perp, TOL)["kind"], "intersect")

    def test_U_INT_22(self):
        """Exact rational oracle vs classifier on 1200 random pairs: kind=='intersect' <=> exact strict crossing (generic position, depths >> tolerance), and the oracle agrees with the fixture cases (crossing True; touching/disjoint/coplanar False)."""
        rnd = random.Random(99)
        for _ in range(1200):
            a, b = rand_tri(rnd, 0.35), rand_tri(rnd, 0.35)
            if ix.tri_area(a) < 1e-4 or ix.tri_area(b) < 1e-4:
                continue
            self.assertEqual(ix.classify_triangle_pair(a, b, TOL)["kind"] == "intersect", ix.exact_proper_crossing(a, b))
        t2 = ((0.2, 0.2, -0.5), (0.2, 0.2, 0.5), (0.6, 0.2, 0.0))
        self.assertTrue(ix.exact_proper_crossing(T0, t2))
        self.assertFalse(ix.exact_proper_crossing(T0, shift(T0, (0, 0, 0.5))))
        self.assertFalse(ix.exact_proper_crossing(T0, shift(T0, (0.25, 0.25, 0))))      # coplanar: no proper crossing
        self.assertFalse(ix.exact_proper_crossing(T0, ((0.25, 0.25, 0.0), (0.5, 0.25, 1.0), (0.25, 0.5, 1.0))))  # apex touch


class MeshLevel(unittest.TestCase):
    def test_U_INT_10(self):
        """Adjacency is not intersection: a closed cube (12 triangles sharing edges/vertices) has 0 definite self-intersections; adjacent pairs are counted as ordinary, not reported."""
        v, t = cube()
        r = ix.self_intersections(mesh("cube", v, t), TOL, BUDGET)
        self.assertEqual((r["definite_count"], r["contact_count"]), (0, 0))
        self.assertGreater(r["adjacent_pairs_ordinary"], 0)
        self.assertEqual(r["degenerate_triangles_excluded"], [])

    def test_U_INT_11(self):
        """Shared edge / shared vertex / fold-over: quad (0,1,2)+(1,3,2) -> nothing; triangle (0,1,3) folded back over (0,1,2) -> adjacent_fold_overlap area 0.125; vertex-sharing triangles touching only at the vertex -> nothing; with the opposite edge piercing -> intersect."""
        c = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0)]
        r = ix.self_intersections(mesh("quad", c, [(0, 1, 2), (1, 3, 2)]), TOL, BUDGET)
        self.assertEqual((r["definite_count"], r["adjacent_pairs_ordinary"]), (0, 1))
        fold = ix.self_intersections(mesh("fold", c[:3] + [(0.5, 0.25, 0)], [(0, 1, 2), (0, 1, 3)]), TOL, BUDGET)
        self.assertEqual(fold["counts"]["adjacent_fold_overlap"], 1)
        self.assertAlmostEqual(fold["definite"][0]["overlap_area"], 0.125, delta=1e-12)
        opp = ix.self_intersections(mesh("opp", c[:3] + [(0.5, -0.25, 0)], [(0, 1, 2), (0, 1, 3)]), TOL, BUDGET)
        self.assertEqual(opp["definite_count"], 0)
        vtx = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (-1, 0, 0.5), (-1, -1, 0.5)]
        self.assertEqual(ix.self_intersections(mesh("vt", vtx, [(0, 1, 2), (0, 3, 4)]), TOL, BUDGET)["definite_count"], 0)
        pierce = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0.3, 0.3, -0.5), (0.3, 0.3, 0.5)]
        r2 = ix.self_intersections(mesh("vp", pierce, [(0, 1, 2), (0, 3, 4)]), TOL, BUDGET)
        self.assertEqual((r2["counts"]["intersect"], r2["definite_count"]), (1, 1))

    def test_U_INT_12(self):
        """Self-intersecting mesh: cube plus a spike triangle through its bottom and top faces -> exactly 2 definite pairs, both involving triangle 12, at z=0 and z=1; welded duplicate vertices at a seam are NOT reported."""
        v, t = cube()
        v += [(0.3, 0.6, -0.5), (0.3, 0.6, 1.5), (0.7, 0.6, 0.5)]
        t = t + [(8, 9, 10)]
        r = ix.self_intersections(mesh("spiked", v, t), TOL, BUDGET)
        self.assertEqual(r["definite_count"], 2)
        self.assertTrue(all(12 in (d["a"]["tri"], d["b"]["tri"]) for d in r["definite"]))
        self.assertEqual(sorted(round(d["location"][2], 9) for d in r["definite"]), [0.0, 1.0])
        # seam: duplicate every vertex of a quad pair (split vertices at identical positions)
        c = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]
        seam = ix.self_intersections(mesh("seam", c, [(0, 1, 2), (3, 4, 5)]), TOL, BUDGET)
        self.assertEqual((seam["definite_count"], seam["contact_count"]), (0, 0))

    def test_U_INT_13(self):
        """Degenerate triangles are excluded and reported, not analysed: cube + collinear triangle through it -> still 0 definite, degenerate list == [12]."""
        v, t = cube()
        v += [(0.3, 0.6, -0.5), (0.3, 0.6, 0.5), (0.3, 0.6, 1.5)]
        r = ix.self_intersections(mesh("deg", v, t + [(8, 9, 10)]), TOL, BUDGET)
        self.assertEqual((r["definite_count"], r["degenerate_triangles_excluded"]), (0, [12]))

    def test_U_INT_14(self):
        """Two meshes: cubes 0.5 apart -> NONE_DETECTED and exact distance 0.5; offset (0.5,0.4,0.3) -> DEFINITE with proper crossings (a symmetric (0.5,0.5,0.5) offset puts the crossing exactly on a face diagonal = boundary contact); offset (0.5,0,0) -> DEFINITE via coplanar overlap; face-sharing (touching) -> coplanar_overlap, pairwise overlap areas sum to the shared face area 1.0, reported definite; corner-only touch -> POSSIBLE (contact)."""
        va, ta = cube()
        a = mesh("A", va, ta)

        def run(origin):
            vb, tb = cube(origin)
            b = mesh("B", vb, tb)
            r = ix.cross_intersections(a, b, TOL, BUDGET)
            return r, ix.pair_status(r, len(a.active), len(b.active)), b
        r, st, b = run((1.5, 0, 0))
        self.assertEqual(st, "NONE_DETECTED")
        pm = ix.mesh_proximity(a, b, math.inf)
        self.assertAlmostEqual(pm["distance"], 0.5, delta=EPS)
        self.assertAlmostEqual(pm["point_b"][0] - pm["point_a"][0], 0.5, delta=EPS)
        r, st, _ = run((0.5, 0.4, 0.3))                    # generic offset: crossings fall strictly inside triangles
        self.assertEqual(st, "DEFINITE_INTERSECTION")
        self.assertGreater(r["counts"]["intersect"], 0)
        r, st, _ = run((0.5, 0, 0))                        # overlap along X only: faces are coplanar, no proper crossing
        self.assertEqual(st, "DEFINITE_INTERSECTION")
        self.assertGreater(r["counts"]["coplanar_overlap"], 0)
        r, st, _ = run((1.0, 0, 0))                        # shared face x=1
        self.assertEqual(st, "DEFINITE_INTERSECTION")
        self.assertEqual(r["counts"]["intersect"], 0)
        self.assertAlmostEqual(sum(d["overlap_area"] for d in r["definite"]), 1.0, delta=1e-9)   # pairwise overlaps of two partitions of one unit square sum to its area
        r, st, _ = run((1.0, 1.0, 1.0))                    # corner point contact only
        self.assertEqual(st, "POSSIBLE_REQUIRES_INSPECTION")
        self.assertEqual(r["definite_count"], 0)

    def test_U_INT_15(self):
        """Proximity analytics: cubes offset (1.5,1.5,0) -> edge-edge distance sqrt(0.5^2+0.5^2); offset (1.5,1.5,1.5) -> corner distance sqrt(3)*0.5; radius smaller than the gap -> None."""
        va, ta = cube()
        a = mesh("A", va, ta)
        for off, exp in (((1.5, 1.5, 0), math.hypot(0.5, 0.5)), ((1.5, 1.5, 1.5), math.sqrt(3) * 0.5)):
            vb, tb = cube(off)
            pm = ix.mesh_proximity(a, mesh("B", vb, tb), math.inf)
            self.assertAlmostEqual(pm["distance"], exp, delta=EPS)
            self.assertAlmostEqual(geom.dist(pm["point_a"], pm["point_b"]), exp, delta=EPS)
        vb, tb = cube((1.5, 0, 0))
        self.assertIsNone(ix.mesh_proximity(a, mesh("B", vb, tb), 0.4))
        self.assertIsNotNone(ix.mesh_proximity(a, mesh("B", vb, tb), 0.5 + 1e-9))

    def test_U_INT_16(self):
        """Acceleration is exact: on random triangle soups the BVH candidate sets equal brute-force box overlap for self and cross queries, and branch-and-bound min distance equals the brute-force minimum."""
        rnd = random.Random(411)

        def soup(n, off):
            tris, coords = [], []
            for k in range(n):
                c = (rnd.uniform(0, 1) + off, rnd.uniform(0, 1), rnd.uniform(0, 1))
                for _ in range(3):
                    coords.append((c[0] + rnd.uniform(-.1, .1), c[1] + rnd.uniform(-.1, .1), c[2] + rnd.uniform(-.1, .1)))
                tris.append((3 * k, 3 * k + 1, 3 * k + 2))
            return coords, tris
        ca, ta = soup(60, 0.0)
        cb, tb = soup(60, 1.4)
        a, b = mesh("A", ca, ta), mesh("B", cb, tb)
        m = 0.02
        brute_self = sorted((i, j) for i in a.active for j in a.active if i < j
                            and ix.box_overlap(a.boxes[i], a.boxes[j], m))
        self.assertEqual(a.bvh.self_pairs(m, BUDGET), brute_self)
        brute_cross = sorted((i, j) for i in a.active for j in b.active if ix.box_overlap(a.boxes[i], b.boxes[j], m))
        self.assertEqual(a.bvh.cross_pairs(b.bvh, m, BUDGET), brute_cross)
        brute_min = min(ix.triangle_distance(a.corners[i], b.corners[j])[0] for i in a.active for j in b.active)
        self.assertAlmostEqual(ix.mesh_proximity(a, b, math.inf)["distance"], brute_min, delta=1e-12)

    def test_U_INT_17(self):
        """Cannot-determine paths: candidate budget exceeded raises BudgetExceeded; a mesh with no usable triangles gives UNABLE_TO_DETERMINE; open/non-manifold topology yields reliability notes (result stays a geometric fact, interpretation flagged)."""
        v, t = cube()
        with self.assertRaises(ix.BudgetExceeded):
            ix.self_intersections(mesh("cube", v, t), TOL, 3)
        empty = mesh("E", [(0, 0, 0)], [])
        self.assertEqual(ix.pair_status(ix.cross_intersections(mesh("A", v, t), empty, TOL, BUDGET),
                                        len(mesh("A", v, t).active), 0), "UNABLE_TO_DETERMINE")
        self.assertEqual(ix.pair_status(None, 5, 5), "UNABLE_TO_DETERMINE")
        notes = ix.reliability_notes({"boundary_edges": 4, "nonmanifold_edges": 1, "wire_edges": 0})
        self.assertEqual(len(notes), 2)
        self.assertEqual(ix.reliability_notes(geom.topology(8, [], [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
                                                                   (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)])), [])

    def test_U_INT_CFG(self):
        """Config pinning: the tolerances these tests assume (plane 1e-6, contact 1e-6, area 1e-12, weld 1e-6) equal quality_rules.json; any change to those thresholds fails here and must be made consciously."""
        cfg = json.load(open(os.path.join(os.path.dirname(HERE), "config", "quality_rules.json"), encoding="utf-8"))["tolerances"]
        self.assertEqual(cfg["tri_plane_coplanar"]["value"], TOL.plane)
        self.assertEqual(cfg["tri_contact"]["value"], TOL.contact)
        self.assertEqual(cfg["degenerate_face_area"]["value"], TOL.area)
        self.assertEqual(cfg["coincident_vertex"]["value"], WELD)
        self.assertEqual(cfg["max_candidate_pairs"]["value"], BUDGET)
        self.assertEqual(cfg["proximity_search_radius"]["status"], "EXPERIMENTAL")


if __name__ == "__main__":
    unittest.main()
