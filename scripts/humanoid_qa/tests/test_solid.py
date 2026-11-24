"""Pure-Python analytic fixtures for solid overlap (core/solid.py).

Every expected value is derived by hand from the fixture geometry (stated in each
docstring). Tolerances are explicit; U_SOL_CFG pins the config values these tests
assume so a change to the config forces a conscious test edit.
Units: metres / cubic metres.
"""

import json
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))  # scripts/

from humanoid_qa.core import geom, solid  # noqa: E402

NULL = 1e-12          # null volume (m^3), pinned by U_SOL_CFG
AREA = 1e-12          # degenerate face area (m^2)
ITERS = 30            # bisection steps for the pure fixtures: T/2^30 < 2e-9 m
AGREE = 1e-4
IDENT = 1e-4
PARAMS = {"null_volume": NULL, "degenerate_area": AREA, "iters": ITERS,
          "agree_rel": AGREE, "identity_rel": IDENT,
          "axial_scan_samples": 32, "tri_contact": 1e-6, "tri_plane_coplanar": 1e-6, "coincident_vertex": 1e-6}
PARAMS_NO_CERT = {k: v for k, v in PARAMS.items() if k not in ("tri_contact", "tri_plane_coplanar", "coincident_vertex")}
EPS = 1e-9
PURE = solid.ConvexBackend


class Mesh:
    """Triangle mesh builder with coordinate welding (shared corners share indices)."""

    def __init__(self):
        self.coords, self.index, self.tris = [], {}, []

    def v(self, p):
        key = tuple(round(x, 12) for x in p)
        if key not in self.index:
            self.index[key] = len(self.coords)
            self.coords.append(tuple(float(x) for x in p))
        return self.index[key]

    def quad(self, a, b, c, d):
        ia, ib, ic, id_ = self.v(a), self.v(b), self.v(c), self.v(d)
        self.tris += [(ia, ib, ic), (ia, ic, id_)]

    def box(self, lo, hi, reverse=False):
        x0, y0, z0 = lo
        x1, y1, z1 = hi
        c = lambda x, y, z: (x, y, z)  # noqa: E731
        faces = [
            (c(x0, y0, z0), c(x0, y1, z0), c(x0, y1, z1), c(x0, y0, z1)),   # -X
            (c(x1, y0, z0), c(x1, y0, z1), c(x1, y1, z1), c(x1, y1, z0)),   # +X
            (c(x0, y0, z0), c(x0, y0, z1), c(x1, y0, z1), c(x1, y0, z0)),   # -Y
            (c(x0, y1, z0), c(x1, y1, z0), c(x1, y1, z1), c(x0, y1, z1)),   # +Y
            (c(x0, y0, z0), c(x1, y0, z0), c(x1, y1, z0), c(x0, y1, z0)),   # -Z
            (c(x0, y0, z1), c(x0, y1, z1), c(x1, y1, z1), c(x1, y0, z1)),   # +Z
        ]
        for q in faces:
            if not reverse:                        # quads above wind inward; reverse to outward
                q = tuple(reversed(q))
            self.quad(*q)
        return self

    def done(self):
        return self.coords, self.tris


def cube(lo=(0, 0, 0), hi=(1, 1, 1), reverse=False):
    return Mesh().box(lo, hi, reverse).done()


def shift_pts(coords, s):
    return [(p[0] + s[0], p[1] + s[1], p[2] + s[2]) for p in coords]


def rot_z_about(coords, deg, centre):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    out = []
    for x, y, z in coords:
        dx, dy = x - centre[0], y - centre[1]
        out.append((centre[0] + c * dx - s * dy, centre[1] + s * dx + c * dy, z))
    return out


def L_prism(h=1.0):
    """L-shaped prism (non-convex). Footprint (0,0)-(2,0)-(2,1)-(1,1)-(1,2)-(0,2),
    area 3, so volume = 3 * h."""
    P = [(0, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2)]
    m = Mesh()
    bottom = [m.v((x, y, 0.0)) for x, y in P]
    top = [m.v((x, y, h)) for x, y in P]
    for a, b, c in ((0, 1, 2), (0, 2, 3), (0, 3, 5), (3, 4, 5)):
        m.tris.append((bottom[a], bottom[c], bottom[b]))   # bottom faces down
        m.tris.append((top[a], top[b], top[c]))             # top faces up
    for i in range(6):
        j = (i + 1) % 6
        m.tris.append((bottom[i], bottom[j], top[j]))
        m.tris.append((bottom[i], top[j], top[i]))
    return m.done()


def plain_cube_box(lo, hi):
    from humanoid_qa.tests.test_refine import plain_cube
    return plain_cube(lo, hi)


def pyramid():
    """Square pyramid, apex (0,0,-0.5), base corners (+-1,+-1,0.5), outward winding."""
    from humanoid_qa.tests.test_refine import _orient
    coords = [(0.0, 0.0, -0.5), (-1.0, -1.0, 0.5), (1.0, -1.0, 0.5), (1.0, 1.0, 0.5), (-1.0, 1.0, 0.5)]
    polys = [(1, 2, 3, 4), (0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 1)]
    return coords, _orient(coords, polys, centre=(0.0, 0.0, 0.0))


def vol(coords, tris):
    return geom.signed_volume(coords, [tuple(t) for t in tris])


def run(ca, ta, cb, tb):
    return solid.measure_pair(ca, ta, cb, tb, PURE, PARAMS)


class Fixtures(unittest.TestCase):
    def test_U_SOL_01(self):
        """Two axis-aligned cubes, A=[0,1]^3 and B=A+(0.5,0.25,0.125). Overlap is the box [0.5,1]x[0.25,1]x[0.125,1]: volume 0.5*0.75*0.875 = 0.328125 m^3. The six directional separations are the distances from each A face to the opposite B face: +X 1.5, -X 0.5, +Y 1.25, -Y 0.75, +Z 1.125, -Z 0.875; the minimum (-X, 0.5 m) equals the box minimum overlap extent, which is the exact MTD for boxes."""
        ca, ta = cube()
        cb, tb = cube((0.5, 0.25, 0.125), (1.5, 1.25, 1.125))
        r = run(ca, ta, cb, tb)
        self.assertEqual(r["status"], "VOLUME_MEASURED", r)
        self.assertAlmostEqual(r["overlap_volume_m3"], 0.328125, delta=1e-12)
        expect = {"+X": 1.5, "-X": 0.5, "+Y": 1.25, "-Y": 0.75, "+Z": 1.125, "-Z": 0.875}
        for k, v in expect.items():
            self.assertAlmostEqual(r["axial"][k]["t"], v, delta=EPS * 10 + 2e-9, msg=k)
        self.assertEqual(r["separation_direction"], "-X")
        self.assertAlmostEqual(r["separation_bound_m"], 0.5, delta=2e-9)

    def test_U_SOL_02(self):
        """Zero overlap, three cases that must NOT be confused with inconclusive results: separated (B shifted +1.5 in X: gap 0.5), face contact (+1 in X), edge contact (+1,+1 in X,Y), vertex contact (+1,+1,+1). Each returns NULL_OVERLAP with zero volume, and the gate passes (status is a verdict, not a refusal)."""
        ca, ta = cube()
        for s in ((1.5, 0, 0), (1, 0, 0), (1, 1, 0), (1, 1, 1)):
            cb, tb = cube(s, (s[0] + 1, s[1] + 1, s[2] + 1))
            r = run(ca, ta, cb, tb)
            self.assertEqual(r["status"], "NULL_OVERLAP", f"shift {s}: {r['status']} {r['reasons']}")
            self.assertEqual(r["volumes"]["AB"]["PURE_CONVEX"], 0.0, f"shift {s}")
            self.assertIsNone(r["separation_bound_m"])

    def test_U_SOL_03(self):
        """Contained solid: B=[0.25,0.75]^3 inside A=[0,1]^3. Overlap is B itself: volume 0.5^3 = 0.125 m^3. Moving A along any axis, B stays inside until A's face passes B's face; the minimum separation is 0.75 m (A.max - B.min on -X, equal to B.max - A.min on +X), which is the minimum translation distance for a containment."""
        ca, ta = cube()
        cb, tb = cube((0.25, 0.25, 0.25), (0.75, 0.75, 0.75))
        r = run(ca, ta, cb, tb)
        self.assertEqual(r["status"], "VOLUME_MEASURED", r)
        self.assertAlmostEqual(r["overlap_volume_m3"], 0.125, delta=1e-12)
        self.assertAlmostEqual(r["separation_bound_m"], 0.75, delta=2e-9)
        for k in ("+X", "-X", "+Y", "-Y", "+Z", "-Z"):
            self.assertAlmostEqual(r["axial"][k]["t"], 0.75, delta=2e-9, msg=k)

    def test_U_SOL_04(self):
        """Rotated overlapping boxes: A=[0,1]^3 and B = the same unit cube rotated 45 deg about the vertical axis through (0.5,0.5), same z range. The horizontal cross-section is the intersection of a unit square and a 45-deg-rotated unit square with the same centre, a regular octagon with area 2*sqrt(2)-2 (four corner triangles of leg 1-1/sqrt(2) removed from a unit square). Height 1, so volume 2*sqrt(2)-2 = 0.8284271247 m^3."""
        ca, ta = cube()
        cb, tb = cube()
        cb = rot_z_about(cb, 45.0, (0.5, 0.5))
        cb = [(x, y, z) for x, y, z in cb]
        r = run(ca, ta, cb, tb)
        self.assertEqual(r["status"], "VOLUME_MEASURED", r)
        self.assertAlmostEqual(r["overlap_volume_m3"], 2 * math.sqrt(2) - 2, delta=1e-9)
        self.assertGreater(r["separation_bound_m"], 0.0)

    def test_U_SOL_05(self):
        """Reversed face orientation: A = cube with every face wound inward (signed volume -1). A raw convex backend refuses it (NonConvexError, planes face the wrong way). After orient_components the copy is reversed (reported) and the overlap with B=A+(0.5,0.5,0.5) is the 0.5^3 = 0.125 m^3 box. The source triangles are not modified."""
        ca, ta_rev = cube(reverse=True)
        self.assertLess(vol(ca, ta_rev), 0.0)
        cb, tb = cube((0.5, 0.5, 0.5), (1.5, 1.5, 1.5))
        with self.assertRaises(solid.NonConvexError):
            PURE(ca, ta_rev, cb, tb)
        before = list(ta_rev)
        r = run(ca, ta_rev, cb, tb)
        self.assertEqual(r["status"], "VOLUME_MEASURED", r)
        self.assertAlmostEqual(r["overlap_volume_m3"], 0.125, delta=1e-12)
        self.assertEqual(r["orientation"]["A"]["reversed_components"], [0])
        self.assertEqual(list(ta_rev), before)

    def test_U_SOL_06(self):
        """Non-convex geometry: an L-shaped prism (volume 3 m^3) overlapped with a cube. The pure convex backend must REFUSE (UNSUPPORTED_BY_BACKEND, no value). The general backend (Blender, B-VOL tests) measures it. Refusing is the correct behaviour here: clipping a non-convex operand by facet planes would be wrong."""
        cl, tl = L_prism()
        self.assertAlmostEqual(vol(cl, tl), 3.0, delta=1e-9)
        cb, tb = cube((0.5, 0.5, 0.25), (1.5, 1.5, 1.25))
        r = run(cl, tl, cb, tb)
        self.assertEqual(r["status"], "UNSUPPORTED_BY_BACKEND", r)
        self.assertNotIn("overlap_volume_m3", r)
        self.assertEqual(r["volumes"], {})

    def test_U_SOL_07(self):
        """Refusals, not values: open cube (one face missing) -> OPEN_BOUNDARY; two cubes welded along an edge -> NON_MANIFOLD; cube plus a collinear triangle -> DEGENERATE_FACE; one face reversed inside a closed cube -> WINDING_INCONSISTENT. All four give UNABLE_TO_DETERMINE, which is distinct from NULL_OVERLAP (a verdict)."""
        cb, tb = cube((0.5, 0.5, 0.5), (1.5, 1.5, 1.5))
        ca, ta = cube()
        open_ca, open_ta = ca, ta[2:]
        cases = {"OPEN_BOUNDARY": (open_ca, open_ta)}
        m = Mesh().box((0, 0, 0), (1, 1, 1)).box((1, 1, 0), (2, 2, 1))
        cases["NON_MANIFOLD"] = m.done()
        c2, t2 = cube()
        extra = [len(c2), len(c2) + 1, len(c2) + 2]
        c2 = list(c2) + [(0.2, 0.2, 2.0), (0.4, 0.4, 2.0), (0.6, 0.6, 2.0)]
        cases["DEGENERATE_FACE"] = (c2, list(t2) + [tuple(extra)])
        wrong = list(ta)
        a, b, c = wrong[0]
        wrong[0] = (a, c, b)
        cases["WINDING_INCONSISTENT"] = (ca, wrong)
        for name, (c_, t_) in cases.items():
            r = run(c_, t_, cb, tb)
            self.assertEqual(r["status"], "UNABLE_TO_DETERMINE", f"{name}: {r['status']}")
            self.assertIn(name, r["reasons"]["A"], f"{name}: {r['reasons']}")
            self.assertNotEqual(r["status"], "NULL_OVERLAP")

    def test_U_SOL_08(self):
        """Orientation per component: two disjoint unit cubes in one shell, the second reversed (volumes +1 and -1). orient_components reverses exactly component 1 and the oriented shell has volume +2 m^3 (sum of two unit cubes). Zero-volume components are reported, not reversed (none here)."""
        c1, t1 = cube()
        c2, t2 = cube((3, 0, 0), (4, 1, 1), reverse=True)
        coords = list(c1) + list(c2)
        tris = list(t1) + [(a + len(c1), b + len(c1), c + len(c1)) for a, b, c in t2]
        self.assertEqual(len(solid.components(tris, len(coords))), 2)
        self.assertLess(vol(coords, [tris[i] for i in range(12, 24)]), 0.0)
        oriented, info = solid.orient_components(coords, tris, NULL)
        self.assertEqual(info["components"], 2)
        self.assertEqual(sorted(info["reversed_components"]), [1])
        self.assertEqual(info["zero_volume_components"], [])
        self.assertAlmostEqual(vol(coords, oriented), 2.0, delta=1e-12)

    def test_U_SOL_09(self):
        """Bisection contract on a synthetic step: overlap is 1 for t < 0.3 and 0 otherwise. The bisection returns 0.3 within 1e-9 and the returned value is verified (its volume is null). Zero overlap at t=0 reports NO_OVERLAP_AT_ZERO; a bracket that does not separate reports BRACKET_NOT_SEPARATING. Neither is passed off as a depth."""
        f = lambda t: 1.0 if t < 0.3 else 0.0  # noqa: E731
        r = solid.bisect_separation(f, (1, 0, 0), 1.0, 40, NULL)
        self.assertEqual(r["status"], "VERIFIED")
        self.assertAlmostEqual(r["t"], 0.3, delta=1e-9)
        self.assertLessEqual(f(r["t"]), NULL)
        self.assertEqual(solid.bisect_separation(lambda t: 0.0, (1, 0, 0), 1.0, 10, NULL)["status"],
                         "NO_OVERLAP_AT_ZERO")
        self.assertEqual(solid.bisect_separation(lambda t: 1.0, (1, 0, 0), 1.0, 10, NULL)["status"],
                         "BRACKET_NOT_SEPARATING")

    def test_U_SOL_10(self):
        """Shell orientation and gating preserve analytic volumes: a cube at (0,0,0) overlapped with an identical cube at offset 0.25 along X: overlap 0.75 m^3; the pure value is the same whether the operands are given in either argument order (symmetry relative difference 0)."""
        ca, ta = cube()
        cb, tb = cube((0.25, 0, 0), (1.25, 1, 1))
        r1 = run(ca, ta, cb, tb)
        r2 = run(cb, tb, ca, ta)
        self.assertAlmostEqual(r1["overlap_volume_m3"], 0.75, delta=1e-12)
        self.assertAlmostEqual(r2["overlap_volume_m3"], 0.75, delta=1e-12)
        self.assertEqual(r1["symmetry_rel_diff"], 0.0)

    def test_U_SOL_11(self):
        """Non-monotone overlap (disconnected target). Synthetic volume along the ray, T=1: overlap for t<0.2, a GAP (no overlap) for 0.2<=t<0.5, overlap again for 0.5<=t<0.7, none for t>=0.7. The legacy single bisection from [0,1] lands on the LAST transition, 0.7: a verified separating translation, but not the smallest. The scanned search (20 samples, spacing 0.05) finds the first sampled null translation, 0.2, within 1e-9, and flags that overlap reappears after it at samples 0.5..0.65. Both returned values are verified (null volume). Neither is called an exact minimum: minimal_along_axis is NOT_ESTABLISHED."""
        def f(t):
            return 1.0 if (t < 0.2 or 0.5 <= t < 0.7) else 0.0
        legacy = solid.bisect_separation(f, (1, 0, 0), 1.0, 40, NULL)
        self.assertAlmostEqual(legacy["t"], 0.7, delta=1e-9)
        r = solid.search_separation(f, 1.0, 40, NULL, 20)
        self.assertEqual(r["status"], "VERIFIED")
        self.assertAlmostEqual(r["t"], 0.2, delta=1e-9)
        self.assertLessEqual(f(r["t"]), NULL)
        self.assertTrue(r["scan"]["overlap_reappears_after_first_null"])
        self.assertEqual([round(x, 2) for x in r["scan"]["reappearing_sample_t"]], [0.5, 0.55, 0.6, 0.65])
        self.assertEqual(r["minimal_along_axis"], "NOT_ESTABLISHED")
        self.assertEqual(solid.search_separation(lambda t: 0.0, 1.0, 10, NULL, 20)["status"], "NO_OVERLAP_AT_ZERO")
        self.assertEqual(solid.search_separation(lambda t: 1.0, 1.0, 10, NULL, 20)["status"], "BRACKET_NOT_SEPARATING")

    def test_U_SOL_12(self):
        """Detection limit, stated honestly. A gap (0.31 <= t < 0.32) narrower than the scan spacing (0.05) is not sampled: the scanned search then agrees with the legacy bisection (0.7, last transition), still a verified separation (volume null at the returned t) but not the smallest, and the reappearance flag is False. A uniform scan can therefore never prove minimality; the output says NOT_ESTABLISHED in both cases."""
        def f(t):
            return 0.0 if (0.31 <= t < 0.32 or t >= 0.7) else 1.0
        r = solid.search_separation(f, 1.0, 40, NULL, 20)
        self.assertAlmostEqual(r["t"], 0.7, delta=1e-9)
        self.assertFalse(r["scan"]["overlap_reappears_after_first_null"])
        self.assertLessEqual(f(r["t"]), NULL)
        self.assertEqual(r["minimal_along_axis"], "NOT_ESTABLISHED")

    def test_U_SOL_13(self):
        """The axial bound is not the minimum translation distance. A = unit cube [0,1]^3. B = a slab whose face is the plane x+y = c with c = 2 - sqrt(2)/4, so the cube's corner line x=y=1 is sunk 0.25 m (measured along the face normal (1,1,0)/sqrt(2)) into B. Analytic minimum translation distance MTD = 0.25 m (move A along -(1,1,0)/sqrt(2)). Along an axis the corner must travel 2-c = sqrt(2)/4 = 0.353553 m: -X and -Y tie. The measured bound equals 0.353553 within 3e-6 m (the crossing certificate tolerates a shallow crossing up to tri_contact = 1e-6 m, i.e. up to sqrt(2)e-6 m of axial travel for this 45 degree face), is >= MTD, and is sqrt(2) times MTD to 1e-5: an upper bound, not the minimum."""
        s2 = math.sqrt(2.0)
        c0 = 2.0 - s2 / 4.0
        u0, u1, v0, v1, z0, z1 = c0 / s2, c0 / s2 + 3.0, -3.0, 3.0, -2.0, 3.0
        ub, tb_ = plain_cube_box((u0, v0, z0), (u1, v1, z1))
        cb = [((u - v) / s2, (u + v) / s2, z) for u, v, z in ub]
        ca, ta = cube()
        r = run(ca, ta, cb, tb_)
        self.assertEqual(r["status"], "VOLUME_MEASURED", r.get("reasons"))
        self.assertAlmostEqual(r["separation_bound_m"], s2 / 4.0, delta=3e-6)
        self.assertIn(r["separation_direction"], ("-X", "-Y"))
        mtd = 0.25
        self.assertGreaterEqual(r["separation_bound_m"], mtd)
        self.assertAlmostEqual(r["separation_bound_m"] / mtd, s2, delta=1e-5)

    def test_U_SOL_14(self):
        """A null-VOLUME test alone can under-report the separation for a sharp tip. A = square pyramid, apex (0,0,-0.5), base corners (+-1,+-1,0.5) (45 degree faces), B = slab [-2,2]x[-2,2]x[-1,0]. Moving A up by t leaves the tip h=0.5-t below the slab top: overlap V = 4 h^3/3. The true separating translation along +Z is 0.5 m. With only the 1e-12 m^3 null volume the search stops at h = (7.5e-13)^(1/3) = 9.09e-5 m short, t=0.4999091: a 'separating translation' that is not separating. With the surface-crossing certificate (tri_contact 1e-6 m) the returned t is within [0.5-2e-6, 0.5+1e-6]."""
        ca, ta = pyramid()
        cb, tb_ = plain_cube_box((-2, -2, -1), (2, 2, 0))
        short = 0.5 - (7.5e-13) ** (1.0 / 3.0)
        r0 = solid.measure_pair(ca, ta, cb, tb_, PURE, PARAMS_NO_CERT)
        self.assertAlmostEqual(r0["axial"]["+Z"]["t"], short, delta=2e-6)
        self.assertLess(r0["axial"]["+Z"]["t"], 0.5 - 5e-5)           # not a separation: the tip is still inside
        r1 = solid.measure_pair(ca, ta, cb, tb_, PURE, PARAMS)
        t1 = r1["axial"]["+Z"]
        self.assertEqual(t1["status"], "VERIFIED")
        self.assertGreaterEqual(t1["t"], 0.5 - 2e-6)
        self.assertLessEqual(t1["t"], 0.5 + 1e-6)
        self.assertGreater(t1["certificate"]["adjustments"], 0)
        self.assertEqual(t1["certificate"]["crossing_pairs"], 0)

    def test_U_SOL_15(self):
        """Certificate leaves a correct box result unchanged: for A=[0,1]^3 and B=A+(0.5,0.25,0.125) the certified axial values equal the face distances (+X 1.5, -X 0.5, +Y 1.25, -Y 0.75, +Z 1.125, -Z 0.875 m) within 2e-9 and no certificate needed an adjustment (face-to-face contact is not a proper crossing)."""
        ca, ta = cube()
        cb, tb = cube((0.5, 0.25, 0.125), (1.5, 1.25, 1.125))
        r = run(ca, ta, cb, tb)
        for k, v in {"+X": 1.5, "-X": 0.5, "+Y": 1.25, "-Y": 0.75, "+Z": 1.125, "-Z": 0.875}.items():
            self.assertAlmostEqual(r["axial"][k]["t"], v, delta=2e-9, msg=k)
            self.assertTrue(r["axial"][k]["certificate"]["ok"], k)
            self.assertEqual(r["axial"][k]["certificate"]["adjustments"], 0, k)
        self.assertEqual(r["axial_nonmonotone_directions"], [])

    def test_U_SOL_CFG(self):
        """Config pinning: the solid-measurement parameters these tests use (null volume 1e-12 m^3, degenerate face area 1e-12 m^2, bisection 18 steps in the production run, solver and identity agreement 1e-4) equal quality_rules.json. A config change fails here and must be made consciously."""
        cfg = json.load(open(os.path.join(os.path.dirname(HERE), "config", "quality_rules.json"), encoding="utf-8"))["tolerances"]
        self.assertEqual(cfg["penetration_null_volume"]["value"], NULL)
        self.assertEqual(cfg["degenerate_face_area"]["value"], AREA)
        self.assertEqual(cfg["penetration_bisection_iterations"]["value"], 18)
        self.assertEqual(cfg["volume_solver_agreement_rel"]["value"], AGREE)
        self.assertEqual(cfg["volume_identity_rel"]["value"], IDENT)
        self.assertEqual(cfg["penetration_null_volume"]["status"], "PROVISIONAL")
        self.assertEqual(cfg["penetration_axial_scan_samples"]["value"], PARAMS["axial_scan_samples"])
        self.assertEqual(cfg["tri_contact"]["value"], PARAMS["tri_contact"])
        self.assertEqual(cfg["tri_plane_coplanar"]["value"], PARAMS["tri_plane_coplanar"])
        self.assertEqual(cfg["coincident_vertex"]["value"], PARAMS["coincident_vertex"])
        self.assertEqual(cfg["zero_length_edge"]["value"], 1e-6)


if __name__ == "__main__":
    unittest.main()
