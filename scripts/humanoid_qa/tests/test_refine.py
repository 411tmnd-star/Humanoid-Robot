"""Pure-Python analytic fixtures for measurement-copy refinement (core/refine.py) and its
integration in solid.measure_pair. Every expected value is derived from the fixture
geometry stated in each docstring. Units: metres / cubic metres.
"""

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))  # scripts/

from humanoid_qa.core import geom, refine, solid  # noqa: E402

AREA = 1e-12
EDGE = 1e-6
NULL = 1e-12
W = 1e-13             # width of the sliver strip in the fixtures (m): area 1e-13 m^2 < 1e-12 m^2
PARAMS = {"null_volume": NULL, "degenerate_area": AREA, "iters": 30, "agree_rel": 1e-4,
          "identity_rel": 1e-4, "refine_edge_tol": EDGE}
CORNERS = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]


def _orient(coords, polys, centre=(0.5, 0.5, 0.5)):
    """Fan-triangulate planar polygons from their FIRST listed vertex with outward winding, decided
    against the solid's centre (convex solids only)."""
    tris = []
    for q in polys:
        pts = [coords[i] for i in q]
        n = solid._newell(pts)
        cen = tuple(sum(p[k] for p in pts) / len(pts) for k in range(3))
        if geom.dot(n, geom.sub(cen, centre)) < 0:
            q = tuple(reversed(q))
        tris += [(q[0], q[i], q[i + 1]) for i in range(1, len(q) - 1)]
    return tris


def sliver_cube(w=W):
    """Unit cube whose bottom-front edge a-b is doubled: a2=a+(0,w,0), b2=b+(0,w,0). The bottom face
    runs from a2-b2 inward; a strip (a,a2,b2,b) of width w lies IN the plane z=0 between the front
    face and the bottom face; a2 and b2 are also vertices of the left / right faces (they lie on
    their edges a-d and b-c). Every point lies on the unit cube's surface, so the enclosed solid is
    exactly the unit cube (1 m^3). The strip triangles and the two thin side triangles have area
    <= w/2 = 5e-14 m^2 (< 1e-12: degenerate); the doubled vertices are w = 1e-13 m apart."""
    coords = [tuple(map(float, p)) for p in CORNERS] + [(0.0, w, 0.0), (1.0, w, 0.0)]
    polys = [(8, 3, 2, 9),            # bottom, starts at the doubled edge
             (0, 8, 9, 1),            # the strip
             (0, 1, 5, 4), (3, 7, 6, 2), (7, 4, 0, 8, 3), (6, 5, 1, 9, 2), (4, 5, 6, 7)]
    return coords, _orient(coords, polys)


def needle_cube():
    """Unit cube with an extra vertex m=(0.5,0,0) at the midpoint of the bottom-front edge, used by
    the front and bottom faces (a watertight T-vertex). The fan triangles (a,m,b) are collinear:
    zero area, edges 0.5 / 0.5 / 1.0 m, NO short edge, so no merge can repair them. Volume 1 m^3."""
    coords = [tuple(map(float, p)) for p in CORNERS] + [(0.5, 0.0, 0.0)]
    polys = [(3, 2, 1, 8, 0), (0, 8, 1, 5, 4), (3, 7, 6, 2), (0, 4, 7, 3), (1, 2, 6, 5), (4, 5, 6, 7)]
    return coords, _orient(coords, polys)


def seam_cube():
    """Unit cube where the bottom face uses a bit-identical copy (index 8) of corner 0."""
    coords = [tuple(map(float, p)) for p in CORNERS] + [(0.0, 0.0, 0.0)]
    polys = [(8, 3, 2, 1), (0, 1, 5, 4), (3, 7, 6, 2), (0, 4, 7, 3), (1, 2, 6, 5), (4, 5, 6, 7)]
    return coords, _orient(coords, polys)


def plain_cube(lo=(0, 0, 0), hi=(1, 1, 1)):
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    coords = [(x, y, z) for z in (z0, z1) for (x, y) in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
    polys = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (2, 3, 7, 6), (0, 4, 7, 3), (1, 2, 6, 5)]
    centre = ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
    return [tuple(map(float, p)) for p in coords], _orient(coords, polys, centre)


def gate_fn(c, t):
    return solid.gate(c, t, AREA)


class Fixtures(unittest.TestCase):
    def test_U_REF_00(self):
        """Fixture sanity: the sliver cube is a closed, consistently wound shell of signed volume +1 m^3 whose ONLY gate reason is DEGENERATE_FACE (4 triangles of area <= 5e-14 m^2: the strip and the two thin side triangles); the seam cube has bit-identical duplicate vertices and is refused for OPEN_BOUNDARY."""
        c, t = sliver_cube()
        self.assertAlmostEqual(geom.signed_volume(c, t), 1.0, delta=1e-12)
        self.assertEqual(solid.gate(c, t, AREA), ["DEGENERATE_FACE"])
        c2, t2 = seam_cube()
        self.assertAlmostEqual(geom.signed_volume(c2, t2), 1.0, delta=1e-12)
        self.assertIn("OPEN_BOUNDARY", solid.gate(c2, t2, AREA))

    def test_U_REF_01(self):
        """Exact reindexing: merging the bit-identical duplicate corner of the seam cube closes it (no boundary edges), moves no vertex, drops no face, and leaves the volume exactly 1 m^3: classification EXACT_EQUIVALENT."""
        c, t = seam_cube()
        r = refine.exact_reindex(c, t)
        self.assertEqual(len(r["coords"]), 8)
        self.assertEqual(len(r["tris"]), 12)
        self.assertEqual(r["removed"], [])
        self.assertEqual(solid.gate(r["coords"], r["tris"], AREA), [])
        rep = refine.preservation_report(c, t, r, AREA, EDGE)
        cls, fails = refine.accept(rep, EDGE)
        self.assertEqual(fails, [])
        self.assertEqual(cls, "EXACT_EQUIVALENT")
        self.assertEqual(rep["max_vertex_displacement_m"], 0.0)
        self.assertAlmostEqual(rep["after"]["signed_volume_m3"], 1.0, delta=1e-12)

    def test_U_REF_02(self):
        """Exact reindexing does nothing for the sliver cube (its duplicates differ by 1e-13 m, not bit-identical): no cluster is found, so the refinement chain must go on to the edge collapse (least-invasive order)."""
        c, t = sliver_cube()
        r = refine.exact_reindex(c, t)
        self.assertEqual(r["clusters"], [])
        self.assertEqual(len(r["tris"]), len(t))

    def test_U_REF_03(self):
        """Removing the degenerate faces alone is NOT acceptable: dropping the 4 degenerate triangles of the sliver cube leaves an open surface (the strip bounded a closed shell) with boundary edges, and accept() rejects it as NOT_EQUIVALENT."""
        c, t = sliver_cube()
        r = refine.remove_degenerate(c, t, AREA)
        self.assertEqual(len(r["removed"]), 4)
        rep = refine.preservation_report(c, t, r, AREA, EDGE)
        self.assertGreater(rep["after"]["boundary_edges"], 0)
        cls, fails = refine.accept(rep, EDGE)
        self.assertEqual(cls, "NOT_EQUIVALENT")
        self.assertTrue(any("topology" in f for f in fails), fails)

    def test_U_REF_04(self):
        """Edge collapse of the sliver cube: 2 vertices merged (8+2 -> 8), exactly the 4 degenerate triangles removed (16 -> 12), no boundary / non-manifold / degenerate / zero-length element left, volume 1 m^3 within 1e-12 and vertex displacement w/2 = 5e-14 m, classification NUMERICALLY_EQUIVALENT. Surviving faces are the original faces under the vertex map."""
        c, t = sliver_cube()
        r = refine.collapse_short_edges(c, t, EDGE)
        self.assertEqual((len(r["coords"]), len(r["tris"]), len(r["removed"])), (8, 12, 4))
        rep = refine.preservation_report(c, t, r, AREA, EDGE)
        a = rep["after"]
        self.assertEqual((a["boundary_edges"], a["nonmanifold_edges"], a["degenerate_faces"],
                          a["zero_length_edges"], a["winding_inconsistent_edges"]), (0, 0, 0, 0, 0))
        self.assertTrue(rep["surviving_faces_remap_exact"])
        self.assertAlmostEqual(a["signed_volume_m3"], 1.0, delta=1e-12)
        self.assertAlmostEqual(rep["max_vertex_displacement_m"], W / 2, delta=1e-16)
        cls, fails = refine.accept(rep, EDGE)
        self.assertEqual((cls, fails), ("NUMERICALLY_EQUIVALENT", []))
        self.assertEqual(rep["vertices_moved"], 4)

    def test_U_REF_05(self):
        """refine_for_gate on the sliver cube applies exact reindex (no-op) then edge collapse and ends REFINED with the gate empty; on a gate-clean cube it reports NOT_NEEDED and returns no copy; the input lists are not modified."""
        c, t = sliver_cube()
        c0, t0 = list(c), list(t)
        out = refine.refine_for_gate(c, t, AREA, EDGE, gate_fn)
        self.assertEqual(out["status"], "REFINED")
        self.assertEqual([s["op"] for s in out["steps"]], ["EXACT_REINDEX", "COLLAPSE_SHORT_EDGES"])
        self.assertFalse(out["steps"][0]["applied"])
        self.assertEqual(out["steps"][1]["gate_after"], [])
        self.assertEqual((c, t), (c0, t0))
        pc, pt = plain_cube()
        self.assertEqual(refine.refine_for_gate(pc, pt, AREA, EDGE, gate_fn)["status"], "NOT_NEEDED")

    def test_U_REF_06(self):
        """A degenerate face with NO short edge (the needle cube: collinear triangles (a,m,b) with edges 0.5 / 0.5 / 1.0 m) cannot be repaired by merging: refine_for_gate reports NOT_APPLICABLE and measure_pair keeps the pair UNABLE_TO_DETERMINE with reasons DEGENERATE_FACE and REFINEMENT_NOT_APPLICABLE (no value is produced)."""
        c, t = needle_cube()
        self.assertEqual(solid.gate(c, t, AREA), ["DEGENERATE_FACE"])
        out = refine.refine_for_gate(c, t, AREA, EDGE, gate_fn)
        self.assertEqual(out["status"], "NOT_APPLICABLE")
        cb, tb = plain_cube((0.5, 0.25, 0.125), (1.5, 1.25, 1.125))
        r = solid.measure_pair(c, t, cb, tb, solid.ConvexBackend, PARAMS)
        self.assertEqual(r["status"], "UNABLE_TO_DETERMINE")
        self.assertIn("DEGENERATE_FACE", r["reasons"]["A"])
        self.assertIn("REFINEMENT_NOT_APPLICABLE", r["reasons"]["A"])
        self.assertNotIn("overlap_volume_m3", r)

    def test_U_REF_07(self):
        """Merging that destroys geometry is rejected: a plate 1 m x 1 m x 5e-7 m (thickness below the 1e-6 m edge tolerance) collapsed by edge length folds its top and bottom faces onto each other and removes all side faces; accept() returns NOT_EQUIVALENT (coincident faces / degenerate result), so a thin real solid is never silently flattened."""
        c, t = plain_cube((0, 0, 0), (1, 1, 5e-7))
        r = refine.collapse_short_edges(c, t, EDGE)
        self.assertGreater(len(r["clusters"]), 0)
        rep = refine.preservation_report(c, t, r, AREA, EDGE)
        cls, fails = refine.accept(rep, EDGE)
        self.assertEqual(cls, "NOT_EQUIVALENT", fails)
        self.assertTrue(fails)

    def test_U_REF_08(self):
        """measure_pair on the refined sliver cube against the cube B=[0.5,1.5]x[0.25,1.25]x[0.125,1.125]: overlap box 0.5 x 0.75 x 0.875 = 0.328125 m^3 (exact convex backend). geometry_basis names the refined operand and its class; refinement_effect compares the unrefined shells through the same backend (equal to 1e-12 m^3); the six axial separations equal the face distances +X 1.5, -X 0.5, +Y 1.25, -Y 0.75, +Z 1.125, -Z 0.875 m (unchanged by the refinement)."""
        c, t = sliver_cube()
        cb, tb = plain_cube((0.5, 0.25, 0.125), (1.5, 1.25, 1.125))
        r = solid.measure_pair(c, t, cb, tb, solid.ConvexBackend, PARAMS)
        self.assertEqual(r["status"], "VOLUME_MEASURED", r.get("reasons"))
        self.assertAlmostEqual(r["overlap_volume_m3"], 0.328125, delta=1e-12)
        self.assertEqual(r["geometry_basis"], "REFINED_COPY:A=NUMERICALLY_EQUIVALENT")
        eff = r["refinement_effect"]
        self.assertEqual(eff["status"], "COMPARED")
        self.assertTrue(eff["ok"])
        for k, v in {"+X": 1.5, "-X": 0.5, "+Y": 1.25, "-Y": 0.75, "+Z": 1.125, "-Z": 0.875}.items():
            self.assertAlmostEqual(r["axial"][k]["t"], v, delta=2e-9, msg=k)
        self.assertEqual(r["refinement"]["A"]["status"], "REFINED")
        self.assertNotIn("B", r["refinement"])

    def test_U_REF_09(self):
        """Refinement verdict logic with stub kernels: when the unrefined shells give a different volume (1.1 vs 1.0 m^3, 10 percent) the pair is REFINEMENT_AFFECTS_RESULT, not validated; when the unrefined shells cannot be run it is REFINEMENT_UNCONFIRMED; when they agree it is validated. The refinement must never decide the number alone."""
        c, t = sliver_cube()
        cb, tb = plain_cube((0.5, 0.25, 0.125), (1.5, 1.25, 1.125))

        class Stub:
            solvers = ("K1", "K2")

            def __init__(self, ca, ta, cb_, tb_, mode):
                self.refined = len(ca) == 8
                self.mode = mode

            def close(self):
                return None

            def volume(self, kind, solver, shift=(0.0, 0.0, 0.0)):
                if not self.refined and self.mode == "raise":
                    raise RuntimeError("kernel failed on unrefined shell")
                base = 1.0 if (self.refined or self.mode == "same") else 1.1
                return {"AB": base, "BA": base, "AMB": 1.0 - base}[kind] if shift == (0.0, 0.0, 0.0) else 0.0

        for mode, expect in (("differ", "REFINEMENT_AFFECTS_RESULT"), ("raise", "REFINEMENT_UNCONFIRMED"),
                             ("same", "VOLUME_VALIDATED")):
            r = solid.measure_pair(c, t, cb, tb, lambda a, b, c_, d, m=mode: Stub(a, b, c_, d, m), PARAMS)
            self.assertEqual(r["status"], expect, (mode, r["status"], r.get("refinement_effect")))

    def test_U_CMP_PEN_01(self):
        """Penetration regression comparator on synthetic documents (old keys axial_depth_*, new keys axial_separation_*): an identical pair is counted identical; a volume differing by 1e-3 relative is reported over tolerance; a separation change of 2e-6 m with no recorded cause but inside the old bisection resolution is explained by resolution; a change of 0.5 m with no recorded cause is UNEXPLAINED; a change with a recorded certificate adjustment is explained by the certificate; a validated pair that is no longer validated is a status transition."""
        from humanoid_qa.core import compare_pen
        def row(a, v, bound, d="+Z", st="VOLUME_VALIDATED", key="old", **extra):
            r = {"a": a, "b": "X", "status": st, "overlap_volume_m3": v}
            if key == "old":
                r.update(axial_depth_bound_m=bound, axial_depth_direction=d)
            else:
                r.update(axial_separation_bound_m=bound, axial_separation_direction=d)
            r.update(extra)
            return r
        old = {"pairs": [row("p1", 1e-3, 0.1), row("p2", 1e-3, 0.1), row("p3", 1e-3, 0.1), row("p4", 1e-3, 0.1),
                         row("p5", 1e-3, 0.1), row("p6", 1e-3, 0.1)]}
        new = {"pairs": [row("p1", 1e-3, 0.1, key="new"), row("p2", 1.001e-3, 0.1, key="new"),
                         row("p3", 1e-3, 0.1 + 2e-6, key="new"), row("p4", 1e-3, 0.6, key="new"),
                         row("p5", 1e-3, 0.1 + 1e-4, key="new", axial_certificate_added_m={"+Z": 1e-4}),
                         row("p6", None, None, key="new", st="UNABLE_TO_DETERMINE")]}
        c = compare_pen.compare(old, new)
        self.assertEqual(c["volume"]["compared"], 5)
        self.assertEqual(c["volume"]["bitwise_equal"], 4)
        self.assertEqual([r["pair"][0] for r in c["volume"]["over_tol"]], ["p2", "p6"][:1] + ["p6"])
        self.assertEqual(c["separation"]["identical"], 2)
        reasons = {r["pair"][0]: r["explained_by"] for r in c["separation"]["changed"]}
        self.assertTrue(any("resolution" in x for x in reasons["p3"]))
        self.assertEqual(reasons["p4"], [])
        self.assertTrue(any("certificate" in x for x in reasons["p5"]))
        self.assertEqual([r["pair"][0] for r in c["separation"]["unexplained"]], ["p4"])
        self.assertIn("VOLUME_VALIDATED -> UNABLE_TO_DETERMINE", c["status_transitions"])
        self.assertEqual((c["baseline_validated"], c["baseline_validated_still_validated"]), (6, 5))


if __name__ == "__main__":
    unittest.main()
