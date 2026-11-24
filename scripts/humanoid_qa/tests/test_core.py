"""Pure-Python unit tests (stdlib unittest; no bpy, no third-party packages).

Every expected value is derived analytically from the synthetic input, never
copied from the production asset. Test IDs are encoded in method names
(test_U_GEOM_01 -> U-GEOM-01) and are referenced by config/quality_rules.json.
"""

import hashlib
import json
import math
import os
import random
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))  # scripts/

from humanoid_qa.core import classify, compare, evidence, gate, geom, glb, rig, rules  # noqa: E402

EPS = 1e-9


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def trs(t, deg_z, s):
    """Translation @ RotZ @ Scale, row-major."""
    r = rot_z(deg_z)
    return [[r[i][0] * s, r[i][1] * s, r[i][2] * s, t[i]] for i in range(3)] + [[0, 0, 0, 1]]


CUBE_V = [(x, y, z) for x in (0, 1) for y in (0, 1) for z in (0, 1)]
# quads with consistent indexing for the 8 corners above (index = 4x + 2y + z)
CUBE_F = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]


class Geom(unittest.TestCase):
    def test_U_GEOM_01(self):
        """Box bounds: a 2 x 3 x 4 box from (1,1,1) has dims (2,3,4) and centre (2,2.5,3)."""
        pts = [(1 + 2 * x, 1 + 3 * y, 1 + 4 * z) for x, y, z in CUBE_V]
        bb = geom.bbox(pts)
        self.assertEqual(geom.bbox_dims(bb), (2, 3, 4))
        self.assertEqual(geom.bbox_centre(bb), (2, 2.5, 3))

    def test_U_GEOM_02(self):
        """Local vs world: unit cube under T(1,2,3) R_z(90) S(2) spans x[-1,1], y[2,4], z[3,5]."""
        m = trs((1, 2, 3), 90, 2)
        lo, hi = geom.bbox(geom.mat_apply(m, p) for p in CUBE_V)
        for a, b in zip(lo, (-1, 2, 3)):
            self.assertAlmostEqual(a, b, delta=EPS)
        for a, b in zip(hi, (1, 4, 5)):
            self.assertAlmostEqual(a, b, delta=EPS)
        self.assertEqual(geom.bbox_dims(geom.bbox(CUBE_V)), (1, 1, 1))  # local unchanged

    def test_U_GEOM_03(self):
        """Percentile: [0..100] gives p50=50, p99=99, p0=0; interpolation 0.5 between [0,1]."""
        s = [float(i) for i in range(101)]
        self.assertEqual(geom.percentile(s, 50), 50)
        self.assertEqual(geom.percentile(s, 99), 99)
        self.assertEqual(geom.percentile(s, 0), 0)
        self.assertAlmostEqual(geom.percentile([0.0, 1.0], 50), 0.5)
        with self.assertRaises(ValueError):
            geom.percentile([], 50)

    def test_U_GEOM_04(self):
        """k-d tree NN is exact: equals brute force for 400 queries (incl. far outside the data) over 600 points with duplicates and a flat axis."""
        rnd = random.Random(411)
        pts = [(rnd.uniform(-1, 1), rnd.uniform(-1, 1), rnd.uniform(0, 2)) for _ in range(500)]
        pts += [pts[3]] * 50                                       # exact duplicates
        pts += [(rnd.uniform(-1, 1), 0.25, 1.0) for _ in range(50)]  # degenerate (flat) axes
        qs = [(rnd.uniform(-1.5, 1.5), rnd.uniform(-1.5, 1.5), rnd.uniform(-0.5, 2.5)) for _ in range(300)]
        qs += [(rnd.uniform(-50, 50), rnd.uniform(-50, 50), rnd.uniform(-50, 50)) for _ in range(100)]
        tree = geom.KDTree(pts)
        for q in qs:
            brute = min(geom.dist(q, p) for p in pts)
            d, i = tree.nearest(q)
            self.assertAlmostEqual(d, brute, delta=1e-12)
            self.assertAlmostEqual(geom.dist(q, pts[i]), d, delta=1e-12)

    def test_U_GEOM_05(self):
        """Mirror compare: exact mirror pair -> 0 both ways; pair offset by +0.05 in Z -> every distance 0.05."""
        rnd = random.Random(7)
        a = [(rnd.uniform(0.1, 0.5), rnd.uniform(-0.2, 0.2), rnd.uniform(0, 1)) for _ in range(200)]
        b = [geom.mirror_x(p) for p in a]
        r = geom.mirror_compare(a, b, 0.0, 1e-6, 0.01)
        self.assertLess(r["forward"]["max"], 1e-12)
        self.assertLess(r["backward"]["max"], 1e-12)
        # shift far smaller than point spacing would not be exact; use a set
        # whose points are >= 0.2 apart so the nearest match stays the twin
        a2 = [(0.3, 0.0, 0.0), (0.3, 0.5, 0.0), (0.6, 0.0, 0.5)]
        b2 = [(-x, y, z + 0.05) for x, y, z in a2]
        r2 = geom.mirror_compare(a2, b2, 0.0, 1e-3, 0.01)
        self.assertAlmostEqual(r2["forward"]["max"], 0.05, delta=1e-12)
        self.assertAlmostEqual(r2["forward"]["min"], 0.05, delta=1e-12)
        self.assertEqual(r2["forward"]["count_over_tolerance"], 3)
        # non-zero mirror plane: plane x=0.1 maps x=0.3 -> -0.1
        # 2*0.1 - 0.3 is not exactly -0.1 in binary floating point; compare to 1e-12
        for got, exp in zip(geom.mirror_x((0.3, 1, 2), 0.1), (-0.1, 1, 2)):
            self.assertAlmostEqual(got, exp, delta=1e-12)

    def test_U_GEOM_06(self):
        """Coincident pairs: 3 points at one location + 1 at 1e-7 + 1 far -> C(4,2)=6 pairs at tol 1e-6."""
        pts = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (1e-7, 0, 0), (1, 1, 1)]
        self.assertEqual(len(geom.coincident_pairs(pts, 1e-6)), 6)
        self.assertEqual(len(geom.coincident_pairs([(0, 0, 0), (2e-6, 0, 0)], 1e-6)), 0)

    def test_U_GEOM_07(self):
        """Topology: closed cube 0 boundary/1 comp; open quad 4 boundary; 3 faces on one edge -> 1 non-manifold; wire edge, loose vertex, 2 comps."""
        t = geom.topology(8, [], CUBE_F)
        self.assertEqual((t["edges"], t["faces"], t["boundary_edges"], t["nonmanifold_edges"],
                          t["connected_components"]), (12, 6, 0, 0, 1))
        q = geom.topology(4, [], [(0, 1, 2, 3)])
        self.assertEqual(q["boundary_edges"], 4)
        # three triangles sharing edge (0,1)
        nm = geom.topology(5, [], [(0, 1, 2), (0, 1, 3), (1, 0, 4)])
        self.assertEqual(nm["nonmanifold_edges"], 1)
        # quad + separate wire edge (4,5) + loose vertex 6
        w = geom.topology(7, [(4, 5)], [(0, 1, 2, 3)])
        self.assertEqual((w["wire_edges"], w["loose_vertices"], w["connected_components"]), (1, 1, 3))

    def test_U_GEOM_08(self):
        """Degenerate: unit square area 1; collinear triangle area 0 -> 1 degenerate face; a duplicated vertex gives 1 zero-length edge."""
        self.assertAlmostEqual(geom.polygon_area([(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]), 1.0)
        coords = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (0, 1, 0), (0, 1, 0)]
        t = geom.topology(5, [], [(0, 1, 2), (0, 1, 3, 4)], coords, 1e-12, 1e-6)
        self.assertEqual(t["degenerate_faces"], 1)
        self.assertEqual(t["zero_length_edges"], 1)

    def test_U_GEOM_09(self):
        """Right axis = forward x up: facing -Y with +Z up -> right is -X; facing +Y -> +X."""
        self.assertEqual(geom.right_axis((0, -1, 0), (0, 0, 1)), (-1, 0, 0))
        self.assertEqual(geom.right_axis((0, 1, 0), (0, 0, 1)), (1, 0, 0))

    def test_U_GEOM_10(self):
        """Empty inputs: bbox -> None, empty index -> (inf,-1), zero vector normalise raises."""
        self.assertIsNone(geom.bbox([]))
        self.assertEqual(geom.KDTree([]).nearest((0, 0, 0)), (math.inf, -1))
        self.assertEqual(geom.nearest_distances([(0, 0, 0)], []), [math.inf])
        with self.assertRaises(ValueError):
            geom.normalize((0, 0, 0))
        with self.assertRaises(ValueError):
            geom.GridIndex([], 0)


def bone(name, parent, head, tail, deform=True):
    return {"name": name, "parent": parent, "head_w": head, "tail_w": tail,
            "length": geom.dist(head, tail), "use_deform": deform}


class Rig(unittest.TestCase):
    def test_U_RIG_01(self):
        """Graph: Root->(A->B, C) has 1 root, depth 2, 2 leaves, 1 component, no cycle."""
        bs = [bone("Root", None, (0, 0, 0), (0, 0, 1)), bone("A", "Root", (0, 0, 1), (0, 0, 2)),
              bone("B", "A", (0, 0, 2), (0, 0, 3)), bone("C", "Root", (0, 0, 1), (1, 0, 1))]
        g = rig.graph_summary(bs)
        self.assertEqual((g["roots"], g["max_depth"], g["leaf_count"], g["component_count"], g["cycle_nodes"]),
                         (["Root"], 2, 2, 1, []))

    def test_U_RIG_02(self):
        """Graph faults: A<->B cycle detected; parent 'Ghost' reported missing; two islands -> 2+ components."""
        bs = [{"name": "A", "parent": "B"}, {"name": "B", "parent": "A"},
              {"name": "C", "parent": "Ghost"}, {"name": "D", "parent": None}]
        g = rig.graph_summary(bs)
        self.assertEqual(g["cycle_nodes"], ["A", "B"])
        self.assertEqual(g["missing_parent_refs"], ["C"])
        self.assertGreaterEqual(g["component_count"], 3)

    def test_U_RIG_03(self):
        """Side parsing: suffix, dot, infix and unsided forms give the shared base name."""
        cases = {"DEF_Thigh_L": ("suffix_L", "DEF_Thigh"), "DEF_Thigh": ("none", "DEF_Thigh"),
                 "MECH_Wrist_Flex.R": ("dot_R", "MECH_Wrist_Flex"),
                 "MECH_Shoulder_L_FlexExt": ("infix_L", "MECH_Shoulder_FlexExt"),
                 "IK_Target.arm.L": ("dot_L", "IK_Target.arm"), "DEF_Palm_Little": ("none", "DEF_Palm_Little")}
        for n, exp in cases.items():
            self.assertEqual(rig.parse_side(n), exp, n)

    def test_U_RIG_04(self):
        """Pairing: X_L<->X, Y_L<->Y_R (R preferred), Z_L unpaired, W_R unpaired."""
        p = rig.pair_by_name(["X", "X_L", "Y_L", "Y_R", "Y", "Z_L", "W_R"])
        got = {(q["l_marked"], q["counterpart"]) for q in p["pairs"]}
        self.assertEqual(got, {("X_L", "X"), ("Y_L", "Y_R")})
        self.assertEqual(p["l_marked_unpaired"], ["Z_L"])
        self.assertEqual(p["r_marked_unpaired"], ["W_R"])

    def test_U_RIG_05(self):
        """Pair metrics: exact mirror -> 0 deltas, 0 deg; twin shifted 0.03 in Y and tilted -> 0.03 head delta, known angle."""
        a = bone("A_L", None, (-0.2, 0, 1), (-0.3, 0, 1))
        b = bone("A", None, (0.2, 0, 1), (0.3, 0, 1))
        m = rig.pair_metrics(a, b, 0.0)
        self.assertLess(m["max_endpoint_delta"], 1e-12)
        self.assertLess(m["direction_angle_deg"], 1e-6)
        c = bone("A", None, (0.2, 0.03, 1), (0.3, 0.03, 1.1))
        m2 = rig.pair_metrics(a, c, 0.0)
        self.assertAlmostEqual(m2["head_delta"], 0.03, delta=1e-12)
        self.assertAlmostEqual(m2["direction_angle_deg"], 45.0, delta=1e-9)

    def test_U_RIG_06(self):
        """Skinning: deform-on bone with group+weights+armature mod -> eligible; deform-on with no group -> not; group with zero weights reported; group without bone reported."""
        bs = [bone("DEF_A", None, (0, 0, 0), (0, 0, 1)), bone("MECH_B", None, (0, 0, 0), (0, 0, 1)),
              bone("DEF_C", None, (0, 0, 0), (0, 0, 1))]
        meshes = [{"name": "m1", "vgroup_names": ["DEF_A", "DEF_C", "Stale"],
                   "vgroup_nonzero": {"DEF_A": 10, "DEF_C": 0, "Stale": 3},
                   "armature_mods": [{"target": "Arm", "use_vertex_groups": True,
                                      "show_viewport": True, "show_render": True}]}]
        s = rig.skinning_layers(bs, meshes, "Arm")
        row = {r["bone"]: r for r in s["bones"]}
        self.assertTrue(row["DEF_A"]["5_eligible_to_deform"])
        self.assertFalse(row["MECH_B"]["2_vertex_group_exists"])
        self.assertFalse(row["MECH_B"]["5_eligible_to_deform"])
        self.assertTrue(row["DEF_C"]["2_vertex_group_exists"])
        self.assertEqual(row["DEF_C"]["3_nonzero_weight_vertices"], 0)
        self.assertEqual(s["vertex_groups_with_zero_weights"], [{"mesh": "m1", "group": "DEF_C"}])
        self.assertEqual(s["vertex_groups_without_bone"][0]["group"], "Stale")
        # same mesh, modifier pointing at another armature -> not bound
        meshes[0]["armature_mods"][0]["target"] = "Other"
        s2 = rig.skinning_layers(bs, meshes, "Arm")
        self.assertFalse({r["bone"]: r for r in s2["bones"]}["DEF_A"]["5_eligible_to_deform"])

    def test_U_RIG_07(self):
        """Constraint refs: missing target, missing object, missing subtarget bone each reported; valid one not."""
        objs = {"Arm": "ARMATURE", "Cam": "CAMERA"}
        bones = {"Arm": {"B1"}}
        cs = [{"owner": "o1", "type": "COPY_ROTATION", "target": None, "subtarget": None},
              {"owner": "o2", "type": "COPY_ROTATION", "target": "Gone", "subtarget": None},
              {"owner": "o3", "type": "COPY_ROTATION", "target": "Arm", "subtarget": "B9"},
              {"owner": "o4", "type": "COPY_ROTATION", "target": "Arm", "subtarget": "B1"},
              {"owner": "o5", "type": "LIMIT_ROTATION", "target": None, "subtarget": None}]
        p = rig.constraint_reference_check(cs, objs, bones)
        self.assertEqual([(x["owner"], x["problem"]) for x in p],
                         [("o1", "missing target"), ("o2", "target object does not exist"),
                          ("o3", "subtarget bone does not exist")])

    def test_U_RIG_08(self):
        """Zero-length bone: direction angle is None instead of a division error."""
        a = bone("Z_L", None, (0, 0, 0), (0, 0, 0))
        b = bone("Z", None, (0, 0, 0), (0, 0, 1))
        self.assertIsNone(rig.pair_metrics(a, b, 0.0)["direction_angle_deg"])

    def test_U_RIG_09(self):
        """Constraint pair diff: identical stacks (side-stripped subtargets) -> no diff; changed influence -> 1 diff."""
        c = lambda sub, inf: {"type": "COPY_ROTATION", "subtarget": sub, "influence": inf,
                              "owner_space": "LOCAL", "target_space": "LOCAL", "mute": False}
        pairs = [{"l_marked": "M_L", "counterpart": "M"}]
        same = {"M_L": [c("CTRL_X_L", 1.0)], "M": [c("CTRL_X", 1.0)]}
        self.assertEqual(rig.constraint_pair_diffs(same, pairs), [])
        diff = {"M_L": [c("CTRL_X_L", 0.5)], "M": [c("CTRL_X", 1.0)]}
        self.assertEqual(len(rig.constraint_pair_diffs(diff, pairs)), 1)

    def test_U_RIG_10(self):
        """Co-location: MECH bone 0.02 from DEF_A head and 1.0 from DEF_B -> nearest DEF_A at 0.02."""
        defs = [bone("DEF_A", None, (0, 0, 0), (0, 0, 1)), bone("DEF_B", None, (1, 0, 0), (1, 0, 1))]
        m = bone("MECH_A", None, (0.02, 0, 0), (0.02, 0, 1))
        r = rig.nearest_def_head(m, defs)
        self.assertEqual(r["def_bone"], "DEF_A")
        self.assertAlmostEqual(r["distance"], 0.02, delta=1e-12)


    def test_U_RIG_11(self):
        """Thumb adjacency: thumb 0.02 from Index MCP and 0.06 from Little MCP -> beside Index; moved past Little -> beside Little."""
        bb = {"DEF_Index_MCP": bone("DEF_Index_MCP", None, (0.32, 0, 1), (0.32, 0, 0.9)),
              "DEF_Little_MCP": bone("DEF_Little_MCP", None, (0.38, 0, 1), (0.38, 0, 0.9)),
              "DEF_Thumb_CMC": bone("DEF_Thumb_CMC", None, (0.30, 0, 1), (0.29, 0, 0.95))}
        r = rig.thumb_adjacency(bb)
        self.assertEqual(r["thumb_beside"], "Index")
        self.assertAlmostEqual(r["dist_to_index_mcp"], 0.02, delta=1e-12)
        bb["DEF_Thumb_CMC"] = bone("DEF_Thumb_CMC", None, (0.40, 0, 1), (0.41, 0, 0.95))
        self.assertEqual(rig.thumb_adjacency(bb)["thumb_beside"], "Little")
        self.assertIsNone(rig.thumb_adjacency(bb, "_L"))

    def test_U_RIG_12(self):
        """Short bones: a 2e-6 m bone gets direction None under a 1e-4 m guard and is listed by short_bones; a 0.05 m bone is not."""
        a = bone("W_L", None, (-0.3, 0, 1), (-0.3 - 2e-6, 0, 1))
        b = bone("W", None, (0.3, 0, 1), (0.3, 2e-6, 1))
        self.assertIsNone(rig.pair_metrics(a, b, 0.0, 1e-4)["direction_angle_deg"])
        self.assertIsNotNone(rig.pair_metrics(a, b, 0.0, 0.0)["direction_angle_deg"])
        long_b = bone("Long", None, (0, 0, 0), (0, 0, 0.05))
        self.assertEqual([r["bone"] for r in rig.short_bones([a, b, long_b], 1e-4)], ["W", "W_L"])

    def test_U_CLS_01(self):
        """Finger mesh names map through the side-dependent table: Finger4_R_A and Finger1_L_A both become FingerIndex; others untouched."""
        fm = {"R": {"4": "Index", "1": "Little"}, "L": {"1": "Index", "4": "Little"}}
        self.assertEqual(classify.canonical_mesh_name("411_Finger4_R_A", fm), "411_FingerIndex_R_A")
        self.assertEqual(classify.canonical_mesh_name("411_Finger1_L_A", fm), "411_FingerIndex_L_A")
        self.assertEqual(classify.canonical_mesh_name("411_Thumb_L", fm), "411_Thumb_L")
        self.assertEqual(classify.canonical_mesh_name("411_Finger9_L_A", fm), "411_Finger9_L_A")


    def test_U_GEOM_11(self):
        """Winding: outward cube has 0 inconsistent edges and volume +1; one flipped face -> 4 inconsistent edges; all faces reversed -> 0 inconsistent, volume -1."""
        self.assertEqual(geom.winding_inconsistencies(CUBE_F), 0)
        self.assertAlmostEqual(geom.signed_volume(CUBE_V, CUBE_F), 1.0, delta=1e-12)
        flipped = list(CUBE_F)
        flipped[0] = tuple(reversed(flipped[0]))
        self.assertEqual(geom.winding_inconsistencies(flipped), 4)
        rev = [tuple(reversed(f)) for f in CUBE_F]
        self.assertEqual(geom.winding_inconsistencies(rev), 0)
        self.assertAlmostEqual(geom.signed_volume(CUBE_V, rev), -1.0, delta=1e-12)

    def test_U_GEOM_12(self):
        """Signed volume scales: a 2 x 3 x 4 box has volume +24; a single open quad (one face) is not judged inconsistent (edges have 1 face)."""
        pts = [(2 * x, 3 * y, 4 * z) for x, y, z in CUBE_V]
        self.assertAlmostEqual(geom.signed_volume(pts, CUBE_F), 24.0, delta=1e-9)
        self.assertEqual(geom.winding_inconsistencies([(0, 1, 2, 3)]), 0)


class Evidence(unittest.TestCase):
    def test_U_EVID_01(self):
        """sha256 of known bytes matches hashlib; ledger append keeps earlier lines byte-identical."""
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "f.bin")
            with open(p, "wb") as f:
                f.write(b"411")
            self.assertEqual(evidence.sha256_file(p), hashlib.sha256(b"411").hexdigest())
            led = os.path.join(d, "ledger.jsonl")
            evidence.append_ledger(led, {"run_id": "a"})
            first = open(led, "rb").read()
            evidence.append_ledger(led, {"run_id": "b"})
            both = open(led, "rb").read()
            self.assertTrue(both.startswith(first))
            self.assertEqual(len(both.splitlines()), 2)
            self.assertEqual(evidence.clean(float("inf")), "inf")

    def test_U_RULE_01(self):
        """Findings: MEASURED_FACT on an EXPERIMENTAL method is rejected; qualification downgrades on failed or missing tests."""
        base = dict(title="t", rule_category="C_ENGINEERING_DIAGNOSTIC", severity="LOW",
                    evidence_class="MEASURED_FACT", affected=[], measured={}, method_ids=["M"],
                    method_status="EXPERIMENTAL", threshold=None, threshold_status="n/a",
                    interpretation="", confidence_and_limits="", next_action="",
                    owner_judgement_required=False, id="X-001")
        with self.assertRaises(ValueError):
            rules.validate_finding(base)
        cfg = {"methods": {"A": {"declared_status": "QUALIFIED", "validated_by": ["T1"]},
                           "B": {"declared_status": "QUALIFIED", "validated_by": ["T2"]},
                           "C": {"declared_status": "QUALIFIED", "validated_by": ["T3"]}}}
        q = rules.qualify_methods(cfg, {"T1": "PASS", "T2": "FAIL"})
        self.assertEqual((q["A"]["status"], q["B"]["status"], q["C"]["status"]),
                         ("QUALIFIED", "FAILED_VALIDATION", "EXPERIMENTAL"))


class Gate(unittest.TestCase):
    CFG_UNRES = {"coordinate_convention": {"status": "OWNER_DECISION_REQUIRED",
                                           "side_dependent_adjudication_allowed": False,
                                           "outcome": "B"}}
    CFG_RES = {"coordinate_convention": {"status": "RESOLVED", "side_dependent_adjudication_allowed": True,
                                         "outcome": "A"}}

    def test_U_GATE_01(self):
        """Unresolved convention: gated() returns UNRESOLVED without calling fn; require raises; anatomical_side is UNRESOLVED."""
        called = []
        r = gate.gated(self.CFG_UNRES, "op", lambda: called.append(1))
        self.assertEqual((r["status"], called), ("UNRESOLVED", []))
        with self.assertRaises(gate.SideConventionUnresolved):
            gate.require_side_convention(self.CFG_UNRES, "op")
        self.assertEqual(gate.anatomical_side(self.CFG_UNRES, "L", "-X", -1), "UNRESOLVED")

    def test_U_GATE_02(self):
        """Resolved convention: fn runs; right_sign_x=-1 (facing -Y) makes -X RIGHT and +X LEFT; midline stays midline; a half-set config (flag true, status not RESOLVED) still refuses."""
        r = gate.gated(self.CFG_RES, "op", lambda: 41 + 1)
        self.assertEqual((r["status"], r["result"]), ("OK", 42))
        self.assertEqual(gate.anatomical_side(self.CFG_RES, "L", "-X", -1), "RIGHT")
        self.assertEqual(gate.anatomical_side(self.CFG_RES, "L", "+X", -1), "LEFT")
        self.assertEqual(gate.anatomical_side(self.CFG_RES, None, "midline", -1), "midline")
        self.assertEqual(gate.anatomical_side(self.CFG_RES, "L", "+X", None), "UNRESOLVED")
        half = {"coordinate_convention": {"status": "OWNER_DECISION_REQUIRED",
                                          "side_dependent_adjudication_allowed": True}}
        self.assertFalse(gate.side_adjudication_allowed(half))


class Glb(unittest.TestCase):
    def test_U_GLB_01(self):
        """GLB round trip: skin joints [DEF_A, MECH_B, Prop] summarise as DEF:1 MECH:1 OTHER:1; bad magic and wrong length are rejected."""
        g = {"asset": {"version": "2.0"},
             "nodes": [{"name": "DEF_A"}, {"name": "MECH_B"}, {"name": "Prop"}],
             "skins": [{"joints": [0, 1, 2]}], "meshes": [{}], "animations": []}
        data = glb.build_glb(g)
        got = glb.skin_summary(glb.read_glb_json(data))
        self.assertEqual(got["skins"][0]["layers"], {"DEF": 1, "MECH": 1, "OTHER": 1})
        self.assertEqual((got["skin_count"], got["node_count"]), (1, 3))
        with self.assertRaises(ValueError):
            glb.read_glb_json(b"XXXX" + data[4:])
        with self.assertRaises(ValueError):
            glb.read_glb_json(data[:-4])


def _fake_run(root, name, sha, metric, methods, findings, tests):
    d = os.path.join(root, name)
    os.makedirs(d)
    evidence.write_json(os.path.join(d, "run_manifest.json"),
                        {"run_id": name, "qa_version": "t", "source_after": {"sha256": sha}})
    evidence.write_json(os.path.join(d, "findings.json"), {"findings": findings})
    evidence.write_json(os.path.join(d, "measurement_methods.json"),
                        {"methods": {k: {"status": v} for k, v in methods.items()}})
    evidence.write_json(os.path.join(d, "geometry.json"),
                        {"metrics": [{"metric": "h", "value": metric, "unit": "m"}]})
    evidence.write_json(os.path.join(d, "test_results.json"),
                        {"tests": [{"id": k, "result": v} for k, v in tests.items()]})
    return d


class Compare(unittest.TestCase):
    def test_U_CMP_01(self):
        """Run comparison: identical runs -> NO_REGRESSIONS; same source hash but metric moved 0.01 -> NONDETERMINISM; method downgrade, new HIGH finding and a newly failing test are each flagged; a changed source hash is reported but a metric change then is not nondeterminism."""
        f = lambda t, sev: {"title": t, "severity": sev, "evidence_class": "MEASURED_FACT"}  # noqa: E731
        with tempfile.TemporaryDirectory() as root:
            a = _fake_run(root, "a", "H1", 2.0, {"M": "QUALIFIED"}, [f("x", "LOW")], {"T": "PASS"})
            b_same = _fake_run(root, "b_same", "H1", 2.0, {"M": "QUALIFIED"}, [f("x", "LOW")], {"T": "PASS"})
            r = compare.compare(compare.load_run(a), compare.load_run(b_same))
            self.assertEqual((r["verdict"], r["regressions"]), ("NO_REGRESSIONS", []))
            b_bad = _fake_run(root, "b_bad", "H1", 2.01, {"M": "EXPERIMENTAL"},
                              [f("x", "LOW"), f("y", "HIGH")], {"T": "FAIL"})
            kinds = sorted(x["kind"] for x in compare.compare(compare.load_run(a), compare.load_run(b_bad))["regressions"])
            self.assertEqual(kinds, ["METHOD_DOWNGRADE", "NEW_HIGH_FINDING", "NONDETERMINISM", "TEST_REGRESSION"])
            b_src = _fake_run(root, "b_src", "H2", 2.01, {"M": "QUALIFIED"}, [f("x", "LOW")], {"T": "PASS"})
            rs = compare.compare(compare.load_run(a), compare.load_run(b_src))
            self.assertFalse(rs["same_source_hash"])
            self.assertEqual(rs["regressions"], [])
            self.assertEqual(rs["changes"][0]["kind"], "source_changed")
            with self.assertRaises(ValueError):
                compare.load_run(root)


def result_id(test):
    name = test.id().split(".")[-1]          # test_U_GEOM_01
    return name[len("test_"):].replace("_", "-")


def main(out_path=None):
    from humanoid_qa.tests import test_intersect, test_refine, test_solid
    suite = unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__]),
                                unittest.defaultTestLoader.loadTestsFromModule(test_intersect),
                                unittest.defaultTestLoader.loadTestsFromModule(test_solid),
                                unittest.defaultTestLoader.loadTestsFromModule(test_refine)])
    records = []

    class Recorder(unittest.TextTestResult):
        def addSuccess(self, t):
            super().addSuccess(t)
            records.append((t, "PASS", ""))

        def addFailure(self, t, err):
            super().addFailure(t, err)
            records.append((t, "FAIL", self._exc_info_to_string(err, t)))

        def addError(self, t, err):
            super().addError(t, err)
            records.append((t, "ERROR", self._exc_info_to_string(err, t)))

    runner = unittest.TextTestRunner(verbosity=2, resultclass=Recorder)
    res = runner.run(suite)
    out = {"suite": "pure_unit", "python": sys.version.split()[0],
           "tests": [{"id": result_id(t), "purpose": (t.shortDescription() or ""),
                      "result": r, "detail": d} for t, r, d in records]}
    if out_path:
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=2)
    return res.wasSuccessful()


if __name__ == "__main__":
    ok = main(sys.argv[1] if len(sys.argv) > 1 else None)
    sys.exit(0 if ok else 1)
