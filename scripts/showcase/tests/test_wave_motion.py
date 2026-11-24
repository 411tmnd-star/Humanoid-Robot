"""Pure tests for the hello-wave loop and the committed rig data. No Blender needed."""
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1]))
import wave_motion as W  # noqa: E402

ROOT = HERE.parents[3]
RIG = ROOT / "proof" / "rig_hierarchy_v20.json"


class WaveMotion(unittest.TestCase):
    def test_every_control_stays_inside_its_documented_limit(self):
        self.assertEqual(W.outside_limits(), [])

    def test_loop_closes_and_starts_at_rest(self):
        self.assertEqual(W.wave_angles(W.WAVE_FRAMES), W.wave_angles(0))
        self.assertTrue(all(abs(v) < 1e-12 for v in W.wave_angles(0).values()))

    def test_no_jump_between_frames(self):
        worst = max(abs(W.wave_angles(f + 1)[k] - W.wave_angles(f)[k]) for f in range(W.WAVE_FRAMES) for k in W.WAVE_POSE)
        self.assertLess(worst, 12.0)  # degrees per frame at 30 fps

    def test_the_hand_really_waves(self):
        elbow = [W.wave_angles(f)["CTRL_Elbow"] for f in range(W.RAISE_END, W.LOWER_START + 1)]
        signs = [e > 112 for e in elbow if abs(e - 112) > 1e-6]
        crossings = sum(a != b for a, b in zip(signs, signs[1:]))
        self.assertGreaterEqual(crossings, 2 * W.CYCLES - 1)
        self.assertGreater(max(elbow) - min(elbow), 25)

    def test_a_limit_violation_is_detected(self):
        keep = W.WAVE_POSE["CTRL_Elbow"]
        try:
            W.WAVE_POSE["CTRL_Elbow"] = 128
            self.assertTrue(any("CTRL_Elbow" in b for b in W.outside_limits()))
        finally:
            W.WAVE_POSE["CTRL_Elbow"] = keep

    def test_smooth_is_flat_at_both_ends(self):
        self.assertEqual((W.smooth(0), W.smooth(1), W.smooth(-3), W.smooth(9)), (0.0, 1.0, 0.0, 1.0))
        self.assertLess(W.smooth(0.01), 1e-4)


class RigData(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.doc = json.loads(RIG.read_text(encoding="utf-8"))
        cls.bones = {b["name"]: b for b in cls.doc["bones"]}

    def test_layer_counts(self):
        self.assertEqual(self.doc["bone_count"], 184)
        self.assertEqual(self.doc["layers"], {"DEF": 56, "MECH": 68, "CTRL": 60})

    def test_only_def_bones_deform(self):
        self.assertTrue(all(b["deform"] == (b["layer"] == "DEF") for b in self.doc["bones"]))

    def test_one_root_and_no_missing_parent(self):
        self.assertEqual([b["name"] for b in self.doc["bones"] if b["parent"] is None], ["DEF_Pelvis"])
        self.assertTrue(all(b["parent"] in self.bones for b in self.doc["bones"] if b["parent"]))

    def test_wave_limits_equal_the_rig_limits(self):
        for bone, (axis, lo, hi) in W.LIMITS.items():
            self.assertEqual([round(v) for v in self.bones[bone]["limits_deg"][axis]], [lo, hi], bone)

    def test_left_and_right_heads_mirror(self):
        pairs = [(b, b[:-2]) for b in self.bones if b.startswith("DEF_") and b.endswith("_L")]
        self.assertEqual(len(pairs), 26)
        worst = max(abs(self.bones[a]["head"][0] + self.bones[c]["head"][0]) for a, c in pairs)
        self.assertLess(worst, 0.002)


if __name__ == "__main__":
    unittest.main()
