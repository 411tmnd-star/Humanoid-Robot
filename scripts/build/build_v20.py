"""build_v20.py -- SCRIPT_VERSION 1.0

Purpose: derive blender/robot_armature_v20.blend (leaner midsection, lightly thinned legs). v17/v18/v19 files are never modified.
Headless:  python3 -I scripts/build/build_v20.py [DST.blend]

The grooves, pelvis fit, yoke and seams all depend on the torso surface, so v20 is rebuilt through the same deterministic pipeline rather than
patched on top of v19:   v17 --build_v18.py (new contour + leg thinning)--> temp --build_v19.py (hand joints)--> v20
Hands, fingers, head, neck, feet, bones, constraints and the studio are produced by exactly the same code as in v19.

v20 parameters (waist / chest 0.790, rim / chest 0.830, z = 1.20 / chest 0.810, abdomen depth x0.95 over z 1.0-1.28, thigh + upper calf x0.955).
"""
import os
import subprocess
import sys
import tempfile

import bpy

SCRIPT_VERSION = "1.0"
ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
B = os.path.join(ROOT, "scripts", "build")
PARAMS = ["--waist", "0.79", "--rim", "0.83", "--mid", "0.81", "--abdomen", "0.95", "--thin", "0.955"]


def run(args):
    r = subprocess.run([sys.executable, "-I"] + args, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"step failed: {args[:2]}\n{r.stdout[-1500:]}\n{r.stderr[-1500:]}")
    return r.stdout


def main(dst):
    with tempfile.TemporaryDirectory() as tmp:
        a, b = os.path.join(tmp, "v18x.blend"), os.path.join(tmp, "v19x.blend")
        run([os.path.join(B, "build_v18.py"), os.path.join(ROOT, "blender", "robot_armature_v17.blend"), a] + PARAMS)
        run([os.path.join(B, "build_v19.py"), a, b])
        bpy.ops.wm.open_mainfile(filepath=b)
        sc = bpy.context.scene
        arm = bpy.data.objects["RobotArmature"]
        if arm.data.pose_position != "POSE":
            raise RuntimeError("armature not in POSE mode")
        sc["v20_build_version"] = SCRIPT_VERSION
        sc["v20_params"] = " ".join(PARAMS)
        bpy.ops.wm.save_as_mainfile(filepath=dst)
    print("saved", dst)


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if x.endswith(".blend")]
    main(a[0] if a else os.path.join(ROOT, "blender", "robot_armature_v20.blend"))
