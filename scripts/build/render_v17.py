"""render_v17.py -- SCRIPT_VERSION 1.0

Purpose: diagnostic renders, warehouse hidden. Design shots use the neutral REST pose (the geometry as authored).
The extra shot "zerocmd" (front view) uses POSE with every pose channel reset: it shows what the rig itself does at
zero command and exists to document the finger-frame defect (see docs/phase_v17_refine.md).
Headless:  python3 -I scripts/build/render_v17.py BLEND OUT_DIR [SAMPLES] [shots,comma,separated]
Shots: front, threeq, side, head, hand, zerocmd.  Cycles CPU, denoising on.  Never saves the .blend.
"""
import os
import sys

import bpy
from mathutils import Vector

SCRIPT_VERSION = "1.0"

SHOTS = {
    # name: (camera location, look-at target, focal length mm, (width, height))
    "front":  ((0.0, -6.5, 1.15), (0.0, 0.0, 1.15), 70, (900, 1300)),
    "threeq": ((4.6, -5.2, 1.45), (0.0, 0.0, 1.15), 70, (900, 1300)),
    "side":   ((7.0, 0.0, 1.15), (0.0, 0.0, 1.15), 70, (900, 1300)),
    "head":   ((0.95, -2.3, 2.05), (0.0, -0.03, 1.98), 85, (1000, 1000)),
    "hand":   ((-1.05, -1.45, 0.98), (-0.38, 0.0, 0.93), 85, (1000, 1000)),
    "zerocmd": ((0.0, -6.5, 1.15), (0.0, 0.0, 1.15), 70, (900, 1300)),
}


def main(blend, out_dir, samples, names):
    if not os.path.exists(blend):
        raise RuntimeError(f"not found: {blend}")
    bpy.ops.wm.open_mainfile(filepath=blend)
    sc = bpy.context.scene
    for o in bpy.data.objects:
        if o.name.startswith(("ENV_", "CINE_")):
            o.hide_render = True
            o.hide_viewport = True
    arm = bpy.data.objects["RobotArmature"]
    arm.animation_data.action = None
    for pb in arm.pose.bones:
        pb.location = (0, 0, 0)
        pb.rotation_euler = (0, 0, 0)
        pb.rotation_quaternion = (1, 0, 0, 0)
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = samples
    sc.cycles.use_denoising = True
    sc.render.image_settings.file_format = "PNG"
    cd = bpy.data.cameras.new("SnapCam")
    cd.clip_end = 100
    cam = bpy.data.objects.new("SnapCam", cd)
    sc.collection.objects.link(cam)
    sc.camera = cam
    os.makedirs(out_dir, exist_ok=True)
    for n in names:
        loc, tgt, lens, res = SHOTS[n]
        arm.data.pose_position = "POSE" if n == "zerocmd" else "REST"
        cd.lens = lens
        sc.render.resolution_x, sc.render.resolution_y = res
        cam.location = Vector(loc)
        cam.rotation_euler = (Vector(tgt) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
        sc.render.filepath = os.path.join(out_dir, n + ".png")
        bpy.ops.render.render(write_still=True)


if __name__ == "__main__":
    a = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    a = [x for x in a if not x.endswith(".py")]
    main(a[0], a[1], int(a[2]) if len(a) > 2 else 128, a[3].split(",") if len(a) > 3 else list(SHOTS))
