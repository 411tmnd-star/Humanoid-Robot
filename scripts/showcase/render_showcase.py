"""render_showcase.py -- portfolio renders of the v20 robot: hero still, six turntable panels, wave loop frames.

  blender -b --factory-startup --python-exit-code 3 --python scripts/showcase/render_showcase.py -- \
      --blend robot_armature_v20.blend --out out --device OPTIX
  add --dry-run to check limits, framing and loop closure without rendering anything.

Framing gate: the robot must fill 75 % to 82 % of the frame height in every shot. It is measured from projected
geometry before rendering, then measured again from a rendered alpha mask. A failed gate stops the run.
"""
import argparse
import json
import math
import os
import sys
import time

import bpy
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import showcase_scene as S  # noqa: E402
import wave_motion as W  # noqa: E402

AZIMUTHS = (0, 45, 90, 180, 270, 315)


def set_res(w, h, scale):
    r = bpy.context.scene.render
    r.resolution_x, r.resolution_y, r.resolution_percentage = w, h, int(round(100 * scale))


def set_device(device):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    if device == "CPU":
        sc.cycles.device = "CPU"
        return "CPU"
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = device
    prefs.refresh_devices()
    names = []
    for d in prefs.devices:
        d.use = d.type == device
        if d.use:
            names.append(d.name)
    if not names:
        raise RuntimeError(f"no {device} device found")
    sc.cycles.device = "GPU"
    return f"{device}: {', '.join(names)}"


def quality(samples):
    sc = bpy.context.scene
    sc.cycles.samples = samples
    sc.cycles.use_adaptive_sampling = True
    sc.cycles.adaptive_threshold = 0.01
    sc.cycles.use_denoising = True
    sc.cycles.denoiser = "OPENIMAGEDENOISE"
    if hasattr(sc.cycles, "denoising_use_gpu"):
        sc.cycles.denoising_use_gpu = sc.cycles.device == "GPU"
    sc.cycles.max_bounces = 8
    sc.render.use_persistent_data = True
    sc.render.image_settings.file_format = "PNG"
    sc.render.image_settings.color_mode = "RGB"
    sc.render.film_transparent = False


def still(path):
    bpy.context.scene.render.filepath = path
    t = time.time()
    bpy.ops.render.render(write_still=True)
    return round(time.time() - t, 2)


def mask_fill(path):
    """Second, independent framing check: render only the robot as alpha and measure its rows."""
    sc = bpy.context.scene
    r, cyc = sc.render, bpy.data.objects["Studio_Cyc"]
    keep = (r.resolution_percentage, sc.cycles.samples, sc.cycles.use_denoising)
    cyc.hide_render = True
    r.resolution_percentage, sc.cycles.samples, sc.cycles.use_denoising = max(8, int(keep[0] * 0.25)), 4, False
    r.film_transparent, r.image_settings.color_mode = True, "RGBA"
    still(path)
    img = bpy.data.images.load(path)
    w, h = img.size
    px = img.pixels[:]
    rows = [y for y in range(h) if any(px[(y * w + x) * 4 + 3] > 0.5 for x in range(w))]
    bpy.data.images.remove(img)
    cyc.hide_render = False
    r.resolution_percentage, sc.cycles.samples, sc.cycles.use_denoising = keep
    r.film_transparent, r.image_settings.color_mode = False, "RGB"
    return (rows[-1] - rows[0] + 1) / h if rows else 0.0


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument("--blend", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--jobs", default="hero,turntable,wave")
    ap.add_argument("--device", default="OPTIX", choices=["OPTIX", "CUDA", "CPU"])
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--hero-samples", type=int, default=512)
    ap.add_argument("--panel-samples", type=int, default=256)
    ap.add_argument("--wave-samples", type=int, default=64)
    ap.add_argument("--wave-step", type=int, default=1)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)
    jobs = [j for j in a.jobs.split(",") if j]
    os.makedirs(a.out, exist_ok=True)
    out = os.path.abspath(a.out)
    gates, timing = {}, {}
    rep = {"script_version": S.SCRIPT_VERSION, "blender": bpy.app.version_string, "blend": os.path.basename(a.blend),
           "dry_run": a.dry_run, "jobs": jobs, "gates": gates, "timing_s": timing}

    def save():
        with open(os.path.join(out, "showcase_report.json"), "w", encoding="utf-8", newline="\n") as f:
            json.dump(rep, f, indent=1)

    t0 = time.time()
    arm = S.open_robot(os.path.abspath(a.blend))
    sc = bpy.context.scene
    rep["bones"] = {p: sum(b.name.startswith(p) for b in arm.data.bones) for p in ("DEF_", "MECH_", "CTRL_")}
    rep["robot_meshes"] = len(S.robot_meshes())
    rep["frozen_seam_meshes"] = S.freeze_seams(arm)
    bad = S.check_limits(arm)
    gates["joint_limits_match_spec"] = {"pass": not bad, "detail": bad}
    bad = W.outside_limits()
    gates["wave_inside_limits"] = {"pass": not bad, "detail": bad,
                                   "range_deg": {k: [round(x, 2) for x in v] for k, v in W.angle_ranges().items()}}
    S.light_studio()
    rep["device"] = "dry-run" if a.dry_run else set_device(a.device)
    root = S.spin_root(arm)
    timing["setup"] = round(time.time() - t0, 2)

    # hero: three-quarter view at the top of the wave
    hero_pose = W.wave_angles(W.HERO_FRAME)
    hero_cam = S.make_camera("SC_Hero", lens=85)
    sc.camera = hero_cam
    set_res(1600, 2000, a.scale)
    S.apply_angles(arm, hero_pose)
    bpy.context.view_layer.update()
    gates["hero_framing_geometry"] = S.frame_robot(hero_cam, 32, 6)
    S.check_frame(gates["hero_framing_geometry"])

    # turntable: neutral pose, orthographic, the robot turns and the light stays
    S.reset_pose(arm)
    tt_cam = S.make_camera("SC_Turn", ortho=True)
    sc.camera = tt_cam
    set_res(900, 1200, a.scale)
    bpy.context.view_layer.update()
    S.frame_robot(tt_cam, 0, 5)
    panels = {}
    for az in AZIMUTHS:
        root.rotation_euler[2] = math.radians(az)
        bpy.context.view_layer.update()
        b = S.projected_box(tt_cam, S.robot_points())
        panels[str(az)] = {"x0": b[0], "x1": b[1], "y0": b[2], "y1": b[3], "fill_height": b[3] - b[2], "fill_width": b[1] - b[0]}
        S.check_frame(panels[str(az)])
    root.rotation_euler[2] = 0
    gates["turntable_framing_geometry"] = {"pass": all(p["pass"] for p in panels.values()), "panels": panels,
                                           "ortho_scale": tt_cam.data.ortho_scale}

    # wave loop: one fixed camera that frames the union of the motion
    act = S.key_wave(arm)
    sc.frame_start, sc.frame_end, sc.render.fps = 0, W.WAVE_FRAMES - 1, W.FPS
    wave_cam = S.make_camera("SC_Wave", lens=85)
    sc.camera = wave_cam
    set_res(720, 900, a.scale)
    union = []
    for f in (0, 10, 20, 24, W.HERO_FRAME, 40, 50, 60, 80):
        sc.frame_set(f)
        union.append(S.robot_points(step=9))
    gates["wave_framing_geometry"] = S.frame_robot(wave_cam, 14, 4, pts=np.concatenate(union))
    S.check_frame(gates["wave_framing_geometry"])
    sc.frame_set(0)
    start = {b: arm.pose.bones[b].matrix.translation.copy() for b in ("DEF_Wrist", "DEF_Head", "DEF_Middle_MCP")}
    sc.frame_set(W.WAVE_FRAMES)
    gap = max((arm.pose.bones[b].matrix.translation - p).length for b, p in start.items())
    gates["wave_loop_closes"] = {"pass": gap < 1e-5, "gap_m": gap}
    sc.frame_set(W.HERO_FRAME)
    travel = (arm.pose.bones["DEF_Middle_MCP"].matrix.translation - start["DEF_Middle_MCP"]).length
    gates["wave_hand_travel"] = {"pass": travel > 0.5, "metres": round(travel, 4)}

    rep["pass_pre_render"] = all(g["pass"] for g in gates.values())
    save()
    print("PRE-RENDER GATES", "PASS" if rep["pass_pre_render"] else "FAIL", json.dumps({k: g["pass"] for k, g in gates.items()}), flush=True)
    if not rep["pass_pre_render"]:
        raise SystemExit(4)
    if a.dry_run:
        return

    if "hero" in jobs:
        arm.animation_data.action = None
        S.reset_pose(arm)
        S.apply_angles(arm, hero_pose)
        sc.camera = hero_cam
        S.set_glow(-2.4)
        set_res(1600, 2000, a.scale)
        quality(a.hero_samples)
        mf = mask_fill(os.path.join(out, "_mask_hero.png"))
        gates["hero_framing_mask"] = {"pass": S.FILL_MIN <= mf <= S.FILL_MAX, "fill_height": round(mf, 4)}
        timing["hero"] = still(os.path.join(out, "hero_master.png"))
        save()
        print("DONE hero", timing["hero"], "s, mask fill", round(mf, 4), flush=True)
    if "turntable" in jobs:
        arm.animation_data.action = None
        S.reset_pose(arm)
        sc.camera = tt_cam
        S.set_glow(0.0)
        set_res(900, 1200, a.scale)
        quality(a.panel_samples)
        timing["turntable"] = {}
        for az in AZIMUTHS:
            root.rotation_euler[2] = math.radians(az)
            timing["turntable"][str(az)] = still(os.path.join(out, f"turn_{az:03d}.png"))
            print("DONE panel", az, flush=True)
        root.rotation_euler[2] = 0
        save()
    if "wave" in jobs:
        arm.animation_data.action = act
        sc.camera = wave_cam
        S.set_glow(-1.0)
        set_res(720, 900, a.scale)
        quality(a.wave_samples)
        os.makedirs(os.path.join(out, "wave"), exist_ok=True)
        t = time.time()
        for f in range(0, W.WAVE_FRAMES, a.wave_step):
            path = os.path.join(out, "wave", f"{f:04d}.png")
            if os.path.exists(path):
                continue  # resume after an interruption
            sc.frame_set(f)
            still(path)
            if f % 10 == 0:
                print("DONE wave frame", f, round(time.time() - t, 1), "s", flush=True)
        timing["wave"] = round(time.time() - t, 2)
    rep["pass"] = all(g["pass"] for g in gates.values())
    save()
    print("SHOWCASE", "PASS" if rep["pass"] else "FAIL", flush=True)
    if not rep["pass"]:
        raise SystemExit(5)


main()
