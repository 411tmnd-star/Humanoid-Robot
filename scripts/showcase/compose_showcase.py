"""compose_showcase.py -- turn raw showcase renders into the files under proof/. Needs Pillow and ffmpeg, not Blender.

  python scripts/showcase/compose_showcase.py --renders out --proof proof [--allow-partial]

Writes 01_hero_scene.png, 02_turntable_sheet.png, 03_hello_wave_motion.mp4, 03_hello_wave_motion.webp and
compose_report.json (size, pixel dimensions, SHA-256 and gate for every file). Every committed image stays under 2 MB.
"""
import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

PNG_LIMIT = 1_900_000
WEBP_LIMIT = 1_800_000
FPS = 30
PANELS = (("turn_000.png", "0"), ("turn_045.png", "45"), ("turn_090.png", "90"),
          ("turn_180.png", "180"), ("turn_270.png", "270"), ("turn_315.png", "315"))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_png_under(img, path, limit):
    """Save as PNG, stepping the width down until the file fits. Returns the final size."""
    width = img.width
    while True:
        out = img if width == img.width else img.resize((width, round(img.height * width / img.width)), Image.LANCZOS)
        out.save(path, optimize=True)
        if path.stat().st_size <= limit or width <= 640:
            return out.size
        width = int(width * 0.92)


def turntable_sheet(renders):
    tiles = [Image.open(renders / n).convert("RGB") for n, _ in PANELS]
    w, h = tiles[0].size
    gap = max(4, w // 90)
    sheet = Image.new("RGB", (3 * w + 4 * gap, 2 * h + 3 * gap), (11, 14, 18))
    draw = ImageDraw.Draw(sheet)
    size = max(14, h // 30)
    try:
        font = ImageFont.load_default(size=size)
    except TypeError:
        font = ImageFont.load_default()
    for k, (tile, (_, label)) in enumerate(zip(tiles, PANELS)):
        x, y = gap + (k % 3) * (w + gap), gap + (k // 3) * (h + gap)
        sheet.paste(tile, (x, y))
        draw.text((x + w // 24, y + h // 30), f"{label}°", font=font, fill=(230, 237, 243))
    return sheet


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"{cmd[0]} failed: {r.stderr[-800:]}")


def wave_video(frames_dir, mp4):
    run(["ffmpeg", "-y", "-v", "error", "-framerate", str(FPS), "-i", str(frames_dir / "%04d.png"), "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", "-c:v", "libx264", "-preset", "slow", "-crf", "18", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)])


def wave_webp(frames, webp, limit):
    """Animated WebP, every second frame (15 fps), stepping quality then width down until it fits."""
    src = [Image.open(f).convert("RGB") for f in frames[::2]]
    for width in (600, 540, 480, 420, 360):
        ims = [i.resize((min(width, i.width), round(i.height * min(width, i.width) / i.width)), Image.LANCZOS) for i in src]
        for q in (82, 74, 66, 58):
            ims[0].save(webp, save_all=True, append_images=ims[1:], duration=round(2000 / FPS), loop=0, quality=q, method=4)
            if webp.stat().st_size < limit:
                return {"width": ims[0].width, "quality": q, "frames": len(ims)}
    raise RuntimeError("wave webp does not fit under the size limit")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--renders", required=True, type=Path)
    ap.add_argument("--proof", required=True, type=Path)
    ap.add_argument("--allow-partial", action="store_true", help="skip the missing-frame check (smoke tests only)")
    a = ap.parse_args()
    a.proof.mkdir(parents=True, exist_ok=True)
    rep = {"files": {}, "gates": {}}

    hero = a.proof / "01_hero_scene.png"
    save_png_under(Image.open(a.renders / "hero_master.png").convert("RGB"), hero, PNG_LIMIT)
    sheet = a.proof / "02_turntable_sheet.png"
    save_png_under(turntable_sheet(a.renders), sheet, PNG_LIMIT)

    frames = sorted((a.renders / "wave").glob("[0-9][0-9][0-9][0-9].png"))
    if len(frames) != 90 and not a.allow_partial:
        raise SystemExit(f"expected 90 wave frames, found {len(frames)}")
    mp4, webp = a.proof / "03_hello_wave_motion.mp4", a.proof / "03_hello_wave_motion.webp"
    if len(frames) == 90:
        wave_video(a.renders / "wave", mp4)
    else:
        tmp = a.renders / "_seq"
        tmp.mkdir(exist_ok=True)
        for i, f in enumerate(frames):
            shutil.copy2(f, tmp / f"{i:04d}.png")
        wave_video(tmp, mp4)
    rep["webp"] = wave_webp(frames, webp, WEBP_LIMIT)

    for p, limit in ((hero, 2_000_000), (sheet, 2_000_000), (webp, WEBP_LIMIT), (mp4, 2_000_000)):
        size = p.stat().st_size
        dims = None if p.suffix == ".mp4" else list(Image.open(p).size)
        rep["files"][p.name] = {"bytes": size, "sha256": sha256(p), "pixels": dims}
        rep["gates"][f"{p.name}_under_{limit}_bytes"] = size < limit
    rep["wave_source_frames"] = len(frames)
    rep["pass"] = all(rep["gates"].values())
    (a.proof / "compose_report.json").write_text(json.dumps(rep, indent=1) + "\n", encoding="utf-8", newline="\n")
    print("COMPOSE", "PASS" if rep["pass"] else "FAIL", json.dumps(rep["gates"]))
    if not rep["pass"]:
        raise SystemExit(6)


if __name__ == "__main__":
    main()
