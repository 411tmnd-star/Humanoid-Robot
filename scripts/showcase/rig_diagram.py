"""rig_diagram.py -- draw the robot's joints and control layers from proof/rig_hierarchy_v20.json. No Blender needed.

  python scripts/showcase/rig_diagram.py proof/rig_hierarchy_v20.json proof/04_rig_diagram.png
"""
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

INK, SOFT, GRID, PAPER = "#1B2129", "#5B6673", "#D5DBE1", "#FFFFFF"
COLOR = {"DEF": "#1F6FEB", "MECH": "#B4591B", "CTRL": "#0E7C8C"}


def label(name):
    for p in ("MECH_", "DEF_", "CTRL_"):
        name = name.removeprefix(p)
    return name.replace("_", " ")


def degrees(v):
    return f"{v:+.0f}" if v else "0"


def main(src, dst):
    doc = json.loads(Path(src).read_text(encoding="utf-8"))
    bones = {b["name"]: b for b in doc["bones"]}
    fig = plt.figure(figsize=(14, 9), dpi=120, facecolor=PAPER)
    ax = fig.add_axes([0.02, 0.05, 0.30, 0.84])
    hand = fig.add_axes([0.335, 0.05, 0.20, 0.84])
    tab = fig.add_axes([0.565, 0.05, 0.42, 0.84])

    for b in doc["bones"]:
        if b["layer"] != "DEF":
            continue
        x, z = (b["head"][0], b["tail"][0]), (b["head"][2], b["tail"][2])
        ax.plot(x, z, color=COLOR["DEF"], lw=2.6, solid_capstyle="round", zorder=2)
        ax.plot(x[0], z[0], "o", ms=4.2, color=PAPER, mec=COLOR["DEF"], mew=1.4, zorder=3)
    ax.set_aspect("equal")
    ax.set_xlim(-0.75, 0.75)
    ax.set_ylim(-0.08, 2.45)
    ax.axis("off")
    ax.set_title(f"Deform skeleton, front view ({doc['layers']['DEF']} DEF bones)", color=INK, fontsize=11.5, loc="left")
    for bone, text, dx in (("DEF_Head", "Head", 0.14), ("DEF_Neck", "Neck", 0.14), ("DEF_UpperArm", "Shoulder", 0.12),
                           ("DEF_Elbow", "Elbow", 0.12), ("DEF_Wrist", "Wrist", 0.12), ("DEF_Thigh", "Hip", -0.42),
                           ("DEF_Shin", "Knee", 0.14), ("DEF_Foot", "Ankle", 0.14)):
        if bone in bones:
            h = bones[bone]["head"]
            ax.annotate(text, (h[0], h[2]), (h[0] + dx, h[2]), fontsize=9.5, color=INK, va="center",
                        arrowprops=dict(arrowstyle="-", color=GRID, lw=1), ha="left")

    fingers = [b for b in doc["bones"] if b["layer"] == "DEF" and not b["name"].endswith("_L")
               and any(k in b["name"] for k in ("Palm", "Thumb", "Index", "Middle", "Ring", "Little", "Wrist"))]
    for b in fingers:
        hand.plot((b["head"][1], b["tail"][1]), (b["head"][2], b["tail"][2]), color=COLOR["DEF"], lw=2.8, solid_capstyle="round", zorder=2)
        hand.plot(b["head"][1], b["head"][2], "o", ms=4.5, color=PAPER, mec=COLOR["DEF"], mew=1.4, zorder=3)
    hand.set_aspect("equal")
    hand.axis("off")
    hand.set_title(f"Right hand, side view ({len(fingers)} bones)", color=INK, fontsize=11.5, loc="left")

    rows = [b for b in doc["bones"] if b["layer"] == "MECH" and b["limits_deg"] and not b["name"].endswith("_L") and "_L_" not in b["name"]]
    tab.axis("off")
    tab.set_xlim(0, 1)
    n = len(rows)
    tab.set_ylim(n + 1.6, -0.6)
    tab.set_title("Actuated joints, right side (MECH layer, measured limits in degrees)", color=INK, fontsize=11.5, loc="left")
    tab.text(0.0, 0, "Joint", fontsize=9.5, color=SOFT, va="center")
    tab.text(0.40, 0, "Axis", fontsize=9.5, color=SOFT, va="center")
    tab.text(0.49, 0, "Range", fontsize=9.5, color=SOFT, va="center")
    for i, b in enumerate(rows, start=1):
        axis, (lo, hi) = next(iter(b["limits_deg"].items()))
        tab.plot((0, 1), (i - 0.5, i - 0.5), color=GRID, lw=0.6)
        tab.text(0.0, i, label(b["name"]), fontsize=9, color=INK, va="center")
        tab.text(0.42, i, axis.upper(), fontsize=9, color=INK, va="center")
        tab.text(0.49, i, f"{degrees(lo)} to {degrees(hi)}", fontsize=9, color=INK, va="center")
        x = lambda d: 0.70 + (d + 60) / 200 * 0.29  # noqa: E731  shared scale: -60 to +140 degrees
        tab.plot((x(-60), x(140)), (i, i), color=GRID, lw=2, solid_capstyle="round")
        tab.plot((x(max(lo, -60)), x(min(hi, 140))), (i, i), color=COLOR["MECH"], lw=4.5, solid_capstyle="round")
        tab.plot((x(0), x(0)), (i - 0.28, i + 0.28), color=INK, lw=0.8)
    tab.text(0.70, n + 1.1, "−60", fontsize=8, color=SOFT, ha="center")
    tab.text(0.70 + 60 / 200 * 0.29, n + 1.1, "0", fontsize=8, color=SOFT, ha="center")
    tab.text(0.99, n + 1.1, "+140", fontsize=8, color=SOFT, ha="center")

    lay = doc["layers"]
    fig.text(0.02, 0.945, f"411 humanoid rig: {doc['bone_count']} bones in three layers", fontsize=17, color=INK, weight="bold")
    fig.text(0.02, 0.915, f"CTRL ({lay['CTRL']}, operator)   →   MECH ({lay['MECH']}, joint type and limits)   →   DEF ({lay['DEF']}, skin and export)",
             fontsize=11.5, color=SOFT)
    fig.text(0.02, 0.015, f"Generated from {Path(src).name} (source {doc['source_blend']}, sha256 {doc['source_sha256'][:12]})", fontsize=8.5, color=SOFT)
    fig.savefig(dst, facecolor=PAPER)
    print("DIAGRAM", dst, Path(dst).stat().st_size, "bytes,", n, "joints")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
