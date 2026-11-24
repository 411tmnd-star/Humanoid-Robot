"""v19_hand_check.py -- SCRIPT_VERSION 1.0
Purpose: measure the v19 hand refinement against v18: knuckle / shell diameters, thumb sphere size, skin weights on every hand mesh
(evaluated, so Subdivision / Bevel are included), crowding (knuckle length vs finger pitch).
Headless:  python3 -I scripts/validation/v19_hand_check.py V18.blend V19.blend OUT.md   (exit 1 on a failed check)
"""
import sys
import bpy
from mathutils import Vector

FAILS = []


def chk(ok, n):
    if not ok:
        FAILS.append(n)
    return "PASS" if ok else "**FAIL**"


def measure(path):
    bpy.ops.wm.open_mainfile(filepath=path)
    arm = bpy.data.objects["RobotArmature"]
    arm.data.pose_position = "POSE"
    R = {"kn": {}, "sh": {}, "tj": {}}
    for o in bpy.data.objects:
        if o.type != "MESH" or not o.name.startswith("411_"):
            continue
        W = [o.matrix_world @ v.co for v in o.data.vertices]
        ext = lambda axis: max(p[axis] for p in W) - min(p[axis] for p in W)
        if "Knuckle" in o.name:
            # diameter = extent perpendicular to the cylinder axis: smallest two principal extents ~ use bounding of the longest-axis removal
            e = sorted([ext(0), ext(1), ext(2)])
            R["kn"][o.name] = (e[0], e[2])        # approx diameter, approx length (axis-aligned bbox of an oblique cylinder: upper bound)
        elif o.name.startswith("411_ThumbJoint"):
            R["tj"][o.name] = max(ext(0), ext(1), ext(2))
    # exact knuckle radius from vertex distance to the cylinder axis: use bone x axis (flex axis) of the object's group
    for o in bpy.data.objects:
        if o.type == "MESH" and "Knuckle" in o.name:
            b = arm.data.bones[o.vertex_groups[0].name]
            M = arm.matrix_world
            ax = (M.to_3x3() @ b.matrix_local.to_3x3()).col[0].normalized()
            W = [o.matrix_world @ v.co for v in o.data.vertices]
            c = sum(W, Vector()) / len(W)
            ln = max(abs((p - c).dot(ax)) for p in W) * 2
            rad = max(((p - c) - ax * (p - c).dot(ax)).length for p in W)
            R["kn"][o.name] = (2 * rad, ln)
    # shell radius: same, around the bone's y axis
    for o in bpy.data.objects:
        if o.type == "MESH" and o.name.startswith("411_") and o.material_slots and o.material_slots[0].material and \
           o.material_slots[0].material.name == "BotWhite" and o.vertex_groups and \
           any(k in o.vertex_groups[0].name for k in ("_MCP", "_PIP", "_Tip", "Thumb_MCP", "Thumb_IP")) and "Forearm" not in o.name:
            b = arm.data.bones[o.vertex_groups[0].name]
            M = arm.matrix_world
            ay = (M.to_3x3() @ b.matrix_local.to_3x3()).col[1].normalized()
            W = [o.matrix_world @ v.co for v in o.data.vertices]
            c = sum(W, Vector()) / len(W)
            R["sh"][o.name] = 2 * max(((p - c) - ay * (p - c).dot(ay)).length for p in W)
    # evaluated skin weights
    dg = bpy.context.evaluated_depsgraph_get()
    bad, tot, objs = 0, 0, 0
    for o in list(R["kn"]) + list(R["sh"]) + list(R["tj"]):
        ev = bpy.data.objects[o].evaluated_get(dg)
        me = ev.to_mesh()
        bad += sum(1 for v in me.vertices if len(v.groups) == 0 or abs(sum(g.weight for g in v.groups) - 1.0) > 1e-4)
        tot += len(me.vertices)
        objs += 1
        ev.to_mesh_clear()
    R["w"] = (bad, tot, objs)
    return R


def main(p18, p19, out):
    a, b = measure(p18), measure(p19)
    L = ["# v19 hand refinement check\n", "| quantity | v18 | v19 | target | result |", "|---|---|---|---|---|"]
    kd18 = sum(v[0] for v in a["kn"].values()) / len(a["kn"])
    kd19 = sum(v[0] for v in b["kn"].values()) / len(b["kn"])
    red = 1 - kd19 / kd18
    L.append(f"| finger / thumb knuckle diameter, mean of {len(b['kn'])} (mm) | {kd18*1e3:.2f} | {kd19*1e3:.2f} | -35..-40 % | {chk(0.35 <= red <= 0.40, 'knuckle reduction')} ({-red*100:.1f} %) |")
    kdi = [b['kn'][n][0] / a['kn'][n][0] for n in b['kn']]
    L.append(f"| per-knuckle diameter ratio min / max | | {min(kdi):.3f} / {max(kdi):.3f} | 0.60-0.65 | {chk(all(0.60 <= r <= 0.65 for r in kdi), 'every knuckle in range')} |")
    kl18 = sum(v[1] for v in a["kn"].values()) / len(a["kn"])
    kl19 = sum(v[1] for v in b["kn"].values()) / len(b["kn"])
    sd18 = sum(a["sh"].values()) / len(a["sh"])
    sd19 = sum(b["sh"].values()) / len(b["sh"])
    L.append(f"| knuckle axial length, mean (mm) | {kl18*1e3:.1f} | {kl19*1e3:.1f} | < 32 mm finger pitch | {chk(kl19 < 0.030, 'no crowding')} |")
    L.append(f"| white shell diameter, mean of {len(b['sh'])} (mm) | {sd18*1e3:.2f} | {sd19*1e3:.2f} | slimmer | {chk(sd19 < sd18 * 0.9, 'shells slimmer')} ({(sd19/sd18-1)*100:+.1f} %) |")
    L.append(f"| knuckle diameter / shell diameter (the joint is now secondary) | {kd18/sd18:.2f} | {kd19/sd19:.2f} | < 1 | {chk(kd19 < sd19, 'joint smaller than shell')} |")
    t18, t19 = sum(a["tj"].values()) / 2, sum(b["tj"].values()) / 2
    L.append(f"| thumb CMC sphere diameter (mm) | {t18*1e3:.1f} | {t19*1e3:.1f} | -37.5 % | {chk(abs(t19 / t18 - 0.625) < 0.01, 'thumb sphere')} |")
    L.append(f"| hand meshes checked / evaluated vertices / unweighted or weight != 1 | | {b['w'][2]} / {b['w'][1]} / {b['w'][0]} | 0 | {chk(b['w'][0] == 0, 'hand weights')} |")
    L.append("\n**" + ("HAND CHECKS PASSED" if not FAILS else "FAILED: " + "; ".join(FAILS)) + "**")
    t = "\n".join(L)
    print(t)
    open(out, "w").write(t + "\n")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.endswith(".py")]
    main(a[0], a[1], a[2])
