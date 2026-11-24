"""v20_check.py -- SCRIPT_VERSION 1.0
Purpose: v20 (leaner midsection + light leg thinning) against v19. Proves (a) what was asked changed by the asked amount and
(b) everything that must not change is IDENTICAL: bones (head / tail / roll / constraints), hands, fingers, head, neck, feet, visor, yoke, hip sockets.
Headless:  python3 -I scripts/validation/v20_check.py V19.blend V20.blend OUT.md     (exit 1 on a failed check)
"""
import math
import sys

import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

FAILS = []
CHANGED = ("411_Torso", "411_PelvicShell", "411_WaistRing_New", "411_WaistRing_Backing", "411_Thigh_L", "411_Thigh_R", "411_Shin_L", "411_Shin_R")


def chk(ok, n):
    if not ok:
        FAILS.append(n)
    return "PASS" if ok else "**FAIL**"


def load(path):
    bpy.ops.wm.open_mainfile(filepath=path)
    arm = bpy.data.objects["RobotArmature"]
    D = {"bones": {}, "verts": {}, "cons": {}, "mods": {}}
    for b in arm.data.bones:
        D["bones"][b.name] = (tuple(b.head_local), tuple(b.tail_local), tuple(round(x, 7) for r in b.matrix_local.to_3x3() for x in r))
    for pb in arm.pose.bones:
        D["cons"][pb.name] = [(c.type, getattr(c, "subtarget", None), c.influence, tuple(round(getattr(c, a), 6) for a in ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z") if hasattr(c, a))) for c in pb.constraints] + [tuple(pb.lock_rotation)]
    for o in bpy.data.objects:
        if o.type == "MESH" and o.name.startswith("411_"):
            M = o.matrix_world
            D["verts"][o.name] = [tuple(M @ v.co) for v in o.data.vertices]
            D["mods"][o.name] = [m.type for m in o.modifiers]
    arm.data.pose_position = "REST"
    for o in bpy.data.objects:
        for m in o.modifiers:
            if m.name == "V18_Groove":
                m.show_viewport = False
    bpy.context.view_layer.update()
    t = bpy.data.objects["411_Torso"]
    b = BVHTree.FromObject(t, bpy.context.evaluated_depsgraph_get())
    hw = lambda z: b.ray_cast(Vector((1, 0, z)), Vector((-1, 0, 0)))[0].x
    dp = lambda z: -b.ray_cast(Vector((0, -1, z)), Vector((0, 1, 0)))[0].y
    wc = max(hw(1.26 + 0.01 * k) for k in range(34))
    zs = [0.97 + 0.01 * k for k in range(0, 40)]
    D["T"] = dict(wc=wc, ww=hw(1.11), wr=hw(0.975), wm=hw(1.20), dw=dp(1.11), dr=dp(0.975),
                  dc=max(dp(1.26 + 0.01 * k) for k in range(34)), zmin=min(zs, key=hw), prof=[hw(z) for z in zs])

    def ring(n):
        o = bpy.data.objects[n]
        W = [o.matrix_world @ v.co for v in o.data.vertices]
        zm = (min(p.z for p in W) + max(p.z for p in W)) / 2
        r = []
        for sel in ([p for p in W if p.z > zm], [p for p in W if p.z <= zm]):
            c = sum(sel, Vector()) / len(sel)
            r.append(2 * sum((p - c).length for p in sel) / len(sel))
        return r
    D["leg"] = {n: ring(n) for n in ("411_Thigh_R", "411_Thigh_L", "411_Shin_R", "411_Shin_L")}
    pts = [Vector(p) for n, v in D["verts"].items() for p in v]
    D["bbox"] = ([min(p[i] for p in pts) for i in range(3)], [max(p[i] for p in pts) for i in range(3)])
    sh = D["verts"]["411_PelvicShell"]
    D["shell"] = (max(abs(p[0]) for p in sh), max(abs(p[1]) for p in sh))
    D["sock"] = [sum(p[0] for p in D["verts"][f"411_HipSocket_{s}"]) / len(D["verts"][f"411_HipSocket_{s}"]) for s in "LR"]
    return D


def main(p19, p20, out):
    a, b = load(p19), load(p20)
    L = ["# v20 check (v20 vs v19)\n", "## 1. What changed, by how much\n", "| quantity | v19 | v20 | target | result |", "|---|---|---|---|---|"]
    ta, tb = a["T"], b["T"]
    L.append(f"| waist / chest at z 1.11 | {ta['ww']/ta['wc']:.3f} | {tb['ww']/tb['wc']:.3f} | 0.78-0.80 | {chk(0.78 <= tb['ww']/tb['wc'] <= 0.80, 'waist ratio in range')} |")
    L.append(f"| rim / chest at z 0.975 | {ta['wr']/ta['wc']:.3f} | {tb['wr']/tb['wc']:.3f} | below v19 (hips not widened) | {chk(tb['wr'] < ta['wr'], 'rim narrower')} |")
    L.append(f"| z of the narrowest torso width in 0.97-1.36 (m) | {ta['zmin']:.2f} | {tb['zmin']:.2f} | at the waist, 1.11 | {chk(abs(tb['zmin']-1.11) < 0.025, 'narrowest at waist')} |")
    mono_dn = all(tb['prof'][i] >= tb['prof'][i + 1] - 1e-4 for i in range(1, 14))   # z 0.98..1.11 narrowing (z 0.97 is inside the rounded hem edge)
    mono_up = all(tb['prof'][i] <= tb['prof'][i + 1] + 1e-4 for i in range(14, 33))  # z 1.11..1.30 widening (no re-swell before the chest)
    L.append(f"| width is monotone: narrowing 0.98 -> 1.11 and widening 1.11 -> 1.30 (no second bulge; hem bevel below z 0.98 excluded) | {'no' if ta['zmin'] > 1.15 else 'yes'} | {'yes' if mono_dn and mono_up else 'no'} | yes | {chk(mono_dn and mono_up, 'monotone taper')} |")
    L.append(f"| abdomen depth at waist / chest depth | {ta['dw']/ta['dc']:.3f} | {tb['dw']/tb['dc']:.3f} | flatter | {chk(tb['dw']/tb['dc'] < ta['dw']/ta['dc'] - 0.05, 'belly flatter')} |")
    L.append(f"| chest half-width (m) | {ta['wc']:.4f} | {tb['wc']:.4f} | unchanged (+-0.5 %) | {chk(abs(tb['wc']/ta['wc']-1) < 0.005, 'chest unchanged')} |")
    L.append(f"| pelvis shell half-width x / y (m) | {a['shell'][0]:.4f} / {a['shell'][1]:.4f} | {b['shell'][0]:.4f} / {b['shell'][1]:.4f} | not wider | {chk(b['shell'][0] <= a['shell'][0] + 1e-6 and b['shell'][1] <= a['shell'][1] + 1e-6, 'hips not widened')} |")
    for n in ("411_Thigh_R", "411_Thigh_L"):
        ra, rb = a["leg"][n], b["leg"][n]
        L.append(f"| {n} diameter top / bottom (mm) | {ra[0]*1e3:.1f} / {ra[1]*1e3:.1f} | {rb[0]*1e3:.1f} / {rb[1]*1e3:.1f} | -4 to -5 % | {chk(all(0.95 <= y/x <= 0.96 for x, y in zip(ra, rb)), 'thigh thinning ' + n)} ({(rb[0]/ra[0]-1)*100:+.1f} %, {(rb[1]/ra[1]-1)*100:+.1f} %) |")
    for n in ("411_Shin_R", "411_Shin_L"):
        ra, rb = a["leg"][n], b["leg"][n]
        L.append(f"| {n} diameter upper (top) / ankle end (mm) | {ra[0]*1e3:.1f} / {ra[1]*1e3:.1f} | {rb[0]*1e3:.1f} / {rb[1]*1e3:.1f} | upper -4..-5 %, ankle unchanged | {chk(0.95 <= rb[0]/ra[0] <= 0.96 and abs(rb[1]/ra[1]-1) < 1e-6, 'calf thinning ' + n)} ({(rb[0]/ra[0]-1)*100:+.1f} %) |")
    r = [b["leg"][n][1] / b["leg"][n][0] for n in ("411_Shin_R", "411_Shin_L")]
    L.append(f"| shin ankle / upper diameter (v17 taper rule 0.65-0.75) | {a['leg']['411_Shin_R'][1]/a['leg']['411_Shin_R'][0]:.3f} | {r[0]:.3f} | 0.65-0.75 | {chk(all(0.65 <= x <= 0.75 for x in r), 'shin taper rule')} |")
    L.append(f"| robot height / bbox min / max | {a['bbox'][1][2]:.3f} / {[round(x,3) for x in a['bbox'][0]]} / {[round(x,3) for x in a['bbox'][1]]} | {b['bbox'][1][2]:.3f} / {[round(x,3) for x in b['bbox'][0]]} / {[round(x,3) for x in b['bbox'][1]]} | height exact, extents within 0.5 mm | {chk(abs(a['bbox'][1][2] - b['bbox'][1][2]) < 1e-6 and max(abs(x - y) for x, y in zip(a['bbox'][0] + a['bbox'][1], b['bbox'][0] + b['bbox'][1])) < 5e-4, 'bbox')} (largest extent change {1e3*max(abs(x - y) for x, y in zip(a['bbox'][0] + a['bbox'][1], b['bbox'][0] + b['bbox'][1])):.2f} mm) |")
    L.append(f"| hip socket centre x L / R (m) | {a['sock'][0]:.4f} / {a['sock'][1]:.4f} | {b['sock'][0]:.4f} / {b['sock'][1]:.4f} | unchanged | {chk(all(abs(x - y) < 1e-9 for x, y in zip(a['sock'], b['sock'])), 'hip sockets unchanged')} |")

    L += ["\n## 2. What must NOT change (identical to v19)\n", "| set | compared | max difference | result |", "|---|---|---|---|"]
    bd = max(max(abs(x - y) for x, y in zip(a["bones"][n][0] + a["bones"][n][1] + a["bones"][n][2], b["bones"][n][0] + b["bones"][n][1] + b["bones"][n][2])) for n in a["bones"])
    L.append(f"| all {len(a['bones'])} bones (DEF/MECH/CTRL): head, tail, rest-frame axes | {len(a['bones'])} | {bd:.2e} | {chk(bd < 1e-9 and a['bones'].keys() == b['bones'].keys(), 'bones identical')} |")
    cm = sum(1 for n in a["cons"] if a["cons"][n] != b["cons"].get(n))
    L.append(f"| constraints, limits, influences, lock flags, all pose bones | {len(a['cons'])} | {cm} differences | {chk(cm == 0, 'constraints identical')} |")
    same = [n for n in a["verts"] if n not in CHANGED and not n.startswith("411_Visor_Glass") or n == "411_Visor_Glass"]
    unchanged = [n for n in a["verts"] if n not in CHANGED]
    worst, wn = 0.0, ""
    for n in unchanged:
        va, vb = a["verts"][n], b["verts"][n]
        if len(va) != len(vb):
            worst, wn = 9.9, n + " (vertex count)"
            break
        d = max((max(abs(x - y) for x, y in zip(p, q)) for p, q in zip(va, vb)), default=0.0)
        if d > worst:
            worst, wn = d, n
    L.append(f"| base vertices of the other {len(unchanged)} meshes (hands, fingers, head, neck, feet, visor, yoke, hip sockets, arms, ...) | {sum(len(a['verts'][n]) for n in unchanged)} | {worst:.2e} ({wn}) | {chk(worst < 1e-7, 'other meshes identical')} |")
    mm = [n for n in a["mods"] if a["mods"][n] != b["mods"].get(n)]
    L.append(f"| modifier stacks of all meshes | {len(a['mods'])} | {len(mm)} differences | {chk(not mm and a['mods'].keys() == b['mods'].keys(), 'modifier stacks identical')} |")
    L.append("\n**" + ("V20 CHECKS PASSED" if not FAILS else "FAILED: " + "; ".join(FAILS)) + "**")
    t = "\n".join(L)
    print(t)
    open(out, "w").write(t + "\n")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    x = [v for v in sys.argv[1:] if not v.endswith(".py")]
    main(x[0], x[1], x[2])
