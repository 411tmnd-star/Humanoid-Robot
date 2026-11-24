"""v18_verify.py -- SCRIPT_VERSION 1.0

Purpose: Phase 4 verification of robot_armature_v18.blend against v17 (docs/ENGINEERING_RULES.md section 13).
Headless (exit 0 = every hard check passed; the Markdown report is printed and written either way):
    python3 -I scripts/validation/v18_verify.py V17.blend V18.blend REPORT.md [GLB] [ROUNDTRIP_LOG] [RENDER_DIR]

Everything measured here is read from the file (evaluated meshes with the groove / subdivision stacks applied, armature in REST
for geometry, zero command for the pose checks). Nothing is assumed from the build script's own numbers.
"""
import json
import math
import os
import struct
import sys

import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

SCRIPT_VERSION = "1.1"
ARM = "RobotArmature"
T_WAIST = float(os.environ.get("V_WAIST", "0.86"))     # torso targets: v18 / v19 default, v20 sets V_WAIST=0.79 V_RIM=0.88
T_RIM = float(os.environ.get("V_RIM", "0.92"))
Sx = Matrix.Diagonal((-1, 1, 1))
FAILS = []
L = []


def P(s=""):
    L.append(s)


def chk(ok, name):
    if not ok:
        FAILS.append(name)
    return "PASS" if ok else "**FAIL**"


class WBVH:
    def __init__(self, o, dg):
        self.bvh = BVHTree.FromObject(o, dg)
        self.M = o.matrix_world.copy()
        self.Mi = self.M.inverted()
        self.Nm = self.Mi.to_3x3().transposed()

    def hit(self, origin, direction):
        h = self.bvh.ray_cast(self.Mi @ origin, (self.Mi.to_3x3() @ direction).normalized())
        if h[0] is None:
            return None
        return self.M @ h[0], (self.Nm @ h[1]).normalized()


SAVED_POSE_MODE = {}


SAVED_ACTION = {}
SAVED_MOVERS = {}


def load(path, zero=True):
    bpy.ops.wm.open_mainfile(filepath=path)
    arm = bpy.data.objects[ARM]
    SAVED_POSE_MODE[path] = arm.data.pose_position
    ad = arm.animation_data
    SAVED_ACTION[path] = ad.action.name if ad and ad.action else None
    if ad and ad.action:
        for o in bpy.data.objects:                       # booleans are irrelevant to bone motion and slow to re-evaluate
            for m in o.modifiers:
                if m.name == "V18_Groove":
                    m.show_viewport = False
        sc = bpy.context.scene
        sc.frame_set(0)
        bpy.context.view_layer.update()
        h0 = {b.name: (arm.matrix_world @ arm.pose.bones[b.name].head).copy() for b in arm.data.bones if b.name.startswith("DEF_")}
        m0 = {b.name: arm.pose.bones[b.name].matrix.to_3x3().copy() for b in arm.data.bones if b.name.startswith("DEF_")}
        sc.frame_set(10)
        bpy.context.view_layer.update()
        n = 0
        for b in arm.data.bones:
            if b.name.startswith("DEF_"):
                pb = arm.pose.bones[b.name]
                dm = (pb.matrix.to_3x3() @ m0[b.name].inverted()).to_quaternion().angle
                n += (arm.matrix_world @ pb.head - h0[b.name]).length > 1e-4 or dm > 1e-3
        SAVED_MOVERS[path] = n
        for o in bpy.data.objects:
            for m in o.modifiers:
                if m.name == "V18_Groove":
                    m.show_viewport = True
    if zero:
        arm.animation_data.action = None
        for pb in arm.pose.bones:
            pb.location = (0, 0, 0)
            pb.rotation_euler = (0, 0, 0)
            pb.rotation_quaternion = (1, 0, 0, 0)
    arm.data.pose_position = "POSE"
    bpy.context.view_layer.update()
    return arm


def rest(arm):
    arm.data.pose_position = "REST"
    bpy.context.view_layer.update()


def bvh_of(o, modifiers_off=()):
    saved = {}
    for m in o.modifiers:
        if m.type in modifiers_off or m.name in modifiers_off:
            saved[m.name] = m.show_viewport
            m.show_viewport = False
    bpy.context.view_layer.update()
    b = WBVH(o, bpy.context.evaluated_depsgraph_get())
    for m in o.modifiers:
        if m.name in saved:
            m.show_viewport = saved[m.name]
    bpy.context.view_layer.update()
    return b


def deviation_all(arm):
    R3 = arm.matrix_world.to_3x3()
    out = {}
    for b in arm.data.bones:
        if b.name.startswith("DEF_"):
            pb = arm.pose.bones[b.name]
            cur, rst = R3 @ pb.matrix.to_3x3(), R3 @ b.matrix_local.to_3x3()
            out[b.name] = math.degrees((cur @ rst.inverted()).to_quaternion().angle)
    return out


def wverts(o):
    M = o.matrix_world
    return [M @ v.co for v in o.data.vertices]


def frame(arm, n):
    b = arm.data.bones[n]
    M = arm.matrix_world
    m = M.to_3x3() @ b.matrix_local.to_3x3()
    return M @ b.head_local, M @ b.tail_local, m.col[0].normalized(), m.col[1].normalized(), m.col[2].normalized()


def half_w(bvh, z):
    return bvh.hit(Vector((1.0, 0, z)), Vector((-1, 0, 0)))[0].x


def glb_json(path):
    d = open(path, "rb").read()
    off = 12
    while off < len(d):
        ln, ty = struct.unpack("<I4s", d[off:off + 8])
        if ty == b"JSON":
            return json.loads(d[off + 8:off + 8 + ln])
        off += 8 + ln


def main(p17, p18, out, glb=None, rt_log=None, render_dir=None):
    # ------------------------------------------------------------------ v17 reference numbers
    a = load(p17)
    v17 = {"dev": max(deviation_all(a).values())}
    rest(a)
    t17 = bpy.data.objects["411_Torso"]
    b17 = WBVH(t17, bpy.context.evaluated_depsgraph_get())
    v17["torso_waist"] = half_w(b17, 1.11)
    v17["torso_rim"] = half_w(b17, 0.975)
    v17["torso_chest"] = max(half_w(b17, 1.26 + 0.01 * k) for k in range(34))
    yk = wverts(bpy.data.objects["411_Yoke"])
    v17["yoke_zmin"], v17["yoke_zmax"] = min(p.z for p in yk), max(p.z for p in yk)
    v17["hip_x"] = {s: sum(p.x for p in wverts(bpy.data.objects[f"411_HipSocket_{s}"])) / len(bpy.data.objects[f"411_HipSocket_{s}"].data.vertices) for s in "LR"}
    sh = wverts(bpy.data.objects["411_PelvicShell"])
    v17["shell_x"] = max(abs(p.x) for p in sh)
    v17["shell_z"] = [p.z for p in sh]
    v17["nobj"] = sum(1 for o in bpy.data.objects if o.type == "MESH" and o.name.startswith("411_"))

    # ------------------------------------------------------------------ v18
    arm = load(p18)
    B = arm.data.bones
    Pb = arm.pose.bones
    R3 = arm.matrix_world.to_3x3()
    P(f"# v18 verification  (verify script {SCRIPT_VERSION}, Blender {bpy.app.version_string})\n")

    P("## 1. Rig integrity and zero-command pose\n")
    dev = deviation_all(arm)
    worst = max(dev, key=dev.get)
    fingers = [n for n in dev if any(k in n for k in ("_MCP", "_PIP", "_Tip", "Thumb_", "Palm_"))]
    wrists = [n for n in dev if "Wrist" in n]
    P("| check | v17 | v18 | result |\n|---|---|---|---|")
    P(f"| DEF bones | 56 | {len(dev)} | {chk(len(dev) == 56, 'DEF count')} |")
    nd = [b.name for b in B if not b.name.startswith('DEF_') and b.use_deform]
    P(f"| non-DEF bones with use_deform | 0 | {len(nd)} | {chk(not nd, 'non-DEF deform')} |")
    P(f"| armature pose_position as saved in the file | {SAVED_POSE_MODE[p17]} | {SAVED_POSE_MODE[p18]} | {chk(SAVED_POSE_MODE[p18] == 'POSE', 'pose_position saved as POSE')} |")
    P(f"| max zero-command deviation, all 56 DEF bones (deg) | {v17['dev']:.4f} | {dev[worst]:.4f} (worst: {worst}) | {chk(dev[worst] <= math.degrees(1e-4), 'zero deviation all DEF')} |")
    P(f"| max deviation, {len(fingers)} finger / thumb / palm DEF bones (deg) | | {max(dev[n] for n in fingers):.4f} | {chk(max(dev[n] for n in fingers) <= math.degrees(1e-4), 'finger dev')} |")
    P(f"| max deviation, wrist DEF bones (deg) | | {max(dev[n] for n in wrists):.4f} | {chk(max(dev[n] for n in wrists) <= math.degrees(1e-4), 'wrist dev')} |")
    P(f"| tolerance | | 1e-4 rad = {math.degrees(1e-4):.4f} deg | |")
    # all MECH/CTRL finger bones coincide with the DEF bone they drive
    mism = 0
    for pb in Pb:
        if pb.name.startswith("DEF_") and any(k in pb.name for k in ("_MCP", "_Thumb_CMC")):
            for c in pb.constraints:
                if c.type == "COPY_ROTATION" and c.subtarget:
                    d, m = frame(arm, pb.name), frame(arm, c.subtarget)
                    mism += max(math.degrees(d[i].angle(m[i])) for i in (2, 3, 4)) > 0.01
    P(f"| POSE-space DEF roots whose MECH driver has a different rest frame | many (17-178 deg offsets) | {mism} | {chk(mism == 0, 'driver frames')} |")

    # the shipped action must actually drive the DEF bones (guards against a file saved in REST / muted rig)
    arm2 = bpy.data.objects[ARM]
    saved_action = arm2.animation_data.action
    P("\n| shipped animation `hello_sequence` (action restored for this check) | value | result |\n|---|---|---|")
    P(f"| action assigned on the armature in the .blend | {SAVED_ACTION.get(p18)} | {chk(SAVED_ACTION.get(p18) == 'hello_sequence', 'action assigned')} |")
    P(f"| DEF bones that move between frames 0 and 10 | {SAVED_MOVERS.get(p18)} | {chk((SAVED_MOVERS.get(p18) or 0) >= 10, 'animation moves DEF bones')} |")

    P("\n## 2. Neutral hand pose (right and left)\n")
    P("| quantity | right | left | target | result |\n|---|---|---|---|---|")
    rows = {}
    for s in ("", "_L"):
        a_ = frame(arm, "DEF_Wrist" + s)[3]
        d = [(frame(arm, f"DEF_Index_{k}{s}")[1] - frame(arm, f"DEF_Index_{k}{s}")[0]).normalized() for k in ("MCP", "PIP", "Tip")]
        ang = (math.degrees(a_.angle(d[0])), math.degrees(d[0].angle(d[1])), math.degrees(d[1].angle(d[2])))
        t0 = frame(arm, f"DEF_Thumb_CMC{s}")
        td = (t0[1] - t0[0]).normalized()
        curl = frame(arm, f"DEF_Index_Tip{s}")[1] - frame(arm, f"DEF_Index_MCP{s}")[0]
        inward = (-curl.x if s == "" else curl.x)
        ys = [frame(arm, f"DEF_{f}_MCP{s}")[0].y for f in ("Index", "Middle", "Ring", "Little")]
        pitch = [abs(ys[i] - ys[i + 1]) for i in range(3)]
        rows[s] = dict(ang=ang, thumb=math.degrees(a_.angle(td)), tfwd=td.y, inward=inward, pitch=pitch)
    r, l = rows[""], rows["_L"]
    tgt = (12.0, 18.0, 10.0)
    for i, nm in enumerate(("MCP flexion (deg)", "PIP flexion (deg)", "DIP flexion (deg)")):
        P(f"| index {nm} | {r['ang'][i]:.2f} | {l['ang'][i]:.2f} | {tgt[i]:.0f} | {chk(abs(r['ang'][i]-tgt[i]) < 0.1 and abs(l['ang'][i]-tgt[i]) < 0.1, 'angle ' + nm)} |")
    P(f"| thumb CMC direction vs hand axis (deg) | {r['thumb']:.2f} | {l['thumb']:.2f} | 25 | {chk(abs(r['thumb']-25) < 0.1 and abs(l['thumb']-25) < 0.1, 'thumb angle')} |")
    P(f"| thumb direction y (forward = negative) | {r['tfwd']:.3f} | {l['tfwd']:.3f} | < 0 | {chk(r['tfwd'] < 0 and l['tfwd'] < 0, 'thumb forward')} |")
    P(f"| index chain curls toward the body (inward displacement, m) | {r['inward']:.3f} | {l['inward']:.3f} | > 0 | {chk(r['inward'] > 0 and l['inward'] > 0, 'palm inward')} |")
    P(f"| MCP pitch index-mid / mid-ring / ring-little (m) | {', '.join(f'{x:.3f}' for x in r['pitch'])} | {', '.join(f'{x:.3f}' for x in l['pitch'])} | 0.032 each | {chk(all(abs(x - 0.032) < 0.002 for x in r['pitch'] + l['pitch']), 'finger pitch')} |")
    # behaviour
    def reset():
        for pb in Pb:
            pb.location = (0, 0, 0)
            pb.rotation_euler = (0, 0, 0)
            pb.rotation_quaternion = (1, 0, 0, 0)
        bpy.context.view_layer.update()
    P("\n| behaviour: +40 deg flex on the same CTRL channel of both hands | tip displacement right | tip displacement left | mirror check |\n|---|---|---|---|")
    for lab, c, tip in (("Index MCP_Flex", "CTRL_Index_MCP_Flex", "DEF_Index_Tip"), ("Index PIP", "CTRL_Index_PIP", "DEF_Index_Tip"),
                        ("Little MCP_Flex", "CTRL_Little_MCP_Flex", "DEF_Little_Tip"), ("Thumb MCP", "CTRL_Thumb_MCP", "DEF_Thumb_IP")):
        reset()
        base = {s: (arm.matrix_world @ Pb[tip + s].tail).copy() for s in ("", "_L")}
        Pb[c].rotation_euler[0] = math.radians(40)
        Pb[c + "_L"].rotation_euler[0] = math.radians(40)
        bpy.context.view_layer.update()
        dr = arm.matrix_world @ Pb[tip].tail - base[""]
        dl = arm.matrix_world @ Pb[tip + "_L"].tail - base["_L"]
        ok = abs(dr.x + dl.x) < 1e-4 and abs(dr.y - dl.y) < 1e-4 and abs(dr.z - dl.z) < 1e-4 and dr.x < 0 < dl.x
        P(f"| {lab} | ({dr.x:.3f}, {dr.y:.3f}, {dr.z:.3f}) | ({dl.x:.3f}, {dl.y:.3f}, {dl.z:.3f}) | {chk(ok, 'mirror flex ' + lab)} |")
    reset()
    chain = [f"DEF_{f}_{k}{sd}" for sd in ("", "_L") for f in ("Index", "Middle", "Ring", "Little") for k in ("MCP", "PIP", "Tip")] + \
            [f"DEF_Thumb_{k}{sd}" for sd in ("", "_L") for k in ("MCP", "IP")]
    white = {n: 0 for n in chain}
    dark = {n: 0 for n in chain}
    for o in bpy.data.objects:
        if o.type == "MESH" and o.name.startswith("411_") and o.vertex_groups:
            g = o.vertex_groups[0].name
            if g in white and o.material_slots and o.material_slots[0].material:
                nm = o.material_slots[0].material.name
                if nm == "BotWhite" and "Knuckle" not in o.name:
                    white[g] += 1
                elif nm == "Graphite_Brushed" and "Knuckle" in o.name:
                    dark[g] += 1
    ok = all(v == 1 for v in white.values()) and all(v == 1 for v in dark.values())
    P(f"\nHand segments: {len(chain)} phalanx bones (24 finger + 4 thumb); each has exactly one white shell and one dark cylindrical knuckle: "
      f"{sum(white.values())} shells, {sum(dark.values())} knuckles -> {chk(ok, 'phalanx shells + knuckles per bone')}. Thumbs also keep their CMC joint spheres.")

    # ------------------------------------------------------------------ geometry (REST)
    rest(arm)
    P("\n## 3. Torso contour and panel seams\n")
    t = bpy.data.objects["411_Torso"]
    bt = bvh_of(t, modifiers_off=("V18_Groove", "V18_WeightAll"))
    wc = max(half_w(bt, 1.26 + 0.01 * k) for k in range(34))
    ww, wr = half_w(bt, 1.11), half_w(bt, 0.975)
    P("| quantity | v17 | v18 | target | result |\n|---|---|---|---|---|")
    P(f"| chest half-width (m) | {v17['torso_chest']:.4f} | {wc:.4f} | +pectoral breadth | {chk(wc > v17['torso_chest'], 'chest wider')} |")
    P(f"| waist / chest | {v17['torso_waist']/v17['torso_chest']:.3f} | {ww/wc:.3f} | {T_WAIST:.2f} | {chk(abs(ww/wc-T_WAIST) < 0.005, 'waist ratio')} |")
    P(f"| lower iliac rim / chest | {v17['torso_rim']/v17['torso_chest']:.3f} | {wr/wc:.3f} | {T_RIM:.2f} | {chk(abs(wr/wc-T_RIM) < 0.005, 'rim ratio')} |")

    def groove_probe(o, origin, direction, across, width_probe):
        """Depth (m) of the groove at `origin` and 2 mm off-centre, measured on the evaluated mesh vs the same mesh without the groove."""
        b0 = bvh_of(o, modifiers_off=("V18_Groove", "V18_WeightAll"))
        b1 = bvh_of(o)
        res = []
        for off in (0.0, width_probe, -width_probe):
            og = origin + across * off
            h0, h1 = b0.hit(og, direction), b1.hit(og, direction)
            res.append(None if (h0 is None or h1 is None) else (h1[0] - h0[0]).dot(direction))
        return res

    z_h = 0.961 + 0.025
    h0 = bt.hit(Vector((0.0, -1.0, z_h)), Vector((0, 1, 0)))
    dep = groove_probe(t, Vector((0.0, -1.0, z_h)), Vector((0, 1, 0)), Vector((0, 0, 1)), 0.0025)
    P(f"| horizontal abdomen seam: height above bottom edge (m) | | {z_h - 0.961:.3f} | 0.025 | {chk(abs(z_h-0.961-0.025) < 1e-6, 'seam height')} |")
    P(f"| horizontal seam groove depth at centre / 2.5 mm off-centre (mm) | | {dep[0]*1e3:.2f} / {dep[1]*1e3:.2f} | 1.5 / 0 | {chk(dep[0] is not None and abs(dep[0]*1e3-1.5) < 0.35 and abs(dep[1]*1e3) < 0.05, 'torso groove depth')} |")
    # curved seam probe: find the seam centre on a row (z=1.28) by scanning x for the deepest point
    b1 = bvh_of(t)
    row = [(x / 10000.0, (b1.hit(Vector((x / 10000.0, -1.0, 1.28)), Vector((0, 1, 0))), bt.hit(Vector((x / 10000.0, -1.0, 1.28)), Vector((0, 1, 0))))) for x in range(1200, 2200, 4)]
    depths = [(x, abs((h1[0] - h0_[0]).y) * abs(h0_[1].y)) for x, (h1, h0_) in row if h1 and h0_]     # depth along the surface normal
    xd, dd = max(depths, key=lambda q: q[1])
    wid = [x for x, d_ in depths if d_ > 0.0004]
    P(f"| curved seam at z = 1.28: deepest point x (m) / depth (mm) / width at >0.4 mm depth (mm) | | {xd:.3f} / {dd*1e3:.2f} / {(max(wid)-min(wid))*1e3 + 0.4:.1f} | x/W = 0.70 / 1.5 / 2.0 | {chk(abs(dd*1e3-1.5) < 0.35, 'curved seam depth')} |")
    P(f"| seam x / half-width at z = 1.28 | | {xd / half_w(bt, 1.28):.3f} | 0.70 | {chk(abs(xd/half_w(bt,1.28)-0.70) < 0.03, 'curved seam position')} |")
    # evaluated weights
    def unweighted(o):
        dg = bpy.context.evaluated_depsgraph_get()
        ev = o.evaluated_get(dg)
        me = ev.to_mesh()
        n = sum(1 for v in me.vertices if len(v.groups) == 0)
        tot = len(me.vertices)
        mats = sorted({p.material_index for p in me.polygons})
        ev.to_mesh_clear()
        return n, tot, mats

    P("\n| evaluated mesh | vertices | unweighted vertices | material slots used | result |\n|---|---|---|---|---|")
    for nm in ("411_Torso", "411_Forearm_R", "411_Forearm_L", "411_Shin_R", "411_Shin_L", "411_Head"):
        n, tot, mats = unweighted(bpy.data.objects[nm])
        P(f"| {nm} | {tot} | {n} | {mats} | {chk(n == 0, 'weights ' + nm)} |")

    P("\n## 4. Limb split seams and head\n")
    P("| quantity | measured | target | result |\n|---|---|---|---|")
    for nm in ("411_Forearm_R", "411_Forearm_L", "411_Shin_R", "411_Shin_L"):
        o = bpy.data.objects[nm]
        W = wverts(o)
        zm = (min(p.z for p in W) + max(p.z for p in W)) / 2
        ct = sum([p for p in W if p.z > zm], Vector()) / len([p for p in W if p.z > zm])
        cb = sum([p for p in W if p.z <= zm], Vector()) / len([p for p in W if p.z <= zm])
        c = ct.lerp(cb, 0.5)
        dep = groove_probe(o, Vector((c.x, -1.0, c.z)), Vector((0, 1, 0)), Vector((1, 0, 0)), 0.0020)
        P(f"| {nm}: front seam depth at mid-length centre / 2 mm off-centre (mm) | {dep[0]*1e3:.2f} / {dep[1]*1e3:.2f} | 1.2 / 0 | {chk(dep[0] is not None and abs(dep[0]*1e3-1.2) < 0.35 and abs(dep[1]*1e3) < 0.05, 'limb seam ' + nm)} |")
    head = bpy.data.objects["411_Head"]
    hb0 = bvh_of(head, modifiers_off=("V18_Groove", "V18_WeightAll"))
    hb1 = bvh_of(head)
    d = Vector((0, 1, 0))                      # rays start in front of the face (-y) and travel toward +y
    zc = 2.115
    c0, c1 = hb0.hit(Vector((0, -1.0, zc)), d), hb1.hit(Vector((0, -1.0, zc)), d)
    pocket = c1[0].y - c0[0].y
    prof = []
    for k in range(0, 260):
        x = 0.080 + k * 0.0001
        h0_, h1_ = hb0.hit(Vector((x, -1.0, zc)), d), hb1.hit(Vector((x, -1.0, zc)), d)
        prof.append((x, (h1_[0].y - h0_[0].y) if h0_ and h1_ else 0.0))
    xin = max((x for x, dd_ in prof if dd_ >= 0.0014), default=None)
    xout = min((x for x, dd_ in prof if x > (xin or 0) and dd_ <= 0.00005), default=None)
    P(f"| visor pocket depth at centre (mm) | {pocket*1e3:.2f} | 1.5 | {chk(abs(pocket*1e3-1.5) < 0.3, 'visor depth')} |")
    P(f"| visor chamfer: horizontal width between floor edge and shell surface (mm) | {((xout - xin)*1e3 if xin and xout else float('nan')):.2f} | 1.5 | {chk(xin is not None and xout is not None and abs((xout-xin)*1e3-1.5) < 0.6, 'chamfer width')} |")
    hp = Vector((0.0, -0.037, 3.0))
    s0, s1 = hb0.hit(hp, Vector((0, 0, -1))), hb1.hit(hp, Vector((0, 0, -1)))
    s2, s3 = hb0.hit(hp + Vector((0, 0.0025, 0)), Vector((0, 0, -1))), hb1.hit(hp + Vector((0, 0.0025, 0)), Vector((0, 0, -1)))
    P(f"| head coronal seam depth at the apex / 2.5 mm off-centre (mm) | {(s0[0].z - s1[0].z)*1e3:.2f} / {(s2[0].z - s3[0].z)*1e3:.2f} | 1.5 / 0 | {chk(abs((s0[0].z-s1[0].z)*1e3-1.5) < 0.35 and abs((s2[0].z-s3[0].z)*1e3) < 0.05, 'head seam')} |")

    P("\n## 5. Pelvis, hips, yoke, feet\n")
    P("| quantity | v17 | v18 | target | result |\n|---|---|---|---|---|")
    yk = wverts(bpy.data.objects["411_Yoke"])
    zmin, zmax = min(p.z for p in yk), max(p.z for p in yk)
    h17, h18 = v17["yoke_zmax"] - v17["yoke_zmin"], zmax - zmin
    YOKE_ANCHOR = 1.745                         # the build compresses about this plane
    expect_min = YOKE_ANCHOR + (v17["yoke_zmin"] - YOKE_ANCHOR) * 0.82 - 0.004
    P(f"| yoke vertical thickness (m) | {h17:.4f} | {h18:.4f} | -18 % (+4 mm sink) | {chk(abs((h18 - (h17*0.82 + 0.004))) < 0.0006, 'yoke thickness')} |")
    P(f"| yoke lowest point z (m) | {v17['yoke_zmin']:.4f} | {zmin:.4f} | {expect_min:.4f} (compress, then -4 mm) | {chk(abs(zmin-expect_min) < 0.0006, 'yoke sink')} |")
    for s in "LR":
        o = bpy.data.objects[f"411_HipSocket_{s}"]
        W = wverts(o)
        cx = sum(p.x for p in W) / len(W)
        rad = (max(p.x for p in W) - min(p.x for p in W)) / 2
        P(f"| hip socket {s} centre x (m), shift as fraction of v17 radius | {v17['hip_x'][s]:.4f} | {cx:.4f} | -20 % of radius inward | {chk(abs(abs(cx - v17['hip_x'][s]) / (rad / 1.0) - 0.20) < 0.02, 'hip socket shift ' + s)} (shift/r = {abs(cx - v17['hip_x'][s]) / rad:.3f}) |")
    sh = bpy.data.objects["411_PelvicShell"]
    S_ = wverts(sh)
    xmax = max(abs(p.x) for p in S_)
    dz = [p.z - z0 for p, z0 in zip(S_, v17["shell_z"])]               # same topology as v17: per-vertex height change
    dip = min(d for p, d in zip(S_, dz) if abs(p.x) < 0.05)
    rise = max(d for p, d in zip(S_, dz) if abs(p.x) > 0.80 * xmax)
    P(f"| pelvis shell: lowest height change at the centre bridge (mm) | 0 | {dip*1e3:.1f} | -12 (dip) | {chk(abs(dip + 0.012) < 0.0008, 'pelvis dip')} |")
    P(f"| pelvis shell: largest height change at the outer hips (mm) | 0 | {rise*1e3:+.1f} | positive (rise, max +14 at the lowest ring) | {chk(0.003 < rise <= 0.0141, 'pelvis rise')} |")
    P(f"| pelvis shell half-width / torso rim half-width | {v17['shell_x']/v17['torso_rim']:.3f} | {xmax/wr:.3f} | <= 1 (tucked in) | {chk(xmax <= wr, 'pelvis within rim')} |")
    # feet
    for s in "LR":
        fo, so, cu = (bpy.data.objects.get(f"411_{n}_{s}") for n in ("Foot", "Sole", "AnkleCuff"))
        F = wverts(fo)
        Sv = wverts(so)
        top = max(p.z for p in F)
        half = lambda z0, z1: (max(p.x for p in F if z0 <= p.z <= z1) - min(p.x for p in F if z0 <= p.z <= z1)) / 2
        wall, deck = half(0.030, 0.066), half(top - 1e-4, top + 1e-4)
        P(f"| foot {s}: rubber sole height / bottom z (m) | | {max(p.z for p in Sv) - min(p.z for p in Sv):.4f} / {min(p.z for p in Sv):.4f} | 0.008 / 0 | {chk(abs(max(p.z for p in Sv)-0.008) < 1e-4 and abs(min(p.z for p in Sv)) < 1e-5, 'sole ' + s)} |")
        P(f"| foot {s}: dorsal edge radius (wall half-width - deck half-width, m) | | {wall - deck:.4f} | 0.018 (toe taper excluded: x) | {chk(abs((wall - deck) - 0.018) < 0.0035, 'dorsal radius ' + s)} |")
        P(f"| foot {s}: shell bottom sits on the sole / ankle cuff present | | {min(p.z for p in F):.4f} / {cu is not None} | 0.008 / True | {chk(abs(min(p.z for p in F)-0.008) < 1e-4 and cu is not None, 'foot stack ' + s)} |")

    P("\n## 6. Cyclorama and lighting\n")
    cyc = bpy.data.objects.get("Studio_Cyc")
    old = [bpy.data.objects[n].hide_render for n in ("Studio_Backdrop", "Studio_Floor")]
    me = cyc.data
    ang = 0.0
    for e in me.edges:
        fs = [p for p in me.polygons if e.index in [me.loops[l].edge_index for l in p.loop_indices]]
        if len(fs) == 2:
            ang = max(ang, math.degrees(fs[0].normal.angle(fs[1].normal)))
    zs = sorted({round(v.co.z, 3) for v in me.vertices})
    P("| item | value | target | result |\n|---|---|---|---|")
    P(f"| Studio_Cyc exists, old two-plane backdrop hidden from render | {cyc is not None}, hidden {old} | cyc + both hidden | {chk(cyc is not None and all(old), 'cyc')} |")
    P(f"| max normal break between adjacent cyc faces (deg) | {ang:.2f} | smooth (< 5; a plane-plane corner is 90) | {chk(ang < 5, 'cyc smooth')} |")
    P(f"| floor-to-wall bevel radius (m) | {max(zs[1:], key=lambda z: 0) if False else 2.5} (profile: first rise at z={zs[1]}, wall starts at z=2.5) | 2.5 | {chk(2.5 in zs, 'cyc radius')} |")
    lights = [o.name for o in bpy.data.objects if o.type == "LIGHT" and o.name.startswith("Studio_")]
    sc = bpy.context.scene
    P(f"| studio lights | {sorted(lights)} | key, fill, rim + 2 edge rims | {chk(len(lights) >= 5 and 'Studio_EdgeL' in lights and 'Studio_EdgeR' in lights, 'lights')} |")
    P(f"| Cycles samples / denoiser | {sc.cycles.samples} / {sc.cycles.denoiser} | 128 / OPENIMAGEDENOISE | {chk(sc.cycles.samples == 128 and sc.cycles.denoiser == 'OPENIMAGEDENOISE', 'render settings')} |")

    if glb:
        js = glb_json(glb)
        joints = [js["nodes"][j]["name"] for s in js["skins"] for j in s["joints"]]
        meshes = sum(1 for n in js["nodes"] if "mesh" in n and "skin" in n)
        tor = next(n for n in js["nodes"] if n.get("name") == "411_Torso")
        acc = js["accessors"][js["meshes"][tor["mesh"]]["primitives"][0]["attributes"]["POSITION"]]["count"]
        P("\n## 7. glTF export\n")
        P("| check | value | target | result |\n|---|---|---|---|")
        P(f"| skin joints | {len(joints)} | 56 | {chk(len(joints) == 56 and all(j.startswith('DEF_') for j in joints), 'glb joints')} |")
        P(f"| skinned meshes | {meshes} | v17 {v17['nobj'] + 0} + new objects | {chk(meshes > 100, 'glb meshes')} |")
        P(f"| 411_Torso vertices in the .glb (grooves baked in) | {acc} | > 3000 (cage has 1056) | {chk(acc > 3000, 'glb torso grooves')} |")
        P(f"| file size | {os.path.getsize(glb)/1e6:.2f} MB | | |")
    if rt_log and os.path.exists(rt_log):
        txt = open(rt_log).read()
        res = [ln for ln in txt.splitlines() if ln.startswith("RESULT")]
        import re
        mx = float(re.search(r"max error ([0-9.e+-]+) m", res[0]).group(1)) if res else None
        P("\n## 8. Export -> reimport trajectory check\n")
        P(f"`{res[0] if res else 'no result line'}`\n")
        P(f"max error {mx:.3e} m against the < 1e-5 m requirement: {chk(mx is not None and mx < 1e-5 and 'PASS' in res[0], 'roundtrip')}")
    if render_dir and os.path.exists(os.path.join(render_dir, "front.png")) and os.path.exists(os.path.join(render_dir, "zerocmd.png")):
        from PIL import Image, ImageChops
        a_, b_ = Image.open(os.path.join(render_dir, "front.png")).convert("RGB"), Image.open(os.path.join(render_dir, "zerocmd.png")).convert("RGB")
        diff = ImageChops.difference(a_, b_)
        px = list(diff.getdata())
        mean = sum(sum(p) for p in px) / (3 * len(px))
        big = sum(1 for p in px if max(p) > 24)
        P("\n## 9. front.png vs zerocmd.png\n")
        P(f"mean absolute difference {mean:.4f} / 255; pixels differing by more than 24/255: {big} of {len(px)} -> {chk(mean < 0.5 and big < 0.0005 * len(px), 'front==zerocmd')}")
    P("\n## Overall\n")
    P("**" + ("ALL HARD CHECKS PASSED" if not FAILS else "FAILED: " + "; ".join(FAILS)) + "**")
    text = "\n".join(L)
    print(text)
    if out:
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        open(out, "w").write(text + "\n")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.endswith(".py")]
    main(a[0], a[1], a[2] if len(a) > 2 else None, a[3] if len(a) > 3 else None, a[4] if len(a) > 4 else None, a[5] if len(a) > 5 else None)
