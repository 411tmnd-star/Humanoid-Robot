"""v17_verify.py -- SCRIPT_VERSION 1.0

Purpose: Phase 4 verification of robot_armature_v17.blend against v16 (docs/ENGINEERING_RULES.md section 13).
Headless (exit 0 = every hard check passed, 1 = a hard check failed; the Markdown report is printed either way):
    python3 -I scripts/validation/v17_verify.py V16.blend V17.blend [REPORT.md]

Hard checks (fail the run): 56 DEF bones, 0 non-DEF deform bones, wrist frames mirror exactly, DEF_Wrist rest frame
== MECH_Wrist_Dev rest frame (zero deviation at zero command), mirrored wrist command -> mirrored hand motion,
thumb QA-003 mirror preserved, taper / neck / pelvis / hand-scale targets met, mesh<->bone alignment scaled 1.65x.
Reported but NOT hard-failed: the finger-chain zero-command deviation (pre-existing, outside this change).
"""
import math
import os
import subprocess
import sys

import bpy
from mathutils import Matrix, Quaternion, Vector

SCRIPT_VERSION = "1.0"
ARM = "RobotArmature"
Sx = Matrix.Diagonal((-1, 1, 1))
FAILS = []


def load(path):
    bpy.ops.wm.open_mainfile(filepath=path)
    arm = bpy.data.objects[ARM]
    arm.animation_data.action = None
    for pb in arm.pose.bones:
        pb.location = (0, 0, 0)
        pb.rotation_euler = (0, 0, 0)
        pb.rotation_quaternion = (1, 0, 0, 0)
    arm.data.pose_position = "POSE"
    bpy.context.view_layer.update()
    return arm


def W(o):
    M = o.matrix_world
    return [M @ v.co for v in o.data.vertices]


def frame(arm, name):
    b = arm.data.bones[name]
    M = arm.matrix_world
    m = M.to_3x3() @ b.matrix_local.to_3x3()
    return M @ b.head_local, M @ b.tail_local, m.col[0].normalized(), m.col[1].normalized(), m.col[2].normalized()


def ang(a, b):
    return math.degrees(a.angle(b))


def check(ok, msg):
    if not ok:
        FAILS.append(msg)
    return "PASS" if ok else "**FAIL**"


def rings(o):
    P = W(o)
    zmid = (min(p.z for p in P) + max(p.z for p in P)) / 2
    out = []
    for sel in ([p for p in P if p.z > zmid], [p for p in P if p.z <= zmid]):
        c = sum(sel, Vector()) / len(sel)
        out.append(sum((p - c).length for p in sel) / len(sel))
    return out  # (proximal, distal)


def deviation(arm, name):
    pb = arm.pose.bones[name]
    Mw = arm.matrix_world.to_3x3()
    cur, rest = Mw @ pb.matrix.to_3x3(), Mw @ pb.bone.matrix_local.to_3x3()
    return math.degrees((cur @ rest.inverted()).to_quaternion().angle)


def mirror_residual(arm, ctrl_r, ctrl_l, axis, th, def_r, def_l, sign):
    for pb in arm.pose.bones:
        pb.rotation_euler = (0, 0, 0)
    arm.pose.bones[ctrl_r].rotation_euler[axis] = th
    arm.pose.bones[ctrl_l].rotation_euler[axis] = sign * th
    bpy.context.view_layer.update()
    Mw = arm.matrix_world.to_3x3()

    def delta(n):
        pb = arm.pose.bones[n]
        return (Mw @ pb.matrix.to_3x3()) @ (Mw @ pb.bone.matrix_local.to_3x3()).inverted()

    R, L = delta(def_r), delta(def_l)
    res = math.degrees((Sx @ R @ Sx).to_quaternion().rotation_difference(L.to_quaternion()).angle)
    for pb in arm.pose.bones:
        pb.rotation_euler = (0, 0, 0)
    bpy.context.view_layer.update()
    return res


def metrics(path):
    arm = load(path)
    m = {}
    B = arm.data.bones
    m["def_count"] = sum(1 for b in B if b.name.startswith("DEF_"))
    m["nondef_deform"] = sum(1 for b in B if not b.name.startswith("DEF_") and b.use_deform)
    # wrist frames
    wr = {}
    for n in ("DEF_Wrist", "MECH_Wrist_Flex", "MECH_Wrist_Dev", "CTRL_Wrist_Flex", "CTRL_Wrist_Dev"):
        fr, fl = frame(arm, n), frame(arm, n + "_L")
        rr = Bone_roll(arm, n)
        rl = Bone_roll(arm, n + "_L")
        wr[n] = dict(
            roll_R=rr, roll_L=rl, len_R=B[n].length, len_L=B[n + "_L"].length,
            head=(Sx @ fr[0] - fl[0]).length, tail=(Sx @ fr[1] - fl[1]).length,
            y=ang(Sx @ fr[3], fl[3]), x=ang(Sx @ fr[2], fl[2]), z=ang(-(Sx @ fr[4]), fl[4]),
            lock_R=tuple(arm.pose.bones[n].lock_rotation), lock_L=tuple(arm.pose.bones[n + "_L"].lock_rotation))
    m["wrist"] = wr
    for s in ("", "_L"):
        a, b = frame(arm, "DEF_Wrist" + s), frame(arm, "MECH_Wrist_Dev" + s)
        m["wframe" + s] = max(ang(a[i], b[i]) for i in (2, 3, 4))
    # behaviour: mirrored wrist command -> mirrored DEF_Wrist motion (sign that mirrors, residual in degrees)
    beh = {}
    for label, cr, cl, ax, th in (("Flex X +30", "CTRL_Wrist_Flex", "CTRL_Wrist_Flex_L", 0, math.radians(30)),
                                   ("Dev Z +15", "CTRL_Wrist_Dev", "CTRL_Wrist_Dev_L", 2, math.radians(15))):
        beh[label] = {s: mirror_residual(arm, cr, cl, ax, th, "DEF_Wrist", "DEF_Wrist_L", s) for s in (1, -1)}
    # ankle control, known-good convention
    beh["Ankle Dorsi X +20 (control)"] = {s: mirror_residual(arm, "CTRL_Ankle_Dorsi", "CTRL_Ankle_Dorsi_L", 0, math.radians(20),
                                                              "DEF_Foot", "DEF_Foot_L", s) for s in (1, -1)}
    m["behaviour"] = beh
    m["dev"] = {n: deviation(arm, n) for n in ("DEF_Wrist", "DEF_Wrist_L", "DEF_Palm_Index", "DEF_Palm_Index_L")}
    fingers = [b.name for b in B if b.name.startswith("DEF_") and any(k in b.name for k in ("_MCP", "_PIP", "_Tip", "Thumb_"))]
    m["finger_dev_max"] = max(deviation(arm, n) for n in fingers)
    m["finger_dev_n"] = len(fingers)
    # thumb mirror (QA-003)
    th = 0.0
    for n in ("DEF_Thumb_CMC", "DEF_Thumb_MCP", "DEF_Thumb_IP"):
        a, b = frame(arm, n), frame(arm, n + "_L")
        th = max(th, (Sx @ a[0] - b[0]).length)
    m["thumb_mirror_err"] = th
    # geometry
    for key, nm in (("forearm", "411_Forearm_R"), ("shin", "411_Shin_R")):
        p, d = rings(bpy.data.objects[nm])
        m[key] = (p, d, d / p)
    neck = W(bpy.data.objects["411_Neck"])
    m["neck_r"] = max(math.hypot(p.x, p.y) for p in neck)
    sh = W(bpy.data.objects["411_PelvicShell"])
    m["shell_x"] = max(abs(p.x) for p in sh)
    m["shell_y"] = max(abs(p.y) for p in sh)
    dg = bpy.context.evaluated_depsgraph_get()
    arm.data.pose_position = "REST"
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    to = bpy.data.objects["411_Torso"].evaluated_get(dg)
    me = to.to_mesh()
    band = [v.co for v in me.vertices if 0.96 <= v.co.z <= 1.00]
    m["torso_x"] = max(abs(p.x) for p in band)
    m["torso_y"] = max(abs(p.y) for p in band)
    to.to_mesh_clear()
    yk = W(bpy.data.objects["411_Yoke"])
    m["yoke"] = (max(abs(p.x) for p in yk), max(abs(p.y) for p in yk), max(p.z for p in yk) - min(p.z for p in yk))
    foot = W(bpy.data.objects["411_Foot_R"])
    m["foot"] = (max(p.x for p in foot) - min(p.x for p in foot), max(p.y for p in foot) - min(p.y for p in foot),
                 max(p.z for p in foot), min(p.z for p in foot))
    # hand: extent from the wrist pivot, and mesh<->bone alignment
    arm.data.pose_position = "REST"
    bpy.context.view_layer.update()
    piv = arm.matrix_world @ B["DEF_Wrist"].head_local
    hand_objs = [o for o in bpy.data.objects if o.type == "MESH" and o.name.endswith(("_R", "_R_A", "_R_B")) and
                 o.name.startswith(("411_Palm_", "411_Finger", "411_Thumb_", "411_ThumbJoint_"))]
    pts = [p for o in hand_objs for p in W(o)]
    m["hand_reach"] = max((p - piv).length for p in pts)
    m["hand_width"] = (max(p.x for p in pts) - min(p.x for p in pts), max(p.y for p in pts) - min(p.y for p in pts))
    m["hand_n"] = len(hand_objs)
    # all 22 hand meshes (both sides): distance from mesh centroid to the head of the bone it is skinned to
    al = {}
    for o in bpy.data.objects:
        if o.type == "MESH" and o.name.startswith(("411_Palm_", "411_Finger", "411_Thumb_", "411_ThumbJoint_")):
            vg = o.vertex_groups[0].name
            pts_o = W(o)
            c = sum(pts_o, Vector()) / len(pts_o)
            al[o.name] = (c - arm.matrix_world @ B[vg].head_local).length
    m["hand_align"] = al
    bl = [(arm.matrix_world @ B[n].tail_local - arm.matrix_world @ B[n].head_local).length
          for n in ("DEF_Index_MCP", "DEF_Index_PIP", "DEF_Thumb_MCP", "DEF_Palm_Little")]
    m["bone_len"] = bl
    wrist_mesh = W(bpy.data.objects["411_Wrist_R"])
    m["wrist_sphere_r"] = (max(p.x for p in wrist_mesh) - min(p.x for p in wrist_mesh)) / 2
    m["nmesh"] = sum(1 for o in bpy.data.objects if o.type == "MESH" and o.name.startswith("411_"))
    return m


def Bone_roll(arm, name):
    b = arm.data.bones[name]
    return math.degrees(b.AxisRollFromMatrix(b.matrix_local.to_3x3())[1])


def main(p16, p17, out):
    a, b = metrics(p16), metrics(p17)
    L = []
    P = L.append
    P(f"# v17 verification  (verify script {SCRIPT_VERSION}, Blender {bpy.app.version_string})\n")
    P("## 1. Rig integrity\n")
    P("| check | v16 | v17 | result |\n|---|---|---|---|")
    P(f"| DEF bones | {a['def_count']} | {b['def_count']} | {check(b['def_count'] == 56, 'DEF count')} |")
    P(f"| non-DEF bones with use_deform | {a['nondef_deform']} | {b['nondef_deform']} | {check(b['nondef_deform'] == 0, 'non-DEF deform')} |")
    P(f"| thumb QA-003 mirror error, max over CMC/MCP/IP head (m) | {a['thumb_mirror_err']:.6f} | {b['thumb_mirror_err']:.6f} | {check(b['thumb_mirror_err'] < 1e-4, 'thumb mirror')} |")
    P(f"| robot meshes (411_*) | {a['nmesh']} | {b['nmesh']} | +2 new (visor glass, visor strip) |")
    P("\n## 2. Wrist: L vs R, world space (mirror across X; convention x_L=S x_R, y_L=S y_R, z_L=-S z_R)\n")
    P("`Roll` is Blender's bone roll in armature space (deg). Angle columns are L vs mirrored-R, in degrees; head/tail in metres.\n")
    P("| bone | roll R | roll L | length R | length L | head err | tail err | y err | x err | z err | locks R | locks L |")
    P("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for tag, mm in (("v16", a), ("v17", b)):
        for n, w in mm["wrist"].items():
            P(f"| {tag} {n} | {w['roll_R']:.4f} | {w['roll_L']:.4f} | {w['len_R']:.4f} | {w['len_L']:.4f} | {w['head']:.5f} | {w['tail']:.5f} | "
              f"{w['y']:.3f} | {w['x']:.3f} | {w['z']:.3f} | {tuple(int(x) for x in w['lock_R'])} | {tuple(int(x) for x in w['lock_L'])} |")
    for n, w in b["wrist"].items():
        check(w["head"] < 1e-4 and w["tail"] < 1e-4 and max(w["x"], w["y"], w["z"]) < 0.01, f"wrist mirror {n}")
        check(w["lock_R"] == w["lock_L"] or n.startswith(("DEF", "CTRL")), f"wrist lock flags {n}")
    P("\n| rest-frame match, DEF_Wrist vs MECH_Wrist_Dev (max axis angle, deg) | v16 | v17 | result |\n|---|---|---|---|")
    for s, lab in (("", "R"), ("_L", "L")):
        P(f"| {lab} | {a['wframe' + s]:.3f} | {b['wframe' + s]:.5f} | {check(b['wframe' + s] < 0.01, 'wframe' + s)} |")
    P("\n| zero-command rest deviation (deg) | v16 | v17 | result |\n|---|---|---|---|")
    for n in ("DEF_Wrist", "DEF_Wrist_L", "DEF_Palm_Index", "DEF_Palm_Index_L"):
        P(f"| {n} | {a['dev'][n]:.2f} | {b['dev'][n]:.4f} | {check(b['dev'][n] < 0.01, 'dev ' + n)} |")
    P("\n| mirrored wrist command: residual of L hand motion vs mirrored R (deg) | L cmd = +th | L cmd = -th | best |\n|---|---|---|---|")
    for lab in b["behaviour"]:
        va, vb = a["behaviour"][lab], b["behaviour"][lab]
        P(f"| v16 {lab} | {va[1]:.2f} | {va[-1]:.2f} | {min(va.values()):.2f} |")
        P(f"| v17 {lab} | {vb[1]:.2f} | {vb[-1]:.2f} | {min(vb.values()):.2f} |")
    for lab in ("Flex X +30", "Dev Z +15"):
        check(min(b["behaviour"][lab].values()) < 0.5, "mirrored wrist behaviour " + lab)
    P("\n## 3. Silhouette: before vs after\n")
    P("| quantity | v16 | v17 | target | result |\n|---|---|---|---|---|")
    f_a, f_b = a["forearm"], b["forearm"]
    P(f"| Forearm radius, proximal (elbow end) m | {f_a[0]:.4f} | {f_b[0]:.4f} | unchanged | {check(abs(f_b[0]-f_a[0]) < 1e-4, 'forearm top')} |")
    P(f"| Forearm radius, distal (wrist end) m | {f_a[1]:.4f} | {f_b[1]:.4f} | 0.65-0.75 x proximal | |")
    P(f"| **Forearm distal / proximal** | {f_a[2]:.3f} | {f_b[2]:.3f} | 0.65-0.75 | {check(0.65 <= f_b[2] <= 0.75, 'forearm ratio')} |")
    s_a, s_b = a["shin"], b["shin"]
    P(f"| Shin radius, proximal (knee end) m | {s_a[0]:.4f} | {s_b[0]:.4f} | unchanged | {check(abs(s_b[0]-s_a[0]) < 1e-4, 'shin top')} |")
    P(f"| Shin radius, distal (ankle end) m | {s_a[1]:.4f} | {s_b[1]:.4f} | 0.65-0.75 x proximal | |")
    P(f"| **Shin distal / proximal** | {s_a[2]:.3f} | {s_b[2]:.3f} | 0.65-0.75 | {check(0.65 <= s_b[2] <= 0.75, 'shin ratio')} |")
    P(f"| Neck outer radius m | {a['neck_r']:.4f} | {b['neck_r']:.4f} | -45..-50 % | {check(0.50 <= b['neck_r']/a['neck_r'] <= 0.55, 'neck ratio')} |")
    P(f"| **Neck ratio v17/v16** | 1.000 | {b['neck_r']/a['neck_r']:.3f} | 0.50-0.55 | |")
    P(f"| Yoke half-width x / half-depth y / height z (m) | {a['yoke'][0]:.3f} / {a['yoke'][1]:.3f} / {a['yoke'][2]:.3f} | {b['yoke'][0]:.3f} / {b['yoke'][1]:.3f} / {b['yoke'][2]:.3f} | tighter, less bulk | {check(b['yoke'][2] < a['yoke'][2] and b['yoke'][0] < a['yoke'][0], 'yoke')} |")
    P(f"| Pelvis shell half-width x (m) | {a['shell_x']:.3f} | {b['shell_x']:.3f} | <= torso waist x | |")
    P(f"| Torso waist half-width x at z 0.96-1.00 (m) | {a['torso_x']:.3f} | {b['torso_x']:.3f} | reference | |")
    P(f"| **Pelvis x / Torso x** | {a['shell_x']/a['torso_x']:.3f} | {b['shell_x']/b['torso_x']:.3f} | <= 1.0 | {check(b['shell_x'] <= b['torso_x'], 'pelvis x')} |")
    P(f"| Pelvis shell half-depth y (m) | {a['shell_y']:.3f} | {b['shell_y']:.3f} | <= torso waist y | |")
    P(f"| Torso waist half-depth y (m) | {a['torso_y']:.3f} | {b['torso_y']:.3f} | reference | |")
    P(f"| **Pelvis y / Torso y** | {a['shell_y']/a['torso_y']:.3f} | {b['shell_y']/b['torso_y']:.3f} | <= 1.0 | {check(b['shell_y'] <= b['torso_y'], 'pelvis y')} |")
    P("\n## 4. End effectors\n")
    P("| quantity | v16 | v17 | target | result |\n|---|---|---|---|---|")
    hs = b["hand_reach"] / a["hand_reach"]
    P(f"| Hand reach from wrist pivot, max over hand meshes (m) | {a['hand_reach']:.4f} | {b['hand_reach']:.4f} | | |")
    P(f"| **Hand scale (reach v17/v16)** | 1.000 | {hs:.4f} | 1.65 | {check(abs(hs-1.65) < 0.005, 'hand scale')} |")
    for i, n in enumerate(("DEF_Index_MCP", "DEF_Index_PIP", "DEF_Thumb_MCP", "DEF_Palm_Little")):
        r = b["bone_len"][i] / a["bone_len"][i]
        P(f"| {n} bone length ratio (v17/v16) | {a['bone_len'][i]:.4f} | {b['bone_len'][i]:.4f} | 1.65 | {check(abs(r-1.65) < 0.005, 'bone scale ' + n)} |")
    ratios = {n: b["hand_align"][n] / a["hand_align"][n] for n in a["hand_align"] if a["hand_align"][n] > 1e-4}
    worst = max(ratios.items(), key=lambda kv: abs(kv[1] - 1.65))
    P(f"| Mesh-centroid-to-bone-head distance ratio v17/v16, all {len(ratios)} hand meshes (L and R), min / max | | {min(ratios.values()):.3f} / {max(ratios.values()):.3f} | 1.65 (worst: {worst[0]}) | {check(len(ratios) >= 20 and abs(worst[1]-1.65) < 0.02, 'hand mesh/bone alignment (all sides)')} |")
    P(f"| Hand width x (m) / hand mesh count (R) | {a['hand_width'][0]:.3f} / {a['hand_n']} | {b['hand_width'][0]:.3f} / {b['hand_n']} | | |")
    P(f"| Hand span vs wrist-tube diameter at wrist (distal r x2) | {a['hand_width'][0]/(2*a['forearm'][1]):.2f} | {b['hand_width'][0]/(2*b['forearm'][1]):.2f} | | |")
    P(f"| Foot footprint x (width) / y (length) m | {a['foot'][0]:.3f} / {a['foot'][1]:.3f} | {b['foot'][0]:.3f} / {b['foot'][1]:.3f} | length = 1.4 x 0.1618 = 0.2265 | {check(abs(b['foot'][1]-0.22652) < 0.003, 'foot length')} |")
    P(f"| Foot z top / sole z (m) | {a['foot'][2]:.3f} / {a['foot'][3]:.3f} | {b['foot'][2]:.3f} / {b['foot'][3]:.4f} | flat sole z=0 | {check(abs(b['foot'][3]) < 1e-6, 'sole flat')} |")
    P("\n## 5. Known defect NOT fixed here (pre-existing, reported)\n")
    P("DEF finger / thumb roots copy rotation from their MECH driver in POSE space (absolute orientation), but the MECH finger "
      "bones share one common rack orientation unrelated to each finger. At zero command the finger chain is therefore rotated "
      "away from its rest orientation:\n")
    P(f"| | v16 | v17 |\n|---|---|---|\n| max rest deviation over {a['finger_dev_n']} DEF finger/thumb bones (deg) | {a['finger_dev_max']:.1f} | {b['finger_dev_max']:.1f} |")
    P("\nStatus: **BLOCKED** (needs a spec-level change to `specifications/hand/fingers.md` axes; see docs/phase_v17_refine.md).")
    P("\n## Overall\n")
    P("**" + ("ALL HARD CHECKS PASSED" if not FAILS else "FAILED: " + "; ".join(FAILS)) + "**")
    text = "\n".join(L)
    print(text)
    if out:
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        with open(out, "w") as f:
            f.write(text + "\n")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    args = [x for x in sys.argv[1:] if not x.endswith(".py")]
    main(args[0], args[1], args[2] if len(args) > 2 else None)
