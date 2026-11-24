"""build_v17.py -- SCRIPT_VERSION 1.0

Purpose: derive blender/robot_armature_v17.blend from v16 (never modifies v16).
Headless:  python3 -I scripts/build/build_v17.py [SRC.blend] [DST.blend]

Phases (all explicit bone / object names, no selection state; mesh edits are vertex/cage edits, no remesh):
  P1  Wrist frames.  v16 defect: DEF_Wrist{,_L} are zero-length bones with an arbitrary roll, and they copy
      MECH_Wrist_Dev in POSE space (an ABSOLUTE orientation copy).  Their rest frame did not equal the MECH
      frame, so at zero command the whole hand (palm, thumb, fingers) was rotated 113 deg (R) / 87 deg (L)
      about the wrist.  Fix: give DEF_Wrist a real length and the exact rest frame of MECH_Wrist_Dev, and make
      the left wrist set (DEF/MECH/CTRL) the exact world mirror of the right (explicit roll, not symmetrize,
      because symmetrize would also move the skinned arm chain).  Left lock flags match the right.
  P2  Silhouette.  Forearm / shin tapered per ring, neck + attachments narrowed, yoke tightened, pelvis shell
      and waist ring tucked and smoothed.
  P3  End effectors + head.  Hand meshes AND hand-chain bones (DEF/MECH/CTRL) scaled 1.65x about the wrist
      head; feet lengthened with a flat sole; recessed visor (dish + projected glass panel + cyan strip);
      satin-white helmet / graphite / glass materials.

Fails loudly (RuntimeError) on any unmet prerequisite or failed post-condition.
"""
import math
import os
import sys

import bpy
import bmesh
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree

SCRIPT_VERSION = "1.0"
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SRC_DEFAULT = os.path.join(ROOT, "blender", "robot_armature_v16.blend")
DST_DEFAULT = os.path.join(ROOT, "blender", "robot_armature_v17.blend")
ARM = "RobotArmature"

# ---- tunables (every number the user specified, plus the ones I chose and why) --------------------------
TAPER_RATIO = 0.70            # distal radius / proximal radius for forearm and shin (user: 0.65-0.75)
NECK_SCALE = 0.525            # radius x0.525 = -47.5 % (user: 45-50 % reduction)
YOKE_SXY_Z = (0.90, 0.86, 0.65)   # x, y, z scale; z anchored at the top of the yoke
YOKE_TOP_Z = 1.745
PELVIS_SHELL_XY = (0.90, 0.635)   # -> 0.207 x 0.146, inside torso waist 0.221 x 0.156
WAIST_RING_XY = (0.97, 0.73)      # -> 0.201 x 0.151
HAND_SCALE = 1.65
SHIN_DEPTH_OLD = 0.1618       # v16 shin ankle-end diameter (2 x 0.0809); foot length = 1.4 x this
FOOT_LEN = 1.4 * SHIN_DEPTH_OLD
FOOT_WIDTH = 0.112            # = ankle sphere width (0.115) so the foot reads as seated under the joint
FOOT_TOP_Z = 0.085
FOOT_HEEL_Y = 0.30 * FOOT_LEN  # heel 30 % of the length behind the ankle axis (y=0), toes toward -y
VISOR = dict(ax=0.092, az=0.118, zc=2.115, n=3.5, depth=0.010)

HAND_BONE_KEYS = ("Palm_", "_MCP", "_PIP", "_DIP", "_Tip", "Thumb_")
HAND_MESH_PREFIXES = ("411_Palm_", "411_Finger", "411_Thumb_", "411_ThumbJoint_")


def S(v):
    return Vector((-v[0], v[1], v[2]))


def require(cond, msg):
    if not cond:
        raise RuntimeError("v17 build: " + msg)


def obj(name):
    o = bpy.data.objects.get(name)
    require(o is not None, f"missing object {name}")
    return o


def world_verts(o):
    M = o.matrix_world
    return [M @ v.co for v in o.data.vertices]


def edit_verts(o, fn):
    """Apply fn(world_position) -> world_position to every vertex. Objects may carry a baked rotation."""
    require(o.data.users == 1, f"{o.name}: mesh data shared by {o.data.users} users")
    require(o.parent is None, f"{o.name}: unexpected parent")
    M = o.matrix_world
    sc = M.to_scale()
    require(all(abs(c - 1.0) < 1e-6 for c in sc), f"{o.name}: non-unit scale {tuple(sc)}")
    Mi = M.inverted()
    for v in o.data.vertices:
        v.co = Mi @ fn(M @ v.co)
    o.data.update()


def edit_verts_indexed(o, fn):
    require(o.data.users == 1, f"{o.name}: mesh data shared")
    M = o.matrix_world
    Mi = M.inverted()
    for v in o.data.vertices:
        v.co = Mi @ fn(v.index, M @ v.co)
    o.data.update()


def smooth_step(a, b, x):
    t = max(0.0, min(1.0, (x - a) / (b - a)))
    return t * t * (3 - 2 * t)


# =========================================================================================================
# P1
# =========================================================================================================
WRIST_R = ("MECH_Wrist_Flex", "MECH_Wrist_Dev", "CTRL_Wrist_Flex", "CTRL_Wrist_Dev")


def p1_wrists(arm):
    M = arm.matrix_world
    R3 = M.to_3x3()
    Mi = M.inverted()
    Ri = Mi.to_3x3()
    B = arm.data.bones
    for n in ("DEF_Wrist", "DEF_Wrist_L") + WRIST_R + tuple(n + "_L" for n in WRIST_R):
        require(n in B, f"missing bone {n}")

    def frame(name):
        b = B[name]
        m = R3 @ b.matrix_local.to_3x3()
        return M @ b.head_local, M @ b.tail_local, m.col[1].normalized(), m.col[2].normalized(), b.length

    # new targets (world space), computed before any edit
    mech_h, mech_t, my, mz, mlen = frame("MECH_Wrist_Dev")
    def_h, _, _, _, _ = frame("DEF_Wrist")
    targets = {"DEF_Wrist": (def_h, def_h + my * mlen, mz)}
    for n in WRIST_R:                                   # L = exact mirror of R (convention verified on legs)
        h, t, _y, z, _l = frame(n)
        targets[n + "_L"] = (S(h), S(t), -S(z))
    targets["DEF_Wrist_L"] = (S(targets["DEF_Wrist"][0]), S(targets["DEF_Wrist"][1]), -S(targets["DEF_Wrist"][2]))

    for n in targets:                                    # children must not be connected to a bone whose tail moves
        for c in B[n].children:
            require(not c.use_connect, f"{c.name} is connected to {n}")

    bpy.context.view_layer.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    try:
        E = arm.data.edit_bones
        for n, (h, t, z) in targets.items():
            eb = E[n]
            eb.head = Mi @ h
            eb.tail = Mi @ t
            eb.align_roll(Ri @ z)
    finally:
        bpy.ops.object.mode_set(mode="OBJECT")

    # lock flags: left matches right (spec section 8: each MECH bone owns one axis)
    P = arm.pose.bones
    for n in ("MECH_Wrist_Flex", "MECH_Wrist_Dev"):
        P[n + "_L"].lock_rotation = tuple(P[n].lock_rotation)

    # post-conditions
    for n, (h, t, z) in targets.items():
        f = frame(n)
        require((f[0] - h).length < 1e-5 and (f[1] - t).length < 1e-5, f"{n}: head/tail not applied")
        require(f[3].angle(z) < 1e-4, f"{n}: roll not applied")
    for s in ("", "_L"):
        a = R3 @ B["DEF_Wrist" + s].matrix_local.to_3x3()
        b = R3 @ B["MECH_Wrist_Dev" + s].matrix_local.to_3x3()
        require((a - b).to_3x3().col[0].length + (a - b).to_3x3().col[1].length + (a - b).to_3x3().col[2].length < 1e-4,
                f"DEF_Wrist{s} rest frame != MECH_Wrist_Dev{s}")
    return {"edited_bones": sorted(targets)}


# =========================================================================================================
# P2
# =========================================================================================================
def ring_taper(o, ratio):
    """Two 24-vertex rings (prism). Distal ring scaled about its own centroid so radius = ratio * proximal."""
    W = world_verts(o)
    zmid = (min(p.z for p in W) + max(p.z for p in W)) / 2
    top = [i for i, p in enumerate(W) if p.z > zmid]
    bot = [i for i, p in enumerate(W) if p.z <= zmid]
    require(len(top) == 24 and len(bot) == 24, f"{o.name}: expected two 24-vertex rings")

    def rad(g):
        c = sum((W[i] for i in g), Vector()) / len(g)
        return c, sum((W[i] - c).length for i in g) / len(g)

    ct, rt = rad(top)
    cb, rb = rad(bot)
    k = ratio * rt / rb
    bset = set(bot)
    idx = {i: n for n, i in enumerate(range(len(W)))}
    edit_verts_indexed(o, lambda i, p: (cb + (p - cb) * k) if i in bset else p)
    return rt, rb, ratio * rt


def tuck_pelvis(sx_sy, o):
    edit_verts(o, lambda p: Vector((p.x * sx_sy[0], p.y * sx_sy[1], p.z)))


def smooth_shade(o, sharp_deg):
    me = o.data
    bm = bmesh.new()
    bm.from_mesh(me)
    for e in bm.edges:
        e.smooth = (not e.is_manifold) or e.calc_face_angle(0.0) < math.radians(sharp_deg)
    for f in bm.faces:
        f.smooth = True
    bm.to_mesh(me)
    bm.free()
    me.update()
    if not any(m.type == "WEIGHTED_NORMAL" for m in o.modifiers):
        wn = o.modifiers.new("WeightedNormal", "WEIGHTED_NORMAL")
        wn.keep_sharp = True
        wn.weight = 50
        arm_idx = [i for i, m in enumerate(o.modifiers) if m.type == "ARMATURE"]
        if arm_idx:
            o.modifiers.move(len(o.modifiers) - 1, arm_idx[0])


def p2_silhouette():
    out = {}
    for side in ("L", "R"):
        out[f"forearm_{side}"] = ring_taper(obj(f"411_Forearm_{side}"), TAPER_RATIO)
        out[f"shin_{side}"] = ring_taper(obj(f"411_Shin_{side}"), TAPER_RATIO)

    k = NECK_SCALE
    edit_verts(obj("411_Neck"), lambda p: Vector((p.x * k, p.y * k, p.z)))
    edit_verts(obj("411_N_JointRing"), lambda p: Vector((p.x * k, p.y * k, p.z)))
    for n in ("411_N_LED_Amber", "411_N_LED_Cyan", "411_N_LED_Red", "411_N_ServicePort", "411_N_CableGuard"):
        o = obj(n)
        W = world_verts(o)
        c = sum(W, Vector()) / len(W)
        d = Vector((c.x * (k - 1), c.y * (k - 1), 0.0))      # translate toward the axis, keep part size
        edit_verts(o, lambda p, d=d: p + d)

    sx, sy, sz = YOKE_SXY_Z
    edit_verts(obj("411_Yoke"), lambda p: Vector((p.x * sx, p.y * sy, YOKE_TOP_Z + (p.z - YOKE_TOP_Z) * sz)))

    tuck_pelvis(PELVIS_SHELL_XY, obj("411_PelvicShell"))
    tuck_pelvis(WAIST_RING_XY, obj("411_WaistRing_New"))
    tuck_pelvis(WAIST_RING_XY, obj("411_WaistRing_Backing"))
    smooth_shade(obj("411_PelvicShell"), 45.0)
    smooth_shade(obj("411_WaistRing_New"), 30.0)
    for side in ("L", "R"):                       # hip sockets are the pelvis's spherical joints: same faceting
        smooth_shade(obj(f"411_HipSocket_{side}"), 45.0)
    return out


# =========================================================================================================
# P3
# =========================================================================================================
def p3_hands(arm):
    M = arm.matrix_world
    Mi = M.inverted()
    B = arm.data.bones
    pivots = {"": M @ B["DEF_Wrist"].head_local, "_L": M @ B["DEF_Wrist_L"].head_local}

    def side(name):
        # bones end in _L; finger meshes are named 411_FingerN_L_A / _B, so test the segment, not the suffix
        return "_L" if (name.endswith("_L") or "_L_" in name) else ""

    hand_bones = [b.name for b in B
                  if b.name.startswith(("DEF_", "MECH_", "CTRL_")) and any(k in b.name for k in HAND_BONE_KEYS)]
    require(len(hand_bones) >= 40, f"unexpected hand-chain bone count {len(hand_bones)}")
    inset = set(hand_bones)
    for n in hand_bones:      # a connected bone is only safe if its parent is scaled with it
        require(not B[n].use_connect or B[n].parent.name in inset, f"{n} is connected to a parent outside the hand chain")
    new = {}
    for n in hand_bones:
        P = pivots[side(n)]
        h = M @ B[n].head_local
        t = M @ B[n].tail_local
        new[n] = (P + (h - P) * HAND_SCALE, P + (t - P) * HAND_SCALE)
    bpy.context.view_layer.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    try:
        E = arm.data.edit_bones
        for n, (h, t) in new.items():
            eb = E[n]
            z = (eb.matrix.to_3x3().col[2]).copy()      # keep the roll: remember local Z, re-align after move
            eb.head = Mi @ h
            eb.tail = Mi @ t
            eb.align_roll(z)
    finally:
        bpy.ops.object.mode_set(mode="OBJECT")

    meshes = [o for o in bpy.data.objects if o.type == "MESH" and o.name.startswith(HAND_MESH_PREFIXES)]
    require(len(meshes) == 2 * (1 + 8 + 1 + 1), f"unexpected hand mesh count {len(meshes)}")
    n_left = sum(1 for o in meshes if side(o.name) == "_L")
    require(n_left == len(meshes) // 2, f"left/right hand mesh split is {n_left}/{len(meshes) - n_left}")
    for o in meshes:
        P = pivots[side(o.name)]
        edit_verts(o, lambda p, P=P: P + (p - P) * HAND_SCALE)
    return {"hand_bones": len(hand_bones), "hand_meshes": len(meshes)}


def p3_feet():
    n_sup = 3.0
    for s in ("L", "R"):
        o = obj(f"411_Foot_{s}")
        W = world_verts(o)
        require(len(W) == 96, f"{o.name}: unexpected vertex count")
        zmid = 0.075
        groups = {"bot": [i for i, p in enumerate(W) if p.z <= zmid], "top": [i for i, p in enumerate(W) if p.z > zmid]}
        require(len(groups["bot"]) == 48 and len(groups["top"]) == 48, f"{o.name}: unexpected topology")
        cx = sum(p.x for p in W) / len(W)
        cy_new = FOOT_HEEL_Y - FOOT_LEN / 2.0   # foot centre y: heel is FOOT_HEEL_Y behind the ankle axis (toes -y)
        z_top_old = max(p.z for p in W)
        spec = {"bot": (FOOT_WIDTH / 2, FOOT_LEN / 2, 0.0),
                "top": (FOOT_WIDTH / 2 * 0.90, FOOT_LEN / 2 * 0.90, FOOT_TOP_Z - z_top_old)}
        newpos = {}
        for g, ids in groups.items():
            xs = [W[i].x for i in ids]
            ys = [W[i].y for i in ids]
            gcx, gcy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
            hx, hy = (max(xs) - min(xs)) / 2, (max(ys) - min(ys)) / 2
            HX, HY, dz = spec[g]
            mapped = {}
            for i in ids:
                u, w = (W[i].x - gcx) / hx, (W[i].y - gcy) / hy
                rho = math.hypot(u, w)
                pn = (abs(u) ** n_sup + abs(w) ** n_sup) ** (1 / n_sup)
                f = rho / pn if pn > 1e-9 else 1.0           # round toward a squared oval (superellipse n=3)
                mapped[i] = (u * f, w * f)
            # renormalise: the source ring is already a rounded square, so the remap alone overshoots 8-10 %
            umin, umax = min(m[0] for m in mapped.values()), max(m[0] for m in mapped.values())
            wmin, wmax = min(m[1] for m in mapped.values()), max(m[1] for m in mapped.values())
            for i, (u, w) in mapped.items():
                un = (u - (umax + umin) / 2) / ((umax - umin) / 2)
                wn = (w - (wmax + wmin) / 2) / ((wmax - wmin) / 2)
                newpos[i] = Vector((cx + un * HX, cy_new + wn * HY, W[i].z + dz))
        edit_verts_indexed(o, lambda i, p: newpos[i])
        zmin = min(p.z for p in world_verts(o))
        require(abs(zmin) < 1e-6, f"{o.name}: sole not at z=0 ({zmin})")
    return {"foot_len": FOOT_LEN, "foot_width": FOOT_WIDTH}


def principled(mat):
    return next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")


def make_material(name, base, rough, metal=0.0, coat=None, emit=None, aniso=None):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    p = principled(m)
    p.inputs["Base Color"].default_value = (*base, 1.0)
    p.inputs["Roughness"].default_value = rough
    p.inputs["Metallic"].default_value = metal
    if coat is not None:
        p.inputs["Coat Weight"].default_value = coat
    if aniso is not None:
        p.inputs["Anisotropic"].default_value = aniso
    if emit is not None:
        p.inputs["Emission Color"].default_value = (*emit[0], 1.0)
        p.inputs["Emission Strength"].default_value = emit[1]
    return m


def p3_materials_and_head(arm):
    helmet = make_material("Helmet_SatinWhite", (0.92, 0.92, 0.92), 0.32, coat=0.25)
    glass = make_material("Visor_BlackGlass", (0.002, 0.002, 0.003), 0.08)
    cyan = make_material("Visor_CyanEmit", (0.0, 0.02, 0.03), 0.35, emit=((0.0, 0.75, 0.9), 1.6))
    graphite = make_material("Graphite_Brushed", (0.055, 0.058, 0.064), 0.45, metal=0.6, aniso=0.35)

    head = obj("411_Head")
    for o in bpy.data.objects:
        if o.type == "MESH" and o.name.startswith("411_") and o is not head:
            for sl in o.material_slots:
                if sl.material and sl.material.name in ("BotBlack_Matte", "BotBlack_Gloss"):
                    sl.material = graphite
    head.material_slots[0].material = helmet
    head.material_slots[1].material = glass

    # ---- visor dish: smooth inward displacement of the front vertices (no topology change) -------------
    V = VISOR
    me = head.data
    n = V["n"]

    def s_of(x, z):
        return (abs(x / V["ax"]) ** n + abs((z - V["zc"]) / V["az"]) ** n) ** (1 / n)

    yc = -0.037
    me.update()
    normals = [v.normal.copy() for v in me.vertices]
    moved = 0
    for v, nrm in zip(me.vertices, normals):
        if v.co.y < yc and nrm.y < -0.25:
            w = 1.0 - smooth_step(0.82, 1.12, s_of(v.co.x, v.co.z))
            if w > 0:
                v.co = v.co - nrm * (V["depth"] * w)
                moved += 1
    me.update()
    require(moved > 150, f"visor dish moved only {moved} vertices")
    for p in me.polygons:
        c = p.center
        p.material_index = 1 if (c.y < yc and p.normal.y < -0.25 and s_of(c.x, c.z) <= 0.80) else 0

    # ---- glass panel + cyan strip, projected onto the evaluated head surface ---------------------------
    for o in bpy.data.objects:
        if o.type == "MESH":
            o.select_set(False)
    prev_pose = arm.data.pose_position
    arm.data.pose_position = "REST"
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    bvh = BVHTree.FromObject(head, dg)

    def project(x, z, lift):
        hit = bvh.ray_cast(Vector((x, -1.0, z)), Vector((0, 1, 0)))
        require(hit[0] is not None, f"visor ray missed at x={x:.3f} z={z:.3f}")
        return hit[0] + hit[1] * lift

    def build(name, nu, nv, mapper, lift, mat):
        bm = bmesh.new()
        grid = [[bm.verts.new(project(*mapper(i / nu * 2 - 1, j / nv * 2 - 1), lift)) for i in range(nu + 1)]
                for j in range(nv + 1)]
        for j in range(nv):
            for i in range(nu):
                bm.faces.new((grid[j][i], grid[j][i + 1], grid[j + 1][i + 1], grid[j + 1][i]))
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        if sum(f.normal.y for f in bm.faces) > 0:           # face normals must point to the viewer (-y)
            bmesh.ops.reverse_faces(bm, faces=bm.faces)
        for f in bm.faces:
            f.smooth = True
        me2 = bpy.data.meshes.new(name)
        bm.to_mesh(me2)
        bm.free()
        ob = bpy.data.objects.new(name, me2)
        head.users_collection[0].objects.link(ob)
        ob.data.materials.append(mat)
        vg = ob.vertex_groups.new(name="DEF_Head")
        vg.add(list(range(len(me2.vertices))), 1.0, "REPLACE")
        mod = ob.modifiers.new("Armature", "ARMATURE")
        mod.object = arm
        return ob

    def panel_map(u, v, k=0.95):
        m = max(abs(u), abs(v))
        if m == 0:
            return (0.0, V["zc"])
        s = m / ((abs(u) ** n + abs(v) ** n) ** (1 / n))
        return (V["ax"] * k * u * s, V["zc"] + V["az"] * k * v * s)

    zs = V["zc"] - 0.085
    strip_map = lambda u, v: (0.060 * u, zs + 0.003 * v)
    for nm in ("411_Visor_Glass", "411_Visor_Strip"):
        require(bpy.data.objects.get(nm) is None, f"{nm} already exists")
    build("411_Visor_Glass", 48, 60, panel_map, 0.0006, glass)
    build("411_Visor_Strip", 32, 1, strip_map, 0.0016, cyan)
    arm.data.pose_position = prev_pose

    # the old camera lens/strip stood 0.05 m proud of the old face; seat them on the new panel
    for nm in ("411_S_PrimaryLens", "411_S_PrimaryStrip"):
        o = obj(nm)
        edit_verts(o, lambda p: p + Vector((0.0, 0.032, 0.0)))
    return {"dish_vertices": moved, "materials": [helmet.name, glass.name, cyan.name, graphite.name]}


# =========================================================================================================
def main(src, dst):
    require(os.path.exists(src), f"source not found: {src}")
    bpy.ops.wm.open_mainfile(filepath=src)
    require(bpy.context.scene.get("v17_build_version") is None, "source is already a v17 file; rebuild from v16")
    arm = obj(ARM)
    require(arm.type == "ARMATURE", f"{ARM} is not an armature")
    defs = [b.name for b in arm.data.bones if b.name.startswith("DEF_")]
    require(len(defs) == 56, f"expected 56 DEF bones in source, found {len(defs)}")
    require(not [b for b in arm.data.bones if not b.name.startswith("DEF_") and b.use_deform], "non-DEF deform bones in source")
    arm.animation_data.action_slot  # touch: the active action must exist
    report = {"P1": p1_wrists(arm)}
    report["P2"] = p2_silhouette()
    report["P3_hands"] = p3_hands(arm)
    report["P3_feet"] = p3_feet()
    report["P3_head"] = p3_materials_and_head(arm)
    require(len([b for b in arm.data.bones if b.name.startswith("DEF_")]) == 56, "DEF bone count changed")
    require(not [b for b in arm.data.bones if not b.name.startswith("DEF_") and b.use_deform], "non-DEF deform bone appeared")
    bpy.context.scene["v17_build_version"] = SCRIPT_VERSION
    bpy.ops.wm.save_as_mainfile(filepath=dst)
    for k, v in report.items():
        print(k, v)
    print("saved", dst)


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if a.endswith(".blend")]
    main(args[0] if len(args) > 0 else SRC_DEFAULT, args[1] if len(args) > 1 else DST_DEFAULT)
