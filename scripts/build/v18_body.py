"""v18_body.py -- SCRIPT_VERSION 1.0

Purpose: v18 Phases 2-3 body geometry. Cage / vertex edits plus NON-destructive groove modifiers:
  stack on a grooved mesh:  [existing modifiers] -> BOOLEAN (DIFFERENCE, exact, hidden cutter) -> VERTEX_WEIGHT_MIX
  (sets every vertex to weight 1.0 of the mesh's single DEF group, because the exact boolean creates unweighted vertices)
  -> ARMATURE.  Groove faces take material slot 1 (or 2 on the head) through the cutter's own face material index.
The glTF exporter applies the modifier stack, so the grooves, pocket and weights are in the .glb.
"""
import math

import bmesh
import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

SCRIPT_VERSION = "1.0"
SEAM_W, SEAM_D = 0.002, 0.0015          # torso / head seam width, depth
LIMB_W, LIMB_D = 0.0015, 0.0012         # forearm / shin split seam width, depth
MARGIN = 0.004                          # cutter stands this far proud of the surface so it always cuts through


def req(c, m):
    if not c:
        raise RuntimeError("v18 body: " + m)


def O(n):
    o = bpy.data.objects.get(n)
    req(o is not None, f"missing object {n}")
    return o


# ---------------------------------------------------------------------------------------------------------
# generic helpers
# ---------------------------------------------------------------------------------------------------------
def world_verts(o):
    M = o.matrix_world
    return [M @ v.co for v in o.data.vertices]


def edit_world(o, fn):
    """fn(index, world_position) -> world_position."""
    req(o.data.users == 1, f"{o.name}: shared mesh")
    M = o.matrix_world
    Mi = M.inverted()
    for v in o.data.vertices:
        v.co = Mi @ fn(v.index, M @ v.co)
    o.data.update()


class WorldBVH:
    """BVH of an evaluated object (armature in REST) queried in WORLD space; the object may carry a baked rotation."""
    def __init__(self, o, dg):
        self.bvh = BVHTree.FromObject(o, dg)
        self.M = o.matrix_world.copy()
        self.Mi = self.M.inverted()
        self.Nm = self.Mi.to_3x3().transposed()

    def ray_cast(self, origin, direction):
        d = (self.Mi.to_3x3() @ direction)
        h = self.bvh.ray_cast(self.Mi @ origin, d.normalized())
        if h[0] is None:
            return (None, None, None, None)
        return (self.M @ h[0], (self.Nm @ h[1]).normalized(), h[2], h[3])


def rest_bvh(arm, o):
    arm.data.pose_position = "REST"
    bpy.context.view_layer.update()
    return WorldBVH(o, bpy.context.evaluated_depsgraph_get())


def ray(bvh, origin, direction):
    h = bvh.ray_cast(origin, direction)
    req(h[0] is not None, f"ray missed from {tuple(round(x, 3) for x in origin)} dir {tuple(direction)}")
    return h[0], h[1]


def smoothstep(a, b, x):
    t = max(0.0, min(1.0, (x - a) / (b - a)))
    return t * t * (3 - 2 * t)


def interp_knots(knots, z):
    """Smoothstep interpolation through (z, value) knots sorted ascending; clamped outside."""
    if z <= knots[0][0]:
        return knots[0][1]
    for (z0, v0), (z1, v1) in zip(knots, knots[1:]):
        if z <= z1:
            return v0 + (v1 - v0) * smoothstep(z0, z1, z)
    return knots[-1][1]


# ---------------------------------------------------------------------------------------------------------
# cutters
# ---------------------------------------------------------------------------------------------------------
def add_tube(bm, points, width, depth, margin, mat_index):
    """Closed rectangular solid swept along [(position, surface_normal)]: from -depth to +margin along the normal."""
    rings = []
    n_pts = len(points)
    for i, (p, n) in enumerate(points):
        t = (points[min(i + 1, n_pts - 1)][0] - points[max(i - 1, 0)][0]).normalized()
        b = t.cross(n).normalized()
        nn = b.cross(t).normalized()
        rings.append([bm.verts.new(p + b * width / 2 - nn * depth), bm.verts.new(p - b * width / 2 - nn * depth),
                      bm.verts.new(p - b * width / 2 + nn * margin), bm.verts.new(p + b * width / 2 + nn * margin)])
    faces = []
    for i in range(n_pts - 1):
        for k in range(4):
            faces.append(bm.faces.new((rings[i][k], rings[i][(k + 1) % 4], rings[i + 1][(k + 1) % 4], rings[i + 1][k])))
    faces.append(bm.faces.new(rings[0][::-1]))
    faces.append(bm.faces.new(rings[-1]))
    for f in faces:
        f.material_index = mat_index
    return faces


def grid_points(project, mapper, nu, nv, lift):
    return [[project(*mapper(i / nu * 2 - 1, j / nv * 2 - 1), lift) for i in range(nu + 1)] for j in range(nv + 1)]


def add_grid(bm, pts, flip=False):
    nv, nu = len(pts) - 1, len(pts[0]) - 1
    verts = [[bm.verts.new(p) for p in row] for row in pts]
    faces = []
    for j in range(nv):
        for i in range(nu):
            q = (verts[j][i], verts[j][i + 1], verts[j + 1][i + 1], verts[j + 1][i])
            faces.append(bm.faces.new(q[::-1] if flip else q))
    return verts, faces


def make_cutter(name, build, like):
    bm = bmesh.new()
    build(bm)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    for _ in range(3):
        me.materials.append(None)
    c = bpy.data.objects.new(name, me)
    like.users_collection[0].objects.link(c)
    c.display_type = "WIRE"
    c.hide_render = True
    c.hide_set(True)
    return c


def add_groove_stack(target, cutter, vgroup):
    for m in list(target.modifiers):
        if m.type in ("BOOLEAN", "VERTEX_WEIGHT_MIX") and m.name.startswith("V18"):
            target.modifiers.remove(m)
    bm = target.modifiers.new("V18_Groove", "BOOLEAN")
    bm.operation = "DIFFERENCE"
    bm.operand_type = "OBJECT"
    bm.object = cutter
    bm.solver = "EXACT"
    bm.use_self = True
    bm.material_mode = "INDEX"
    wm = target.modifiers.new("V18_WeightAll", "VERTEX_WEIGHT_MIX")
    wm.vertex_group_a = vgroup
    wm.mix_mode = "SET"
    wm.mix_set = "ALL"
    wm.default_weight_a = 1.0
    wm.default_weight_b = 1.0
    # order: ... -> BOOLEAN -> VERTEX_WEIGHT_MIX -> ARMATURE
    def idx(t):
        return [m.type for m in target.modifiers].index(t)
    target.modifiers.move(idx("BOOLEAN"), idx("ARMATURE"))
    target.modifiers.move(idx("VERTEX_WEIGHT_MIX"), idx("ARMATURE"))
    order = [m.type for m in target.modifiers]
    req(order.index("BOOLEAN") < order.index("VERTEX_WEIGHT_MIX") < order.index("ARMATURE"), f"{target.name}: bad modifier order {order}")


def ensure_slot(o, mat, index):
    while len(o.data.materials) <= index:
        o.data.materials.append(mat)
    if o.data.materials[index] is None:
        o.data.materials[index] = mat
    return index


# ---------------------------------------------------------------------------------------------------------
# torso contour
# ---------------------------------------------------------------------------------------------------------
def half_width(bvh, z):
    return ray(bvh, Vector((1.0, 0.0, z)), Vector((-1, 0, 0)))[0].x


def torso_contour(arm, pec=1.03, waist_ratio=0.86, rim_ratio=0.92, mid_ratio=None, abdomen_depth=1.0):
    """Re-contour the torso cage. Default (mid_ratio None) is the v18 behaviour (3 solved heights).
    v20: mid_ratio adds a fourth solved height (z = 1.20) so the NARROWEST point sits at the waist (z = 1.11) and the width then grows
    monotonically into the hips; abdomen_depth < 1 additionally flattens the front-to-back depth over z 1.0-1.28 (belly)."""
    t = O("411_Torso")
    orig = [v.co.copy() for v in t.data.vertices]
    Z_RIM, Z_WAIST, Z_MID = 0.975, 1.11, 1.20       # measurement heights: lower iliac rim, waist, upper-waist check point
    if mid_ratio is None:
        knots_z = (0.961, Z_WAIST, 1.26, 1.43, 1.56)
        f = {"rim": 1.12, "waist": 1.12, "pec": pec}
    else:
        knots_z = (0.961, Z_WAIST, Z_MID, 1.30, 1.43, 1.56)
        f = {"rim": 1.0, "waist": 1.0, "mid": 1.0, "pec": pec}
    M = t.matrix_world
    req((M - Matrix.Identity(4)).to_3x3().col[0].length < 1e-9, "torso is expected to have an identity transform")
    depth_knots = [(0.961, 1.0), (1.02, abdomen_depth), (1.16, abdomen_depth), (1.28, 1.0)]
    for it in range(14):
        if mid_ratio is None:
            kn = [(knots_z[0], f["rim"]), (knots_z[1], f["waist"]), (knots_z[2], 1.0), (knots_z[3], f["pec"]), (knots_z[4], 1.0)]
        else:
            kn = [(knots_z[0], f["rim"]), (knots_z[1], f["waist"]), (knots_z[2], f["mid"]), (knots_z[3], 1.0), (knots_z[4], f["pec"]), (knots_z[5], 1.0)]
        for v, c in zip(t.data.vertices, orig):
            sc_ = interp_knots(kn, c.z)
            v.co = Vector((c.x * sc_, c.y * sc_ * interp_knots(depth_knots, c.z), c.z))
        t.data.update()
        bvh = rest_bvh(arm, t)
        wc = max(half_width(bvh, 1.26 + 0.01 * k) for k in range(0, 34))
        ww, wr, wm = half_width(bvh, Z_WAIST), half_width(bvh, Z_RIM), half_width(bvh, Z_MID)
        errs = [abs(ww / wc - waist_ratio), abs(wr / wc - rim_ratio)] + ([abs(wm / wc - mid_ratio)] if mid_ratio is not None else [])
        if max(errs) < 0.001:
            break
        f["waist"] *= (waist_ratio * wc) / ww
        f["rim"] *= (rim_ratio * wc) / wr
        if mid_ratio is not None:
            f["mid"] *= (mid_ratio * wc) / wm
    dep = lambda z: -ray(bvh, Vector((0.0, -1.0, z)), Vector((0, 1, 0)))[0].y
    dc = max(dep(1.26 + 0.01 * k) for k in range(0, 34))
    prof = [(round(z, 2), round(half_width(bvh, z), 4)) for z in (0.975, 1.03, 1.11, 1.20, 1.30, 1.43)]
    return {"z_waist": Z_WAIST, "z_rim": Z_RIM, "chest_half": wc, "waist_half": ww, "rim_half": wr, "waist_ratio": ww / wc,
            "rim_ratio": wr / wc, "mid_ratio": wm / wc, "depth_chest": dc, "depth_waist_ratio": dep(Z_WAIST) / dc,
            "depth_rim_ratio": dep(Z_RIM) / dc, "profile": prof, "iterations": it + 1}


def thin_legs(thigh=0.955, calf_top=0.955):
    """Light proportional thinning of the leg prisms (2 x 24-vertex rings each): thigh both rings, shin PROXIMAL ring only
    (upper calf), each about its own centroid. Lengths and ring centres do not change."""
    out = {}
    for side in ("L", "R"):
        for name, k_top, k_bot in ((f"411_Thigh_{side}", thigh, thigh), (f"411_Shin_{side}", calf_top, 1.0)):
            o = O(name)
            W = world_verts(o)
            zm = (min(p.z for p in W) + max(p.z for p in W)) / 2
            top = [i for i, p in enumerate(W) if p.z > zm]
            bot = [i for i, p in enumerate(W) if p.z <= zm]
            req(len(top) == 24 and len(bot) == 24, f"{name}: expected two 24-vertex rings")
            ct = sum((W[i] for i in top), Vector()) / 24
            cb = sum((W[i] for i in bot), Vector()) / 24
            ts, bs = set(top), set(bot)
            edit_world(o, lambda i, p, ts=ts, bs=bs, ct=ct, cb=cb, k_top=k_top, k_bot=k_bot:
                       ct + (p - ct) * k_top if i in ts else (cb + (p - cb) * k_bot if i in bs else p))
            out[name] = (k_top, k_bot)
    return out


# ---------------------------------------------------------------------------------------------------------
# torso seams
# ---------------------------------------------------------------------------------------------------------
def catmull(points, n_out):
    """Centripetal-ish Catmull-Rom through 3D control points; returns n_out samples."""
    P = [points[0]] + points + [points[-1]]
    out = []
    segs = len(points) - 1
    for k in range(n_out):
        u = k / (n_out - 1) * segs
        i = min(int(u), segs - 1)
        t = u - i
        p0, p1, p2, p3 = P[i], P[i + 1], P[i + 2], P[i + 3]
        out.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
    return out


def torso_seams(arm, graphite):
    t = O("411_Torso")
    bvh = rest_bvh(arm, t)
    ratio_knots = [(0.975, 0.60), (1.06, 0.60), (1.16, 0.62), (1.28, 0.70), (1.42, 0.80), (1.54, 0.86)]   # seam x / half-width
    zs = [z for z, _ in ratio_knots]
    pts_ctrl = []
    for z, r in ratio_knots:
        x = r * half_width(bvh, z)
        pts_ctrl.append(Vector((x, 0.0, z)))
    samples = catmull(pts_ctrl, 140)
    sides = {}
    for sgn in (1, -1):
        pts = []
        for p in samples:
            h, n = ray(bvh, Vector((sgn * p.x, -1.0, p.z)), Vector((0, 1, 0)))
            pts.append((h, n))
        sides[sgn] = pts
    z_h = 0.961 + 0.025
    xr = ratio_knots[0][1] * half_width(bvh, z_h)
    horiz = []
    for k in range(61):
        x = -xr + 2 * xr * k / 60
        horiz.append(ray(bvh, Vector((x, -1.0, z_h)), Vector((0, 1, 0))))

    def build(bm):
        for sgn in (1, -1):
            add_tube(bm, sides[sgn], SEAM_W, SEAM_D, MARGIN, 1)
        add_tube(bm, horiz, SEAM_W, SEAM_D, MARGIN, 1)

    cutter = make_cutter("CUT_Torso", build, t)
    ensure_slot(t, graphite, 1)
    add_groove_stack(t, cutter, t.vertex_groups[0].name)
    return {"seam_samples": len(samples), "horizontal_z": z_h}


# ---------------------------------------------------------------------------------------------------------
# limb split seams
# ---------------------------------------------------------------------------------------------------------
def limb_seam(arm, name, graphite, t0=0.16, t1=0.90, n=60):
    o = O(name)
    W = world_verts(o)
    zmid = (min(p.z for p in W) + max(p.z for p in W)) / 2
    top = [p for p in W if p.z > zmid]
    bot = [p for p in W if p.z <= zmid]
    ct = sum(top, Vector()) / len(top)
    cb = sum(bot, Vector()) / len(bot)
    bvh = rest_bvh(arm, o)
    pts = []
    for k in range(n):
        c = ct.lerp(cb, t0 + (t1 - t0) * k / (n - 1))
        pts.append(ray(bvh, Vector((c.x, -1.0, c.z)), Vector((0, 1, 0))))
    cutter = make_cutter("CUT_" + name[4:], lambda bm: add_tube(bm, pts, LIMB_W, LIMB_D, MARGIN, 1), o)
    ensure_slot(o, graphite, 1)
    add_groove_stack(o, cutter, o.vertex_groups[0].name)
    return len(pts)


# ---------------------------------------------------------------------------------------------------------
# head: restore, visor pocket with 45 deg chamfer, glass, strip, coronal seam
# ---------------------------------------------------------------------------------------------------------
VISOR = dict(ax=0.092, az=0.118, zc=2.115, n=3.5)
POCKET_DEPTH = 0.0015        # glass floor sits 1.5 mm below the shell surface
CHAMFER = 0.0015             # horizontal width of the 45 deg chamfer at the shell surface


def restore_head_from(v16_path):
    head = O("411_Head")
    me = head.data
    with bpy.data.libraries.load(v16_path, link=False) as (src, dst):
        req(me.name in src.meshes, f"mesh {me.name} not in v16")
        dst.meshes = [me.name]
    tmp = dst.meshes[0]
    req(len(tmp.vertices) == len(me.vertices), "head vertex count differs from v16")
    for v, s in zip(me.vertices, tmp.vertices):
        v.co = s.co
    me.update()
    bpy.data.meshes.remove(tmp)
    for p in me.polygons:
        p.material_index = 0
    return head


def head_features(arm, helmet, glass, graphite, cyan):
    head = O("411_Head")
    bvh = rest_bvh(arm, head)
    V = VISOR
    n = V["n"]

    def proj(x, z, lift):
        h, nm = ray(bvh, Vector((x, -1.0, z)), Vector((0, 1, 0)))
        return h + nm * lift

    def mapper(k):
        def f(u, v):
            m = max(abs(u), abs(v))
            if m == 0:
                return (0.0, V["zc"])
            s = m / ((abs(u) ** n + abs(v) ** n) ** (1 / n))
            return (V["ax"] * k * u * s, V["zc"] + V["az"] * k * v * s)
        return f

    k0 = 0.985
    k1 = k0 + (POCKET_DEPTH + MARGIN) / 0.105
    nu, nv = 40, 50
    floor = grid_points(proj, mapper(k0), nu, nv, -POCKET_DEPTH)
    topg = grid_points(proj, mapper(k1), nu, nv, MARGIN)

    # coronal seam: plane y = yc through the apex, between the two side pins at +-90 deg (pins sit at z ~ 2.12)
    yc = -0.037
    cz = 2.12
    seam = []
    for k in range(121):
        phi = math.radians(-86 + 172 * k / 120)
        d = Vector((math.sin(phi), 0.0, math.cos(phi)))
        h, nm = ray(bvh, Vector((0.0, yc, cz)) + d * 0.6, -d)
        seam.append((h, nm))

    def build(bm):
        fv, ff = add_grid(bm, floor, flip=True)
        tv, tf = add_grid(bm, topg, flip=False)
        for f in ff:
            f.material_index = 1
        for f in tf:
            f.material_index = 2
        per = [(0, i) for i in range(nu)] + [(j, nu) for j in range(nv)] + [(nv, nu - i) for i in range(nu)] + [(nv - j, 0) for j in range(nv)]
        for a, b in zip(per, per[1:] + per[:1]):
            f = bm.faces.new((fv[a[0]][a[1]], fv[b[0]][b[1]], tv[b[0]][b[1]], tv[a[0]][a[1]]))
            f.material_index = 2
        add_tube(bm, seam, SEAM_W, SEAM_D, MARGIN, 2)

    cutter = make_cutter("CUT_Head", build, head)
    for i, m in enumerate((helmet, glass, graphite)):
        ensure_slot(head, m, i)
        head.data.materials[i] = m
    add_groove_stack(head, cutter, head.vertex_groups[0].name)

    # glass panel and cyan strip on the pocket floor
    def rebuild(name, pts, flip, mat):
        o = O(name)
        bm = bmesh.new()
        add_grid(bm, pts, flip=flip)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        if sum(f.normal.y for f in bm.faces) > 0:
            bmesh.ops.reverse_faces(bm, faces=bm.faces)
        for f in bm.faces:
            f.smooth = True
        o.data.clear_geometry()
        bm.to_mesh(o.data)
        bm.free()
        vg = o.vertex_groups.get("DEF_Head") or o.vertex_groups.new(name="DEF_Head")   # clear_geometry drops group names
        vg.add(list(range(len(o.data.vertices))), 1.0, "REPLACE")
        o.data.materials.clear()
        o.data.materials.append(mat)
        o.data.update()

    kp = k0 - 0.004
    rebuild("411_Visor_Glass", grid_points(proj, mapper(kp), 48, 60, -POCKET_DEPTH + 0.0004), False, glass)
    zs = V["zc"] - 0.085
    strip_map = lambda u, v: (0.060 * u, zs + 0.003 * v)
    rebuild("411_Visor_Strip", grid_points(proj, strip_map, 32, 1, -POCKET_DEPTH + 0.0016), False, cyan)
    return {"pocket_depth_mm": POCKET_DEPTH * 1e3, "chamfer_mm": CHAMFER * 1e3, "k_floor": k0, "k_top": round(k1, 4)}


# ---------------------------------------------------------------------------------------------------------
# Phase 3: pelvis, hip sockets, yoke
# ---------------------------------------------------------------------------------------------------------
def pelvis_chassis(arm, shell_fit=0.975, ring_fit=0.99, rise=0.014, dip=0.012, taper=0.10):
    """Fit the pelvis shell / waist ring to the (re-contoured) torso rim, then sculpt the shell into a brief-like chassis:
    outer hips rise, the centre bridge dips, and the lower rings taper inward."""
    t = O("411_Torso")
    bvh = rest_bvh(arm, t)
    z_rim = 0.975
    wx = half_width(bvh, z_rim)
    dy = -ray(bvh, Vector((0.0, -1.0, z_rim)), Vector((0, 1, 0)))[0].y

    sh = O("411_PelvicShell")
    W = world_verts(sh)
    cx, cy = max(abs(p.x) for p in W), max(abs(p.y) for p in W)
    sx, sy = shell_fit * wx / cx, shell_fit * dy / cy
    zmin, zmax = min(p.z for p in W), max(p.z for p in W)

    def shape(i, p):
        x, y = p.x * sx, p.y * sy
        h = (p.z - zmin) / (zmax - zmin)
        w = (1.0 - h) ** 1.5
        u = min(1.0, abs(x) / (cx * sx))
        z = p.z + w * (rise * u * u - dip * (1.0 - u * u))
        k = 1.0 - taper * w
        return Vector((x * k, y * k, z))

    edit_world(sh, shape)
    for name in ("411_WaistRing_New", "411_WaistRing_Backing"):
        o = O(name)
        Wr = world_verts(o)
        rx, ry = max(abs(p.x) for p in Wr), max(abs(p.y) for p in Wr)
        fx, fy = ring_fit * wx / (rx if name.endswith("New") else rx * 1.15), ring_fit * dy / (ry if name.endswith("New") else ry * 1.15)
        edit_world(o, lambda i, p, fx=fx, fy=fy: Vector((p.x * fx, p.y * fy, p.z)))
    return {"torso_rim_half_x": wx, "torso_rim_half_y": dy, "shell_scale": (sx, sy)}


def hip_sockets(depth_fraction=0.20):
    out = {}
    for side in ("L", "R"):
        o = O(f"411_HipSocket_{side}")
        W = world_verts(o)
        radius = (max(p.x for p in W) - min(p.x for p in W)) / 2
        dx = depth_fraction * radius
        sgn = 1.0 if sum(p.x for p in W) > 0 else -1.0
        edit_world(o, lambda i, p, dx=dx, sgn=sgn: Vector((p.x - sgn * dx, p.y, p.z)))
        out[side] = (radius, dx)
    return out


def yoke(extra_compress=0.18, sink=0.004, z_top=1.745):
    o = O("411_Yoke")
    W = world_verts(o)
    zmin0 = min(p.z for p in W)
    k = 1.0 - extra_compress
    zmin1 = z_top + (zmin0 - z_top) * k
    h1 = z_top - zmin1

    def f(i, p):
        z = z_top + (p.z - z_top) * k
        z -= sink * (1.0 - smoothstep(zmin1, zmin1 + 0.4 * h1, z))      # lower V-edge sinks into the chest
        return Vector((p.x, p.y, z))

    edit_world(o, f)
    return {"height_before": z_top - zmin0, "height_after": h1 + 0.0, "sink_mm": sink * 1e3}


# ---------------------------------------------------------------------------------------------------------
# Phase 3: feet (procedural shell with an exact dorsal radius), rubber sole, ankle cuff
# ---------------------------------------------------------------------------------------------------------
def _superellipse(theta, a, b, n=3.0):
    c, s = math.cos(theta), math.sin(theta)
    return (a * math.copysign(abs(c) ** (2 / n), c), b * math.copysign(abs(s) ** (2 / n), s))


def foot_solid(cx, cy, a, b, z_bottom, z_top, radius, toe_taper=0.20, segs=48, wall_rings=2, arc_rings=8, bottom_round=0.0):
    """Closed lofted solid: vertical wall then a quarter-circle dorsal edge of `radius` into a flat top deck."""
    bm = bmesh.new()
    rings = []
    zs = []
    for k in range(wall_rings):
        zs.append((z_bottom + (z_top - radius - z_bottom) * k / wall_rings, 0.0))
    for k in range(arc_rings + 1):
        phi = math.radians(90.0 * k / arc_rings)
        zs.append((z_top - radius + radius * math.sin(phi), radius * (1.0 - math.cos(phi))))
    for z, inset in zs:
        ring = []
        for i in range(segs):
            th = 2 * math.pi * i / segs
            x, y = _superellipse(th, a - inset, b - inset)
            t = max(0.0, (-y / max(b, 1e-6) - 0.15) / 0.85)             # toe is the -y end
            x *= 1.0 - toe_taper * (t * t * (3 - 2 * t))
            ring.append(bm.verts.new((cx + x, cy + y, z)))
        rings.append(ring)
    for r0, r1 in zip(rings, rings[1:]):
        for i in range(segs):
            bm.faces.new((r0[i], r0[(i + 1) % segs], r1[(i + 1) % segs], r1[i]))
    bm.faces.new(rings[0][::-1])
    bm.faces.new(rings[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    for f in bm.faces:
        f.smooth = True
    return bm


def _replace_mesh(o, bm, vgroup, mat):
    me = o.data
    me.clear_geometry()
    bm.to_mesh(me)
    bm.free()
    vg = o.vertex_groups.get(vgroup) or o.vertex_groups.new(name=vgroup)
    vg.add(list(range(len(me.vertices))), 1.0, "REPLACE")
    me.materials.clear()
    me.materials.append(mat)
    o.rotation_mode = "XYZ"
    o.rotation_euler = (0, 0, 0)
    o.location = (0, 0, 0)
    o.scale = (1, 1, 1)
    me.update()


def feet(arm, white, rubber, graphite, foot_len, foot_width, heel_y, sole_h=0.008, dorsal_r=0.018, top_z=0.085):
    out = {}
    cy = heel_y - foot_len / 2.0
    for side in ("L", "R"):
        o = O(f"411_Foot_{side}")
        W = world_verts(o)
        cx = sum(p.x for p in W) / len(W)
        bm = foot_solid(cx, cy, foot_width / 2, foot_len / 2, sole_h, top_z, dorsal_r)
        _replace_mesh(o, bm, "DEF_Foot" if side == "R" else "DEF_Foot_L", white)
        for m in list(o.modifiers):
            if m.type == "BEVEL":
                o.modifiers.remove(m)
        # rubber sole pad, 1.5 mm proud of the shell footprint
        name = f"411_Sole_{side}"
        so = bpy.data.objects.get(name)
        if so is None:
            me = bpy.data.meshes.new(name)
            so = bpy.data.objects.new(name, me)
            o.users_collection[0].objects.link(so)
            am = so.modifiers.new("Armature", "ARMATURE")
            am.object = arm
        bm2 = foot_solid(cx, cy, foot_width / 2 + 0.0015, foot_len / 2 + 0.0015, 0.0, sole_h, 0.0015, segs=48, wall_rings=1, arc_rings=3)
        _replace_mesh(so, bm2, "DEF_Foot" if side == "R" else "DEF_Foot_L", rubber)
        if not any(m.type == "BEVEL" for m in so.modifiers):
            bv = so.modifiers.new("Bevel", "BEVEL")
            bv.width = 0.0012
            bv.segments = 2
            bv.limit_method = "ANGLE"
            idx = [i for i, m in enumerate(so.modifiers) if m.type == "ARMATURE"][0]
            so.modifiers.move(len(so.modifiers) - 1, idx)
        # ankle cuff: lathe ring around the shin axis, seated on the foot top
        an = O(f"411_Ankle_{side}")
        Wa = world_verts(an)
        c = sum(Wa, Vector()) / len(Wa)
        sh = O(f"411_Shin_{side}")
        Ws = world_verts(sh)
        zmid = (min(p.z for p in Ws) + max(p.z for p in Ws)) / 2
        ct = sum([p for p in Ws if p.z > zmid], Vector()) / len([p for p in Ws if p.z > zmid])
        cb = sum([p for p in Ws if p.z <= zmid], Vector()) / len([p for p in Ws if p.z <= zmid])
        axis = (ct - cb).normalized()
        # axis-aligned profile (radius, height above the foot top): flared lower lip tucks onto the foot deck
        prof_out = [(0.071, 0.000), (0.069, 0.006), (0.0645, 0.014), (0.0635, 0.022), (0.0665, 0.030), (0.0675, 0.034)]
        prof_in = [(0.0555, 0.034), (0.0555, 0.020), (0.0575, 0.000)]
        prof = prof_out + prof_in
        base = Vector((c.x, c.y, 0.0)) + Vector((0, 0, top_z - 0.003))
        cuff_bm = bmesh.new()
        segs = 40
        rings = []
        b3 = Matrix.Rotation(0.0, 3, "Z")
        # orientation: local Z -> shin axis
        zaxis = axis
        xaxis = Vector((1, 0, 0)) - zaxis * zaxis.x
        xaxis.normalize()
        yaxis = zaxis.cross(xaxis)
        for r, h in prof:
            ring = []
            for i in range(segs):
                th = 2 * math.pi * i / segs
                p = base + xaxis * (r * math.cos(th)) + yaxis * (r * math.sin(th)) + zaxis * h
                ring.append(cuff_bm.verts.new(p))
            rings.append(ring)
        for r0, r1 in zip(rings, rings[1:] + rings[:1]):
            for i in range(segs):
                cuff_bm.faces.new((r0[i], r0[(i + 1) % segs], r1[(i + 1) % segs], r1[i]))
        bmesh.ops.recalc_face_normals(cuff_bm, faces=cuff_bm.faces)
        for f in cuff_bm.faces:
            f.smooth = True
        cname = f"411_AnkleCuff_{side}"
        co = bpy.data.objects.get(cname)
        if co is None:
            me = bpy.data.meshes.new(cname)
            co = bpy.data.objects.new(cname, me)
            o.users_collection[0].objects.link(co)
            am = co.modifiers.new("Armature", "ARMATURE")
            am.object = arm
        _replace_mesh(co, cuff_bm, "DEF_Foot" if side == "R" else "DEF_Foot_L", graphite)
        bv = co.modifiers.get("Bevel") or co.modifiers.new("Bevel", "BEVEL")
        bv.width = 0.0012
        bv.segments = 2
        bv.limit_method = "ANGLE"
        idx = [i for i, m in enumerate(co.modifiers) if m.type == "ARMATURE"][0]
        co.modifiers.move([m.name for m in co.modifiers].index("Bevel"), idx)
        out[side] = {"foot_center_x": cx, "cuff_axis": tuple(round(v, 3) for v in axis)}
    return out


# ---------------------------------------------------------------------------------------------------------
# Phase 4: infinity cyclorama and lighting rig
# ---------------------------------------------------------------------------------------------------------
def cyclorama(floor_y0=-16.0, wall_y=4.0, bevel_r=2.5, wall_h=14.0, half_w=18.0, arc_segs=32):
    """Curved cyc wall: flat floor -> quarter-circle of `bevel_r` -> vertical wall. Old two-plane backdrop is hidden, not deleted."""
    for n in ("Studio_Backdrop", "Studio_Floor"):
        o = O(n)
        o.hide_render = True
        o.hide_viewport = True
    prof = [(floor_y0, 0.0), (wall_y - bevel_r - 6.0, 0.0), (wall_y - bevel_r, 0.0)]
    for k in range(1, arc_segs):
        a = math.pi / 2 * k / arc_segs
        prof.append((wall_y - bevel_r + bevel_r * math.sin(a), bevel_r - bevel_r * math.cos(a)))
    prof += [(wall_y, bevel_r), (wall_y, wall_h)]
    bm = bmesh.new()
    left = [bm.verts.new((-half_w, y, z)) for y, z in prof]
    right = [bm.verts.new((half_w, y, z)) for y, z in prof]
    for i in range(len(prof) - 1):
        bm.faces.new((left[i], right[i], right[i + 1], left[i + 1]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    for f in bm.faces:
        f.smooth = True
    name = "Studio_Cyc"
    me = bpy.data.meshes.get(name) or bpy.data.meshes.new(name)
    me.clear_geometry()
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.get(name) or bpy.data.objects.new(name, me)
    if o.name not in bpy.context.scene.collection.objects:
        O("Studio_Floor").users_collection[0].objects.link(o)
    mat = bpy.data.materials.get("Studio_Cyc_Mat") or bpy.data.materials.new("Studio_Cyc_Mat")
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
    tc = nt.nodes.new("ShaderNodeTexCoord")
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    nt.links.new(tc.outputs["Object"], sep.inputs[0])
    nt.links.new(sep.outputs["Z"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], bsdf.inputs["Base Color"])
    nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    ramp.color_ramp.elements[0].position = 0.0
    ramp.color_ramp.elements[0].color = (0.46, 0.43, 0.40, 1.0)     # warm grey at the floor
    ramp.color_ramp.elements[1].position = 6.0
    ramp.color_ramp.elements[1].color = (0.17, 0.165, 0.165, 1.0)  # darker neutral grey high on the wall
    bsdf.inputs["Roughness"].default_value = 0.92
    bsdf.inputs["Specular IOR Level"].default_value = 0.15
    me.materials.clear()
    me.materials.append(mat)
    return {"profile_points": len(prof), "bevel_radius": bevel_r}


def lighting_rig():
    """3-point + dual edge rims. Existing Key/Fill/Rim keep their names; two strip rims are added behind-left / behind-right."""
    key, fill, rim = O("Studio_Key"), O("Studio_Fill"), O("Studio_Rim")
    key.data.energy, key.data.size, key.data.color = 210.0, 2.4, (1.0, 0.97, 0.93)
    fill.data.energy, fill.data.size, fill.data.color = 70.0, 5.0, (0.92, 0.95, 1.0)
    rim.data.energy, rim.data.size, rim.data.color = 330.0, 2.0, (1.0, 0.85, 0.65)
    made = []
    for side, x in (("L", -2.4), ("R", 2.4)):
        name = f"Studio_Edge{side}"
        ld = bpy.data.lights.get(name) or bpy.data.lights.new(name, "AREA")
        ld.shape = "RECTANGLE"
        ld.size, ld.size_y = 0.35, 3.2
        ld.energy = 430.0
        ld.color = (0.95, 0.97, 1.0)
        lo = bpy.data.objects.get(name) or bpy.data.objects.new(name, ld)
        if lo.name not in bpy.context.scene.collection.objects:
            key.users_collection[0].objects.link(lo)
        lo.location = Vector((x, 2.2, 1.4))
        look = Vector((0.0, 0.0, 1.2)) - lo.location
        lo.rotation_euler = look.to_track_quat("-Z", "Y").to_euler()
        made.append(name)
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.samples = 128
    sc.cycles.use_denoising = True
    sc.cycles.denoiser = "OPENIMAGEDENOISE"
    return {"lights": made, "samples": 128, "denoiser": sc.cycles.denoiser}


def smooth_joint_spheres(levels=2):
    """The joint spheres are coarse UV spheres (smooth-shaded, faceted geometry). Where a white shell intersects one, the
    boundary follows the facets (visible in close-ups). A Subdivision modifier placed before the Armature fixes it
    non-destructively and is baked into the glTF by the exporter."""
    names = []
    for base in ("Shoulder", "Elbow", "Wrist", "Knee", "Ankle", "HipSocket", "ThumbJoint"):
        for side in ("L", "R"):
            names.append(f"411_{base}_{side}")
    done = []
    for n in names:
        o = O(n)
        if any(m.type == "SUBSURF" for m in o.modifiers):
            continue
        sm = o.modifiers.new("Subdivision", "SUBSURF")
        sm.levels = levels
        sm.render_levels = levels
        idx = [i for i, m in enumerate(o.modifiers) if m.type == "ARMATURE"][0]
        o.modifiers.move(len(o.modifiers) - 1, idx)
        done.append(n)
    return done
