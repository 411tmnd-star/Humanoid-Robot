"""v18_hand_rig.py -- SCRIPT_VERSION 1.0

Purpose: v18 Phase 1. Eliminate the finger / thumb zero-command displacement and give the hands a natural neutral pose.
Imported by build_v18.py (it needs an open scene that already is robot_armature_v17.blend).

Root cause being fixed (see docs/phase_v18_refine.md): DEF_*_MCP and DEF_Thumb_CMC copy rotation from their MECH driver in
POSE space (absolute orientation) while the MECH/CTRL finger bones all shared one "rack" frame unrelated to the DEF frames
(rack X axis was vertical, so even a corrected offset would have curled fingers about the wrong axis).

Fix (option 1 of the brief, plus a defined axis contract):
  * DEF finger / thumb / palm bones are re-posed (heads, tails) and RE-ROLLED so that local X = flexion axis
    (positive = toward the palm) and local Z = abduction axis (= palm normal, positive = away from the index side for
    the right hand). Heads/tails of connected chains stay consistent.
  * MECH and CTRL bones are placed on the exact head/tail/roll of the DEF bone they drive. The POSE-space absolute copy is then
    an identity at rest, and LOCAL copies act about the right axes.
  * Left hand = world mirror of the right using x_L = -S x_R, y_L = S y_R, z_L = S z_R: the SAME flex value curls both hands,
    abduction mirrors with opposite sign. Left Z limits are the right limits negated.
  * Neutral pose: palms inward, MCP 12 / PIP 18 / DIP 10 deg flexion (cumulative 12 / 30 / 40 from the hand axis), thumb 25 deg
    forward of the hand axis with 8 / 8 deg of curl.
"""
import math

import bpy
import bmesh
from mathutils import Matrix, Vector

SCRIPT_VERSION = "1.0"
S = Matrix.Diagonal((-1, 1, 1))

FINGERS = ("Index", "Middle", "Ring", "Little")
CURL = (math.radians(12.0), math.radians(18.0), math.radians(10.0))   # MCP, PIP, DIP joint flexion
FINGER_PITCH = 0.032
THUMB_OPPOSE = math.radians(25.0)
THUMB_CURL = (math.radians(8.0), math.radians(8.0))


def _req(c, m):
    if not c:
        raise RuntimeError("v18 hand rig: " + m)


def reflect(v):
    return Vector((-v[0], v[1], v[2]))


class Frames:
    """World-space (head, tail, y, z) targets by bone name."""
    def __init__(self):
        self.d = {}

    def set(self, name, head, tail, ztarget):
        self.d[name] = (head, tail, ztarget)


def _bone_w(arm, name):
    b = arm.data.bones[name]
    M = arm.matrix_world
    return M @ b.head_local, M @ b.tail_local, (M.to_3x3() @ b.matrix_local.to_3x3())


def _rot_about(axis, origin, angle):
    R = Matrix.Rotation(angle, 3, axis)
    return lambda p: origin + R @ (p - origin), R


def plan_right_hand(arm):
    """Compute every target frame for the RIGHT hand chain (world space). No scene edits."""
    M = arm.matrix_world
    B = arm.data.bones
    W, _, wm = _bone_w(arm, "DEF_Wrist")
    a = wm.col[1].normalized()                            # forearm / hand axis, pointing toward the fingertips
    # rotation of the whole hand about the arm axis: choose the sign that sends the thumb forward (-Y)
    thumb_head, _, _ = _bone_w(arm, "DEF_Thumb_CMC")
    palm_c = sum((_bone_w(arm, f"DEF_Palm_{f}")[0] + _bone_w(arm, f"DEF_Palm_{f}")[1] for f in FINGERS), Vector()) / 8
    chosen = None
    for ang in (math.radians(90), math.radians(-90)):
        mover, R = _rot_about(a, W, ang)
        if (mover(thumb_head) - mover(palm_c)).y < -0.005:
            chosen = (ang, mover, R)
            break
    _req(chosen is not None, "could not find a rotation that sends the thumb forward")
    ang, mover, Rhand = chosen

    # frame vectors, orthogonal to the hand axis
    def perp(v):
        v = v - a * v.dot(a)
        return v.normalized()
    p = perp(Vector((-1, 0, 0)))                          # palm side: inward for the +X hand
    fwd = perp(Vector((0, -1, 0)))                        # thumb opposition direction: forward
    e = a.cross(p).normalized()                           # flexion axis: +rotation about e carries a toward p

    plan = Frames()
    info = {"hand_axis": a, "palm_dir": p, "forward": fwd, "flex_axis": e, "rot_deg": math.degrees(ang), "mover": mover, "Rhand": Rhand}
    mcp_heads = {}

    def seg_frame(head, d, axis):
        d = d.normalized()
        x = (axis - d * axis.dot(d)).normalized()          # flexion axis made exactly orthogonal to the segment
        z = x.cross(d).normalized()
        return head, head + d * 1.0, z                    # tail length applied by caller

    # palm bones: keep length and endpoints (rotated), re-roll so local X is the flexion axis
    for f in FINGERS:
        h, t, _ = _bone_w(arm, f"DEF_Palm_{f}")
        h2, t2 = mover(h), mover(t)
        x_t = (e - (t2 - h2).normalized() * e.dot((t2 - h2).normalized())).normalized()
        plan.set(f"DEF_Palm_{f}", h2, t2, x_t.cross((t2 - h2).normalized()).normalized())

    # fingers. The inherited v16 MCP mounts overlap in pairs, so they are re-spaced evenly across the palm:
    # lateral (forward -> back) offsets 1.5 / 0.5 / -0.5 / -1.5 x FINGER_PITCH about the centroid, one palm-plane depth.
    rotated = {f: mover(_bone_w(arm, f"DEF_{f}_MCP")[0]) for f in FINGERS}
    centroid = sum(rotated.values(), Vector()) / 4
    lateral = dict(zip(FINGERS, (1.5, 0.5, -0.5, -1.5)))
    axial = dict(zip(FINGERS, (0.0, 0.004, 0.0, -0.006)))           # middle reaches furthest, little shortest
    for f in FINGERS:
        names = (f"DEF_{f}_MCP", f"DEF_{f}_PIP", f"DEF_{f}_Tip")
        c = centroid - a * centroid.dot(a) - p * centroid.dot(p)       # centroid projected onto the lateral axis only
        head = (a * centroid.dot(a) + p * centroid.dot(p) + c * 0 + a * axial[f]
                + fwd * (centroid.dot(fwd) + lateral[f] * FINGER_PITCH))
        mcp_heads[f] = head
        cum = 0.0
        for nm, curl in zip(names, CURL):
            L = B[nm].length
            cum += curl
            d = a * math.cos(cum) + p * math.sin(cum)
            hd, tl, z = seg_frame(head, d, e)
            tail = head + d * L
            plan.set(nm, head, tail, z)
            head = tail
    # thumb chain (connected): CMC -> MCP -> IP
    tn = ("DEF_Thumb_CMC", "DEF_Thumb_MCP", "DEF_Thumb_IP")
    h0, _, _ = _bone_w(arm, tn[0])
    head = mover(h0)
    t_base = a * math.cos(THUMB_OPPOSE) + fwd * math.sin(THUMB_OPPOSE)
    curls = (0.0, THUMB_CURL[0], THUMB_CURL[0] + THUMB_CURL[1])
    for nm, c in zip(tn, curls):
        L = B[nm].length
        d = (t_base * math.cos(c) + p * math.sin(c)).normalized()
        ax = d.cross(p).normalized()                        # thumb flexion axis for this segment (+rot carries d toward p)
        x = ax
        z = x.cross(d).normalized()
        tail = head + d * L
        plan.set(nm, head, tail, z)
        head = tail

    # MECH / CTRL sit exactly on the DEF bone they drive
    follow = {}
    for f in FINGERS:
        follow[f"MECH_{f}_MCP_Flex"] = f"DEF_{f}_MCP"
        follow[f"CTRL_{f}_MCP_Flex"] = f"DEF_{f}_MCP"
        follow[f"MECH_{f}_MCP_Abd"] = f"DEF_{f}_MCP"
        follow[f"CTRL_{f}_MCP_Abd"] = f"DEF_{f}_MCP"
        follow[f"MECH_{f}_PIP"] = f"DEF_{f}_PIP"
        follow[f"CTRL_{f}_PIP"] = f"DEF_{f}_PIP"
        follow[f"MECH_{f}_DIP"] = f"DEF_{f}_Tip"
    for f in ("Ring", "Little"):
        follow[f"MECH_Palm_{f}"] = f"DEF_Palm_{f}"
        follow[f"CTRL_Palm_{f}"] = f"DEF_Palm_{f}"
    for pre in ("MECH", "CTRL"):
        follow[f"{pre}_Thumb_CMC_Flex"] = "DEF_Thumb_CMC"
        follow[f"{pre}_Thumb_CMC_Abd"] = "DEF_Thumb_CMC"
        follow[f"{pre}_Thumb_MCP"] = "DEF_Thumb_MCP"
        follow[f"{pre}_Thumb_IP"] = "DEF_Thumb_IP"
    for dst, src in follow.items():
        if dst in B:                                       # CTRL_Middle_* / CTRL_*_DIP do not exist: skip, not an error
            plan.d[dst] = plan.d[src]
    info["mcp_heads"] = mcp_heads
    return plan, info


def mirrored(plan):
    """Left-hand targets: head/tail mirrored, z target = S z (x_L = -S x_R, y_L = S y_R, z_L = S z_R)."""
    out = Frames()
    for n, (h, t, z) in plan.d.items():
        out.d[n + "_L"] = (reflect(h), reflect(t), reflect(z))
    return out


def apply_plan(arm, plan):
    """Edit-mode write of head / tail / roll for every planned bone. Connected children take the parent's tail."""
    Mi = arm.matrix_world.inverted()
    Ri = Mi.to_3x3()
    bpy.context.view_layer.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    try:
        E = arm.data.edit_bones
        missing = [n for n in plan.d if n not in E]
        _req(not missing, f"planned bones missing: {missing[:5]}")
        for n, (h, t, z) in plan.d.items():
            eb = E[n]
            eb.head = Mi @ h
            eb.tail = Mi @ t
            eb.align_roll(Ri @ z)
        for n in plan.d:                                  # connected bones must still be connected and coincident
            eb = E[n]
            if eb.use_connect:
                _req((eb.head - eb.parent.tail).length < 1e-5, f"{n}: connected head != parent tail")
    finally:
        bpy.ops.object.mode_set(mode="OBJECT")


def fix_limits_and_locks(arm):
    """Left Z limits = right Z limits negated (z_L = S z_R); X limits and lock flags copied from the right."""
    changed = []
    P = arm.pose.bones
    for pb in P:
        if not pb.name.endswith("_L"):
            continue
        r = P.get(pb.name[:-2])
        if r is None or not any(k in pb.name for k in ("Palm_", "_MCP", "_PIP", "_DIP", "Thumb_")):
            continue
        pb.lock_rotation = tuple(r.lock_rotation)
        for cl, cr in zip([c for c in pb.constraints if c.type == "LIMIT_ROTATION"],
                          [c for c in r.constraints if c.type == "LIMIT_ROTATION"]):
            for ax in "xyz":
                use = getattr(cr, "use_limit_" + ax)
                setattr(cl, "use_limit_" + ax, use)
                lo, hi = getattr(cr, "min_" + ax), getattr(cr, "max_" + ax)
                if ax == "z":
                    lo, hi = -hi, -lo
                setattr(cl, "min_" + ax, lo)
                setattr(cl, "max_" + ax, hi)
            changed.append(pb.name)
    return changed


def rigid_follow(o, old, new):
    """Move mesh o by the rigid transform that takes the old bone frame (4x4 world) to the new one."""
    T = new @ old.inverted()
    M = o.matrix_world
    Mi = M.inverted()
    for v in o.data.vertices:
        v.co = Mi @ (T @ (M @ v.co))
    o.data.update()


def bone_world_matrix(arm, name):
    b = arm.data.bones[name]
    return arm.matrix_world @ b.matrix_local


def snapshot_mats(arm, names):
    return {n: bone_world_matrix(arm, n).copy() for n in names}
