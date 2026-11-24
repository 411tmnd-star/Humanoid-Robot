"""v18_hand_mesh.py -- SCRIPT_VERSION 1.0

Purpose: v18 hand geometry. Rebuilds the finger shells as three white phalanges separated by dark cylindrical knuckles, on
the re-posed DEF bones. Existing meshes are re-used (names, vertex groups, modifiers kept); only new knuckle / distal
phalanx objects are added. Everything is skinned 100 % to one DEF bone like the rest of the robot.
"""
import math

import bpy
import bmesh
from mathutils import Matrix, Vector

SCRIPT_VERSION = "1.0"
R_PHAL = (0.0135, 0.0125, 0.0115)     # proximal, middle, distal shell radius (m)
R_KNUCKLE = 0.0150                    # knuckle cylinder radius
W_KNUCKLE = 0.029                     # knuckle cylinder length along the flexion axis
GAP = 0.008                           # white shell stops this far from the joint centre (tucks into the knuckle)


def _basis(d, x_hint):
    d = d.normalized()
    x = (x_hint - d * x_hint.dot(d)).normalized()
    y = d.cross(x).normalized()          # right-handed (x, y, d)
    return Matrix(((x[0], y[0], d[0]), (x[1], y[1], d[1]), (x[2], y[2], d[2])))


def _mesh_from_bm(bm, o):
    me = o.data
    me.clear_geometry()
    bm.to_mesh(me)
    bm.free()
    for p in me.polygons:
        p.use_smooth = True
    me.update()


def _link_new(name, like):
    me = bpy.data.meshes.new(name)
    o = bpy.data.objects.new(name, me)
    like.users_collection[0].objects.link(o)
    return o


def _finish(o, arm, bone, mat, bevel):
    if o.vertex_groups.get(bone) is None:
        vg = o.vertex_groups.new(name=bone)
    else:
        vg = o.vertex_groups[bone]
    vg.add(list(range(len(o.data.vertices))), 1.0, "REPLACE")
    if not o.material_slots:
        o.data.materials.append(mat)
    else:
        o.material_slots[0].material = mat
    for m in list(o.modifiers):
        if m.type in ("BEVEL", "ARMATURE"):
            o.modifiers.remove(m)
    bv = o.modifiers.new("Bevel", "BEVEL")
    bv.width = bevel
    bv.segments = 3
    bv.limit_method = "ANGLE"
    bv.angle_limit = math.radians(35)
    am = o.modifiers.new("Armature", "ARMATURE")
    am.object = arm
    o.rotation_mode = "XYZ"
    o.rotation_euler = (0, 0, 0)
    o.location = (0, 0, 0)
    o.scale = (1, 1, 1)


def cylinder(o, center, axis, x_hint, r0, r1, length, segs=16):
    bm = bmesh.new()
    B = _basis(axis, x_hint).to_4x4()
    B.translation = center
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=segs, radius1=r0, radius2=r1, depth=length, matrix=B)
    _mesh_from_bm(bm, o)


def bone_axes(arm, name):
    b = arm.data.bones[name]
    M = arm.matrix_world
    m = M.to_3x3() @ b.matrix_local.to_3x3()
    return M @ b.head_local, M @ b.tail_local, m.col[0].normalized(), m.col[1].normalized()


def build_chain(arm, like, white, graphite, label, bones, scale=1.0):
    """bones: [(bone_name, shell_object_or_None, shell_radius)] in proximal -> distal order.
    A knuckle sits at the head of every bone; the shell of each bone runs head+GAP -> tail-GAP (distal: to the tip)."""
    made = []
    for i, (bn, shell_obj, r) in enumerate(bones):
        head, tail, x, y = bone_axes(arm, bn)
        last = i == len(bones) - 1
        r = r * scale
        a = head + y * GAP * scale
        b = tail - (Vector((0, 0, 0)) if last else y * GAP * scale)
        o = shell_obj
        if o is None:
            o = _link_new(f"411_{label}_Phalanx{i + 1}", like)
        cylinder(o, (a + b) / 2, y, x, r * 1.08 if i == 0 else r * 1.04, r * 0.92 if last else r * 0.97, (b - a).length)
        _finish(o, arm, bn, white, 0.0035 if not last else 0.006)
        made.append(o)
        k = _link_new(f"411_{label}_Knuckle{i + 1}", like)
        cylinder(k, head, x, y, R_KNUCKLE * scale, R_KNUCKLE * scale, W_KNUCKLE * scale)
        _finish(k, arm, bn, graphite, 0.002)
        made.append(k)
    return made
