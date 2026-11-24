"""bpy adapter: read the loaded scene into plain dicts. Read-only.

Nothing here writes to bpy.data except the temporary evaluated mesh created
by evaluated_bounds(), which is released with to_mesh_clear() and never
saved. Base geometry (mesh data) and evaluated geometry (depsgraph) are kept
in separate functions and separate output keys so they are never mixed.
"""

import json

import bmesh
import bpy


def m4(mat):
    return [[float(v) for v in row] for row in mat]


def v3(v):
    return (float(v[0]), float(v[1]), float(v[2]))


def scene_info():
    s = bpy.context.scene
    us = s.unit_settings
    props = {}
    for k in s.keys():
        val = s[k]
        if isinstance(val, str):
            try:
                props[k] = json.loads(val)
            except ValueError:
                props[k] = val
        elif isinstance(val, (int, float, bool)):
            props[k] = val
        else:
            props[k] = str(val)
    return {
        "scene": s.name,
        "unit_system": us.system,
        "length_unit": us.length_unit,
        "scale_length": float(us.scale_length),
        "frame_current": s.frame_current,
        "frame_range": [s.frame_start, s.frame_end],
        "custom_properties": props,
        "blender_version": bpy.app.version_string,
    }


def modifier_rec(m):
    return {
        "name": m.name, "type": m.type,
        "target": m.object.name if getattr(m, "object", None) else None,
        "use_vertex_groups": bool(getattr(m, "use_vertex_groups", False)),
        "show_viewport": bool(m.show_viewport), "show_render": bool(m.show_render),
    }


def objects_info():
    out = []
    for o in sorted(bpy.data.objects, key=lambda x: x.name):
        out.append({
            "name": o.name, "type": o.type, "data": o.data.name if o.data else None,
            "parent": o.parent.name if o.parent else None,
            "parent_type": o.parent_type,
            "parent_bone": o.parent_bone or None,
            "collections": sorted(c.name for c in o.users_collection),
            "hide_viewport": bool(o.hide_viewport), "hide_render": bool(o.hide_render),
            "matrix_world": m4(o.matrix_world),
            "location": v3(o.location), "rotation_euler": v3(o.rotation_euler),
            "rotation_mode": o.rotation_mode, "scale": v3(o.scale),
            "modifiers": [modifier_rec(m) for m in o.modifiers],
            "materials": [s.material.name if s.material else None for s in o.material_slots],
            "vertex_groups": [g.name for g in o.vertex_groups] if o.type == "MESH" else [],
            "constraints": [constraint_rec(o.name, c) for c in o.constraints],
        })
    return out


def mesh_data(o):
    """Base (unevaluated) mesh: local and world coordinates, connectivity and
    a per-group count of vertices with weight > 0."""
    me = o.data
    mw = o.matrix_world
    local = [v3(v.co) for v in me.vertices]
    world = [v3(mw @ v.co) for v in me.vertices]
    edges = [tuple(e.vertices) for e in me.edges]
    faces = [tuple(p.vertices) for p in me.polygons]
    names = [g.name for g in o.vertex_groups]
    nonzero = {n: 0 for n in names}
    unweighted, max_inf, sum_off = 0, 0, 0
    for v in me.vertices:
        inf = [(g.group, g.weight) for g in v.groups if g.weight > 0]
        if not inf:
            unweighted += 1
        max_inf = max(max_inf, len(inf))
        if inf and abs(sum(w for _, w in inf) - 1.0) > 1e-3:
            sum_off += 1
        for gi, _ in inf:
            if gi < len(names):
                nonzero[names[gi]] += 1
    return {
        "name": o.name, "local": local, "world": world, "edges": edges, "faces": faces,
        "vgroup_names": names, "vgroup_nonzero": nonzero,
        "unweighted_vertices": unweighted, "max_influences": max_inf,
        "weight_sum_off_1": sum_off,
        "armature_mods": [modifier_rec(m) for m in o.modifiers if m.type == "ARMATURE"],
    }


def evaluated_bounds(objs):
    """Depsgraph-evaluated world bounds at the file's current frame."""
    dg = bpy.context.evaluated_depsgraph_get()
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    per = {}
    for o in objs:
        ev = o.evaluated_get(dg)
        me = ev.to_mesh()
        try:
            mw = ev.matrix_world
            pts = [mw @ v.co for v in me.vertices]
            if pts:
                olo = [min(p[i] for p in pts) for i in range(3)]
                ohi = [max(p[i] for p in pts) for i in range(3)]
                per[o.name] = {"min": olo, "max": ohi, "vertices": len(pts)}
                for i in range(3):
                    lo[i] = min(lo[i], olo[i])
                    hi[i] = max(hi[i], ohi[i])
        finally:
            ev.to_mesh_clear()
    if not per:
        return {"min": None, "max": None, "per_object": {}}
    return {"min": lo, "max": hi, "per_object": per,
            "frame": bpy.context.scene.frame_current}


def constraint_rec(owner, c):
    return {
        "owner": owner, "name": c.name, "type": c.type,
        "target": c.target.name if getattr(c, "target", None) else None,
        "subtarget": getattr(c, "subtarget", None) or None,
        "pole_target": c.pole_target.name if getattr(c, "pole_target", None) else None,
        "influence": float(c.influence), "mute": bool(c.mute),
        "owner_space": getattr(c, "owner_space", None),
        "target_space": getattr(c, "target_space", None),
        "is_valid": bool(getattr(c, "is_valid", True)),
    }


def armature_info(arm_obj):
    mw = arm_obj.matrix_world
    rot = mw.to_3x3()
    bones = []
    for b in arm_obj.data.bones:
        axes = rot @ b.matrix_local.to_3x3()
        bones.append({
            "name": b.name, "parent": b.parent.name if b.parent else None,
            "head_local": v3(b.head_local), "tail_local": v3(b.tail_local),
            "head_w": v3(mw @ b.head_local), "tail_w": v3(mw @ b.tail_local),
            "length": float(b.length), "use_deform": bool(b.use_deform),
            "use_connect": bool(b.use_connect),
            "axis_x_w": v3(axes.col[0].normalized()), "axis_y_w": v3(axes.col[1].normalized()),
            "axis_z_w": v3(axes.col[2].normalized()),
        })
    cons = []
    for pb in arm_obj.pose.bones:
        for c in pb.constraints:
            cons.append(constraint_rec(pb.name, c))
    ad = arm_obj.animation_data
    return {
        "object": arm_obj.name, "matrix_world": m4(mw),
        "rotation_euler": v3(arm_obj.rotation_euler), "scale": v3(arm_obj.scale),
        "pose_position": arm_obj.data.pose_position,
        "active_action": ad.action.name if ad and ad.action else None,
        "bones": bones, "pose_constraints": cons,
    }


def recalc_flip_count(o):
    """Independent orientation check: how many faces Blender's own
    recalc_face_normals would flip. Works on a temporary bmesh copy."""
    bm = bmesh.new()
    try:
        bm.from_mesh(o.data)
        bm.normal_update()
        bm.faces.ensure_lookup_table()
        before = [f.normal.copy() for f in bm.faces]
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        bm.normal_update()
        bm.faces.ensure_lookup_table()
        return sum(1 for f, v in zip(bm.faces, before) if f.normal.dot(v) < 0)
    finally:
        bm.free()


def mesh_tris(o):
    """Triangles exactly as Blender tessellates them (loop triangles), with
    vertices in WORLD space and the source polygon index of each triangle.
    calc_loop_triangles() refreshes a runtime cache only; mesh data and the
    file are untouched."""
    me = o.data
    me.calc_loop_triangles()
    mw = o.matrix_world
    return {
        "name": o.name,
        "world": [v3(mw @ v.co) for v in me.vertices],
        "tris": [tuple(int(i) for i in t.vertices) for t in me.loop_triangles],
        "poly": [int(t.polygon_index) for t in me.loop_triangles],
    }


def blender_bvh_overlap(world_a, tris_a, world_b, tris_b):
    """Triangle pairs Blender's BVHTree.overlap() reports for two meshes.

    VERIFIED BEHAVIOUR (not assumed): overlap() is NOT a bounding-box
    collector. For two triangles whose boxes overlap but whose surfaces are
    apart it returns nothing, i.e. Blender applies its own float32
    triangle-triangle test. It is therefore used here as an INDEPENDENT
    INTERSECTION IMPLEMENTATION to compare against core/intersect, not as a
    candidate generator. float32 precision means it can miss crossings that
    exact arithmetic confirms (seen on the real asset)."""
    from mathutils.bvhtree import BVHTree
    a = BVHTree.FromPolygons(world_a, tris_a, all_triangles=True, epsilon=0.0)
    b = BVHTree.FromPolygons(world_b, tris_b, all_triangles=True, epsilon=0.0)
    return sorted((int(i), int(j)) for i, j in a.overlap(b))


def blender_bvh_self_overlap(world, tris):
    from mathutils.bvhtree import BVHTree
    t = BVHTree.FromPolygons(world, tris, all_triangles=True, epsilon=0.0)
    return sorted({(min(int(i), int(j)), max(int(i), int(j))) for i, j in t.overlap(t) if i != j})
