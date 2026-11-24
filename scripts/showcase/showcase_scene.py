"""showcase_scene.py -- shared scene setup for the portfolio renders of the v20 robot.

Opens the robot .blend, hides the film set, freezes the boolean panel seams once, lights the studio cyclorama and
solves cameras from projected geometry. Everything happens in memory: the .blend is never saved.
Needs bpy (Blender 5.2, or the bpy 5.2 wheel). The motion itself lives in wave_motion.py, which needs no Blender.
"""
import math

import bpy
import numpy as np
from mathutils import Vector

from wave_motion import CHANNELS, LIMITS, WAVE_FRAMES, wave_angles

SCRIPT_VERSION = "1.0"
ARMATURE = "RobotArmature"
MESH_PREFIX = "411_"
EXCLUDE = {"411_Visor_Patch_backup"}
FILL_MIN, FILL_MAX, FILL_TARGET = 0.75, 0.82, 0.785
WALL_Y = 4.0  # back wall of Studio_Cyc


def robot_meshes():
    return [o for o in bpy.data.objects if o.type == "MESH" and o.name.startswith(MESH_PREFIX) and o.name not in EXCLUDE]


def open_robot(blend):
    bpy.ops.wm.open_mainfile(filepath=blend)
    for o in bpy.data.objects:  # the film set, its cameras and the old two-plane studio are not part of this scene
        if o.name.startswith(("ENV_", "CINE_")) or o.name in EXCLUDE or o.name in ("Studio_Backdrop", "Studio_Floor"):
            o.hide_render = o.hide_viewport = True
    arm = bpy.data.objects[ARMATURE]
    if arm.data.pose_position != "POSE":
        raise RuntimeError("armature is not in POSE mode")
    arm.animation_data.action = None
    reset_pose(arm)
    return arm


def reset_pose(arm):
    for pb in arm.pose.bones:
        pb.location, pb.rotation_euler = (0, 0, 0), (0, 0, 0)
        pb.rotation_quaternion, pb.scale = (1, 0, 0, 0), (1, 1, 1)


def freeze_seams(arm):
    """Evaluate each exact-boolean seam mesh once (armature off) so every later frame is cheap."""
    seam = [o for o in robot_meshes() if any(m.type == "BOOLEAN" for m in o.modifiers)]
    for o in seam:  # switch every boolean off first, so freezing one mesh does not re-evaluate the others
        for m in o.modifiers:
            m.show_viewport = False
    frozen = []
    for o in seam:
        arms = [m for m in o.modifiers if m.type == "ARMATURE"]
        for m in o.modifiers:
            m.show_viewport = m.type != "ARMATURE"
        bpy.context.view_layer.update()
        dg = bpy.context.evaluated_depsgraph_get()
        new = bpy.data.meshes.new_from_object(o.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
        o.data = new
        for m in list(o.modifiers):
            if m.type != "ARMATURE":
                o.modifiers.remove(m)
        for m in arms:
            m.show_viewport = True
        bad = sum(1 for v in new.vertices if not v.groups)
        if bad:
            raise RuntimeError(f"{o.name}: {bad} unweighted vertices after freezing")
        frozen.append([o.name, len(new.vertices)])
    for o in bpy.data.objects:
        if o.name.startswith("CUT_"):
            o.hide_viewport = o.hide_render = True
    return frozen


def check_limits(arm):
    """Differences between wave_motion.LIMITS and the live Limit Rotation constraints (empty list = match)."""
    bad = []
    for bone, (ax, lo, hi) in LIMITS.items():
        con = next((c for c in arm.pose.bones[bone].constraints if c.type == "LIMIT_ROTATION" and not c.mute), None)
        if con is None:
            bad.append(f"{bone}: no limit constraint")
            continue
        got = (round(math.degrees(getattr(con, f"min_{ax}"))), round(math.degrees(getattr(con, f"max_{ax}"))))
        if got != (lo, hi):
            bad.append(f"{bone}: {got} != {(lo, hi)}")
    return bad


def apply_angles(arm, angles):
    for bone, deg in angles.items():
        axis, mech = CHANNELS[bone]
        lo, hi = LIMITS[mech][1:]
        if not lo - 1e-6 <= deg <= hi + 1e-6:
            raise RuntimeError(f"{bone} {deg:.2f} deg is outside {lo}..{hi}")
        e = [0.0, 0.0, 0.0]
        e[axis] = math.radians(deg)
        arm.pose.bones[bone].rotation_euler = e


def key_wave(arm):
    act = bpy.data.actions.new("showcase_wave")
    arm.animation_data.action = act
    for f in range(WAVE_FRAMES + 1):
        ang = wave_angles(f)
        apply_angles(arm, ang)
        for bone in ang:
            arm.pose.bones[bone].keyframe_insert("rotation_euler", frame=f)
    return act


def spin_root(arm):
    """Parent every top-level robot object to one empty, so a turntable turns the robot and keeps the light fixed."""
    root = bpy.data.objects.new("SHOWCASE_Spin", None)
    bpy.context.scene.collection.objects.link(root)
    tops = set()
    for o in robot_meshes() + [arm]:
        while o.parent is not None:
            o = o.parent
        tops.add(o)
    for o in tops:
        o.parent = root
    return root


def _area(name, loc, target, energy, size, color, size_y=None, spread=180):
    d = bpy.data.lights.new(name, "AREA")
    d.energy, d.color, d.size = energy, color, size
    if size_y:
        d.shape, d.size_y = "RECTANGLE", size_y
    d.spread = math.radians(spread)
    o = bpy.data.objects.new(name, d)
    o.location = loc
    o.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    o.visible_camera = False
    bpy.context.scene.collection.objects.link(o)
    return o


def _cyc_material():
    """Dark graphite sweep with a soft glow on the wall. The glow centre (SC_GlowX) is set per camera."""
    mat = bpy.data.materials.new("SC_Cyc")
    mat.use_nodes = True
    nt = mat.node_tree
    p = next(n for n in nt.nodes if n.type == "BSDF_PRINCIPLED")
    p.inputs["Base Color"].default_value = (0.036, 0.043, 0.052, 1)
    p.inputs["Roughness"].default_value = 0.46

    def math_node(op, x, y=None):
        n = nt.nodes.new("ShaderNodeMath")
        n.operation = op
        for i, v in enumerate((x, y)):
            if v is None:
                continue
            if isinstance(v, (int, float)):
                n.inputs[i].default_value = v
            else:
                nt.links.new(v, n.inputs[i])
        return n.outputs[0]

    geo = nt.nodes.new("ShaderNodeNewGeometry")
    xyz = nt.nodes.new("ShaderNodeSeparateXYZ")
    nt.links.new(geo.outputs["Position"], xyz.inputs[0])
    gx = nt.nodes.new("ShaderNodeValue")
    gx.name = "SC_GlowX"
    gx.outputs[0].default_value = 0.0
    dx = math_node("SUBTRACT", xyz.outputs["X"], gx.outputs[0])
    dz = math_node("SUBTRACT", xyz.outputs["Z"], 1.75)
    r2 = math_node("ADD", math_node("DIVIDE", math_node("MULTIPLY", dx, dx), 7.0), math_node("DIVIDE", math_node("MULTIPLY", dz, dz), 3.4))
    glow = math_node("POWER", math.e, math_node("MULTIPLY", r2, -1.0))
    wall = math_node("MULTIPLY", math_node("SUBTRACT", xyz.outputs["Y"], WALL_Y - 2.6), 1 / 2.6)
    wall = math_node("MINIMUM", math_node("MAXIMUM", wall, 0.0), 1.0)
    p.inputs["Emission Color"].default_value = (0.36, 0.52, 0.66, 1)
    nt.links.new(math_node("MULTIPLY", math_node("MULTIPLY", glow, wall), 0.17), p.inputs["Emission Strength"])
    return mat


def set_glow(x):
    next(n for n in bpy.data.materials["SC_Cyc"].node_tree.nodes if n.name == "SC_GlowX").outputs[0].default_value = x


def light_studio():
    """Studio cyclorama from v18, re-lit: key, fill, top, and a cool and a warm edge strip that touch only the robot."""
    sc = bpy.context.scene
    for o in bpy.data.objects:
        if o.type == "LIGHT":
            o.hide_render = True
    only = bpy.data.collections.new("SC_RobotOnly")
    for o in robot_meshes():
        only.objects.link(o)
    _area("SC_Key", (3.0, -4.2, 4.2), (0, 0, 1.35), 430, 2.2, (1.0, 0.97, 0.93))
    _area("SC_Fill", (-4.2, -3.4, 2.0), (0, 0, 1.2), 60, 4.0, (0.80, 0.90, 1.0))
    _area("SC_Top", (0, 0.6, 4.6), (0, 0, 1.6), 120, 1.6, (1, 1, 1), None, 120)
    for name, x, energy, color in (("SC_EdgeCool", -2.3, 340, (0.40, 0.86, 1.0)), ("SC_EdgeWarm", 2.3, 280, (1.0, 0.68, 0.42))):
        o = _area(name, (x, 1.9, 1.8), (0, 0, 1.3), energy, 0.5, color, 2.6, 120)
        o.light_linking.receiver_collection = only
    bg = next(n for n in sc.world.node_tree.nodes if n.type == "BACKGROUND")
    bg.inputs["Color"].default_value = (0.012, 0.016, 0.022, 1)
    bg.inputs["Strength"].default_value = 1.0
    cyc = bpy.data.objects["Studio_Cyc"]
    cyc.data.materials.clear()
    cyc.data.materials.append(_cyc_material())
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.exposure = 0.15


def robot_points(step=3):
    """World-space sample of the evaluated robot surface, as an N x 3 array."""
    dg = bpy.context.evaluated_depsgraph_get()
    out = []
    for o in robot_meshes():
        if o.hide_render:
            continue
        ev = o.evaluated_get(dg)
        me = ev.to_mesh()
        co = np.empty(len(me.vertices) * 3, dtype=np.float32)
        me.vertices.foreach_get("co", co)
        mw = np.array(ev.matrix_world)
        out.append(co.reshape(-1, 3)[::step] @ mw[:3, :3].T + mw[:3, 3])
        ev.to_mesh_clear()
    return np.concatenate(out)


def projected_box(cam, pts):
    """Bounds of pts in normalised frame coordinates (0..1, origin bottom left)."""
    r = bpy.context.scene.render
    dg = bpy.context.evaluated_depsgraph_get()
    proj = np.array(cam.calc_matrix_camera(dg, x=r.resolution_x, y=r.resolution_y))
    view = np.array(cam.matrix_world.inverted())
    clip = np.concatenate([pts, np.ones((len(pts), 1))], axis=1) @ (proj @ view).T
    uv = clip[:, :2] / clip[:, 3:4] * 0.5 + 0.5
    return float(uv[:, 0].min()), float(uv[:, 0].max()), float(uv[:, 1].min()), float(uv[:, 1].max())


def make_camera(name, lens=85, ortho=False):
    d = bpy.data.cameras.new(name)
    d.lens, d.clip_end, d.sensor_fit = lens, 200, "VERTICAL"
    if ortho:
        d.type = "ORTHO"
    o = bpy.data.objects.new(name, d)
    bpy.context.scene.collection.objects.link(o)
    return o


def frame_robot(cam, azimuth, elevation, target_fill=FILL_TARGET, pts=None):
    """Place the camera so the robot fills target_fill of the frame height, centred. Returns the measured box."""
    if pts is None:
        pts = robot_points()
    tgt = Vector((0, 0, float(pts[:, 2].min() + pts[:, 2].max()) / 2))
    az, el = math.radians(azimuth), math.radians(elevation)
    direction = Vector((math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el), math.sin(el)))
    ortho = cam.data.type == "ORTHO"
    lo, hi = (1.0, 12.0) if ortho else (2.0, 60.0)
    r = bpy.context.scene.render
    aspect = r.resolution_x / r.resolution_y
    box = None
    for _ in range(40):
        mid = (lo + hi) / 2
        for _ in range(3):  # re-centre by moving the look-at point, not by lens shift
            if ortho:
                cam.data.ortho_scale = mid
            cam.location = tgt + direction * (9.0 if ortho else mid)
            cam.rotation_euler = (-direction).to_track_quat("-Z", "Y").to_euler()
            bpy.context.view_layer.update()
            box = projected_box(cam, pts)
            size = mid if ortho else 2 * mid * math.tan(cam.data.angle_y / 2)
            axes = cam.matrix_world.to_3x3()
            tgt = tgt + axes.col[1] * (((box[2] + box[3]) / 2 - 0.5) * size) + axes.col[0] * (((box[0] + box[1]) / 2 - 0.5) * size * aspect)
        fill = box[3] - box[2]
        if abs(fill - target_fill) < 5e-4:
            break
        if fill > target_fill:
            lo = mid
        else:
            hi = mid
    return {"x0": box[0], "x1": box[1], "y0": box[2], "y1": box[3], "fill_height": box[3] - box[2],
            "fill_width": box[1] - box[0], "azimuth": azimuth, "elevation": elevation,
            "lens_mm": None if ortho else cam.data.lens, "ortho_scale": cam.data.ortho_scale if ortho else None,
            "location": [round(v, 4) for v in cam.location]}


def check_frame(box):
    ok = FILL_MIN <= box["fill_height"] <= FILL_MAX and box["x0"] > 0.02 and box["x1"] < 0.98 and box["y0"] > 0.02 and box["y1"] < 0.98
    box["pass"] = bool(ok)
    return ok
