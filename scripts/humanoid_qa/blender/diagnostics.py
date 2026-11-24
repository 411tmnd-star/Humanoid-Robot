"""Orthographic diagnostic renders with world-axis markers.

Creates a camera, axis bars and text labels IN MEMORY ONLY and hides
non-body objects from render. The caller must run this after all
measurements and must never save the file afterwards (run_qa never saves;
the source hash check enforces it).

For every view the image-right and image-up world directions are derived
from the camera matrix, so "viewer-left/right" is measured, not asserted.
"""

import math
import os

import bpy

VIEWS = {
    # name: camera location, euler XYZ (radians), label rotation
    "front_from_minusY": ((0.0, -10.0, 1.2), (math.pi / 2, 0.0, 0.0)),
    "side_from_plusX": ((10.0, 0.0, 1.2), (math.pi / 2, 0.0, math.pi / 2)),
    "top_from_plusZ": ((0.0, 0.0, 10.0), (0.0, 0.0, 0.0)),
}


def _mat(name, rgba):
    m = bpy.data.materials.new(name)
    m.diffuse_color = rgba
    return m


def _bar(name, start, end, rgba, thickness=0.012):
    me = bpy.data.meshes.new(name)
    sx, sy, sz = start
    ex, ey, ez = end
    t = thickness
    lo = (min(sx, ex) - t, min(sy, ey) - t, min(sz, ez) - t)
    hi = (max(sx, ex) + t, max(sy, ey) + t, max(sz, ez) + t)
    v = [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    f = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    me.from_pydata(v, [], f)
    o = bpy.data.objects.new(name, me)
    o.color = rgba
    bpy.context.scene.collection.objects.link(o)
    return o


def _label(text, loc, rgba):
    cu = bpy.data.curves.new("HQA_lbl_" + text, "FONT")
    cu.body = text
    cu.size = 0.12
    o = bpy.data.objects.new("HQA_lbl_" + text, cu)
    o.location = loc
    o.color = rgba
    bpy.context.scene.collection.objects.link(o)
    return o


def render_views(out_dir, keep_visible, resolution=900):
    """Render each view to out_dir; return a description per view."""
    scene = bpy.context.scene
    for o in scene.objects:                       # in-memory visibility only
        if o.name not in keep_visible:
            o.hide_render = True
    red, green, blue = (0.9, 0.1, 0.1, 1), (0.1, 0.7, 0.1, 1), (0.15, 0.3, 0.95, 1)
    L = 0.7
    _bar("HQA_axis_+X", (0, 0, 0), (L, 0, 0), red)
    _bar("HQA_axis_+Y", (0, 0, 0), (0, L, 0), green)
    _bar("HQA_axis_+Z", (0, 0, 0), (0, 0, L), blue)
    labels = [_label("+X", (L + 0.05, 0, 0.02), red), _label("-X", (-L - 0.2, 0, 0.02), red),
              _label("+Y", (0, L + 0.05, 0.02), green), _label("-Y", (0, -L - 0.2, 0.02), green),
              _label("+Z", (0.03, 0, L + 0.05), blue)]
    _bar("HQA_axis_-X", (0, 0, 0), (-L, 0, 0), (0.45, 0.05, 0.05, 1), 0.006)
    _bar("HQA_axis_-Y", (0, 0, 0), (0, -L, 0), (0.05, 0.35, 0.05, 1), 0.006)

    cam_data = bpy.data.cameras.new("HQA_cam")
    cam_data.type = "ORTHO"
    cam_data.ortho_scale = 3.0
    cam = bpy.data.objects.new("HQA_cam", cam_data)
    scene.collection.objects.link(cam)
    scene.camera = cam

    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.light = "STUDIO"
    scene.display.shading.color_type = "OBJECT"
    for o in scene.objects:
        if o.type == "MESH" and not o.name.startswith("HQA_") and o.name in keep_visible:
            o.color = (0.75, 0.75, 0.75, 1)
    scene.render.resolution_x = scene.render.resolution_y = resolution
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"

    out = {}
    for name, (loc, rot) in VIEWS.items():
        cam.location, cam.rotation_euler = loc, rot
        for lb in labels:                          # labels face this camera
            lb.rotation_euler = rot
        bpy.context.view_layer.update()
        m3 = cam.matrix_world.to_3x3()
        right, up, back = m3.col[0], m3.col[1], m3.col[2]
        path = os.path.join(out_dir, f"diag_{name}.png")
        scene.render.filepath = path
        bpy.ops.render.render(write_still=True)
        out[name] = {
            "file": os.path.basename(path),
            "camera_location": list(loc),
            "view_direction_world": [round(-c, 6) for c in back],
            "image_right_world": [round(c, 6) for c in right],
            "image_up_world": [round(c, 6) for c in up],
            "rendered": os.path.exists(path),
        }
    return out
