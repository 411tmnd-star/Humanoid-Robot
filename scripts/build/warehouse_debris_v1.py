"""
warehouse_debris_v1.py -- v1.0
Purpose: "Decaying robotics" set dressing for the warehouse shell built by
warehouse_env_v1.py, plus generic industrial debris (crates, barrels, cable
coils) and one establishing-shot camera.

Design decision (narrative, stated up front): the decaying-robot debris is
built by DUPLICATING real mesh data from the hero robot's own mesh objects
(411_* prefixed meshes already in the scene) rather than modeling new junk
from scratch. This is both the cheapest path and a narrative beat: the
warehouse becomes a graveyard of the hero's own kind, not generic scrap --
directly supporting a "self-discovery" story. Duplicates are mesh-data
copies (not linked duplicates, not armature-parented), so they are fully
inert props: posing/animating RobotArmature will never move them.

Scope:
- IN: scattered decaying-robot-part duplicates (rusted-metal material
  override, toppled/half-buried placement), procedural crates + barrels,
  simple cable coils, one establishing camera aimed at the dock-light
  shaft from warehouse_env_v1.py.
- OUT: the hero robot's own placement/pose (a separate, later decision
  once the shot list exists), dust particle systems (skipped for this
  pass -- the volumetric fog + light shaft already carry depth; flagged
  as a follow-up, not silently added and left unverified), final camera
  animation.

Prerequisites (validated live before running):
- warehouse_env_v1.run() must already have been called (checks for the
  "ENV_Warehouse" collection and its rusted-metal material).
- RobotArmature and at least a few 411_* mesh objects must exist in the
  scene (checks a specific candidate list and skips any that are missing
  rather than failing outright -- a mesh naming change shouldn't hard-crash
  the whole set-dressing pass).

Safe to rerun: creates its own "ENV_Debris" collection; aborts with a clear
error if it already exists. Call teardown() first to remove a previous pass.
"""

import bpy
import bmesh
import random
import math
import mathutils

DEBRIS_COLLECTION = "ENV_Debris"
ENV_COLLECTION = "ENV_Warehouse"

# Candidate hero-mesh objects to duplicate as decaying husks. Not all need
# to exist -- missing ones are skipped and reported, not fatal.
DECAY_SOURCE_CANDIDATES = [
    "411_Torso", "411_UpperArm_R", "411_UpperArm_L", "411_Wrist_R",
    "411_Wrist_L", "411_WaistRing_New", "411_ThumbJoint_R",
    "411_Visor_Patch_backup",
]

# Keep a clear circular "stage" at the origin free of debris for wherever
# the hero robot ends up standing.
STAGE_CLEAR_RADIUS = 4.5

RNG_SEED = 411  # deterministic scatter -- same layout every rerun after teardown()


def teardown():
    if DEBRIS_COLLECTION not in bpy.data.collections:
        print(f"warehouse_debris_v1.teardown(): no '{DEBRIS_COLLECTION}' collection found -- nothing to remove.")
        return
    coll = bpy.data.collections[DEBRIS_COLLECTION]
    objs = list(coll.objects)
    for o in objs:
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(coll)
    print(f"warehouse_debris_v1.teardown(): removed {len(objs)} objects and the collection.")


def _random_floor_point(rng, half_w, half_l, margin=1.5):
    for _ in range(50):
        x = rng.uniform(-half_w + margin, half_w - margin)
        y = rng.uniform(-half_l + margin, half_l - margin)
        if math.hypot(x, y) > STAGE_CLEAR_RADIUS:
            return x, y
    return half_w - margin, half_l - margin  # fallback, extremely unlikely to be hit


def _recenter_mesh_to_floor(mesh_data):
    """The source meshes carry vertex coordinates relative to their original
    on-body position (e.g. a torso sits ~1.2m up in its own local space,
    a wrist chunk further out still). Copying the mesh data preserves those
    coordinates verbatim, so placing the new object's origin at floor level
    would leave the actual geometry floating at its original body height --
    confirmed visually in the first render (torso/visor chunks hovering at
    head/chest height against the walls). Fix: shift every vertex down by
    the mesh's own local-space minimum Z, so the lowest point sits at Z=0
    and the object's origin becomes "floor level" for real."""
    bm = bmesh.new()
    bm.from_mesh(mesh_data)
    min_z = min((v.co.z for v in bm.verts), default=0.0)
    for v in bm.verts:
        v.co.z -= min_z
    bm.to_mesh(mesh_data)
    bm.free()


def _settle_on_floor(mesh_data, rotation_euler, scale, target_z):
    """The Z-recenter above only guarantees the mesh's lowest point sits at
    local Z=0 BEFORE rotation. A tumbled husk (random X/Y tilt, per the
    "toppled" look this function is for) rotates around that origin point,
    which lifts the rest of the panel up and away from the floor rather
    than keeping it resting there -- confirmed visually in the second
    render (a torso panel floating against the far wall, mid-tilt). Fix:
    transform every mesh vertex by the actual rotation+scale that will be
    applied to the object, find the resulting minimum Z, and return the
    location.z needed so that minimum lands exactly on target_z."""
    rot_matrix = rotation_euler.to_matrix()
    min_z = None
    for v in mesh_data.vertices:
        co = v.co.copy()
        co.x *= scale
        co.y *= scale
        co.z *= scale
        world_z = (rot_matrix @ co).z
        if min_z is None or world_z < min_z:
            min_z = world_z
    return target_z - (min_z or 0.0)


def _duplicate_as_husk(coll, src_obj, location, rotation, rust_mat, scale_jitter=1.0):
    mesh_copy = src_obj.data.copy()
    _recenter_mesh_to_floor(mesh_copy)
    husk = bpy.data.objects.new(f"ENV_Husk_{src_obj.name}", mesh_copy)
    rot_euler = mathutils.Euler(rotation, 'XYZ')
    settled_z = _settle_on_floor(mesh_copy, rot_euler, scale_jitter, location[2])
    husk.location = (location[0], location[1], settled_z)
    husk.rotation_euler = rot_euler
    husk.scale = (scale_jitter, scale_jitter, scale_jitter)
    # Strip any modifiers copied in from the source (e.g. Armature) -- these
    # are dead props, never deformed.
    coll.objects.link(husk)
    for m in list(husk.modifiers):
        husk.modifiers.remove(m)
    husk.data.materials.clear()
    husk.data.materials.append(rust_mat)
    return husk


def build_decaying_robot_debris(coll, rust_mat, rng):
    half_w, half_l = 12.0 - 2.0, 20.0 - 2.0  # matches warehouse_env_v1 WIDTH/LENGTH minus margin, inlined to avoid a hard import dependency
    placed = []
    skipped = []
    for name in DECAY_SOURCE_CANDIDATES:
        if name not in bpy.data.objects:
            skipped.append(name)
            continue
        src = bpy.data.objects[name]
        x, y = _random_floor_point(rng, half_w, half_l)
        # Toppled and half-sunk: random tumble on X/Y, free spin on Z, and a
        # small negative Z so it reads as resting/embedded in the floor.
        rot = (rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(0, math.tau))
        z = rng.uniform(-0.15, 0.05)
        husk = _duplicate_as_husk(coll, src, (x, y, z), rot, rust_mat,
                                   scale_jitter=rng.uniform(0.9, 1.15))
        placed.append(husk.name)
    return placed, skipped


def _make_material(name, color, roughness, metallic=0.0):
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Roughness"].default_value = roughness
    if "Metallic" in bsdf.inputs:
        bsdf.inputs["Metallic"].default_value = metallic
    return mat


def _add_bmesh_cube(coll, name, size, location, rotation_z, material):
    """No bpy.ops.mesh.primitive_cube_add / transform_apply -- see the note
    in warehouse_env_v1._add_cube: operator-based creation depends on an
    active 3D viewport context this MCP-driven workflow can't guarantee, and
    was confirmed to fail intermittently ("context is incorrect")."""
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=size)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    obj.location = location
    obj.rotation_euler = (0, 0, rotation_z)
    obj.data.materials.append(material)
    coll.objects.link(obj)
    return obj


def _add_bmesh_cylinder(coll, name, radius, depth, location, rotation_euler, material):
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=16,
                           radius1=radius, radius2=radius, depth=depth)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    obj.location = location
    obj.rotation_euler = rotation_euler
    obj.data.materials.append(material)
    coll.objects.link(obj)
    return obj


def _add_bmesh_torus(coll, name, major_radius, minor_radius, location, material,
                      major_segments=24, minor_segments=8):
    """bmesh has no built-in torus primitive (unlike cube/cone) -- built
    manually from the standard parametric torus equation instead of reaching
    for bpy.ops.mesh.primitive_torus_add, for the same context-independence
    reason as the cube/cylinder helpers above."""
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    ring_verts = []
    for i in range(major_segments):
        theta = 2.0 * math.pi * i / major_segments
        ring = []
        for j in range(minor_segments):
            phi = 2.0 * math.pi * j / minor_segments
            x = (major_radius + minor_radius * math.cos(phi)) * math.cos(theta)
            y = (major_radius + minor_radius * math.cos(phi)) * math.sin(theta)
            z = minor_radius * math.sin(phi)
            ring.append(bm.verts.new((x, y, z)))
        ring_verts.append(ring)
    for i in range(major_segments):
        i2 = (i + 1) % major_segments
        for j in range(minor_segments):
            j2 = (j + 1) % minor_segments
            bm.faces.new((ring_verts[i][j], ring_verts[i2][j],
                          ring_verts[i2][j2], ring_verts[i][j2]))
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    obj.location = location
    obj.data.materials.append(material)
    coll.objects.link(obj)
    return obj


def build_crates_and_barrels(coll, rng, count_crates=10, count_barrels=6):
    half_w, half_l = 10.0, 18.0
    wood_mat = _make_material("ENV_CrateWood", (0.22, 0.15, 0.08), roughness=0.85)
    barrel_mat = _make_material("ENV_Barrel", (0.15, 0.28, 0.12), roughness=0.55, metallic=0.3)

    placed = []
    for i in range(count_crates):
        x, y = _random_floor_point(rng, half_w, half_l)
        size = rng.uniform(0.5, 1.1)
        obj = _add_bmesh_cube(coll, f"ENV_Crate_{i:02d}", size,
                               (x, y, size / 2.0), rng.uniform(0, math.tau), wood_mat)
        placed.append(obj.name)

    for i in range(count_barrels):
        x, y = _random_floor_point(rng, half_w, half_l)
        radius = rng.uniform(0.28, 0.35)
        height = rng.uniform(0.85, 0.95)
        toppled = rng.random() < 0.35
        loc = (x, y, height / 2.0 if not toppled else radius)
        rot = (math.radians(90), 0, rng.uniform(0, math.tau)) if toppled else (0, 0, rng.uniform(0, math.tau))
        obj = _add_bmesh_cylinder(coll, f"ENV_Barrel_{i:02d}", radius, height, loc, rot, barrel_mat)
        placed.append(obj.name)

    return placed


def build_cable_coils(coll, rng, count=5):
    coil_mat = _make_material("ENV_Cable", (0.02, 0.02, 0.02), roughness=0.7)
    half_w, half_l = 10.0, 18.0
    placed = []
    for i in range(count):
        x, y = _random_floor_point(rng, half_w, half_l)
        major = rng.uniform(0.4, 0.7)
        obj = _add_bmesh_torus(coll, f"ENV_CableCoil_{i:02d}", major, 0.05,
                                (x, y, 0.05), coil_mat)
        obj.scale.z = 0.3  # flatten into a coiled-on-the-floor look
        placed.append(obj.name)
    return placed


def build_establishing_camera(coll):
    if "ENV_EstablishingCam" in bpy.data.objects:
        print("warehouse_debris_v1: ENV_EstablishingCam already exists -- leaving it as-is.")
        return bpy.data.objects["ENV_EstablishingCam"]
    cam_data = bpy.data.cameras.new("ENV_EstablishingCam")
    cam_data.lens = 35.0  # was 24mm for the wide establishing shot; 35mm at
    # this closer distance keeps the robot as the clear subject while still
    # showing enough surrounding warehouse for context
    cam_obj = bpy.data.objects.new("ENV_EstablishingCam", cam_data)
    # Positioned inside the warehouse, off-center, low-ish, looking toward
    # the broken dock wall on -X to catch the cold light shaft against the
    # warm sodium fixtures overhead -- the two light colors built in
    # warehouse_env_v1.build_lighting() are meant to read as a contrast here.
    # Repositioned for the discovery beat: after retargeting the dock-shaft
    # spot at ROBOT_AIM_POINT (warehouse_env_v1.py) instead of the floor near
    # the opening, the camera needs to actually see the robot standing IN
    # that beam, with the dock opening visible behind/past it for context --
    # not a wide establishing shot that happens to catch the room.
    # x=12.0 (tried previously) is EXACTLY WIDTH/2 -- the East wall's center
    # plane in warehouse_env_v1.py -- which put the camera literally inside
    # solid wall geometry and rendered pure black from inside the mesh.
    # x=9.0 keeps 3m of real clearance from that wall.
    cam_obj.location = (9.0, -5.0, 2.2)
    target_point = (0.0, 0.0, 1.5)  # ROBOT_AIM_POINT from warehouse_env_v1.py, slight headroom bias
    direction = (target_point[0] - cam_obj.location.x,
                 target_point[1] - cam_obj.location.y,
                 target_point[2] - cam_obj.location.z)
    import mathutils
    quat = mathutils.Vector(direction).to_track_quat('-Z', 'Y')
    cam_obj.rotation_euler = quat.to_euler()

    coll.objects.link(cam_obj)
    bpy.context.scene.camera = cam_obj
    return cam_obj


def run(report=True):
    if DEBRIS_COLLECTION in bpy.data.collections:
        raise RuntimeError(
            f"warehouse_debris_v1.run(): '{DEBRIS_COLLECTION}' already exists -- "
            f"call teardown() first if you want to regenerate from scratch."
        )
    if ENV_COLLECTION not in bpy.data.collections:
        raise RuntimeError(
            f"warehouse_debris_v1.run(): '{ENV_COLLECTION}' not found -- "
            f"run warehouse_env_v1.run() first."
        )

    rust_mat = (bpy.data.materials.get("ENV_RustedMetal_Procedural")
                or bpy.data.materials.get("ENV_RustedMetal_Fallback"))
    if rust_mat is None:
        raise RuntimeError(
            "warehouse_debris_v1.run(): no rusted-metal material found from "
            "warehouse_env_v1 -- run that script's run() first."
        )

    coll = bpy.data.collections.new(DEBRIS_COLLECTION)
    bpy.context.scene.collection.children.link(coll)

    rng = random.Random(RNG_SEED)
    husks, skipped_sources = build_decaying_robot_debris(coll, rust_mat, rng)
    crates_barrels = build_crates_and_barrels(coll, rng)
    coils = build_cable_coils(coll, rng)
    cam = build_establishing_camera(coll)

    result = {
        'collection': DEBRIS_COLLECTION,
        'robot_husks_placed': husks,
        'robot_husk_sources_skipped_not_found': skipped_sources,
        'crates_and_barrels_count': len(crates_barrels),
        'cable_coils_count': len(coils),
        'camera': cam.name,
        'note': 'Dust particle system intentionally not added this pass -- '
                'volumetric fog + dock light shaft carry depth for now; flag '
                'if you want particles layered in next.',
    }
    if report:
        print("warehouse_debris_v1.run():", result)
    return result
