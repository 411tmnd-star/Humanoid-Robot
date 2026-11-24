"""
warehouse_env_v1.py -- v1.0
Purpose: Procedural blockout of an abandoned industrial warehouse shell --
floor, four walls (one with a large broken loading-dock opening), a roof
with visible steel trusses, an overhead sodium-light rig, and EEVEE
volumetric fog for atmosphere.

Scope decision (stated up front, per project convention):
- IN: floor slab, walls + broken opening, roof + trusses, an overhead
  light rig, volumetric fog, PolyHaven concrete/rusted-metal materials
  (applied if already downloaded into this .blend; flat fallback colors
  otherwise -- see run() report for which case happened).
- OUT: the robot itself, decaying-machinery/debris set dressing, camera
  rig, final render settings. Those are separate later phases.

This is deliberately independent of which robot rig is loaded: it never
touches RobotArmature or any 411_*/DEF_*/MECH_*/CTRL_* object, and
everything it creates lives in its own "ENV_Warehouse" collection so it
can be deleted or merged into any file without touching rig data.

Asset acquisition (do this BEFORE running, if you want real materials
instead of flat fallback colors -- separate MCP tool calls, not bpy):
  1. mcp__blender__search_polyhaven_assets(asset_type="textures", query="concrete floor")
  2. mcp__blender__download_polyhaven_asset(asset_id=<chosen>, resolution="2k")
  3. Repeat for a rusted corrugated metal texture (walls/roof).
  4. Note the resulting material names and pass them into run() via
     concrete_material_name / metal_material_name -- or just call run()
     with no arguments to get flat placeholder materials now and swap
     them later with a single material-slot reassignment.

Prerequisites (validated live before running):
  - Blender 5.2 LTS, blender-mcp addon connected.
  - Does NOT require RobotArmature to exist -- environment-only.

Safe to rerun: checks for an existing "ENV_Warehouse" collection first;
aborts with a clear error rather than duplicating geometry if found. Call
teardown() first to remove a previous build before regenerating.
"""

import bpy
import math
import mathutils

COLLECTION_NAME = "ENV_Warehouse"

# Chest/head height at the scene origin -- where RobotArmature actually
# stands, confirmed live via arm.matrix_world.translation == (0,0,0) and
# DEF_Head world position ~= (0,0,1.905). Used to aim the dock-shaft spot
# (and available for the camera script) at the subject, not just "somewhere
# near the opening."
ROBOT_AIM_POINT = mathutils.Vector((0.0, 0.0, 1.3))

# All dimensions in meters.
LENGTH = 40.0          # along Y
WIDTH = 24.0           # along X
EAVES_HEIGHT = 9.0     # wall height to roofline
WALL_THICKNESS = 0.3
FLOOR_THICKNESS = 0.2
TRUSS_SPACING = 4.0
TRUSS_DEPTH = 0.6
DOCK_OPENING_WIDTH = 8.0
DOCK_OPENING_HEIGHT = 6.0


def teardown():
    """Remove a previous ENV_Warehouse build (objects + the collection itself)."""
    if COLLECTION_NAME not in bpy.data.collections:
        print(f"warehouse_env_v1.teardown(): no '{COLLECTION_NAME}' collection found -- nothing to remove.")
        return
    coll = bpy.data.collections[COLLECTION_NAME]
    objs = list(coll.objects)
    for o in objs:
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(coll)
    print(f"warehouse_env_v1.teardown(): removed {len(objs)} objects and the collection.")


def _get_or_make_material(name, base_color, roughness=0.85, metallic=0.0):
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs["Base Color"].default_value = (*base_color, 1.0)
        bsdf.inputs["Roughness"].default_value = roughness
        if "Metallic" in bsdf.inputs:
            bsdf.inputs["Metallic"].default_value = metallic
    return mat


def build_procedural_concrete(name="ENV_Concrete_Procedural"):
    """Grungy poured-concrete slab: two octaves of noise drive base-color
    blotching (damp/clean patches) and a bump map for surface roughness,
    with a third large-scale noise carving darker crack/stain streaks."""
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nt = mat.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)

    coord = nt.nodes.new("ShaderNodeTexCoord")
    mapping = nt.nodes.new("ShaderNodeMapping")
    mapping.inputs["Scale"].default_value = (3.0, 3.0, 3.0)
    nt.links.new(coord.outputs["Object"], mapping.inputs["Vector"])

    noise_blotch = nt.nodes.new("ShaderNodeTexNoise")
    noise_blotch.inputs["Scale"].default_value = 8.0
    noise_blotch.inputs["Detail"].default_value = 4.0
    nt.links.new(mapping.outputs["Vector"], noise_blotch.inputs["Vector"])

    noise_stain = nt.nodes.new("ShaderNodeTexNoise")
    noise_stain.inputs["Scale"].default_value = 1.5
    noise_stain.inputs["Detail"].default_value = 2.0
    nt.links.new(mapping.outputs["Vector"], noise_stain.inputs["Vector"])

    ramp_blotch = nt.nodes.new("ShaderNodeValToRGB")
    ramp_blotch.color_ramp.elements[0].position = 0.35
    ramp_blotch.color_ramp.elements[0].color = (0.22, 0.21, 0.20, 1.0)  # damp/dark patch
    ramp_blotch.color_ramp.elements[1].position = 0.65
    ramp_blotch.color_ramp.elements[1].color = (0.42, 0.41, 0.38, 1.0)  # dry/clean patch
    nt.links.new(noise_blotch.outputs["Fac"], ramp_blotch.inputs["Fac"])

    ramp_stain = nt.nodes.new("ShaderNodeValToRGB")
    ramp_stain.color_ramp.elements[0].position = 0.45
    ramp_stain.color_ramp.elements[0].color = (0.10, 0.10, 0.10, 1.0)  # stain streak, near-black
    ramp_stain.color_ramp.elements[1].position = 0.55
    ramp_stain.color_ramp.elements[1].color = (1.0, 1.0, 1.0, 1.0)     # no stain (passthrough via mix factor)
    nt.links.new(noise_stain.outputs["Fac"], ramp_stain.inputs["Fac"])

    mix_stain = nt.nodes.new("ShaderNodeMixRGB")
    mix_stain.blend_type = 'MULTIPLY'
    mix_stain.inputs["Fac"].default_value = 0.6
    nt.links.new(ramp_blotch.outputs["Color"], mix_stain.inputs["Color1"])
    nt.links.new(ramp_stain.outputs["Color"], mix_stain.inputs["Color2"])

    bump_noise = nt.nodes.new("ShaderNodeTexNoise")
    bump_noise.inputs["Scale"].default_value = 25.0
    bump_noise.inputs["Detail"].default_value = 6.0
    nt.links.new(mapping.outputs["Vector"], bump_noise.inputs["Vector"])
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.15
    nt.links.new(bump_noise.outputs["Fac"], bump.inputs["Height"])

    bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.inputs["Roughness"].default_value = 0.92
    if "Metallic" in bsdf.inputs:
        bsdf.inputs["Metallic"].default_value = 0.0
    nt.links.new(mix_stain.outputs["Color"], bsdf.inputs["Base Color"])
    nt.links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])

    out = nt.nodes.new("ShaderNodeOutputMaterial")
    nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    return mat


def build_procedural_rusted_metal(name="ENV_RustedMetal_Procedural"):
    """Corrugated steel: a wave texture drives real geometric ridges via Bump,
    a Voronoi-masked noise blends rust-orange into the base steel color, and
    the rust fraction also locally kills the metallic/roughness so rusted
    patches read as matte oxide rather than shiny steel."""
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nt = mat.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)

    coord = nt.nodes.new("ShaderNodeTexCoord")
    mapping = nt.nodes.new("ShaderNodeMapping")
    mapping.inputs["Scale"].default_value = (1.0, 6.0, 1.0)
    nt.links.new(coord.outputs["Object"], mapping.inputs["Vector"])

    corrugation = nt.nodes.new("ShaderNodeTexWave")
    corrugation.wave_type = 'BANDS'
    corrugation.inputs["Scale"].default_value = 18.0
    corrugation.inputs["Distortion"].default_value = 0.0
    nt.links.new(mapping.outputs["Vector"], corrugation.inputs["Vector"])
    corrugation_bump = nt.nodes.new("ShaderNodeBump")
    corrugation_bump.inputs["Strength"].default_value = 0.4
    nt.links.new(corrugation.outputs["Fac"], corrugation_bump.inputs["Height"])

    rust_scale = nt.nodes.new("ShaderNodeMapping")
    rust_scale.inputs["Scale"].default_value = (4.0, 4.0, 4.0)
    nt.links.new(coord.outputs["Object"], rust_scale.inputs["Vector"])
    rust_voronoi = nt.nodes.new("ShaderNodeTexVoronoi")
    rust_voronoi.inputs["Scale"].default_value = 5.0
    nt.links.new(rust_scale.outputs["Vector"], rust_voronoi.inputs["Vector"])
    rust_noise = nt.nodes.new("ShaderNodeTexNoise")
    rust_noise.inputs["Scale"].default_value = 12.0
    rust_noise.inputs["Detail"].default_value = 5.0
    nt.links.new(rust_scale.outputs["Vector"], rust_noise.inputs["Vector"])

    rust_mix_mask = nt.nodes.new("ShaderNodeMixRGB")
    rust_mix_mask.blend_type = 'MULTIPLY'
    nt.links.new(rust_voronoi.outputs["Distance"], rust_mix_mask.inputs["Color1"])
    nt.links.new(rust_noise.outputs["Fac"], rust_mix_mask.inputs["Color2"])

    rust_ramp = nt.nodes.new("ShaderNodeValToRGB")
    rust_ramp.color_ramp.elements[0].position = 0.3
    rust_ramp.color_ramp.elements[1].position = 0.6
    nt.links.new(rust_mix_mask.outputs["Color"], rust_ramp.inputs["Fac"])

    steel_color = (0.32, 0.33, 0.35, 1.0)
    rust_color = (0.35, 0.14, 0.05, 1.0)
    color_mix = nt.nodes.new("ShaderNodeMixRGB")
    color_mix.inputs["Color1"].default_value = steel_color
    color_mix.inputs["Color2"].default_value = rust_color
    nt.links.new(rust_ramp.outputs["Color"], color_mix.inputs["Fac"])

    rough_mix = nt.nodes.new("ShaderNodeMixRGB")
    rough_mix.inputs["Color1"].default_value = (0.35, 0.35, 0.35, 1.0)  # steel: fairly smooth
    rough_mix.inputs["Color2"].default_value = (0.95, 0.95, 0.95, 1.0)  # rust: rough/matte
    nt.links.new(rust_ramp.outputs["Color"], rough_mix.inputs["Fac"])

    metal_mix = nt.nodes.new("ShaderNodeMixRGB")
    metal_mix.inputs["Color1"].default_value = (0.9, 0.9, 0.9, 1.0)   # steel: metallic
    metal_mix.inputs["Color2"].default_value = (0.05, 0.05, 0.05, 1.0)  # rust: non-metallic oxide
    nt.links.new(rust_ramp.outputs["Color"], metal_mix.inputs["Fac"])

    bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
    nt.links.new(color_mix.outputs["Color"], bsdf.inputs["Base Color"])
    nt.links.new(rough_mix.outputs["Color"], bsdf.inputs["Roughness"])
    nt.links.new(metal_mix.outputs["Color"], bsdf.inputs["Metallic"])
    nt.links.new(corrugation_bump.outputs["Normal"], bsdf.inputs["Normal"])

    out = nt.nodes.new("ShaderNodeOutputMaterial")
    nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    return mat


def _add_cube(coll, name, dims, location, material=None):
    """dims = (x_size, y_size, z_size) in meters, full extent not half-extent.
    Built directly via bmesh rather than bpy.ops.mesh.primitive_cube_add +
    transform_apply -- confirmed live that the operator path fails
    ("context is incorrect") whenever no 3D viewport area is active in the
    window this call happens to land on, which this MCP-driven, non-interactive
    workflow can't guarantee. bmesh has no context/poll() dependency at all."""
    import bmesh as _bmesh
    mesh = bpy.data.meshes.new(name)
    bm = _bmesh.new()
    _bmesh.ops.create_cube(bm, size=1.0)
    _bmesh.ops.scale(bm, vec=(dims[0], dims[1], dims[2]), verts=bm.verts)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    obj.location = location
    coll.objects.link(obj)
    if material:
        obj.data.materials.append(material)
    return obj


def build_floor(coll, mat):
    return _add_cube(
        coll, "ENV_Floor",
        dims=(WIDTH, LENGTH, FLOOR_THICKNESS),
        location=(0, 0, -FLOOR_THICKNESS / 2.0),
        material=mat,
    )


def build_walls(coll, mat):
    walls = []
    half_w, half_l = WIDTH / 2.0, LENGTH / 2.0
    z = EAVES_HEIGHT / 2.0

    # North / south walls (full span along X, at +-Y ends)
    walls.append(_add_cube(coll, "ENV_Wall_North",
                            dims=(WIDTH, WALL_THICKNESS, EAVES_HEIGHT),
                            location=(0, half_l, z), material=mat))
    walls.append(_add_cube(coll, "ENV_Wall_South",
                            dims=(WIDTH, WALL_THICKNESS, EAVES_HEIGHT),
                            location=(0, -half_l, z), material=mat))

    # East wall: solid, full span along Y
    walls.append(_add_cube(coll, "ENV_Wall_East",
                            dims=(WALL_THICKNESS, LENGTH, EAVES_HEIGHT),
                            location=(half_w, 0, z), material=mat))

    # West wall: split into two segments flanking a broken dock opening,
    # centered on the wall, so there's a real gap for light shafts / a hero shot.
    gap = DOCK_OPENING_WIDTH
    remaining = LENGTH - gap
    seg_len = remaining / 2.0
    seg_y_offset = (gap / 2.0) + (seg_len / 2.0)
    walls.append(_add_cube(coll, "ENV_Wall_West_A",
                            dims=(WALL_THICKNESS, seg_len, EAVES_HEIGHT),
                            location=(-half_w, seg_y_offset, z), material=mat))
    walls.append(_add_cube(coll, "ENV_Wall_West_B",
                            dims=(WALL_THICKNESS, seg_len, EAVES_HEIGHT),
                            location=(-half_w, -seg_y_offset, z), material=mat))
    # Lintel above the opening (wall doesn't open all the way to the roof)
    lintel_h = EAVES_HEIGHT - DOCK_OPENING_HEIGHT
    if lintel_h > 0.05:
        walls.append(_add_cube(coll, "ENV_Wall_West_Lintel",
                                dims=(WALL_THICKNESS, gap, lintel_h),
                                location=(-half_w, 0, EAVES_HEIGHT - lintel_h / 2.0),
                                material=mat))
    return walls


def build_roof_and_trusses(coll, roof_mat, truss_mat):
    parts = []
    parts.append(_add_cube(coll, "ENV_Roof",
                            dims=(WIDTH + WALL_THICKNESS, LENGTH + WALL_THICKNESS, 0.15),
                            location=(0, 0, EAVES_HEIGHT + 0.075),
                            material=roof_mat))
    # Steel trusses spanning the width, at intervals along the length.
    n_trusses = int(LENGTH // TRUSS_SPACING) + 1
    start_y = -LENGTH / 2.0
    for i in range(n_trusses):
        y = start_y + i * TRUSS_SPACING
        parts.append(_add_cube(coll, f"ENV_Truss_{i:02d}",
                                dims=(WIDTH, 0.25, TRUSS_DEPTH),
                                location=(0, y, EAVES_HEIGHT - TRUSS_DEPTH / 2.0),
                                material=truss_mat))
    return parts


def build_lighting(coll):
    lights = []
    n_fixtures = int(LENGTH // (TRUSS_SPACING * 2)) + 1
    start_y = -LENGTH / 2.0 + TRUSS_SPACING
    for i in range(n_fixtures):
        y = start_y + i * (TRUSS_SPACING * 2)
        light_data = bpy.data.lights.new(name=f"ENV_SodiumLight_{i:02d}", type='POINT')
        light_data.energy = 800.0
        light_data.color = (1.0, 0.65, 0.25)  # warm, dying sodium-vapor tone
        light_data.shadow_soft_size = 0.3
        light_obj = bpy.data.objects.new(f"ENV_SodiumLight_{i:02d}", light_data)
        light_obj.location = (0, y, EAVES_HEIGHT - 1.2)
        coll.objects.link(light_obj)
        lights.append(light_obj)

    # A single strong shaft through the broken dock opening -- the "hero" light.
    # Aimed via a track-to quaternion at the opening's floor-level center
    # rather than a hand-picked Euler -- verified this is necessary: a
    # guessed (90,0,-90) Euler does not reliably point a spot light's local
    # -Z into the building for an arbitrary placement, and getting this
    # wrong silently kills the one light this scene's mood depends on.
    sun_data = bpy.data.lights.new(name="ENV_DockShaft", type='SPOT')
    sun_data.energy = 7000.0  # was 18000 -- combined with even thin fog that
    # much power volumetrically scattered into a total whiteout at the closer
    # discovery-shot camera distance; 7000 keeps the beam bright without
    # blooming out the whole frame
    # Narrowed from 40deg: at ~16m throw (light to robot), a wide cone floods
    # the whole floor instead of reading as a directed "discovery" beam.
    # 18deg still clears the 8x6m dock opening at this distance/height (checked
    # below) while giving a much more focused shaft on the subject.
    sun_data.spot_size = math.radians(18)
    sun_data.spot_blend = 0.35
    sun_data.color = (0.75, 0.82, 1.0)  # cold daylight vs. the warm sodium fixtures
    sun_obj = bpy.data.objects.new("ENV_DockShaft", sun_data)
    sun_pos = mathutils.Vector((-WIDTH / 2.0 - 4.0, 0.0, EAVES_HEIGHT * 0.75))
    sun_obj.location = sun_pos
    # Aimed at ROBOT_AIM_POINT (chest/head height at the scene origin, where
    # RobotArmature actually stands -- confirmed live, not assumed) instead of
    # the floor just past the opening. That was the exact gap flagged after
    # the first render: the beam hit the floor to the robot's left because it
    # was aimed at the opening's near edge, not through it at the subject.
    aim_target = ROBOT_AIM_POINT.copy()
    direction = aim_target - sun_pos
    sun_obj.rotation_euler = direction.to_track_quat('-Z', 'Y').to_euler()
    coll.objects.link(sun_obj)
    lights.append(sun_obj)

    # Geometry check, not just asserted: does the beam axis actually clear the
    # dock opening (0 to DOCK_OPENING_HEIGHT at x = -WIDTH/2) between the
    # light and the robot, or does it clip the lintel/wall? Interpolate the
    # beam's height at the opening's X plane.
    t = (-WIDTH / 2.0 - sun_pos.x) / (aim_target.x - sun_pos.x)
    beam_height_at_opening = sun_pos.z + t * (aim_target.z - sun_pos.z)
    clears_opening = 0.0 < beam_height_at_opening < DOCK_OPENING_HEIGHT
    print(f"warehouse_env_v1.build_lighting(): dock shaft beam height at opening "
          f"plane = {beam_height_at_opening:.2f}m (opening is 0-{DOCK_OPENING_HEIGHT}m) "
          f"-- clears={clears_opening}")
    if not clears_opening:
        print("warehouse_env_v1.build_lighting(): WARNING -- beam axis does not "
              "clear the opening geometrically; the visible shaft may be dimmer "
              "than expected or partly occluded by the lintel/wall.")
    return lights


def build_atmosphere():
    """EEVEE volumetric fog via the world's volume scatter, plus a bounded fog
    volume so density doesn't apply outside the warehouse shell."""
    scene = bpy.context.scene
    if scene.render.engine not in ('BLENDER_EEVEE', 'BLENDER_EEVEE_NEXT'):
        print(f"warehouse_env_v1: render engine is {scene.render.engine}, not EEVEE -- "
              f"volumetric setup below assumes EEVEE and may not appear correctly.")

    world = scene.world
    if world is None:
        world = bpy.data.worlds.new("ENV_World")
        scene.world = world
    world.use_nodes = True

    # Enable EEVEE volumetrics in the render settings (Next or legacy, whichever exists).
    eevee = getattr(scene, 'eevee', None)
    if eevee and hasattr(eevee, 'use_volumetric_lights'):
        eevee.use_volumetric_lights = True

    # A bounded fog cube using a Volume Scatter shader keeps density localized
    # to the warehouse interior rather than filling all of world space.
    if "ENV_FogVolume" in bpy.data.objects:
        print("warehouse_env_v1: ENV_FogVolume already exists -- leaving it as-is.")
        return
    import bmesh as _bmesh
    fog_mesh = bpy.data.meshes.new("ENV_FogVolume")
    bm = _bmesh.new()
    _bmesh.ops.create_cube(bm, size=1.0)
    _bmesh.ops.scale(bm, vec=(WIDTH, LENGTH, EAVES_HEIGHT), verts=bm.verts)
    bm.to_mesh(fog_mesh)
    bm.free()
    fog_obj = bpy.data.objects.new("ENV_FogVolume", fog_mesh)
    fog_obj.location = (0, 0, EAVES_HEIGHT / 2.0)
    fog_obj.display_type = 'WIRE'  # never rendered as a solid surface

    fog_mat = bpy.data.materials.new("ENV_FogVolume_Mat")
    fog_mat.use_nodes = True
    nt = fog_mat.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    scatter = nt.nodes.new("ShaderNodeVolumeScatter")
    # 0.05 (and even the original 0.035) reads fine from the far establishing
    # camera (~17m throw) but was confirmed way too thick once the camera moved
    # to ~10m for the discovery close-up -- the whole frame washed out into flat
    # haze and the dock-shaft beam lost all definition as a visible "shaft."
    # 0.012 keeps a thin atmospheric layer without erasing the beam.
    scatter.inputs["Density"].default_value = 0.012
    scatter.inputs["Color"].default_value = (0.6, 0.6, 0.65, 1.0)
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    nt.links.new(scatter.outputs["Volume"], out.inputs["Volume"])
    fog_obj.data.materials.append(fog_mat)

    coll = bpy.data.collections[COLLECTION_NAME]
    for c in list(fog_obj.users_collection):
        c.objects.unlink(fog_obj)
    coll.objects.link(fog_obj)
    return fog_obj


def run(concrete_material_name=None, metal_material_name=None, report=True):
    if COLLECTION_NAME in bpy.data.collections:
        raise RuntimeError(
            f"warehouse_env_v1.run(): '{COLLECTION_NAME}' already exists -- "
            f"call teardown() first if you want to regenerate from scratch."
        )

    coll = bpy.data.collections.new(COLLECTION_NAME)
    bpy.context.scene.collection.children.link(coll)

    used_real_materials = bool(concrete_material_name and metal_material_name
                                and concrete_material_name in bpy.data.materials
                                and metal_material_name in bpy.data.materials)
    if used_real_materials:
        concrete_mat = bpy.data.materials[concrete_material_name]
        metal_mat = bpy.data.materials[metal_material_name]
    else:
        # PolyHaven is disabled in this session's Blender panel -- use real
        # procedural PBR node graphs (noise/wave/voronoi driven) instead of
        # flat placeholder colors. Swap in real downloaded materials later
        # by passing their names in; this path needs no external asset.
        concrete_mat = build_procedural_concrete()
        metal_mat = build_procedural_rusted_metal()

    floor = build_floor(coll, concrete_mat)
    walls = build_walls(coll, concrete_mat)
    roof_parts = build_roof_and_trusses(coll, metal_mat, metal_mat)
    lights = build_lighting(coll)
    fog = build_atmosphere()

    result = {
        'collection': COLLECTION_NAME,
        'object_count': len(coll.objects),
        'used_real_polyhaven_materials': used_real_materials,
        'floor': floor.name,
        'wall_count': len(walls),
        'roof_and_truss_count': len(roof_parts),
        'light_count': len(lights),
        'fog_volume': fog.name if fog else None,
        'dimensions_m': {'width': WIDTH, 'length': LENGTH, 'eaves_height': EAVES_HEIGHT},
        'dock_opening': {'width': DOCK_OPENING_WIDTH, 'height': DOCK_OPENING_HEIGHT},
    }
    if report:
        print("warehouse_env_v1.run():", result)
    return result
