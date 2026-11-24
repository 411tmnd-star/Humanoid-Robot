"""
cinematic_pass_v1.py -- v1.0
Purpose: Cinematic camera + lighting + local volumetric pass around the
existing, validated self_discovery_blocking_v1 animation (450 frames,
30fps). Additive only -- per the audit performed live before this script
was written: does not touch RobotArmature, any animation action, the
warehouse/debris collections, or their existing geometry.

Audited state this script assumes (confirmed live, not guessed):
  - RobotArmature: 184 bones, world location (0,0,0), active action
    'self_discovery_blocking_v1'. hello_sequence untouched (4 fcurves).
  - Two pre-existing cameras ('Camera', 'ENV_EstablishingCam'), neither
    with DOF/focus-tracking -- this script adds a NEW camera rather than
    repurposing either.
  - Nine pre-existing lights: ENV_DockShaft (spot, cold, the Phase-1 key),
    six ENV_SodiumLight_* (warm ceiling points), and three white AREA
    lights from an earlier session (Studio_Key/Fill/Rim) this script did
    not create. Studio_Rim is refined (warmed) rather than duplicated;
    Studio_Key is dimmed to avoid competing with the dock shaft as a
    second white key.
  - One pre-existing volume, ENV_FogVolume (whole-warehouse ambient fog,
    Volume Scatter, left as-is). This script adds a SEPARATE, smaller,
    local volume around the camera/hero/light region rather than
    resizing or replacing it.

Prerequisites (validated live before running): RobotArmature, DEF_Chest,
ENV_DockShaft, Studio_Rim, Studio_Key, ENV_FogVolume all exist.

Safe to rerun: checks for 'CINE_FocusTarget'/'CINE_HeroCam'/'CINE_HeroVolume'
first; aborts with a clear error if any already exist. Call teardown()
first to remove a previous pass.
"""

import bpy
import bmesh
import math

ARMATURE_NAME = 'RobotArmature'
NEW_OBJECT_NAMES = ['CINE_FocusTarget', 'CINE_HeroCam', 'CINE_HeroVolume']


def teardown():
    removed = []
    for name in NEW_OBJECT_NAMES:
        if name in bpy.data.objects:
            bpy.data.objects.remove(bpy.data.objects[name], do_unlink=True)
            removed.append(name)
    print(f"cinematic_pass_v1.teardown(): removed {removed or 'nothing (none existed)'}.")


def _validate_prerequisites(arm):
    required_bones = ['DEF_Chest']
    missing_bones = [b for b in required_bones if b not in arm.pose.bones]
    if missing_bones:
        raise RuntimeError(f"cinematic_pass_v1: required bone(s) missing -- {missing_bones}.")
    required_objs = ['ENV_DockShaft', 'Studio_Rim', 'Studio_Key', 'ENV_FogVolume']
    missing_objs = [o for o in required_objs if o not in bpy.data.objects]
    if missing_objs:
        raise RuntimeError(f"cinematic_pass_v1: required object(s) missing -- {missing_objs}.")
    existing_new = [n for n in NEW_OBJECT_NAMES if n in bpy.data.objects]
    if existing_new:
        raise RuntimeError(
            f"cinematic_pass_v1: {existing_new} already exist -- call teardown() first to rebuild."
        )


def build_focus_target(arm):
    """An empty that follows DEF_Chest's world position every frame via a
    Copy Location bone-target constraint -- gives the camera's Track-To and
    DOF focus_object a stable, animation-aware point instead of a fixed
    world coordinate, per the brief's explicit requirement."""
    empty = bpy.data.objects.new('CINE_FocusTarget', None)
    empty.empty_display_type = 'PLAIN_AXES'
    empty.empty_display_size = 0.15
    bpy.context.scene.collection.objects.link(empty)
    con = empty.constraints.new('COPY_LOCATION')
    con.target = arm
    con.subtarget = 'DEF_Chest'
    return empty


def build_camera(focus_target):
    cam_data = bpy.data.cameras.new('CINE_HeroCam')
    cam_data.lens = 85.0
    cam_data.dof.use_dof = True
    cam_data.dof.aperture_fstop = 2.8
    cam_data.dof.focus_object = focus_target
    cam_obj = bpy.data.objects.new('CINE_HeroCam', cam_data)
    bpy.context.scene.collection.objects.link(cam_obj)

    # Slow, controlled dolly-in across the full 450-frame sequence -- start
    # far enough back for an 85mm lens to comfortably frame the whole figure
    # (85mm's narrow FOV needs real distance, not a wide-lens close-up) at a
    # 3/4 angle that also catches the dock-shaft beam, end modestly closer.
    # Checked against the warehouse shell (walls at x=+-12, y=+-20) for
    # clearance -- both positions sit comfortably inside with margin.
    start_loc = (7.5, -12.5, 1.6)
    end_loc = (6.0, -10.0, 1.6)
    cam_obj.location = start_loc
    cam_obj.keyframe_insert(data_path='location', frame=1)
    cam_obj.location = end_loc
    cam_obj.keyframe_insert(data_path='location', frame=450)
    # Blender 5.x actions store curves under a layered structure
    # (action.layers[*].strips[*].channelbags[*].fcurves), not a flat
    # action.fcurves -- confirmed directly earlier this session.
    if cam_obj.animation_data and cam_obj.animation_data.action:
        action = cam_obj.animation_data.action
        for layer in action.layers:
            for strip in layer.strips:
                for channelbag in strip.channelbags:
                    for fc in channelbag.fcurves:
                        for kp in fc.keyframe_points:
                            kp.interpolation = 'BEZIER'
                            kp.handle_left_type = 'AUTO_CLAMPED'
                            kp.handle_right_type = 'AUTO_CLAMPED'

    # Aim via Track-To at the focus target -- rotation then follows both the
    # camera's own dolly motion and the target's animated position
    # automatically, rather than needing hand-keyed rotation to match.
    track = cam_obj.constraints.new('TRACK_TO')
    track.target = focus_target
    track.track_axis = 'TRACK_NEGATIVE_Z'
    track.up_axis = 'UP_Y'

    bpy.context.scene.camera = cam_obj
    return cam_obj


def refine_lighting():
    dock = bpy.data.objects['ENV_DockShaft']
    dock.data.spot_size = math.radians(30.0)  # was 18 deg (Phase 1 tuning for a tighter beam
    # at a closer discovery-shot camera); this pass's brief asks for ~30 deg specifically.

    rim = bpy.data.objects['Studio_Rim']
    rim.data.color = (1.0, 0.72, 0.45)  # was pure white -- warmed per the brief's
    # "warm rear/side Area light" spec, kept subtle (color only, energy unchanged)
    # so it doesn't compete with the cold dock-shaft key.

    key = bpy.data.objects['Studio_Key']
    key.data.energy = 120.0  # was 400 -- a second full-strength white key competed
    # directly with ENV_DockShaft (this pass's designated key light) and risked
    # flattening the single-shaft dramatic read; dimmed to a supporting role
    # rather than removed, since it still helps readability off-axis from the beam.

    return {'dock_spot_size_deg': 30.0, 'rim_color': list(rim.data.color), 'key_energy': key.data.energy}


def build_hero_volume():
    """A local Principled-Volume box covering the light/hero/camera triangle
    (dock shaft at x=-16, hero at origin, camera at x=6-7.5/y=-10 to -12.5)
    -- NOT resizing the existing whole-warehouse ENV_FogVolume, per the
    brief's explicit preference for a dedicated local volume over turning up
    ambient World/whole-room fog."""
    if 'CINE_HeroVolume' in bpy.data.objects:
        raise RuntimeError("cinematic_pass_v1: CINE_HeroVolume already exists.")

    dims = (27.0, 16.0, 7.5)
    center = (-4.5, -6.0, 3.75)

    mesh = bpy.data.meshes.new('CINE_HeroVolume')
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=dims, verts=bm.verts)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new('CINE_HeroVolume', mesh)
    obj.location = center
    obj.display_type = 'WIRE'
    bpy.context.scene.collection.objects.link(obj)

    mat = bpy.data.materials.new('CINE_HeroVolume_Mat')
    mat.use_nodes = True
    nt = mat.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    pv = nt.nodes.new('ShaderNodeVolumePrincipled')
    pv.inputs['Density'].default_value = 0.02
    pv.inputs['Anisotropy'].default_value = 0.7
    pv.inputs['Color'].default_value = (0.65, 0.68, 0.72, 1.0)
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    nt.links.new(pv.outputs['Volume'], out.inputs['Volume'])
    obj.data.materials.append(mat)

    return obj


def run(report=True):
    if ARMATURE_NAME not in bpy.data.objects:
        raise RuntimeError(f"cinematic_pass_v1: armature '{ARMATURE_NAME}' not found -- aborting.")
    arm = bpy.data.objects[ARMATURE_NAME]
    _validate_prerequisites(arm)

    focus_target = build_focus_target(arm)
    camera = build_camera(focus_target)
    lighting = refine_lighting()
    volume = build_hero_volume()

    result = {
        'focus_target': focus_target.name,
        'camera': camera.name,
        'lighting_changes': lighting,
        'hero_volume': volume.name,
    }
    if report:
        print("cinematic_pass_v1.run():", result)
    return result
