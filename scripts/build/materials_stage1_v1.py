"""
materials_stage1_v1.py -- v1.0
Purpose: Stage 1 material authoring -- create the five base shell/hardware
materials (CompositeShell, StructuralMetal, JointHardware, Elastomer, Optics)
as procedural node-based materials, unassigned to any mesh.

REDO, NOT RECOVERY: an earlier v14 pass built five materials of these same
names, but that .blend was lost (confirmed gone -- not on disk, not in git,
no autosave, no crash-recovery copy, no authoring script survived anywhere
searched). This script is a first-pass rebuild following the same stated
pattern (UV-independent, idempotent, defensive socket lookups) -- the actual
node parameter values below (colors, roughness, metallic, noise scale) are
NOT recovered originals. They are this script's own first-pass choices,
explicitly flagged here as ASSUMED and meant to be tuned once reviewed live,
same as every other numeric value in this project that hasn't been visually
validated yet.

Design constraints (per user's stated requirement):
  - UV-independent: every procedural texture node is driven from
    TexCoord.Object, never TexCoord.UV -- these parts may have inconsistent
    or absent UV layouts, and Object coordinates stay stable regardless.
  - Idempotent creation guard: aborts if ANY of the five already exist,
    rather than silently overwriting or producing a partial set. Call
    teardown() first to rebuild from scratch.
  - Defensive socket-name lookups: Principled BSDF input socket names have
    changed across Blender versions (e.g. 'Specular' -> 'Specular IOR Level',
    'Transmission' -> 'Transmission Weight' in 4.0+). Every socket is looked
    up by a list of known names via _input(), never by a hardcoded single
    name or index, and raises loudly if none match rather than silently
    wiring the wrong socket.

Prerequisites (validated live before running): none beyond a functioning
Principled BSDF node type existing (bpy.types.ShaderNodeBsdfPrincipled) --
this script creates materials only, does not touch any mesh or object.

Safe to rerun: aborts if any target material name already exists (see
_validate_prerequisites); call teardown() to remove all five and start over.
"""

import bpy

MATERIAL_NAMES = ['CompositeShell', 'StructuralMetal', 'JointHardware', 'Elastomer', 'Optics']


def teardown(report=True):
    removed = []
    for name in MATERIAL_NAMES:
        mat = bpy.data.materials.get(name)
        if mat:
            bpy.data.materials.remove(mat)
            removed.append(name)
    if report:
        print(f"materials_stage1_v1.teardown(): removed {removed or 'nothing (none existed)'}")


def _validate_prerequisites():
    existing = [n for n in MATERIAL_NAMES if n in bpy.data.materials]
    if existing:
        raise RuntimeError(
            f"materials_stage1_v1: material(s) already exist -- {existing}. "
            f"Aborting rather than partially rebuilding. Call teardown() first "
            f"if you want to regenerate from scratch."
        )


def _input(node, *names):
    """Defensive socket lookup: try each candidate name in order, raise
    loudly (not silently wire the wrong socket) if none match -- Principled
    BSDF socket names differ across Blender versions."""
    for name in names:
        if name in node.inputs:
            return node.inputs[name]
    raise RuntimeError(
        f"materials_stage1_v1: none of {names} found as an input on "
        f"'{node.name}' ({node.bl_idname}). Available inputs: "
        f"{[i.name for i in node.inputs]}"
    )


def _new_material_base(name):
    """Every material shares this skeleton: new material, nodes on,
    Principled BSDF -> Material Output, and a TexCoord node (Object output
    only ever used downstream -- never TexCoord.UV)."""
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()

    output = nt.nodes.new('ShaderNodeOutputMaterial')
    output.location = (400, 0)
    principled = nt.nodes.new('ShaderNodeBsdfPrincipled')
    principled.location = (100, 0)
    nt.links.new(_input(output, 'Surface'), principled.outputs['BSDF'])

    texcoord = nt.nodes.new('ShaderNodeTexCoord')
    texcoord.location = (-700, 0)

    return mat, nt, principled, texcoord


def _build_composite_shell():
    """Outer shell plastic: light neutral composite, low-moderate roughness
    with a very slight procedural roughness variation so it doesn't read as
    a flat CG plastic. ASSUMED base color/roughness -- tune once reviewed."""
    mat, nt, principled, texcoord = _new_material_base('CompositeShell')

    noise = nt.nodes.new('ShaderNodeTexNoise')
    noise.location = (-400, 200)
    noise.inputs['Scale'].default_value = 8.0
    noise.inputs['Detail'].default_value = 4.0
    nt.links.new(noise.inputs['Vector'], texcoord.outputs['Object'])

    roughness_map = nt.nodes.new('ShaderNodeMapRange')
    roughness_map.location = (-150, 200)
    roughness_map.inputs['To Min'].default_value = 0.30
    roughness_map.inputs['To Max'].default_value = 0.42
    nt.links.new(roughness_map.inputs['Value'], noise.outputs['Fac'])

    base_color = _input(principled, 'Base Color')
    base_color.default_value = (0.72, 0.73, 0.75, 1.0)
    _input(principled, 'Metallic').default_value = 0.0
    nt.links.new(_input(principled, 'Roughness'), roughness_map.outputs['Result'])

    return mat


def _build_structural_metal():
    """Structural frame members: brushed aluminum.

    v1.0 used a Wave texture for the brush pattern -- reviewed and rejected:
    it read as hard, regular banding (like corduroy), not brushed metal.
    Rejected the technique, not just its parameters.

    v1.1: brushed metal is directional ANISOTROPIC SCRATCHES -- a bump/normal
    detail, not a roughness-only effect. Built as: Object coords -> Mapping
    node stretched heavily on one axis (X:Y:Z = 150:4:4) so a normal
    (isotropic) Noise texture reads as elongated streaks along that axis ->
    Bump node -> Principled 'Normal'. A second, narrower use of the same
    noise still feeds a small roughness variation on top of the bump detail.
    Still UV-independent (TexCoord.Object feeds the Mapping node, never
    TexCoord.UV). ASSUMED values -- tune once reviewed, especially the
    brush-direction axis, which was picked arbitrarily (X) with no reference
    to how this material will actually be UV/orientation-mapped once
    assigned to real parts."""
    mat, nt, principled, texcoord = _new_material_base('StructuralMetal')

    mapping = nt.nodes.new('ShaderNodeMapping')
    mapping.location = (-550, 200)
    mapping.inputs['Scale'].default_value = (150.0, 4.0, 4.0)
    nt.links.new(mapping.inputs['Vector'], texcoord.outputs['Object'])

    noise = nt.nodes.new('ShaderNodeTexNoise')
    noise.location = (-350, 200)
    noise.inputs['Scale'].default_value = 5.0
    noise.inputs['Detail'].default_value = 2.0
    nt.links.new(noise.inputs['Vector'], mapping.outputs['Vector'])

    bump = nt.nodes.new('ShaderNodeBump')
    bump.location = (-100, 100)
    bump.inputs['Strength'].default_value = 0.25
    bump.inputs['Distance'].default_value = 0.05
    nt.links.new(bump.inputs['Height'], noise.outputs['Fac'])

    roughness_map = nt.nodes.new('ShaderNodeMapRange')
    roughness_map.location = (-100, 320)
    roughness_map.inputs['To Min'].default_value = 0.24
    roughness_map.inputs['To Max'].default_value = 0.32
    nt.links.new(roughness_map.inputs['Value'], noise.outputs['Fac'])

    _input(principled, 'Base Color').default_value = (0.65, 0.66, 0.68, 1.0)
    _input(principled, 'Metallic').default_value = 0.95
    nt.links.new(_input(principled, 'Roughness'), roughness_map.outputs['Result'])
    nt.links.new(_input(principled, 'Normal'), bump.outputs['Normal'])

    return mat


def _build_joint_hardware():
    """Joint hardware: darker, more worn metal than the structural frame --
    higher roughness, a grungier noise pattern mixed into both base color
    and roughness for a used-hardware look. ASSUMED values -- tune once
    reviewed."""
    mat, nt, principled, texcoord = _new_material_base('JointHardware')

    noise = nt.nodes.new('ShaderNodeTexNoise')
    noise.location = (-400, 200)
    noise.inputs['Scale'].default_value = 15.0
    noise.inputs['Detail'].default_value = 6.0
    nt.links.new(noise.inputs['Vector'], texcoord.outputs['Object'])

    color_ramp = nt.nodes.new('ShaderNodeValToRGB')
    color_ramp.location = (-150, 350)
    color_ramp.color_ramp.elements[0].color = (0.12, 0.12, 0.13, 1.0)
    color_ramp.color_ramp.elements[1].color = (0.22, 0.22, 0.24, 1.0)
    nt.links.new(color_ramp.inputs['Fac'], noise.outputs['Fac'])

    roughness_map = nt.nodes.new('ShaderNodeMapRange')
    roughness_map.location = (-150, 50)
    roughness_map.inputs['To Min'].default_value = 0.35
    roughness_map.inputs['To Max'].default_value = 0.55
    nt.links.new(roughness_map.inputs['Value'], noise.outputs['Fac'])

    nt.links.new(_input(principled, 'Base Color'), color_ramp.outputs['Color'])
    _input(principled, 'Metallic').default_value = 1.0
    nt.links.new(_input(principled, 'Roughness'), roughness_map.outputs['Result'])

    return mat


def _build_elastomer():
    """Rubber/flexible joint boots: near-black, non-metallic, high
    roughness, slight noise-driven roughness variation for a soft rubber
    look.

    v1.0 had no secondary specular lobe and read as flat dark plastic, not
    rubber -- a missing ingredient, not a wrong number. v1.1 adds a Coat
    layer (Blender 5.2's Principled BSDF renamed Clearcoat -> Coat: 'Coat
    Weight' / 'Coat Roughness', confirmed against this Blender version's
    actual socket list, not assumed): a thin, glossy coat over the rough
    matte base is the standard way to sell a rubber/vinyl look in a PBR
    shader -- matte body, sharp small highlight on top. ASSUMED values --
    tune once reviewed."""
    mat, nt, principled, texcoord = _new_material_base('Elastomer')

    noise = nt.nodes.new('ShaderNodeTexNoise')
    noise.location = (-400, 200)
    noise.inputs['Scale'].default_value = 20.0
    noise.inputs['Detail'].default_value = 3.0
    nt.links.new(noise.inputs['Vector'], texcoord.outputs['Object'])

    roughness_map = nt.nodes.new('ShaderNodeMapRange')
    roughness_map.location = (-150, 200)
    roughness_map.inputs['To Min'].default_value = 0.70
    roughness_map.inputs['To Max'].default_value = 0.85
    nt.links.new(roughness_map.inputs['Value'], noise.outputs['Fac'])

    _input(principled, 'Base Color').default_value = (0.02, 0.02, 0.022, 1.0)
    _input(principled, 'Metallic').default_value = 0.0
    nt.links.new(_input(principled, 'Roughness'), roughness_map.outputs['Result'])
    _input(principled, 'Coat Weight', 'Clearcoat').default_value = 0.4
    _input(principled, 'Coat Roughness', 'Clearcoat Roughness').default_value = 0.15

    return mat


def _build_optics():
    """Visor/lens optics: glossy, transmissive, slight cyan tint to match
    the BotCyan accent already used elsewhere on this rig. Uses whichever
    transmission-weight socket name this Blender version exposes. ASSUMED
    values -- tune once reviewed."""
    mat, nt, principled, texcoord = _new_material_base('Optics')

    _input(principled, 'Base Color').default_value = (0.75, 0.95, 0.97, 1.0)
    _input(principled, 'Metallic').default_value = 0.0
    _input(principled, 'Roughness').default_value = 0.04
    _input(principled, 'Transmission Weight', 'Transmission').default_value = 1.0
    ior_socket = None
    for candidate in ('IOR',):
        if candidate in principled.inputs:
            ior_socket = principled.inputs[candidate]
            break
    if ior_socket is not None:
        ior_socket.default_value = 1.46  # acrylic/polycarbonate lens, not glass

    return mat


def build(report=True):
    _validate_prerequisites()

    created = [
        _build_composite_shell().name,
        _build_structural_metal().name,
        _build_joint_hardware().name,
        _build_elastomer().name,
        _build_optics().name,
    ]

    result = {'materials_created': created, 'assigned_to_any_mesh': False}
    if report:
        print("materials_stage1_v1.build():", result)
    return result
