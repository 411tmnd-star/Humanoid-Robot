# NAMING CONVENTION — Bones, Drivers, Custom Properties

##{LAYER}{Segment}{Joint}[_{Axis}]


Where:
- `{LAYER}` = `CTRL`, `MECH`, or `DEF`
- `{Segment}` = body part (Wrist, Elbow, Shoulder, Hip, Knee, Ankle, Thumb, Index, etc.)
- `{Joint}` = specific joint (Flex, Dev, Abduction, IK, etc.)
- `{Axis}` = optional, only if the name is ambiguous without it

Examples:
- `CTRL_Wrist_Flex` — animator control for wrist flexion/extension
- `MECH_Wrist_Flex` — mechanical hinge bone for wrist flexion
- `DEF_Wrist` — deformation bone that animates the wrist mesh
- `MECH_Shoulder_IK` — IK target for shoulder

Never:
- `wrist_flex` (no layer prefix)
- `MECH_Wrist_Flexion_Extension` (redundant)
- `MECH_W_F` (cryptic abbreviation)
- `Wrist.flex` (dot notation reserved for bpy hierarchy)

## Bilateral Naming

All bilateral bones use Rigify's `.L` and `.R` suffixes:

- `MECH_Wrist_Flex.L`
- `MECH_Wrist_Flex.R`

A specification is written once, **unsided**, as `MECH_Wrist_Flex`, and
Rigify's mirror operation creates the `.L` and `.R` versions.

Never create `.L` and `.R` by hand; always use Rigify's symmetry tools or
mirror scripts.

## Custom Property Names

Custom properties on bones use the same layering:

{layer}_{property_name}


Examples:
- `mech_min_flexion` — custom property storing the minimum flexion angle
- `mech_max_extension` — maximum extension angle
- `ctrl_blend_ik_fk` — blend slider for IK/FK switching

Always lowercase for custom properties; bone names are CamelCase.

## Constraint Names

Constraints don't need naming in Blender's UI but should be ordered and
intentional. Constraint stack order is:

1. Copy Rotation (from MECH or CTRL source)
2. Limit Rotation (if the joint has hard stops)
3. Pole Constraint (IK only)
4. Damped Track (IK/tracking only)

**Why Copy Rotation before Limit Rotation**: Blender evaluates a bone's
constraint stack top-to-bottom, each constraint acting on the output of the
one before it. Copy Rotation (mode REPLACE, influence 1.0) fully overwrites
the bone's rotation. If Limit Rotation ran first, Copy Rotation would erase
its clamp on every evaluation — the hard stop would silently never apply.
Confirmed empirically against the live rig while building the elbow/forearm
joint (2026-08-31): with Limit Rotation first, over-limit and cross-axis
input passed straight through, unclamped.

Document the constraint stack in the joint's spec file if it's non-obvious.

## Driver Names / Variables

Drivers use explicit variable names:

# Example: wrist flexion driven by control bone's local X rotation
target_bone = 'CTRL_Wrist_Flex'
target_property = 'rotation_euler[0]'  # X rotation in radians
output_property = 'MECH_Wrist_Flex.rotation_euler[0]'

# No single-letter variables; no implicit globals
```

If a driver's expression is longer than one line, it belongs in a function
in a Python script, not inline in Blender's driver editor.


## Armature Collections

Blender 4.0+ uses bone collections for organization. Use:

{LAYER}
├── {Segment}
│ ├── {Joint}
│ └── {Joint}
└── {Segment}
└── {Joint}


Example:

MECH
├── Wrist
│ ├── Flex
│ └── Dev
├── Elbow
│ └── Hinge
└── Shoulder
├── Pitch
├── Roll
└── Yaw

CTRL
├── Wrist
│ ├── Flex
│ └── Dev
└── ...

DEF
├── Wrist
│ └── (all deformation bones)
└── ...


Never nest more than 3 levels; if organization gets deeper, split across
files or reconsider the hierarchy.

## Reserved Names

Never use these as bone names (they're Rigify-internal or bpy-reserved):

- `rig` (Rigify root bone)
- `root` (kinematic root)
- `MCH_*` (Rigify's internal mechanism bones)
- `ORG_*` (Rigify's internal original bones)

## File Organization

Each specification/*.md file names its MECH/CTRL/DEF bones with the standard
convention. Before building a joint in Blender, copy the exact bone names
from its spec — never ad-lib.

Validation checks: every bone in the live .blend matches the spec's naming.
If a name diverges, it's caught and reported before constraints are added.