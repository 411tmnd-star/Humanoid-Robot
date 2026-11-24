# Humanoid robot: engineering rules

## 1. Scope and priorities

A 3D humanoid service robot built in Blender 5.2 LTS with Python (bpy). Target: a rigged, animated .glb that plays
correctly in Three.js. Priority order for every decision:

1. Mechanical correctness — does the joint make sense as a physical mechanism
2. Kinematic correctness — DOF count, axis definition, range match the spec
3. Anatomical plausibility only where the spec calls for it — this is a
   simplified mechanical skeleton, not a 1:1 biological replica
4. Clean Blender architecture (DEF/MECH/CTRL separation, sane hierarchy)
5. Deterministic Python automation
6. Maintainability
7. Validation before modification

## 2. Tooling

Blender 5.2 LTS, Python (bpy), and Rigify for base generation, extended per joint where the actuated DOF differs from
Rigify's default. Documents are never assumed to match the current .blend file: inspect first.

## 3. Documentation Hierarchy

Humanoid-Robot/
├── docs/
│   ├── ENGINEERING_RULES.md  ← this file: rules, not data
│   ├── ARCHITECTURE.md       ← coordinate system, how MECH splices into Rigify
│   ├── NAMING.md
│   ├── KINEMATICS.md         ← joint-type → DOF → solver methodology
│   ├── RIGGING.md            ← Rigify rig-type mapping per segment
│   └── VALIDATION.md         ← test procedure incl. export/reimport check
├── specifications/
│   ├── torso/  { head_neck.md, spine.md, pelvis.md }
│   ├── arm/    { shoulder_complex.md, elbow_forearm.md, wrist.md }
│   ├── hand/   { palm.md, thumb.md, fingers.md }
│   └── legs/   { hip.md, knee.md, ankle_foot.md }
├── scripts/    { build/, rig/, validation/, humanoid_qa/, showcase/ }
└── proof/      renders, rig data and validation reports

This file defines how the work is done. specifications/ defines WHAT each
joint is. No joint-specific number (DOF count, ROM limit, axis assignment)
is ever hard-coded in this file.

## 4. Global Rig Architecture

CTRL   — animator / operator interface
  ↓
MECH   — mechanical & kinematic logic (DOF, joint type, limits, solver)
  ↓
DEF    — deformation / visual output

- CTRL drives MECH. MECH drives DEF. DEF never drives MECH.
- A DEF bone is never driven by two independent MECH sources on the same
  transform channel.
- Rigify generates DEF+CTRL natively; MECH is this project's addition,
  sitting logically between them. How MECH is spliced into the Rigify
  pipeline — pre-metarig bones the generator consumes, vs. post-generation
  constraint overrides — is resolved in docs/ARCHITECTURE.md, not assumed here.

## 5. Export Target Constraint (glTF → Three.js)

The deliverable is a .glb animating correctly in a Three.js runtime, not
"looks right in the Blender viewport." Consequence:

glTF 2.0's animation model has no concept of a constraint or driver — only
baked keyframed TRS per node. Every MECH mechanism (IK, copy-rotation
coupling, driven finger linkages, cross-joint coupling) is an
authoring-time tool; none of it survives export as live logic. Rigify rigs
with IK/constraint stacks are a known source of bone-transform errors on
glTF export and reimport, so:

- Every animation is baked to DEF-only keyframes before being treated as
  complete.
- Validation (§13) for any animated sequence includes an actual export →
  reimport check, not just a viewport check.
- MECH complexity is justified only by how much it simplifies authoring —
  never by runtime behavior, since the runtime only ever sees DEF.

## 6. Naming Convention

{LAYER}_{Segment}_{Joint}[_{Axis}] — e.g. MECH_Wrist_Flex, DEF_Elbow,
CTRL_Hip_IK. Bilateral limbs use Rigify's .L/.R suffix; a spec is written
once, unsided, and mirrored — never duplicated by hand. No bone, driver, or
constraint is ever referenced by selection state, active object, or
viewport state in generated code.

## 7. Joint Classification Methodology

Classify before writing any bpy code (full derivation in docs/KINEMATICS.md):

| Type        | Examples                        | Bio DOF   | Actuated | Solver |
|-------------|----------------------------------|-----------|----------|--------|
| Hinge       | Elbow, knee, PIP/DIP, thumb IP  | 1R        | 1R       | Single revolute, one locked axis, hard min/max |
| Condyloid   | Wrist, MCP, thumb CMC           | 2–3R      | 2R       | Two serial revolutes, axis split per §8 |
| Ball-socket | Shoulder (GH), hip              | 3R        | 3R       | Three serial revolutes / Rigify's native 3-DOF limb head |
| Compound    | Ankle (TC+ST), knee passive rot | 3R (part passive) | 2R ankle / 1R knee | Primary actuated axis + secondary coupled-or-omitted per spec |
| Floating    | Scapulothoracic                 | ~1R+2T    | 0–1R     | No literal analog — approximated via parent range or one elevation DOF, per spec |

A joint never gets more MECH bones than its classified actuated-DOF count
without a written justification in its spec file.

## 8. Axis Ownership

- Each MECH bone in a serial chain owns exactly one rotation axis; the
  other two are locked (min=max=0).
- Axis-to-letter assignment is never assumed from a diagram — it's read
  from the bone's actual local transform before any
  driver/constraint references it, since bone roll sets this per-bone, not
  per-joint-type. Once verified, it's recorded in that joint's spec and
  treated as locked until the bone is re-rolled.
- No cross-talk: two MECH bones never target the same output axis on the
  same downstream DEF bone.

## 9. Constraint Philosophy

Before adding any constraint: state target, owner, space, affected axes,
influence, evaluation order, interaction with existing constraints on the
bone. No redundant constraints; no two systems writing the same transform
channel.

## 10. Drivers

Prefer SINGLE_PROP against explicit custom properties and explicit data
paths. No driver expression depends on selection or viewport state. Every
driver declares variable names, explicit target, source property, output
property.

## 11. IK / Solver Selection

No default to Blender's standard IK constraint just because a target
exists. Classify the joint first (§7): does it genuinely need generic IK,
or is it a locked-axis serial-revolute chain that only looks like it needs
IK? The solver must never introduce motion on a locked axis.

## 12. Script Standards

Every script: has a version number and stated purpose; validates its
prerequisites against the live scene before running; never
destructively deletes without explicit authorization; uses deterministic
names, never selection state; reports what changed; validates the
resulting state; fails loudly rather than silently; is safe to rerun or
detects prior execution.

## 13. Validation

After any structural change, validate: bone existence, hierarchy, parent
relationships, local axes, bone lengths, rotation modes, axis locks,
constraints, drivers, custom properties. For mechanisms, additionally: ROM
against the spec's limits, singularities, unwanted translation/roll,
constraint conflicts. For animated sequences: the export → reimport check
from §5.

## 14. Change Control

Each specifications/*.md file is locked once written. Changing a locked
spec requires stating: what changes, why, what depends on it, how it's
re-validated. No silent swap of a specified mechanism for a simpler one to
ease implementation.

## 15. Communication Standard

Every status is one of IMPLEMENTED / VALIDATED / ASSUMED / UNVERIFIED /
BLOCKED. On failure: state root cause, affected system, proposed
correction, validation procedure. No hiding failures behind workarounds.

## 16. Development Workflow

INSPECT → PLAN → IMPLEMENT → VALIDATE → REPORT

Before modifying an existing rig: inspect the live scene, identify
the relevant bones, compare against the relevant specifications/*.md,
determine what's missing or wrong, make the smallest correct change,
validate.

## 17. Correctness Over Compliance

If any specification contains an anatomical, mechanical, Blender API, or mathematical error, the error is named
and work stops rather than implementing it. This covers axis
assumptions and ROM/DOF figures equally: a number that looks authoritative
isn't verified until checked against either its cited source or the live
scene.