# ARCHITECTURE — DEF/MECH/CTRL Splicing into Rigify

## Overview

This project's rig uses three layers:

- **CTRL** — Animator/operator interface (what an animator/AI system drives)
- **MECH** — Mechanical logic layer (DOF, joint limits, solvers, coupling)
- **DEF** — Deformation bones (what geometry follows)

Rigify natively generates CTRL + DEF. This project adds MECH logically between
them, then re-splices the constraint chains so CTRL → MECH → DEF instead of
the stock CTRL → DEF.

## Coordinate System

All local axes follow **Blender's right-hand convention**:
- **X**: Forward/Back (red axis in viewport)
- **Y**: Left/Right (green axis)
- **Z**: Up/Down (blue axis)

For limbs:
- **X-axis points along the bone** (elbow→wrist = +X along forearm bone)
- **Y-axis points lateral** (perpendicular to limb length, typically out toward thumb/pinky)
- **Z-axis points up** (for standing; perpendicular to both)

Roll is computed per-bone, not assumed. A bone's local axes are always
verified via MCP inspection before any driver/constraint references it.

## Rigify Integration

Rigify generates a metarig → rig workflow. This project:

1. Uses Rigify's generated DEF-bone structure as-is (no modification)
2. Uses Rigify's generated CTRL rig as-is (no modification)
3. Adds MECH bones pre-generation (they're part of the metarig that Rigify
   sees) OR post-generation (as a parallel bone chain that feeds into the
   final constraint tree)

The choice (pre- vs. post-generation) is made per-joint in that joint's
specification file. It's recorded there and never changed silently.

## MECH Bone Naming

A MECH bone is named:

`MECH_{Segment}_{Joint}[_{Axis}]`

Examples:
- `MECH_Wrist_Flex` — wrist flexion/extension mechanism
- `MECH_Wrist_Dev` — wrist radial/ulnar deviation mechanism
- `MECH_Shoulder_IK` — shoulder IK target
- `MECH_Elbow` — elbow hinge

Bilateral: `.L` and `.R` suffixes per Rigify convention.

## Constraint Flow

The constraint graph flows:


CTRL bone
↓ Copy Rotation (influence = 1.0)
MECH bone (limit rotation to single axis, locked on others)
↓ Copy Rotation (influence = 1.0)
DEF bone (visible to glTF export, drives mesh)



Each link is explicit:
- CTRL copy-rotates the MECH bone's local rotation on the specified axis
- MECH bone outputs to the DEF bone via another copy-rotation in the same space

No constraint skips a layer; no bone is driven by two independent CTRL or MECH
sources on the same axis.

## Export Validation

Before any animated sequence is treated as complete, it's:
1. Baked to DEF-only keyframes (all constraints/drivers → static keys)
2. Exported to .glb (File → Export → glTF 2.0)
3. Re-imported into a test .blend
4. Played back visually

If the reimported animation matches the Blender viewport, the mechanism is
validated. If it diverges, the MECH/constraint setup has an error (common
with Rigify IK export bugs — Rigify rigs don't export cleanly without extra
care).

This check lives in scripts/validation/export_reimport_check.py and must pass
before any animation is shipped to the live site.

## Axis Locking

On every MECH bone:

- One axis (the joint's DOF) has `min ≠ max` (free to rotate)
- Two axes have `min = max = 0` (locked; rotation on those axes is 0)

Example for a hinge (1-DOF) joint:



MECH_Elbow.rotation_mode = 'XYZ'
MECH_Elbow.use_inherit_rotation = True
MECH_Elbow.lock_rotation = (False, True, True) # X unlocked, Y/Z locked
MECH_Elbow.lock_rotation_w = True



This is set per-bone in that joint's spec and verified by validation after
every build.

## Singularities & Gimbal Lock

3-DOF joints (shoulder, hip) are ordered as serial revolutes to avoid gimbal
lock in the middle ranges. The order is specified in each joint's spec. If a
joint reaches a singularity during animation, the validator reports it rather
than hiding it.


