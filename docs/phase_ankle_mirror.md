# Phase Spec: Mirror Ankle to Left Side

## Objective
Duplicate the right-side ankle MECH/CTRL bone layer and solver to the
left side. Short phase — same pattern as the main mirror phase, narrower
scope (ankle only).

## Scope
- IN: MECH_Ankle_Dorsi_L, MECH_Ankle_Inversion_L, CTRL_Ankle_Dorsi_L,
  CTRL_Ankle_Inversion_L bones; DEF_Foot_L Copy Rotation constraints;
  IK_Target.foot.L empty; ankle_ik_solver_L.py script.
- OUT: everything else — no other bones, no skinning, no other phases.

## Pre-checks (same discipline as main mirror phase)

1. Confirm right-side ankle bone names exactly as they exist in the live
   rig — print them before mirroring. Do not assume the names match the
   spec doc; read what's actually there.
2. Confirm DEF_Foot_L exists and has no ankle constraints yet (should be
   clean from the main mirror phase, which pre-dated the ankle build).

## Method — bones
Manual duplication with explicit _L renaming, same as the main mirror
phase (no Symmetrize — confirmed earlier that this rig uses no .R/.L
suffix convention).

For each of the four ankle bones (MECH_Ankle_Dorsi, MECH_Ankle_Inversion,
CTRL_Ankle_Dorsi, CTRL_Ankle_Inversion):
1. Duplicate the bone
2. Mirror its head/tail X coordinate (negate X, keep Y and Z)
3. Rename: append _L (or replace _R with _L if the right-side names use
   that convention — check first per pre-check 1)
4. Set rotation_mode = 'XYZ' immediately — per VALIDATION.md gotcha entry 6
5. Re-parent to the correct left-side parent (DEF_Foot_L, or whichever
   bone the right-side ankle bones parent to — check and match)

## Method — DEF_Foot_L constraints
Add the same two Copy Rotation constraints as DEF_Foot (right):
- Copy Rotation from MECH_Ankle_Dorsi_L, Mix: Add, X axis only
- Copy Rotation from MECH_Ankle_Inversion_L, Mix: Add, Z axis only
Same stack order as the right side.

## Method — IK_Target.foot.L empty
Create fresh at the mirrored position of IK_Target.foot.R:
- location = IK_Target.foot.R.location * (-1, 1, 1)
Print the computed location before creating.

## Method — solver script
Copy ankle_ik_solver.py to ankle_ik_solver_L.py, update every bone name
and target name reference explicitly. Print a diff of changed lines after
writing — same discipline as arm/knee solver mirroring.

## Regression tests (quick — right side already validated)
Hand-set CTRL_Ankle_Dorsi_L to 15°/X, confirm MECH_Ankle_Dorsi_L and
DEF_Foot_L both show 15.0°/(1,0,0). Hand-set CTRL_Ankle_Inversion_L to
10°/Z, confirm both show 10.0°/(0,0,1). Reset both. PASS=True required
before proceeding to renders.

## Render (one only — front view)
`preview_ankle_mirror_front.png` — front view, both feet visible, IK
target moved to show right foot inverted while left foot stays at rest
(or vice versa) — confirms left-side solver is wired to the correct
bones and not accidentally driving the right side.

## Save
Save as `robot_armature_v09.blend` after regression tests pass.

## Stop condition
Stop after the render and report:
- Printed bone names from pre-check
- IK_Target.foot.L computed location
- Regression test results
- The render
Do not proceed into skinning, spine CTRL/MECH, or any other phase
without a new instruction.
