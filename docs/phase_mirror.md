# Phase Spec: Mirror Right Side to Left Side

## Objective
Duplicate the right-side arm and leg bone chains (and their MECH/CTRL
layers) to the left side, create mirrored IK target/pole empties, and
produce left-side solver script variants. Result: a fully symmetric
skeleton with both arms and both legs rigged and testable.

## Scope
- IN: bone chain mirroring (DEF/MECH/CTRL layers, right→left), IK empty
  creation for left side, left-side solver scripts (arm + knee).
- OUT: spine CTRL/MECH layer, ankle solver, skinning/weight painting,
  head/neck bones. Those are separate phases.

## Critical pre-checks before touching anything

1. **Confirm the current right-side bone names end in `.R` or `_R`** — run
   a quick print of all bone names in RobotArmature and confirm the naming
   convention before mirroring. Blender's Symmetrize operator depends on
   this to correctly rename `.R` → `.L`. If bones are named without a
   side suffix (e.g. `DEF_Thigh` not `DEF_Thigh.R`), do NOT use
   Symmetrize — manual duplication with explicit renaming is required
   instead. Report which case applies before proceeding.

2. **Check the spine's local X sign flip** (flagged in the spine phase):
   spine bones came out with local X ≈ negative world X, opposite from
   the leg bones. Confirm whether Blender's Symmetrize or manual mirror
   would produce correct left-side roll on the leg/arm chains given this
   asymmetry, or whether left-side bones will need a roll correction pass
   after mirroring.

## Method — bones

### If bones use .R/.L suffix convention (Blender standard):
Use Armature → Symmetrize (from Edit Mode, with all right-side bones
selected). This mirrors bone positions across X=0 and renames `.R` → `.L`
automatically. Verify afterward that:
- Every right-side bone has a corresponding `.L` counterpart
- The `.L` bones sit at the correct mirrored X position (negative of the
  `.R` bone's X)
- Roll is consistent and symmetric — render axis-display front view to
  check, same discipline as the arm placement phase

### If bones do NOT use .R/.L suffix (current rig uses _R/_L or no suffix):
Do NOT use Symmetrize — it won't rename correctly. Instead:
1. Duplicate the right-side subtrees manually (DEF_Thigh subtree under
   DEF_Pelvis; DEF_UpperArm subtree under DEF_Chest)
2. Mirror each duplicated bone's head/tail X coordinate (negate X,
   keep Y and Z identical)
3. Rename every duplicated bone: replace `_R` with `_L` in the name, or
   add `_L` suffix if no side suffix currently exists
4. Re-parent the left-side roots (DEF_Thigh_L, DEF_UpperArm_L) to their
   correct spine attachment point (DEF_Pelvis and DEF_Chest respectively)
5. Set rotation_mode = 'XYZ' on every new bone immediately — per
   VALIDATION.md gotcha entry 6

## Method — IK empties (do NOT mirror these via bone tools)
Create three new empties from scratch at the correct mirrored positions:

- `IK_Target.arm.L`: mirror of `IK_Target.arm.R` — same Y and Z,
  negate X. Compute as: location = IK_Target.arm.R.location * (-1, 1, 1)
- `IK_Pole.arm.L`: mirror of `IK_Pole.arm.R` — same method
- `IK_Target.leg.L`: mirror of `IK_Target.leg.R` — same method

Print the computed locations before creating, same paper-trail discipline.
Do not parent these to the armature — they stay as world-space empties,
same as the right-side targets.

## Method — solver scripts
Do not just copy and rename. Open each script and update every bone name
and target name reference explicitly:

- `scripts/rig/arm_ik_solver_L.py`: copy of arm_ik_solver.py with all
  `.arm.R` → `.arm.L`, `CTRL_Elbow` → `CTRL_Elbow.L` (or `_L` equivalent),
  `CTRL_Forearm_Twist` → left-side equivalent, and any other bone name
  references updated to left-side names. Print a diff of changed lines
  after writing.
- `scripts/rig/knee_ik_solver_L.py`: same treatment for knee_ik_solver.py

## Verification

### Roll symmetry check
Render `preview_mirror_axes_front.png` with bone axes display enabled
(same as the arm placement phase's axis render). Both arms and both legs
should show axis indicators pointing in mirror-symmetric directions —
left arm's Z axis should point the opposite X direction from right arm's
Z axis when viewed from the front. If they don't, the left side needs a
roll correction pass before anything else proceeds.

### Hierarchy check
Print the full bone tree after mirroring — same format as the spine
phase's hierarchy output. Confirm:
- DEF_Pelvis has both DEF_Thigh_R (or DEF_Thigh) AND DEF_Thigh_L as
  children
- DEF_Chest has both DEF_UpperArm_R (or DEF_UpperArm) AND DEF_UpperArm_L
  as children
- MECH/CTRL bones for both knee and elbow exist on both sides

### Sanity test
Rotate DEF_Chest by 10°/Z — both arm chains (left and right) should move
together. Rotate DEF_Pelvis by 10°/Z — both leg chains should move
together. Reset to 0° after confirming.

### Quick solver test (left side only)
Move `IK_Target.arm.L` a small amount and call `arm_ik_solver_L.py` —
confirm the left arm responds. Move `IK_Target.leg.L` and call
`knee_ik_solver_L.py` — confirm the left knee bends. These don't need
full bend/unreachable/regression passes — that was done on the right side.
Just confirm the left-side scripts are wired to the correct bones and
targets and produce a non-zero response.

## Renders (explicit filenames)
- `preview_mirror_front.png` — full-body front view, all bones visible,
  both sides present and visually symmetric
- `preview_mirror_axes_front.png` — same angle, bone axes display ON,
  for roll symmetry check
- `preview_mirror_top.png` — top-down view, confirming left/right
  symmetry across the X=0 centerline

## Save
Save as `robot_armature_v06.blend` after the sanity test passes and
before the solver quick-test — same incremental discipline.

## Stop condition
Stop after this phase and report:
- Which naming convention the bones use (pre-check result)
- Printed mirrored empty locations
- Hierarchy printout confirming both sides
- Sanity test result
- Left-side solver quick-test result (did left arm/knee respond?)
- The three renders

Do not proceed into ankle solver, spine CTRL/MECH, or any other phase
without a new instruction.
