# Phase Spec: Ankle Solver

## Objective
Build the ankle's MECH/CTRL bone layer and an analytic 2R solver — the
most complex joint in this rig (two actuated axes vs. the knee/elbow's
one), but mechanically well-understood from today's empirical axis check.

## Classification (per specifications/legs/ankle.md + today's live test)
**Compound (Mixed DOF)** — 2R actuated, 1R passive (omitted):
- **Actuated DOF 1**: dorsiflexion/plantarflexion, local X axis,
  confirmed empirically: +15°/X swings foot tail in Y-Z plane
  (forward/back + up/down). ROM: open item — not in KINEMATICS.md.
  See ROM Decision below.
- **Actuated DOF 2**: inversion/eversion, local Z axis, confirmed
  empirically: +15°/Z swings foot tail in X-Y plane (side to side).
  KINEMATICS.md's "Z-axis (or Y, per spec)" ambiguity resolved: Z.
  ROM: open item — see ROM Decision below.
- **Passive DOF**: local Y (head-to-tail, foot points forward). Rotation
  around it is a geometric no-op (confirmed: +15°/Y produced zero tail
  movement). Omitted entirely — no bone, same treatment as the knee's
  passive DOF.

## ROM Decision (project owner must decide before solver is built)
KINEMATICS.md contains no ankle ROM figures. Two values needed:

**Dorsiflexion/plantarflexion (local X):**
Suggested range: -30° (plantarflexion, foot pointing down) to +20°
(dorsiflexion, foot pulling up). Reasoning: a service robot mascot
needs enough plantarflexion for a toe-point or walking pose, and enough
dorsiflexion for a flat-footed standing pose. Narrower than human
anatomical norm (~50° plantarflexion / 20° dorsiflexion) on the same
"functional over full biological" reasoning used for knee/elbow.

**Inversion/eversion (local Z):**
Suggested range: -15° to +15°. Reasoning: subtle weight-shift and
ground-contact variation — the mascot doesn't need extreme ankle tilt,
just enough to make standing poses feel natural on uneven ground.

**Confirm or adjust both ranges before proceeding.** These are suggestions
based on the rig's established "functional over full biological" pattern —
the project owner's call, same as the knee's 0°–120° decision.

## Bone layer (MECH/CTRL) — before the solver

### MECH_Ankle_Dorsi
- 1R hinge, local X, handles dorsiflexion/plantarflexion
- Positioned at DEF_Foot's head (the ankle joint location)
- Copy Rotation from CTRL_Ankle_Dorsi → Limit Rotation X: [ROM_min,
  ROM_max], Y/Z locked — same constraint stack as MECH_Knee/MECH_Elbow
- rotation_mode = 'XYZ' immediately after creation

### MECH_Ankle_Inversion
- 1R hinge, local Z, handles inversion/eversion
- Positioned at same ankle joint location (shares the joint point with
  MECH_Ankle_Dorsi — these are conceptually two axes at the same point)
- Copy Rotation from CTRL_Ankle_Inversion → Limit Rotation Z: [ROM_min,
  ROM_max], X/Y locked
- rotation_mode = 'XYZ' immediately after creation

### CTRL_Ankle_Dorsi / CTRL_Ankle_Inversion
- Free animator handles, same role as CTRL_Knee/CTRL_Elbow
- No constraints at rest

### DEF_Foot (existing bone)
- Add Copy Rotation constraints from both MECH_Ankle_Dorsi (X axis) and
  MECH_Ankle_Inversion (Z axis) — this is the first DEF bone in this rig
  driven by two MECH bones simultaneously. Order matters: apply
  dorsiflexion first (Copy Rotation from MECH_Ankle_Dorsi, Mix: Add),
  then inversion (Copy Rotation from MECH_Ankle_Inversion, Mix: Add).
  The Add mix mode accumulates both rotations rather than overwriting.

## Solver design
Unlike the arm and knee solvers (1R — one angle from one geometric
projection), the ankle needs a 2R solve: given the IK target position,
compute both dorsiflexion and inversion angles simultaneously.

The decomposition is sequential, not simultaneous — this works because
the two axes are approximately orthogonal:
1. **Dorsiflexion angle**: project the target direction onto the
   dorsiflexion plane (Y-Z plane in foot-local space), compute the angle
   from the foot's rest forward direction to the projected target. Clamp
   to dorsiflexion ROM.
2. **Inversion angle**: project the target direction onto the inversion
   plane (X-Y plane in foot-local space), compute the lateral offset
   angle. Clamp to inversion ROM.

This sequential decomposition has a known limitation: it introduces a
small coupling error when both axes are simultaneously at large angles.
For the ROM ranges above (max 30° and 15° respectively), the coupling
error is negligible (<2°) — acceptable for a portfolio mascot. Note this
in the solver's docstring so it's not silently forgotten.

## Regression tests (two, one per axis)
Same pattern as arm/knee — hand-set each CTRL bone to a known angle,
confirm MECH and DEF_Foot follow at the same value on the correct axis,
PASS=True. Run before the solver, after wiring the constraint stack.

## Mirroring note
Build right side only in this phase. Left side (_L) follows the same
mirror-and-rename pattern established in the mirror phase — do not
build both sides simultaneously, verify right first.

## Renders (explicit filenames)
- `preview_ankle_rest.png` — side view, rest pose, ankle bones visible
- `preview_ankle_dorsi.png` — side view, foot tipped forward/back
  (dorsiflexion test), solver-driven
- `preview_ankle_inversion.png` — front view, foot tilted sideways
  (inversion test), solver-driven — front view because inversion is
  invisible from the side

## Save
Save as `robot_armature_v08.blend` after constraint stack + regression
tests pass, before solver is written. Save again as the same file after
solver is written and full test pass completes.

## Stop condition
**STOP BEFORE BUILDING ANYTHING** and report the two ROM decisions back
for confirmation. Do not create any bones or write any solver code until
ROM figures are confirmed. Once confirmed, proceed and stop again after
the three renders and full test pass.
