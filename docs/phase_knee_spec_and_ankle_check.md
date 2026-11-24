# Phase Spec: Draft Knee Spec + Verify Ankle Classification (No Solver Code Yet)

## Context
Today's elbow work failed once on an unverified assumption (generic IK
assuming free ball-joint DOF where the joint was actually a locked-axis
hinge, per §7/§11). The knee's twist-mechanism reuse assumption just failed
the same way — KINEMATICS.md classifies the knee as Compound, not Hinge,
with an explicitly *passive* secondary DOF that should be locked to zero,
not solved for. That's twice today an assumption didn't survive contact
with the actual spec. Do not let a third one — about the ankle — pass
through unexamined just because it wasn't the joint in question this time.

## Scope
- IN: writing `specifications/legs/knee.md`, independently checking the
  ankle's classification in KINEMATICS.md rather than assuming it's simple,
  saving the `rotation_mode` fix already made to `DEF_Thigh`/`DEF_Shin`/
  `DEF_Foot`.
- OUT: writing any solver code. That's the next phase, after this spec is
  reviewed.

## Part 1 — `specifications/legs/knee.md`
Pull directly from KINEMATICS.md, don't paraphrase from memory of this
session's chat. Include:
1. **Classification**: Compound (Mixed DOF), with the exact quoted
   justification from KINEMATICS.md for why it's not Hinge.
2. **Actuated DOF**: which axis is the real, solved-for flexion axis, and
   its ROM figures (min/max angle) as stated in the spec — not estimated,
   not copied from the elbow's ROM.
3. **Passive DOF**: explicitly state it is locked to zero rather than
   solved for, per KINEMATICS.md's own methodology description. Quote the
   relevant line so this isn't just this document's paraphrase of it.
4. **Bone naming** for this joint (DEF_/CTRL_/MECH_ pattern, once that
   layer exists — for now, note it applies to DEF_Thigh/DEF_Shin same as
   the placement phase built).
5. **Open items**: anything KINEMATICS.md is ambiguous or silent about for
   this joint — don't fill gaps with assumptions, list them as open
   questions for me to decide.

## Part 2 — Ankle classification (independent check, not inherited)
KINEMATICS.md was quoted as mentioning the ankle has "independent" twist
behavior in the context of the knee's passive DOF description. That's not
the same as the ankle having been classified. Explicitly:
1. Find the ankle's own classification entry in KINEMATICS.md (Hinge,
   Compound, or another category entirely — don't assume it matches the
   knee just because they were mentioned near each other).
2. If the ankle is Compound like the knee, note that in a short
   `specifications/legs/ankle.md` stub (classification + actuated/passive
   DOF only, doesn't need to be as complete as the knee doc yet).
3. If the ankle's classification is genuinely undocumented/ambiguous in
   KINEMATICS.md, say so plainly rather than inferring — this becomes an
   open question for me, not a judgment call to make silently.

## Part 3 — Save the rotation_mode fix
`DEF_Thigh`/`DEF_Shin`/`DEF_Foot` were left at Blender's default
`rotation_mode = QUATERNION` during placement, inconsistent with the rest
of the rig and the cause of a silent no-op during today's testing (setting
`rotation_euler` on a QUATERNION-mode bone does nothing, no error). This
was fixed in the live session to XYZ — confirm it's still applied, then
save. Note the fix and its cause in `specifications/legs/knee.md`'s notes
section (or a shared rig-notes location if one already exists) so this
exact bug class doesn't get silently reintroduced on a future joint.

## Stop condition
Stop after this phase and report back with:
- The full text of `specifications/legs/knee.md`
- The ankle finding (either the stub doc, or the "genuinely undocumented"
  flag)
- Confirmation `rotation_mode` is XYZ on all three leg bones and the file
  is saved

Do not write solver code until I've reviewed the knee spec and the ankle
finding.
