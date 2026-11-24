# HIP — Joint Specification

## Segment Overview

Per docs/ENGINEERING_RULES.md §7, the hip is **Ball-socket**: 3R biological, 3R actuated,
"three serial revolutes" per this rig's established hand-built convention
(no Rigify 3-DOF limb head in use anywhere in this project). Structurally
identical in approach to `specifications/arm/shoulder_complex.md` — same
session, same pattern, copied from the validated `wrist.md` precedent
(serial MECH chain of single-axis stub bones sharing one span, collapsing
to the existing DEF bone via a POSE-space Copy Rotation from the chain's
terminal bone). See `shoulder_complex.md` for the fuller precedent
discussion; this file states the hip-specific numbers and geometry only.

## Insertion Point (verified live)

`DEF_Thigh` (+ `_L`) currently parents directly to `DEF_Pelvis` — no joint
exists between them (zero constraints on `DEF_Thigh`, confirmed live this
session). `DEF_Thigh`'s own head/tail
(`(0.0690, -0.0226, 0.8718)` → `(0.1194, -0.0167, 0.4782)`, right side) is
the hip-to-knee span and is left exactly as-is. `MECH_Knee`/`CTRL_Knee`/
`DEF_Knee` (already built, validated, per `knee.md`) are untouched — they
already parent to `DEF_Thigh`, and `DEF_Thigh`'s own rest transform doesn't
change, so nothing downstream needs re-skinning or re-verification of its
own ROM.

## Joint 1: Flexion/Extension

- **Axis**: local X (hypothesis, matches every hinge-like primary axis in
  this project — elbow, knee, wrist flex, and now shoulder flex/ext).
  Verify empirically before treating as final, per §8.
- **Bone**: `MECH_Hip_FlexExt` (+ `_L`), parent `DEF_Pelvis`, geometry =
  copy of `DEF_Thigh`'s current head/tail/roll.
- **Biological AROM**: ~120-125° flexion (knee bent), ~20-30° extension.
  Standard biomechanics ballpark, not tied to a specific verifiable citation
  this session — flagged per §17 rather than invented.
- **Functional ROM (this spec)**: **-20° (extension) to +115° (flexion)**.
  Deliberately generous on the flexion side relative to the elbow/knee's
  "narrower than biological" pattern elsewhere in this project: the project
  owner's explicit requirement this pass is that the robot can crouch and
  push off the floor for the self-discovery sequence, which needs real hip
  flexion depth (a seated/deep-crouch pose), not just a walking-gait range.

## Joint 2: Abduction/Adduction

- **Axis**: local Z (hypothesis, matches wrist dev / ankle inversion /
  finger MCP abduction / shoulder abd-add convention).
- **Bone**: `MECH_Hip_AbdAdd` (+ `_L`), parent `MECH_Hip_FlexExt`, same span.
- **Biological AROM**: ~45° abduction, ~30° adduction.
- **Functional ROM (this spec)**: **-15° (adduction) to +40° (abduction)**
  — enough for a wider stance or a single-leg weight shift, not full
  gymnastic range.

## Joint 3: Internal/External Rotation

- **Axis**: local Y (hypothesis, matches forearm twist / shoulder rotation
  convention — the long-axis twist).
- **Bone**: `MECH_Hip_Rot` (+ `_L`), parent `MECH_Hip_AbdAdd`, same span.
  Terminal bone of the chain — `DEF_Thigh` gets one new Copy Rotation
  constraint here (POSE space, target `MECH_Hip_Rot`), mirroring
  `DEF_Wrist`'s pattern exactly.
- **Biological AROM**: ~35-45° each direction.
- **Functional ROM (this spec)**: **-30° to +30°**.

## CTRL Layer

`CTRL_Hip_FlexExt` (parent `DEF_Pelvis`) → `CTRL_Hip_AbdAdd` (parent
`CTRL_Hip_FlexExt`) → `CTRL_Hip_Rot` (parent `CTRL_Hip_AbdAdd`) — free
animator handles, no constraints of their own, same span geometry.

## Constraint Stack (Copy Rotation before Limit Rotation, no exceptions)

```
MECH_Hip_FlexExt:
  Copy Rotation (target CTRL_Hip_FlexExt, Local, influence 1.0)
  Limit Rotation (Min X -20, Max X 115; Y/Z locked; Local)

MECH_Hip_AbdAdd:
  Copy Rotation (target CTRL_Hip_AbdAdd, Local, influence 1.0)
  Limit Rotation (Min Z -15, Max Z 40; X/Y locked; Local)

MECH_Hip_Rot:
  Copy Rotation (target CTRL_Hip_Rot, Local, influence 1.0)
  Limit Rotation (Min Y -30, Max Y 30; X/Z locked; Local)

DEF_Thigh (new constraint added, existing parent/geometry unchanged):
  Copy Rotation (target MECH_Hip_Rot, Pose, influence 1.0)
```

All new MECH/CTRL bones: `use_deform = False`, built correctly from the
start (per the project's own corrected precedent on the elbow/wrist).

## IK solver rework (RESOLVED, follow-up session)

`scripts/rig/knee_ik_solver.py`/`_L.py` now read `DEF_Thigh`'s LIVE pose
matrix (a direct swap, since the knee solver already used `DEF_Thigh`
itself as its rotational reference, unlike the elbow's stub-bone
indirection). `scripts/rig/ankle_ik_solver.py`/`_L.py` now read `DEF_Shin`'s
LIVE pose (reflecting both hip and knee) and compose the ankle bone's fixed
rest-relative offset on top of it — confirmed live that `CTRL_Ankle_Dorsi`'s
parent is `DEF_Shin`, so the ankle solver must run **after** the knee solver
for the same leg/frame (documented in that file now, not left implicit).
Regression-tested at rest and combined-pose-tested (hip+knee+ankle together
under a 45° crouch) with zero NaN and zero mesh breakage — see
`specifications/arm/shoulder_complex.md`'s equivalent note for the shared
render evidence.

## Empirical Axis Verification (performed, per §8 — not assumed)

Same test method and same finding as `specifications/arm/shoulder_complex.md`
(see that file for the full derivation): **FlexExt (X) was mirrored
backward on the left hip** (right +15° moved `DEF_Thigh`'s tail forward,
`world Y +0.103`; left +15° moved it backward, `world Y -0.102`) — fixed
identically via `Copy Rotation`'s `invert_x` on `MECH_Hip_L_FlexExt` plus a
negated/swapped Limit Rotation range `(-115°, +20°)`. Re-verified: both
sides now shift `world Y` the same sign for the same input (right `+0.103`,
left `+0.103` after the fix). AbdAdd's opposite-sign `world X` between
sides is correct (adduction/abduction is genuinely mirrored). Rot's
handedness confirmed identical between sides via the same perpendicular-
vector check (no correction needed). Boundary clamp tested: `CTRL_Hip_L_FlexExt`
at 115° and 200° produced identical `DEF_Thigh_L` tail positions. Rest-pose
integrity confirmed: `DEF_Thigh`/`DEF_Thigh_L` tail positions unchanged
(<0.001 diff) after the full build with all new CTRL bones at zero.

## Status

**VALIDATED** at the structural/mechanical/axis level (built, empirically
axis-verified per §8, boundary-clamped, rest-pose-integrity-checked, and
visually confirmed — both hips driven to +45° FlexExt in
`blender/shoulder_hip_pose_check.png` show symmetric forward flexion with
no mesh breakage). **Not yet validated** for animation or export (§13
Layer 3/4) — no animation using these joints exists yet.
