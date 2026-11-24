# SHOULDER COMPLEX — Joint Specification

## Segment Overview

Per docs/ENGINEERING_RULES.md §7's classification table, the shoulder (glenohumeral joint) is
**Ball-socket**: 3R biological, 3R actuated here, solved as "three serial
revolutes" (the table's prescribed methodology for this rig, which has no
Rigify 3-DOF limb head — every joint in this project is hand-built, per
`docs/RIGGING.md`'s precedent). True glenohumeral biomechanics also involves
scapulothoracic contribution (the shoulder blade itself gliding) — per
docs/ENGINEERING_RULES.md §7's Floating classification, that's a separate, much smaller
mechanism this spec explicitly omits. This spec covers only the glenohumeral
3R ball-socket.

This spec's structural pattern (serial MECH chain of single-axis stub bones,
each sharing the same head/tail span, collapsing to one existing DEF bone via
a POSE-space Copy Rotation from the terminal MECH bone) is copied directly
from `specifications/arm/wrist.md`'s validated 2-axis precedent, extended to
three axes — not invented fresh. Confirmed live before writing this spec:
`DEF_Wrist`'s parent is `DEF_Forearm_Twist` (unchanged, not the MECH chain),
with exactly one constraint — Copy Rotation, POSE space both sides, target
`MECH_Wrist_Dev` (the chain's terminal bone).

## Insertion Point (verified live, not assumed)

`DEF_UpperArm` currently parents directly to `DEF_Chest` — no joint exists
between them (confirmed: zero constraints on `DEF_UpperArm`, per the
project's own audit and re-confirmed this session). `DEF_UpperArm`'s own
head/tail (`(0.1845, -0.0564, 1.6108)` → `(0.2414, -0.0450, 1.3165)` in
armature-local space, right side) is the shoulder-to-elbow span, and stays
exactly as-is — no bone in the existing arm chain (elbow, twist, wrist,
hand, fingers) is moved, re-rolled, or re-parented by this spec. The new
shoulder mechanism is inserted purely by adding a MECH+CTRL chain in
parallel and adding one new constraint to `DEF_UpperArm` — the same
non-invasive pattern wrist used for `DEF_Wrist`.

## Joint 1: Flexion/Extension

- **Axis**: local X (hypothesis, per this rig's established convention —
  every hinge-like primary axis in this project is X: elbow, knee, wrist
  flex. **Not yet empirically verified on this specific bone** — per §8,
  this is checked live immediately after the bones are built, before being
  treated as final, same as every other joint in this project.)
- **Bone**: `MECH_Shoulder_FlexExt` (+ `_L`), parent `DEF_Chest`, geometry =
  copy of `DEF_UpperArm`'s current head/tail/roll (a full-span stub bone,
  same pattern as `MECH_Wrist_Flex`/`MECH_Wrist_Dev` sharing one span).
- **Biological AROM**: ~180° flexion, ~60° extension (standard biomechanics
  ballpark — not tied to a specific verifiable citation in this session;
  flagged honestly per §17 rather than attached to an invented reference).
- **Functional ROM (this spec)**: **-40° (extension) to +140° (flexion)**.
  Narrower than full biological AROM for the same reason as every other
  joint here (elbow 130° of 150°, knee 120° of ~135°): this is a mascot
  that needs to gesture and reach, not achieve full overhead extension via
  scapular contribution this rig doesn't model. Flagged for tuning once
  animated.

## Joint 2: Abduction/Adduction

- **Axis**: local Z (hypothesis — matches this rig's convention for every
  other secondary/deviation axis: wrist dev, ankle inversion, finger MCP
  abduction all use Z. Verify empirically before treating as final.)
- **Bone**: `MECH_Shoulder_AbdAdd` (+ `_L`), parent `MECH_Shoulder_FlexExt`,
  same shoulder-to-elbow span geometry.
- **Biological AROM**: ~180° abduction, ~30-45° adduction across the body.
- **Functional ROM (this spec)**: **-20° (adduction) to +140° (abduction)**.

## Joint 3: Internal/External Rotation

- **Axis**: local Y (hypothesis — matches this rig's convention for the one
  other long-axis twist in this project, forearm pronation/supination).
- **Bone**: `MECH_Shoulder_Rot` (+ `_L`), parent `MECH_Shoulder_AbdAdd`, same
  span. This is the chain's terminal bone — `DEF_UpperArm` gets a new Copy
  Rotation constraint (POSE space, target `MECH_Shoulder_Rot`) here, exactly
  mirroring `DEF_Wrist`'s existing pattern.
- **Biological AROM**: ~70-90° each direction.
- **Functional ROM (this spec)**: **-60° (internal) to +60° (external)**.

## CTRL Layer

`CTRL_Shoulder_FlexExt` (parent `DEF_Chest`) → `CTRL_Shoulder_AbdAdd`
(parent `CTRL_Shoulder_FlexExt`) → `CTRL_Shoulder_Rot` (parent
`CTRL_Shoulder_AbdAdd`) — free animator handles, no constraints of their
own, same span geometry, same role as `CTRL_Elbow`/`CTRL_Wrist_Flex`/etc.
throughout this rig.

## Constraint Stack (per MECH bone, order matters — the exact bug found and
fixed on the elbow: Copy Rotation before Limit Rotation, never reversed)

```
MECH_Shoulder_FlexExt:
  Copy Rotation (target CTRL_Shoulder_FlexExt, Local, influence 1.0)
  Limit Rotation (Min X -40, Max X 140; Y/Z locked; Local)

MECH_Shoulder_AbdAdd:
  Copy Rotation (target CTRL_Shoulder_AbdAdd, Local, influence 1.0)
  Limit Rotation (Min Z -20, Max Z 140; X/Y locked; Local)

MECH_Shoulder_Rot:
  Copy Rotation (target CTRL_Shoulder_Rot, Local, influence 1.0)
  Limit Rotation (Min Y -60, Max Y 60; X/Z locked; Local)

DEF_UpperArm (new constraint added, existing parent/geometry unchanged):
  Copy Rotation (target MECH_Shoulder_Rot, Pose, influence 1.0)
```

All new MECH/CTRL bones: `use_deform = False` (per §5 and this project's own
corrected precedent — MECH/CTRL bones were found with `use_deform=True` by
Blender's default on the elbow/wrist build and had to be corrected; built
correctly from the start here).

## IK solver rework (RESOLVED, follow-up session)

`scripts/rig/arm_ik_solver.py`/`_L.py` have been updated: the elbow's
rotation frame is now read from `DEF_UpperArm`'s LIVE pose matrix (which
already reflects the shoulder via its own Copy Rotation constraint) each
call, instead of the elbow bone's static rest transform. Verified live:
`elbow_head_world` correctly sweeps with `CTRL_Shoulder_FlexExt`, and
`flex_axis_world` correctly reorients under `CTRL_Shoulder_AbdAdd` (and is
correctly *invariant* under `CTRL_Shoulder_FlexExt` alone, since rotating a
frame about its own axis can't move that same axis — confirmed
mathematically consistent with the earlier Rot-axis mirroring check, not a
regression). Regression-tested at rest: identical elbow/twist/residual
values to the pre-rework baseline. Full combined crouch+reach pose tested
(hips + shoulder posed together, all leg/arm solvers run) — zero NaN,
zero mesh breakage, confirmed via render
(`blender/ik_rework_crouch_reach_check.png`). Boundary clamping under that
extreme, uncoordinated pose (targets weren't moved to match) is expected
correct fallback behavior per the solver's documented design, not a defect.

## Empirical Axis Verification (performed, per §8 — not assumed)

Built and tested live via `scripts/rig/shoulder_hip_builder_v1.py`'s
`verify_axes()`: for each new MECH bone, rotate it 15° on its assigned
axis, read the resulting world-space shift of `DEF_UpperArm`'s tail, reset.

- **FlexExt (X)**: confirmed real, non-degenerate motion (~0.065-0.08m
  world shift at 15°, matching the expected arc length for this bone's
  ~0.3m span). **Found a genuine bug**: the right and left sides moved in
  *opposite* physical directions for the same +15° input (right: tail
  moved forward, `world Y -0.039`; left: tail moved backward, `world Y
  +0.043`) — traced to `DEF_UpperArm_L`'s own rest roll (authored during
  the original mirror phase, unrelated to this build) not preserving
  "same positive angle = same physical direction" for this axis. **Fixed**
  via `Copy Rotation`'s built-in `invert_x` on `MECH_Shoulder_L_FlexExt`'s
  constraint, with the Limit Rotation range correspondingly negated/swapped
  to `(-140°, +40°)` (the real, physical range post-invert) so an animator
  can use the same `(-40°, +140°)` conceptual convention on both sides.
  Re-verified after the fix: both sides now shift `world Y` in the same
  sign for the same input (confirmed exactly: right `-0.0393`, left
  `-0.0434`).
- **AbdAdd (Z)**: confirmed real motion, opposite `world X` sign between
  sides — this is *correct*, not a bug (abduction is genuinely
  outward-and-opposite on left vs. right).
- **Rot (Y)**: tail-displacement is definitionally zero for this axis
  (twisting a bone around its own length axis can't move a point on that
  axis). Verified instead via a perpendicular reference vector (the DEF
  bone's own local-X axis in world space) rotated 15° and checked against
  each side's own length axis: signed handedness came out identical between
  sides (`0.2588` both), confirming no mirroring correction is needed here
  — matches the (unmirrored) convention already used for forearm twist.
- **Boundary clamp test**: `CTRL_Shoulder_L_FlexExt` driven to 140° and 200°
  produced identical `DEF_UpperArm_L` tail positions (clamped correctly at
  the true boundary); same confirmed at the -40°/-100° extension boundary.
- **Rest-pose integrity**: `DEF_UpperArm`/`DEF_UpperArm_L` world tail
  positions after adding the full mechanism and resetting all new CTRL
  bones to zero matched their pre-build rest positions exactly (diff
  <0.001) — the new mechanism introduces zero rest-pose drift.
- **Visual pose check**: both shoulders driven to +60° FlexExt rendered
  with both arms raising forward symmetrically (`blender/shoulder_hip_pose_check.png`)
  — no mesh breakage, no asymmetric motion.

## Status

**VALIDATED** at the structural/mechanical/axis level (built, empirically
axis-verified per §8, boundary-clamped, rest-pose-integrity-checked, and
visually confirmed via render — not merely described). **Not yet validated**
for animation or export (§13's Layer 3/4) — no animation using these joints
exists yet, and per `docs/env_warehouse_phase1.md`'s equivalent gap, no
export→reimport check has been run for these bones. `hello_sequence`'s
existing 4 fcurves were confirmed unaffected (bone count went 160→184,
exactly the 24 new bones, no stray keyframes).
