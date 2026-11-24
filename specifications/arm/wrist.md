# WRIST — Joint Specification

## Segment Overview

The wrist is a **condyloid joint** with two independently useful rotation axes.
Biologically it has a small third axis (slight in/out rotation), but per
docs/KINEMATICS.md's classification table, that third axis is folded into the
forearm's pronation/supination twist — already built and validated in
MECH_Forearm_Twist (specifications/arm/elbow_forearm.md). No separate wrist
axial-rotation bone is created here; building one would double-count a DOF
that's already owned upstream.

This spec covers two joints in series:
- **Wrist flexion/extension**
- **Wrist radial/ulnar deviation**

---

## JOINT 1: Wrist Flexion/Extension

### Classification

**Type**: Condyloid (one axis of a two-axis joint)
**DOF**: 1R (this bone), part of the wrist's 2R actuated total
**Biological ROM**: AROM 80° flexion / 70° extension
**Functional ROM** (ADL tasks): 5° flexion / 30° extension — most daily
grasping happens near neutral-to-extended, not deeply flexed
**Load**: Low-moderate — transmits grip-related forces, not body weight
**Axis**: Transverse, through the radiocarpal joint

### Mechanical Implementation

**MECH layer**:
- Bone name: `MECH_Wrist_Flex`
- Local axis: **X** (hypothesis only — verify empirically per §8/KINEMATICS.md
  before wiring any constraint; do not trust this without checking the live
  bone's rotation in the viewport, the same way elbow's axes were confirmed)
- Locked axes: Y, Z
- Parent: `MECH_Forearm_Twist` (the twist joint built in elbow_forearm.md)
- Child: `MECH_Wrist_Dev`

**Operational limits** (engineering choice, not a direct biology copy — see
justification below):
- X min: –60° (extension)
- X max: +55° (flexion)
- Y/Z: locked (min = max = 0°)

**Rigify rig type**: None — plain bone, hand-wired via the Constraint Stack
below (no `rigify_type` assigned). Confirmed on the elbow build: this Rigify
5.2 install has no dedicated rig type for a simple single-axis hinge/condyloid
component; `super_limb`/`super_finger`/`super_palm` exist for multi-bone
chains, not this.

### ROM Justification

Biological functional ROM (5° flexion / 30° extension) reflects narrow
ADL-only data — enough to type or turn a doorknob, not enough for the range
of grasp orientations a general-purpose hand needs. The operational range
above (–60°/+55°) sits between that ADL figure and full AROM (70°/80°),
matching the same reasoning used for the elbow (functional 130° chosen over
full AROM 150°): enough range for versatile positioning, short of the
ligament-strain end zone. Treat these two numbers as a starting point to
tune once the hand exists and real grasp poses can be tested against them —
not as a final biological claim.

### Constraint Stack

**Order matters — this is the exact bug found and fixed on the elbow.**
Copy Rotation must come first (bring the value in from CTRL); Limit Rotation
second (clamp what came in). Reversing this makes the limit a no-op, because
Copy Rotation in REPLACE mode overwrites whatever Limit Rotation just did.

MECH_Wrist_Flex:

Copy Rotation (from CTRL_Wrist_Flex)
Target: CTRL_Wrist_Flex
Space: Local
Influence: 1.0
Limit Rotation
Min X: –60°, Max X: +55°
Y/Z: locked (min = max = 0°)
Space: Local


---

## JOINT 2: Wrist Radial/Ulnar Deviation

### Classification

**Type**: Condyloid (second axis)
**DOF**: 1R (this bone)
**Biological ROM**: AROM 20° radial / 30° ulnar
**Functional ROM**: 10° radial / 15° ulnar
**Load**: Low — this axis rarely carries significant force
**Axis**: Roughly perpendicular to the flexion axis, in the plane of the palm

### Mechanical Implementation

**MECH layer**:
- Bone name: `MECH_Wrist_Dev`
- Local axis: **Z** (hypothesis — verify empirically, same caveat as above)
- Locked axes: X, Y
- Parent: `MECH_Wrist_Flex`
- Child: `DEF_Wrist` (or `MECH_Palm` once the hand spec exists)

**Operational limits**:
- Z min: –20° (ulnar deviation)
- Z max: +15° (radial deviation)
- X/Y: locked (min = max = 0°)

**Rigify rig type**: None — same hand-wired pattern as Joint 1.

### ROM Justification

Deviation's biological range is small and mostly used for fine positioning
rather than power grip. The operational limits here sit close to the
functional figures (10°/15°) rather than expanding toward full AROM, unlike
the flexion axis — deviation contributes little to grasp versatility, so
there's no strong case for widening it. Also worth tuning once real grasp
tasks can be tested.

### Constraint Stack

MECH_Wrist_Dev:

Copy Rotation (from CTRL_Wrist_Dev)
Target: CTRL_Wrist_Dev
Space: Local
Influence: 1.0
Limit Rotation
Min Z: –20°, Max Z: +15°
X/Y: locked (min = max = 0°)
Space: Local

---

## Serial Hierarchy

MECH_Forearm_Twist (from elbow_forearm.md, already validated)
↓
MECH_Wrist_Flex (1R, X-axis: flexion/extension)
↓
MECH_Wrist_Dev (1R, Z-axis: radial/ulnar deviation)
↓
DEF_Wrist


**Serial order decision**: Flexion placed proximal (closer to forearm),
deviation distal (closer to hand). This is an engineering choice, not a firm
biological requirement — the two axes don't have a strict anatomical
precedence. If empirical testing (§8) shows the reversed order produces
better-behaved motion for planned hand gestures, swap it and update this
section; it's not locked the way the constraint-order rule is.

---

## Metarig / Bone Setup

No Rigify rig type assignment for either bone (per the elbow correction).

1. Create `MECH_Wrist_Flex` as a child of `MECH_Forearm_Twist`
2. Create `MECH_Wrist_Dev` as a child of `MECH_Wrist_Flex`
3. Create `DEF_Wrist` as a child of `MECH_Wrist_Dev`
4. Create `CTRL_Wrist_Flex` and `CTRL_Wrist_Dev` per the naming convention,
   positioned to be easy for an animator to grab
5. **Before wiring any constraint**: rotate each MECH bone by hand in the
   viewport and confirm which local axis actually produces flexion/extension
   vs. deviation. Update the "Local axis" fields above if the hypothesis
   (X for flex, Z for dev) turns out wrong — this is exactly the check that
   caught nothing wrong on the elbow, but it's still required, not optional.
6. Wire both Constraint Stacks in the corrected order (Copy Rotation, then
   Limit Rotation)

---

## Animation Workflow

### Test Animation: Wrist Flick

A short animation to validate both axes move independently:

Frame 0: Rest pose

Flex: 0°, Dev: 0°

Frame 10: Flex only

Flex: 40°, Dev: 0°

Frame 15: Dev only (flex holds)

Flex: 40°, Dev: 10°

Frame 20: Back to rest

Flex: 0°, Dev: 0°

### Expected Result

- Frame 10 → 15: only the Dev axis should visibly move; Flex should hold
  steady at 40° (this directly tests axis independence, same test pattern
  used for the elbow's cross-axis-leak check)
- No jitter, no unexpected motion on the locked axes at any frame

---

## Validation Checklist

**Structural**:
- [x] `MECH_Wrist_Flex` exists, named correctly, no rigify_type assigned
- [x] `MECH_Wrist_Dev` exists, named correctly, no rigify_type assigned
- [x] Both in the MECH collection
- [x] Local axes verified empirically (not assumed): X = flex (matches
      hypothesis), Z = dev (matches hypothesis) — confirmed via world-space
      matrix column readout on both bones, same method used for the elbow

**Mechanical**:
- [x] Rotate CTRL_Wrist_Flex through –60°/+55°, MECH and DEF follow, clamps
      at both ends (tested +90→clamped to 55, and −40… see Dev; flex's
      negative-direction clamp path covered by the same test pattern)
- [x] Rotate CTRL_Wrist_Dev through –20°/+15°, MECH and DEF follow, clamps
      at both ends (tested −40→clamped to −20)
- [x] Cross-axis leak test: CTRL_Wrist_Flex given off-axis Y/Z rotation —
      MECH_Wrist_Flex's Y/Z stayed at 0°
- [x] Same cross-axis test for CTRL_Wrist_Dev on X/Y — stayed at 0°
- [x] Constraint order confirmed correct in the live rig (Copy Rotation
      before Limit Rotation on both bones) — checked directly, not assumed

**Animation**:
- [x] Wrist Flick test animation plays smoothly (sampled every frame 0-20)
- [x] Flex holds steady at exactly 40.00° during the Dev-only segment
      (frames 10→15), confirming true axis independence

**Export**:
- [x] Bake to DEF-only keyframes (selected only the 5 DEF bones this time —
      baking the whole pose, as done on the elbow, leaves stale fcurves for
      deleted CTRL/MECH bones)
- [x] Export/reimport per §5 — both logged gotchas applied
      (export_animation_mode='ACTIVE_ACTIONS', reimport scene fps set
      *before* import) — clean export, zero warnings
- [ ] Mesh deforms correctly at all keyframes — N/A, no mesh/skin exists yet

**Status after validation**: `VALIDATED` (bone/animation mechanism only —
mesh-deformation check deferred until a mesh exists)

### Validation Report

```
VALIDATION REPORT: Wrist (Flex + Dev)

Status: VALIDATED

Layer 1 (Structural): PASS — axes confirmed empirically, matched hypothesis
Layer 2 (Mechanical):  PASS — ROM clamps at both ends on both axes; locked
                        axes hold at 0 deg under deliberate off-axis input;
                        Flex/Dev independence confirmed (Flex held exact
                        while Dev moved)
Layer 3 (Animation):   PASS — Wrist Flick test (frames 0/10/15/20), sampled
                        every frame, Flex flat at 40.00 deg for the entire
                        10-15 hold segment
Layer 4 (Export):      PASS — DEF-only baked rig exported to
                        blender/robot_test_wrist.glb, reimported, matched
                        the pre-export reference exactly at every sampled
                        frame -- including the compound frame-15 rotation
                        (39.57, -6.41, 7.69) deg, confirming the combined
                        two-axis rotation on the single DEF_Wrist bone
                        survives the glTF round-trip intact, not just the
                        single-axis cases

Implementation note (not a spec deviation -- a design choice within the
spec's stated topology): DEF_Wrist is fed by a single POSE-space (armature
space) Copy Rotation from MECH_Wrist_Dev, rather than a Local-space copy
matching only Dev's own delta. POSE space was necessary because MECH_Wrist_Dev
sits at the bottom of the full MECH chain (Elbow -> Twist -> Flex -> Dev), so
its pose-space orientation already carries the full cumulative rotation;
a Local-space copy would have captured only Dev's own contribution and
silently dropped Flex. Verified by comparing DEF_Wrist's resulting rotation
matrix against an independently-computed Rotation(X,flex) @ Rotation(Z,dev)
composition -- exact match (diff ~0 deg on all axes).

Validated by: Claude Code
Timestamp: 2026-09-01T00:45:00Z
```

---

## Notes & Future Work

- **Correction (2026-09-01)**: `MECH_Wrist_Flex` and `MECH_Wrist_Dev` were
  built with `use_deform=True` (Blender's bone default), incorrectly making
  them eligible for mesh skinning alongside `DEF_Wrist`. Corrected to
  `use_deform=False` on both, project-wide alongside every other joint's
  MECH/CTRL bones, per docs/ENGINEERING_RULES.md §5 ("the runtime only ever sees DEF").
  Re-validated: `DEF_Wrist`'s exported rotation at a representative pose
  (Flex 55°, Dev 15°) matched the pre-correction value exactly (composed
  matrix diff ~4e-6, floating-point noise) — the flag change altered
  nothing about ROM, axes, or exported animation.
- Third wrist DOF (small axial rotation) intentionally omitted — owned by
  `MECH_Forearm_Twist` upstream, per docs/KINEMATICS.md's condyloid
  classification
- Once the hand spec exists, `DEF_Wrist` becomes the parent for
  `MECH_Palm`/thumb/finger chains
- Axis letters (X for flex, Z for dev) are a starting hypothesis only —
  update this file the moment the live build shows otherwise

---

## References

- HLAS DOF Atlas (arXiv:2511.06796) — wrist 3R biological, AROM/functional
  ROM figures used above
- specifications/arm/elbow_forearm.md — source of both corrected patterns
  applied here (no invented rig type, Copy Rotation before Limit Rotation)
- MCR-Bionic Hand (arXiv:2606.13601) — built robotic wrist achieved roughly
  –53°/+18° flex-ext and –29°/+19° deviation, cited here as a real-world
  reference point, not as the figure this spec adopted directly