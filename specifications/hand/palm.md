 PALM — Joint Specification

## Revision v18 — palm frames

**Status: VALIDATED.** The four `DEF_Palm_*` bones were rotated with the hand (palm inward) and re-rolled so local **X** is the flexion axis; `MECH_Palm_{Ring,Little}` and
`CTRL_Palm_{Ring,Little}` sit on the DEF palm bones. The 0–12° arch limit on X is unchanged. The palm mesh (skinned to `DEF_Wrist`) was rotated with the hand about the
wrist head. See `specifications/hand/fingers.md` "Revision v18" for the cause, mirror convention and validation.

## Segment Overview

The palm consists of four metacarpal rays (MC2–MC5). Anatomically, MC2 and
MC3 (index and middle) are nearly rigid at their carpometacarpal joints,
while MC4 and MC5 (ring and little) have increasing mobility toward the
ulnar side — enabling "palm cupping," the concave curl that lets the hand
conform around cylindrical objects during power grasp.

This spec covers four bones in parallel (not serial), all children of
`DEF_Wrist`:
- **MC2 mount** (index base) — rigid
- **MC3 mount** (middle base) — rigid
- **MC4 mount** (ring base) — 1R actuated cupping
- **MC5 mount** (little base) — 1R actuated cupping

Each becomes the parent for its corresponding finger's first phalanx once
`thumb.md` / `fingers.md` exist. This spec only builds the palm mounts
themselves.

---

## MC2 / MC3 — Index and Middle Metacarpal Mounts

### Classification

**Type**: Rigid (not a joint) — CMC2/3 are described in the anatomical
literature as "almost immobile."
**DOF**: 0
**Load**: Primary structural spine of the hand — the index and middle rays
carry the bulk of grip and fingertip force due to their central position
and longest effective moment arm.

### Mechanical Implementation

No MECH bone, no CTRL bone, no constraint stack — there is no DOF to wire.
Per this project's own MECH/DEF philosophy (docs/ENGINEERING_RULES.md §4, reinforced by
KINEMATICS.md's "Floating/Deformable" category), a bone with zero actuated
DOF doesn't need a MECH shadow. It exists purely to give the index and
middle fingers correct anatomical spacing when their phalanx chains attach.

**Bone names**: `DEF_Palm_Index`, `DEF_Palm_Middle`
**Parent**: `DEF_Wrist` (both)
**Rigify rig type**: None — static structural bones, no rigify_type
assigned (consistent with the corrected pattern from elbow_forearm.md).

---

## MC4 — Ring Metacarpal Mount

### Classification

**Type**: Modified CMC gliding joint, functionally a low-range hinge for
this project's purposes.
**DOF**: 1R (cupping flexion)
**Biological ROM**: A "small degree of gliding" — sources describe this as
noticeably less than MC5's range but more than MC2/3's near-zero range. No
precise degree figure was found in the literature search for this spec;
treat the operational limit below as an engineering estimate, not a cited
anatomical value.
**Load**: Low — contributes conforming shape to grip, not primary force
**Axis**: Roughly parallel to the finger MCP flexion axis (transverse
across the hand), since cupping curls the ulnar side of the palm inward
alongside finger flexion, not perpendicular to it.

### Mechanical Implementation

**MECH layer**:
- Bone name: `MECH_Palm_Ring`
- Local axis: **X** (hypothesis — verify empirically per §8, same
  requirement as every other joint in this project; do not trust without
  checking the live bone in the viewport)
- Locked axes: Y, Z
- Parent: `DEF_Wrist`
- Child: `DEF_Palm_Ring` (which becomes the parent for the ring finger's
  first phalanx once fingers.md exists)

**Operational limits**:
- X min: 0°
- X max: 12° (conservative estimate — MC4's mobility is described as
  small relative to MC5; this is a starting point, not a fixed biological
  claim, and should be tuned once real grasp poses can be tested)
- Y/Z: locked (min = max = 0°)

**Rigify rig type**: None — hand-wired, no rigify_type assigned.

### Constraint Stack

**Order**: Copy Rotation first, then Limit Rotation — the corrected order
from elbow_forearm.md's Change Log. This is now the standing rule for
every joint in this project (see docs/NAMING.md), not something to
re-derive per spec.

MECH_Palm_Ring:

Copy Rotation (from CTRL_Palm_Ring)
Target: CTRL_Palm_Ring
Space: Local
Influence: 1.0
Limit Rotation
Min X: 0°, Max X: 12°
Y/Z: locked (min = max = 0°)
Space: Local

---

## MC5 — Little Metacarpal Mount

### Classification

**Type**: The most mobile CMC joint after the thumb — sources describe it
as capable of "flexion and rotation," not just gliding, driven primarily by
the opponens digiti minimi muscle.
**DOF**: 1R actuated (flexion). The muscle that drives this joint
biologically also produces lateral rotation as a coupled side-effect
(same "primary DOF + passive coupled rotation" pattern already used for the
knee) — that coupled rotation is intentionally **not** given its own
actuator here, for the same reason the knee's passive tibial rotation
wasn't: it's a geometric consequence of the joint surface, not an
independently useful DOF for a robotic hand.
**Biological ROM**: Described qualitatively as the largest of any CMC
joint besides the thumb; no single degree figure was found and cited
confidently in this search. Operational limit below is a deliberately
generous estimate relative to MC4, reflecting the anatomical consensus
that MC5 is meaningfully more mobile — treat it as a tuning starting
point.
**Load**: Low — same role as MC4, contributing to grip conformity, with
the little finger side bearing less peak force than index/middle.
**Axis**: Same general direction as MC4's — roughly parallel to finger MCP
flexion.

### Mechanical Implementation

**MECH layer**:
- Bone name: `MECH_Palm_Little`
- Local axis: **X** (hypothesis — verify empirically, same requirement)
- Locked axes: Y, Z
- Parent: `DEF_Wrist`
- Child: `DEF_Palm_Little` (parent for the little finger's first phalanx
  once fingers.md exists)

**Operational limits**:
- X min: 0°
- X max: 20° (larger than MC4's 12°, reflecting MC5's greater mobility —
  still an engineering estimate to tune later, not a precise biological
  citation)
- Y/Z: locked (min = max = 0°)

**Rigify rig type**: None — hand-wired.

### Constraint Stack

MECH_Palm_Little:

Copy Rotation (from CTRL_Palm_Little)
Target: CTRL_Palm_Little
Space: Local
Influence: 1.0
Limit Rotation
Min X: 0°, Max X: 20°
Y/Z: locked (min = max = 0°)
Space: Local

---

## Hierarchy

All four mounts are parallel children of `DEF_Wrist`, not a serial chain
(unlike every joint built so far):

DEF_Wrist
├── DEF_Palm_Index (rigid, 0 DOF)
├── DEF_Palm_Middle (rigid, 0 DOF)
├── MECH_Palm_Ring → DEF_Palm_Ring (1R cupping)
└── MECH_Palm_Little → DEF_Palm_Little (1R cupping)


Positioning note: the four mounts must be spaced across the width of the
hand in roughly their real anatomical order (index nearest the thumb side,
little furthest) — this spec doesn't hard-code coordinates, since that
depends on the hand's actual scale once modeled; get it right by eye
against reference proportions, then record the resulting local
offsets here once built, per §14's usual "verify against the live scene"
requirement.

---

## Animation Workflow

### Test Animation: Cupping Curl

Frame 0: Flat hand

MC4 (Ring): 0°, MC5 (Little): 0°

Frame 10: Full cup

MC4: 12°, MC5: 20°

Frame 20: Back to flat

MC4: 0°, MC5: 0°

### Expected Result

- MC5 should visibly curl further than MC4 at frame 10 (matching the
  anatomical asymmetry this spec is built around)
- MC2/MC3 mounts stay perfectly static throughout — no motion at all,
  since they're not actuated

---

## Validation Checklist

**Structural**:
- [x] `DEF_Palm_Index` and `DEF_Palm_Middle` exist, no rigify_type, no
      MECH bone, correctly parented to `DEF_Wrist`
- [x] `MECH_Palm_Ring` / `MECH_Palm_Little` exist, no rigify_type assigned
- [x] Local axes verified empirically (not assumed): X = transverse
      (world Y), matching the "parallel to finger MCP flexion" hypothesis
      for both MC4 and MC5

**Mechanical**:
- [x] Rotate CTRL_Palm_Ring through 0°–12°, clamps at both ends (tested
      25°→12° and within-range)
- [x] Rotate CTRL_Palm_Little through 0°–20°, clamps at both ends (tested
      −10°→0° and 30°→20°)
- [x] Cross-axis leak test on both — locked axes stayed at 0° under
      deliberate off-axis CTRL input
- [x] Confirmed DEF_Palm_Index / DEF_Palm_Middle never move under any
      input (rest, over-limit, and cross-axis test conditions)
- [x] Constraint order confirmed live: Copy Rotation before Limit Rotation
      on both actuated bones

**Animation**:
- [x] Cupping Curl test plays smoothly (sampled every frame 0-20)
- [x] MC5 visibly curls further than MC4 throughout the curve (12° vs 20°
      peak at frame 10, proportional at every intermediate frame)

**Export**:
- [x] Bake to DEF-only keyframes (8 DEF bones selected explicitly, no
      stray CTRL/MECH fcurves)
- [x] Export/reimport per §5 — export_animation_mode='ACTIVE_ACTIONS' and
      reimport scene fps set before import, both applied; zero warnings
      on export; reimported values matched the pre-export reference
      exactly at every sampled frame; Index confirmed static through the
      reimported animation too

**Status after validation**: `VALIDATED` (bone/animation mechanism only —
mesh-deformation check deferred until a mesh exists)

### Implementation Notes

- **Positioning**: no coordinates were specified in this spec (by design —
  it depends on hand scale, which doesn't exist yet). Built as a
  placeholder spread across world Y (hand width) at DEF_Wrist's tail
  (0.4, 0, 0.8): Index +0.04, Middle +0.013, Ring −0.013, Little −0.04,
  each spanning down 0.05 in Z. **Assumption**: +Y was arbitrarily chosen
  as the thumb side, since no thumb spec exists yet to confirm handedness.
  Revisit once `thumb.md` exists and hand orientation is settled.
- **DEF_Hand removed**: the placeholder end-effector bone from the elbow
  build was deleted per the project owner's explicit decision — the four
  `DEF_Palm_*` bones are now DEF_Wrist's real children, matching this
  spec's hierarchy exactly.
- **File location**: this spec was originally saved to
  `specifications/arm/PALM.md` (wrong folder, wrong case) and was moved to
  `specifications/hand/palm.md` before this build, per the project owner's
  explicit instruction. Referenced here so the history isn't lost.

### Validation Report

```
VALIDATION REPORT: Palm (MC2-MC5 mounts)

Status: VALIDATED

Layer 1 (Structural): PASS — DEF_Palm_Index/Middle rigid & static, correctly
                       parented; MECH_Palm_Ring/Little axes confirmed
                       empirically (X=transverse), matched hypothesis
Layer 2 (Mechanical):  PASS — ROM clamps at both ends on both actuated
                        bones (Ring 0-12deg, Little 0-20deg); locked axes
                        hold at 0deg under off-axis input; Index/Middle
                        confirmed immovable under all test conditions
Layer 3 (Animation):   PASS — Cupping Curl test (frames 0/10/20), sampled
                        every frame, Little consistently curls further
                        than Ring, peak values exact at frame 10
Layer 4 (Export):      PASS — DEF-only baked rig (8 bones) exported to
                        blender/robot_test_palm.glb, reimported with fps
                        set pre-import, matched the pre-export reference
                        exactly at every sampled frame; Index confirmed
                        static in the reimported file too

Validated by: Claude Code
Timestamp: 2026-09-01T02:30:00Z
```

---

## Notes & Future Work

- The two ROM figures in this spec (12° for MC4, 20° for MC5) are the
  least well-sourced numbers in any spec written so far — the literature
  search for this file found strong qualitative agreement that MC5 > MC4
  > MC2/3 in mobility, but no precise degree citation for either. Flag
  this explicitly if it ever needs re-deriving from a stronger source.
- MC5's coupled lateral rotation (from opponens digiti minimi) is
  deliberately omitted, same reasoning as the knee's passive rotation —
  documented here so it isn't mistaken for an oversight later.
- Once `thumb.md` and `fingers.md` exist, each finger's first phalanx
  parents onto its matching mount here (index → DEF_Palm_Index, etc.)
- Thumb is NOT part of this spec — its CMC joint (2R, saddle) is
  significantly more complex and gets its own file.
- **Correction (2026-09-01)**: `MECH_Palm_Ring` and `MECH_Palm_Little`
  (plus `CTRL_Palm_Ring`/`CTRL_Palm_Little`) were built with
  `use_deform=True`, incorrectly making them mesh-skinning-eligible
  alongside `DEF_Palm_Ring`/`DEF_Palm_Little`. Corrected to
  `use_deform=False`, project-wide, per docs/ENGINEERING_RULES.md §5. Re-validated: both
  bones' exported rotation at their max ROM (Ring 12°, Little 20°) matched
  the pre-correction value exactly — no change to ROM, axes, or animation.

---

## References

- Kenhub, ScienceDirect, and Wiley/Anatomical Record sources (2016–2026) —
  qualitative CMC2–5 mobility gradient (2/3 near-rigid, 4 slight glide, 5
  most mobile with coupled rotation)
- Opponens digiti minimi anatomy (multiple sources) — confirms MC5
  flexion is coupled with lateral rotation, the basis for this spec's
  "primary DOF + intentionally omitted coupled rotation" decision
- specifications/arm/elbow_forearm.md, specifications/arm/wrist.md —
  source of the corrected patterns applied throughout (no invented Rigify
  rig type, Copy Rotation before Limit Rotation)

Same as before: Notepad → save as palm.md in specifications/hand/ (you'll need to create that folder first, per the tree in docs/ENGINEERING_RULES.md). Then tell Claude Code to read it and build.

One thing to flag going in, since this spec says it plainly: the two ROM numbers (12° and 20°) are the weakest-sourced figures of anything written so far — real anatomical agreement on the direction (MC5 more mobile than MC4), but no solid degree citation for either. Worth remembering if Claude Code's own inspection step turns up something that makes those look wrong once fingers exist to test grasps against.