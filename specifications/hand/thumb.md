# THUMB — Joint Specification

## Revision v18 — axis contract restored, thumb opposed forward

**Status: VALIDATED.** Authorised change (docs/ENGINEERING_RULES.md §14), see `specifications/hand/fingers.md` "Revision v18" for the shared root cause and mirror convention.

* `DEF_Thumb_{CMC,MCP,IP}` heads / tails / rolls were re-posed and `MECH_Thumb_*` / `CTRL_Thumb_*` placed on the DEF frames they drive (the POSE-space copy into
  `DEF_Thumb_CMC` is now an identity at rest). The thumb is opposed **forward**: the CMC→MCP direction is 25° from the hand axis toward the front, with 8° / 8° of
  relaxed curl at MCP / IP, palm side inward. The DEF chain stays connected.
* Axes: CMC Flex, MCP and IP flex about local **X** (positive toward the palm); CMC Abd about local **Z**. Left Z limits are the right limits negated; the same flex value curls
  both thumbs inward (mirror convention as in fingers.md).
* Re-validation: zero-command deviation 0.0000° on all thumb DEF bones; thumb direction measured 25.0° from the hand axis with y < 0 (forward).

## Segment Overview

The thumb is a 3-joint serial chain carrying 4 actuated rotational DOF
total — the single most functionally important motion in the whole hand:
without opposition, the hand can rake and hook but can't pinch or grip.

- **CMC** (carpometacarpal / trapeziometacarpal) — 2R actuated, condyloid
  (saddle) joint. Per docs/ENGINEERING_RULES.md §7's own classification table
  ("Condyloid | Wrist, MCP, thumb CMC | 2–3R | 2R"), this is the same
  joint-type family as the wrist, built the same way: two serial
  1R revolutes.
- **MCP** (metacarpophalangeal) — 1R actuated, hinge (flexion/extension;
  sources agree ab/adduction at this joint is small enough to omit,
  consistent with how this project treats every joint's negligible
  secondary axis).
- **IP** (interphalangeal — the thumb has only one, unlike the fingers'
  PIP+DIP) — 1R actuated, hinge.

Unlike the fingers (which will get PIP-DIP coupling in `fingers.md`), the
thumb has no middle phalanx and no coupling mechanism — each of its three
joints is independently actuated.

This spec does not build a palm "mount" for the thumb the way `palm.md`
built rigid/actuated mounts for the four fingers — there's no separate
rigid CMC-adjacent structure to model that way. The CMC joint (MC1's
base) is itself the first actuated joint in the chain.

**Total actuated DOF**: 2 (CMC) + 1 (MCP) + 1 (IP) = 4R, matching a common
DOF count for robotic thumb designs. Four MECH bones are built below — one
per actuated DOF, per docs/ENGINEERING_RULES.md §7's "never more MECH bones than the
classified actuated-DOF count" rule.

---

## JOINT 1: CMC Flexion/Extension

### Classification

**Type**: Saddle (biconcave), one axis of the CMC's two-axis pair
**DOF**: 1R (this bone), part of the CMC's 2R actuated total
**Biological ROM**: Two independent studies converge closely — a 200-hand
optoelectronic study (101 subjects) measured 41° total flexion-extension
range; a separate 96-subject age/gender-normative goniometer study
measured 21.7° flexion + 19.5° extension (41.2° total) — the same figure
from a different method and with the flexion/extension split this spec
needs.
**Load**: Moderate — the CMC bears the base reaction force for every
pinch and power grip the hand performs.
**Axis**: In the frontal plane, parallel to the palm.

### Mechanical Implementation

**MECH layer**:
- Bone name: `MECH_Thumb_CMC_Flex`
- Local axis: **X** (hypothesis — verify empirically per §8 before wiring
  any constraint, the same requirement as every joint so far; X is the
  hypothesis because every flexion axis built so far in this project has
  turned out to be local X once verified)
- Locked axes: Y, Z
- Parent: `DEF_Wrist` (no palm mount exists for the thumb)
- Child: `MECH_Thumb_CMC_Abd`

**Operational limits**:
- X min: −20° (extension)
- X max: +22° (flexion)
- Y/Z: locked (min = max = 0°)

**Rigify rig type**: None — plain bone, hand-wired via the Constraint
Stack below (no `rigify_type` assigned).

### Constraint Stack

Copy Rotation first, then Limit Rotation — the standing project-wide rule
from `docs/NAMING.md`.

```
MECH_Thumb_CMC_Flex:

Copy Rotation (from CTRL_Thumb_CMC_Flex)
  Target: CTRL_Thumb_CMC_Flex
  Space: Local
  Influence: 1.0
Limit Rotation
  Min X: −20°, Max X: +22°
  Y/Z: locked (min = max = 0°)
  Space: Local
```

---

## JOINT 2: CMC Abduction/Adduction

### Classification

**Type**: Saddle, second axis of the CMC pair
**DOF**: 1R (this bone)
**Biological ROM**: Strong agreement between the same two sources —
51° (optoelectronic study) and 51.1° ± 5.5° (age/gender-normative study),
essentially identical independent measurements.
**Load**: Moderate — this axis opens the web space for grasping larger
objects (e.g. a cup) and positions the thumb for opposition.
**Axis**: In the sagittal plane, perpendicular to the palm — roughly
perpendicular to the CMC flexion axis above.

### Mechanical Implementation

**MECH layer**:
- Bone name: `MECH_Thumb_CMC_Abd`
- Local axis: **Z** (hypothesis — verify empirically; the wrist's
  analogous second condyloid axis, `MECH_Wrist_Dev`, verified as local Z,
  and this bone is built the same way)
- Locked axes: X, Y
- Parent: `MECH_Thumb_CMC_Flex`
- Child: `MECH_Thumb_MCP`

**Operational limits**:
- Z min: −10° (adduction)
- Z max: +41° (abduction — split of the ~51° measured arc biased toward
  the functionally useful direction, since full abduction is what opens
  the hand for grasp and opposition reach)
- X/Y: locked (min = max = 0°)

**Rigify rig type**: None — same hand-wired pattern as Joint 1.

### Constraint Stack

```
MECH_Thumb_CMC_Abd:

Copy Rotation (from CTRL_Thumb_CMC_Abd)
  Target: CTRL_Thumb_CMC_Abd
  Space: Local
  Influence: 1.0
Limit Rotation
  Min Z: −10°, Max Z: +41°
  X/Y: locked (min = max = 0°)
  Space: Local
```

### Note on omitted coupling

The 200-hand optoelectronic study also measured a small (~21°) axial
rotation coupled to CMC motion during circumduction. This is deliberately
**not** modeled as an independent or coupled DOF here — same reasoning as
`palm.md`'s MC5 coupled rotation and the wrist's third-axis fold-in: real
and measured, but a geometric consequence of the saddle-joint surface, not
an independently useful actuator for a robotic hand, and docs/ENGINEERING_RULES.md's own
table caps thumb CMC at 2R actuated (not 3R). (A separate claim about a
specific CMC-MCP abduction correlation coefficient was considered for
this note but could not be verified against a real source in this
project's research pass — dropped rather than cited unverified, per §17.)

### DEF_Thumb_CMC — Combined Output

The CMC's two MECH bones drive a single combined DEF bone,
`DEF_Thumb_CMC` — the same pattern validated on `wrist.md`'s `DEF_Wrist`,
for the same reason: `MECH_Thumb_CMC_Abd` sits at the bottom of the
two-bone CMC chain, so its **POSE-space** (armature-space) orientation
already carries both axes' cumulative rotation. A Local-space copy would
only capture the Abd bone's own delta and silently drop Flex — verified
empirically on the wrist build (matrix-composition check, exact match)
and reused here without needing to re-derive it. This keeps the CMC at
one DEF bone instead of two, which also gives the MCP joint below one
clear attachment point instead of two.

```
DEF_Thumb_CMC:

Copy Rotation (from MECH_Thumb_CMC_Abd)
  Target: MECH_Thumb_CMC_Abd
  Space: Pose (armature space) — NOT Local
  Influence: 1.0
```

**Parent**: `DEF_Wrist` (DEF chain stays self-contained, parented only to
other DEF bones, per the lesson from the elbow build about keeping the
DEF-only export chain simple to strip CTRL/MECH from later — no
reparenting needed at bake time).

---

## JOINT 3: MCP (Metacarpophalangeal) Flexion/Extension

### Classification

**Type**: Hinge
**DOF**: 1R
**Biological ROM**: Two independent sources converge well — a 300-hand
goniometer study (150 subjects) measured mean flexion 59.5° (range
16–90°) and mean extension 7.9° (range −32 to 58°); Barakat, Field &
Taylor's 2013 prospective study of 100 individuals measured mean flexion
60° (range 43–70°) and mean extension 8.1° (range 0–15°). Both flexion
(~60°) and extension (~8°) agree closely across the two studies.
**Load**: Moderate — primary joint for closing the thumb tip onto an
object during pinch.
**Axis**: Transverse, through the metacarpal head — a true hinge, unlike
the CMC.

### Mechanical Implementation

**MECH layer**:
- Bone name: `MECH_Thumb_MCP`
- Local axis: **X** (hypothesis — verify empirically per §8)
- Locked axes: Y, Z
- Parent: `MECH_Thumb_CMC_Abd`
- Child: `MECH_Thumb_IP`

**Operational limits**:
- X min: −8° (extension)
- X max: +60° (flexion — the round, well-converged mean from both
  sources)
- Y/Z: locked (min = max = 0°)

**Rigify rig type**: None — hand-wired.

### Constraint Stack

```
MECH_Thumb_MCP:

Copy Rotation (from CTRL_Thumb_MCP)
  Target: CTRL_Thumb_MCP
  Space: Local
  Influence: 1.0
Limit Rotation
  Min X: −8°, Max X: +60°
  Y/Z: locked (min = max = 0°)
  Space: Local
```

### DEF_Thumb_MCP

A single MECH bone with no second axis to combine — the simpler
`elbow_forearm.md` pattern (Local space, not Pose space):

```
DEF_Thumb_MCP:

Copy Rotation (from MECH_Thumb_MCP)
  Target: MECH_Thumb_MCP
  Space: Local
  Influence: 1.0
```

**Parent**: `DEF_Thumb_CMC` (DEF chain stays serial and self-contained;
inherits the CMC's cumulative rotation via bone-parent inheritance, then
adds its own local MCP delta via this constraint).

---

## JOINT 4: IP (Interphalangeal) Flexion/Extension

### Classification

**Type**: Hinge
**DOF**: 1R
**Biological ROM**: Barakat, Field & Taylor (2013), the same 100-subject
study, gives mean flexion 88° (range 80–90°, tight and consistent) and
mean extension 12° (range 0–45°, much wider spread — extension hyperlaxity
varies a lot between individuals).
**Load**: Low-moderate — provides the final tip-curl for precision pinch
and fine manipulation.
**Axis**: Transverse, through the distal end of the proximal phalanx.

### Mechanical Implementation

**MECH layer**:
- Bone name: `MECH_Thumb_IP`
- Local axis: **X** (hypothesis — verify empirically per §8)
- Locked axes: Y, Z
- Parent: `MECH_Thumb_MCP`
- Child: none (distal end of the thumb chain)

**Operational limits**:
- X min: 0° (extension — using 0° rather than the measured mean extension
  figure, since that spread (0–45°) reflects individual hyperlaxity
  variance rather than a reliable functional target; 0° is the
  conservative, non-hyperextended choice, same reasoning
  `elbow_forearm.md` used for the elbow's minimum)
- X max: +85° (flexion — slightly conservative relative to the 88° mean,
  same margin-below-full-ROM approach used for the elbow's 130° vs. 150°)
- Y/Z: locked (min = max = 0°)

**Rigify rig type**: None — hand-wired.

### Constraint Stack

```
MECH_Thumb_IP:

Copy Rotation (from CTRL_Thumb_IP)
  Target: CTRL_Thumb_IP
  Space: Local
  Influence: 1.0
Limit Rotation
  Min X: 0°, Max X: +85°
  Y/Z: locked (min = max = 0°)
  Space: Local
```

### DEF_Thumb_IP

```
DEF_Thumb_IP:

Copy Rotation (from MECH_Thumb_IP)
  Target: MECH_Thumb_IP
  Space: Local
  Influence: 1.0
```

**Parent**: `DEF_Thumb_MCP`.

---

## Serial Hierarchy

```
DEF_Wrist (already validated, from wrist.md)
├── MECH_Thumb_CMC_Flex (1R, X-axis)
│   └── MECH_Thumb_CMC_Abd (1R, Z-axis)
│       └── MECH_Thumb_MCP (1R, X-axis)
│           └── MECH_Thumb_IP (1R, X-axis)
└── DEF_Thumb_CMC (combined Flex+Abd, Pose-space copy from MECH_Thumb_CMC_Abd)
    └── DEF_Thumb_MCP (Local-space copy from MECH_Thumb_MCP)
        └── DEF_Thumb_IP (Local-space copy from MECH_Thumb_IP)
```

Four actuated DOF in a single serial chain — the longest MECH chain built
so far in this project (previous joints topped out at 2). Watch for
cumulative orientation drift in Layer 2 validation; this wasn't a problem
for any prior 1–2-bone chain but hasn't been tested at this depth yet.

**Serial order decision**: CMC → MCP → IP follows direct anatomical
precedence (not an engineering choice — the thumb's three joints are
physically in this order along the bone chain). Flexion-before-abduction
*within* the CMC pair is an engineering choice (mirroring the wrist's own
Flex-before-Dev precedent for consistency, not a firm biological
requirement) — flag and swap if empirical testing shows the reverse order
behaves better once real grasp poses can be tested.

**CTRL chain**: `CTRL_Thumb_CMC_Flex → CTRL_Thumb_CMC_Abd →
CTRL_Thumb_MCP → CTRL_Thumb_IP`, parented in the same serial order as the
MECH chain, each positioned to match its corresponding MECH bone.

---

## Metarig / Bone Setup

No Rigify rig type assignment for any bone in this spec.

1. Create `MECH_Thumb_CMC_Flex` as a child of `DEF_Wrist`
2. Create `MECH_Thumb_CMC_Abd` as a child of `MECH_Thumb_CMC_Flex`
3. Create `MECH_Thumb_MCP` as a child of `MECH_Thumb_CMC_Abd`
4. Create `MECH_Thumb_IP` as a child of `MECH_Thumb_MCP`
5. Create `DEF_Thumb_CMC` as a child of `DEF_Wrist` (not of any MECH bone
   — see the DEF-chain-self-contained rationale under Joint 2)
6. Create `DEF_Thumb_MCP` as a child of `DEF_Thumb_CMC`
7. Create `DEF_Thumb_IP` as a child of `DEF_Thumb_MCP`
8. Create the four `CTRL_Thumb_*` bones per the naming convention,
   positioned to be easy for an animator to grab, parented in the same
   serial order as the MECH chain
9. **Before wiring any constraint**: verify each MECH bone's actual local
   axes via MCP inspection (world-space matrix column readout, the method
   used on every joint so far) — the X/Z hypotheses above are starting
   points, not confirmed facts
10. Wire all four Constraint Stacks in the corrected order (Copy
    Rotation, then Limit Rotation), and the two DEF composition
    constraints (Pose-space for `DEF_Thumb_CMC`, Local-space for the
    other two)

**Positioning note**: no coordinates are hard-coded here, same policy as
`palm.md` — hand scale isn't settled yet. When built, the thumb chain
should originate from `DEF_Wrist` offset toward the side `palm.md`
assumed was the thumb side (+Y, per that spec's own placeholder
assumption), and angled somewhat forward/away from the finger mounts'
plane to reflect the thumb's opposable orientation — get it right by eye
against reference proportions, then record the resulting local offsets
here once built.

---

## Animation Workflow

### Test Animation: Opposition Reach

A four-DOF coordinated motion, the most complex test animation in this
project so far:

**Frame 0**: Rest pose — CMC_Flex 0°, CMC_Abd 0°, MCP 0°, IP 0°

**Frame 10**: Full abduction (opening the web space) — CMC_Flex 0°,
CMC_Abd +41°, MCP 0°, IP 0°

**Frame 20**: Opposition curl (thumb sweeps toward little-finger side) —
CMC_Flex +22°, CMC_Abd +20°, MCP +60°, IP +85°

**Frame 30**: Back to rest — all axes 0°

### Expected Result

- Smooth, monotonic motion on all four axes through the full sequence
- No axis exceeds its own operational limit at any frame
- Frame 10 → 20: CMC_Abd should visibly *decrease* (from +41° toward
  +20°) while CMC_Flex, MCP, and IP all increase — a genuine multi-axis
  coordination test, not four independent single-axis checks
- `DEF_Thumb_CMC` at frame 20 should reflect the *combined* Flex+Abd
  rotation (matrix composition check, same verification method used on
  `DEF_Wrist` — not just an Euler-angle eyeball check)
- Independence check (same pattern as the wrist's Flex-holds-steady
  test): hold CMC_Flex and CMC_Abd fixed at frame 20's values while
  animating MCP and IP alone through a short sub-range, confirm CMC
  doesn't drift

---

## Validation Checklist

**Structural**:
- [x] All four MECH bones exist, named correctly, no rigify_type assigned
- [x] All in the MECH collection
- [x] Local axes verified empirically (not assumed). Result: **the
      X/Z hypothesis did not hold cleanly** — this is the first joint
      chain in the project built off-axis (angled forward+sideways from
      DEF_Wrist rather than straight down), so no bone's local axes came
      out aligned to a single world axis the way every prior joint did.
      `MECH_Thumb_CMC_Flex`'s local X = (0, 0.894, 0.447) — Y-dominant,
      directionally consistent with a transverse flexion axis.
      `MECH_Thumb_CMC_Abd`'s local Z = (−0.913, 0.183, −0.365) —
      X-dominant, directionally consistent with "perpendicular to the
      palm." Proceeded with the X/Z assignment on that directional
      match; mechanical correctness was then confirmed independently via
      the matrix-composition checks below, which don't depend on the
      axes being world-aligned.

**Mechanical**:
- [x] CTRL_Thumb_CMC_Flex clamps at both −20° and +22°
- [x] CTRL_Thumb_CMC_Abd clamps at both −10° and +41°
- [x] CTRL_Thumb_MCP clamps at both −8° and +60°
- [x] CTRL_Thumb_IP clamps at both 0° and +85°
- [x] Cross-axis leak test on all four CTRL bones — all locked axes
      stayed at 0° on every corresponding MECH bone
- [x] DEF_Thumb_CMC's combined rotation verified against an independently
      computed Rotation(X,flex) @ Rotation(Z,abd) matrix at three
      different flex/abd combinations (clamped-limit, opposite-limit,
      mid-range) — exact match (<0.001° diff) every time
- [x] Constraint order confirmed live on all four MECH bones (Copy
      Rotation before Limit Rotation) — checked directly
- [x] Checked for cumulative orientation drift across the 4-bone chain.
      **First attempt failed** (up to 2.6° diff) — but the failure was in
      the test's own expected-value formula, which assumed all four
      joints rotate around shared world-aligned axes. Since the thumb
      bends direction at each segment (unlike every prior straight
      chain), each bone has a different rest orientation relative to its
      parent, and that has to be accounted for in the expected
      calculation. Redone using each bone's actual measured rest
      transform: exact match (<0.001° diff) at full-max, full-min, and
      mid-range combinations. The rig itself was correct throughout —
      only the first test formula was wrong.

**Animation**:
- [x] Opposition Reach test animation plays smoothly (sampled every
      frame 0–30)
- [x] CMC_Abd visibly decreases (41°→20°) while CMC_Flex, MCP, and IP all
      increase (frame 10→20)
- [x] CMC holds steady (exactly 22.00°/20.00°) across four different
      MCP/IP sub-range values — independence confirmed

**Export**:
- [x] Baked to DEF-only keyframes (11 DEF bones selected explicitly, no
      stray CTRL/MECH fcurves)
- [x] Export/reimport per §5. Hit a **new gotcha** not previously logged:
      the scene's `frame_end` was still 20 (left over from the elbow
      build) while this animation runs to frame 30 — the bake silently
      truncated at frame 20, and frames past that held frame 20's value
      in the reimported file. Fixed by setting `scene.frame_end` to match
      the actual animation length *before* baking. Re-verified after the
      fix: all sampled frames (0/10/20/30) matched exactly, including the
      DEF_Thumb_CMC composed-rotation check.

**Status after validation**: `VALIDATED` (bone/animation mechanism only —
mesh-deformation check deferred until a mesh exists)

### Validation Report

```
VALIDATION REPORT: Thumb (CMC Flex + CMC Abd + MCP + IP)

Status: VALIDATED

Layer 1 (Structural): PASS — all four MECH bones exist correctly; axes
                       verified empirically, did not land world-axis-
                       aligned (first off-axis joint chain in the
                       project) but matched the hypothesis directionally;
                       mechanical correctness confirmed independently of
                       axis alignment via matrix composition checks
Layer 2 (Mechanical):  PASS — all four joints clamp correctly at both
                        ends; cross-axis locked; DEF_Thumb_CMC's combined
                        rotation matches an independently computed
                        composition matrix exactly at three test points;
                        full 4-bone chain cumulative composition verified
                        exact once the test accounted for each bone's own
                        rest orientation (first test attempt was wrong,
                        not the rig -- corrected and re-verified)
Layer 3 (Animation):   PASS -- Opposition Reach test (frames 0/10/20/30),
                        sampled every frame, CMC_Abd decreases while the
                        other three increase (frame 10-20), CMC holds
                        exactly steady across four independence sub-tests
Layer 4 (Export):      PASS -- DEF-only baked rig (11 bones) exported to
                        blender/robot_test_thumb.glb, reimported with fps
                        set pre-import and frame_end corrected to match
                        the animation's actual length (new gotcha found
                        and fixed), matched the pre-export reference
                        exactly at every sampled frame including the
                        composed CMC rotation

New gotchas logged for future joints:
1. Scene frame_end must be checked/extended before baking if a joint's
   test animation runs longer than whatever the last joint left it at --
   the bake silently truncates rather than erroring.
2. For any joint chain that bends direction (not a straight line from its
   parent), matrix-composition validation must use each bone's actual
   measured rest transform, not an assumed world-axis rotation product --
   naive world-axis composition produces false failures on bent chains
   even when the rig is correct.

Validated by: Claude Code
Timestamp: 2026-09-01T03:01:00Z
```

---

## Notes & Future Work

- The CMC's coupled axial rotation (~21°) is intentionally omitted — same
  reasoning as `palm.md`'s MC5 coupled rotation and the wrist's third-axis
  fold-in. Documented here so it isn't mistaken for an oversight.
- `DEF_Thumb_CMC` parents to `DEF_Wrist` directly rather than to
  `DEF_Palm_Index` (an alternative considered during drafting, on the
  reasoning that the trapezium sits near the palm's radial/index side).
  Kept on `DEF_Wrist` to preserve the fully self-contained DEF-chain
  architecture used throughout this file; this is a positioning choice,
  not a mechanics one — revisit once real hand proportions exist and the
  index-side placement can be judged by eye.
- Thumb positioning (offset from `DEF_Wrist`, orientation relative to the
  finger mounts) depends on `palm.md`'s own placeholder thumb-side
  assumption (+Y) and on hand scale, neither of which is finalized.
  Revisit both together once fingers exist and real grasp poses can be
  tested.
- **Correction (2026-09-01)**: all four thumb MECH bones and all four
  CTRL bones were built with `use_deform=True`, incorrectly making them
  mesh-skinning-eligible alongside `DEF_Thumb_CMC/MCP/IP`. Corrected to
  `use_deform=False`, project-wide, per docs/ENGINEERING_RULES.md §5. Re-validated with
  extra care given the thumb's off-axis rest pose (see Layer 1 note
  above): `DEF_Thumb_CMC`'s combined rotation at CMC_Flex 22°/CMC_Abd 41°
  matched the pre-correction value exactly once compared as a delta from
  its own (non-identity) rest orientation, not as an absolute rotation —
  the same category of check this file's own Layer 2 validation already
  established. `DEF_Thumb_MCP`/`DEF_Thumb_IP` matched exactly too. No
  change to ROM, axes, or exported animation.
- This spec completes the thumb chain. `fingers.md` (index/middle/ring/
  little PP→MP→DP chains) is the only remaining hand spec.

---

## References

- [Normal range-of-motion of trapeziometacarpal joint — PubMed 19762264](https://pubmed.ncbi.nlm.nih.gov/19762264/) — 200-hand optoelectronic study: CMC flexion-extension 41°, abduction-adduction 51°, axial rotation 21°. Verified against live search: figures match exactly.
- [Influence of age and gender on normative passive range of motion values of the carpometacarpal joint of the thumb — ScienceDirect / PubMed 28478845](https://www.sciencedirect.com/science/article/abs/pii/S0894113017300595) — 96-subject goniometer study: CMC flexion 21.7°, extension 19.5°, abduction 51.1°. Verified against live search: figures match exactly; independently converges with the study above.
- [Barakat, Field & Taylor, "The Range of Movement of the Thumb", HAND 8:179-182 (2013), DOI 10.1007/s11552-013-9492-y](https://link.springer.com/article/10.1007/s11552-013-9492-y) — 100-subject study: IPJ flexion 88° (80–90°) / extension 12° (0–45°); MCPJ flexion 60° (43–70°) / extension 8.1° (0–15°). Verified against live search: figures match exactly.
- [AAHS thumb MCP range of motion study, 300 hands](https://meeting.handsurgery.org/abstracts/2020/HSEP245.cgi) — 150-subject goniometer study: MCP flexion 59.5° (16–90°), extension 7.9° (−32 to 58°). Verified against live search: figures match exactly; independently converges with Barakat et al.'s MCP figures.
- `specifications/arm/elbow_forearm.md`, `specifications/arm/wrist.md`,
  `specifications/hand/palm.md` — source of every corrected pattern
  applied throughout this file (no invented Rigify rig type, Copy
  Rotation before Limit Rotation, Pose-space composition for a combined
  multi-axis DEF bone, DEF-only bake selection)
