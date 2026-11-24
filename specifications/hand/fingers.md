# FINGERS — Joint Specification (Index, Middle, Ring, Little)

**Status: VALIDATED** — built in `RobotArmature`, empirically verified
against the live scene, and confirmed to survive a glTF export/reimport
round-trip losslessly. See the Validation Report at the end of this
section's checklist for the full record. This file supersedes two
earlier independent drafts of the same spec; the merge decisions and the
one design this file explicitly rejected are recorded in Notes & Future
Work so the reasoning isn't lost.

## Revision v18 — axis contract restored, rest frames aligned, relaxed neutral pose

**Status: VALIDATED** (`blender/inspection/v18/verification.md`). Authorised change to a locked spec (docs/ENGINEERING_RULES.md §14):

* **What changed.** (1) DEF finger and palm bones were re-posed and *re-rolled*; every `MECH_*` / `CTRL_*` finger and palm bone now sits on the exact
  head / tail / roll of the DEF bone it drives. (2) The hands were rotated back to *palm inward* (about the forearm axis) and given a relaxed neutral
  pose: joint flexion MCP 12° / PIP 18° / DIP 10° (cumulative 12° / 30° / 40° from the hand axis), MCP mounts evenly spaced (pitch 32 mm).
  (3) Left Z limits are the right limits negated. Limits, locks, DOF count and the MECH→DEF constraint topology are unchanged.
* **Why.** `DEF_{Finger}_MCP` copies `MECH_{Finger}_MCP_Abd` in POSE space, which copies an *absolute* orientation. All MECH/CTRL finger bones shared one
  common rack frame (local X pointed down the arm), unrelated to the DEF frames, so at zero command the finger chain sat 17°–178° off its rest orientation, and
  a corrected offset alone would still have curled the fingers about the wrong axis. v18 restores the contract this file already recorded as verified in
  the validation checklist (local X = world +Y flexion, local Z = world −X abduction, for the right hand).
* **Axis contract (verified, replaces every "hypothesis" below).** Local **X = flexion**, positive carries the finger toward the palm (inward); local **Z =
  abduction** = palm normal. Right hand at rest: X = world +Y, Z = world −X. MCP Flex limits on X, MCP Abd limits on Z, PIP on X, as written.
* **Mirror convention (fingers, thumb, palm).** Left bones are the world mirror of the right with x_L = −S·x_R, y_L = S·y_R, z_L = S·z_R (S = diag(−1,1,1)):
  the **same** flex value curls both hands inward; abduction mirrors with the **opposite** sign. (The wrist and legs keep the earlier x_L = S·x_R convention,
  where flexion mirrors with the opposite sign.)
* **Re-validation.** Zero-command deviation of every DEF bone (all 56, including the 30 finger / thumb bones) is 0.0000°; neutral-pose joint angles measured
  12.00° / 18.00° / 10.00°; +40° flex on both hands produces mirrored tip displacement; export → reimport max error < 1e-5 m.
* **Depends on this.** Any animation keying finger `CTRL` channels keeps its meaning (X flex, Z abd). `specifications/hand/thumb.md` and `palm.md` carry the same revision.

## Revision v19 — visual proportions only

Hand joint pieces refined (`docs/phase_v19_refine.md`): knuckle cylinders are 18.6 mm across (was 29.8 mm, −37.5 %) and 25.9 mm long, white phalanx shells 22.4 mm (was 26.4 mm), the thumb CMC sphere 24.0 mm.
No bone, axis, limit, lock or constraint changed; the axis contract and mirror convention of "Revision v18" stand.

## Segment Overview

Four parallel finger chains, same joint *types* across all four (MCP →
PIP → DIP). Each finger's MCP parents onto its matching palm mount from
`palm.md`: `DEF_Palm_Index`, `DEF_Palm_Middle`, `DEF_Palm_Ring`,
`DEF_Palm_Little`.

**Total actuated DOF: 11** — MCP Flex ×4, MCP Abd ×3 (Index, Ring,
Little — each fully independent), PIP Flex ×4. Middle has no abduction
actuator at all. DIP is driven on all four fingers, 0 actuated.

---

## DOF Allocation Decision

**MCP flexion/extension**: actuated on all four fingers — the primary
curling motion every finger needs for any grasp. Not a debatable call.

**MCP abduction/adduction**: actuated independently on **Index, Ring,
and Little**; Middle has no abduction bone at all (not locked — absent).

**Middle locked out**: uncontested across every version of this spec —
anatomically the middle finger is the hand's own reference axis, the
other fingers spread *relative to it*, so its own abduction is
functionally near-zero. No actuator, no bone.

**Index, Ring, and Little each get their own independent actuator**,
matching Index's own established pattern (serial `MCP_Flex → MCP_Abd`,
own `CTRL` bone, own `DEF` composition) rather than sharing one actuator
between any two fingers. This spec deliberately considered and rejected
a cheaper alternative: fusing Ring and Little onto a single shared
actuator (`MECH_RingLittle_MCP_Abd`), modeled on a real published
robotic-hand design, SABD Hand (arXiv:2509.13074), which fuses exactly
those two digits' abduction into one joint and reports a 400% workspace
gain with no loss of grasp success rate.

That design was rejected for this mascot, for a reason specific to what
a shared actuator actually does mechanically: a single input bone driving
both `DEF_Ring_MCP` and `DEF_Little_MCP` produces a **rigid 1:1 lock** —
Ring and Little would always move together, by the same angle, in the
same direction, with no pose in the rig where one moves and the other
stays still. That's an acceptable trade for a hand whose job is grasping
objects (SABD Hand's use case), where symmetric spread of the two ulnar
digits is functionally sufficient. It is not acceptable for a hand whose
job is a portfolio-site mascot's gesture range, where a pose like an
isolated pinky raise or a ring-only adjustment is exactly the kind of
expressive detail a shared actuator makes physically unreachable, not
just harder to animate. Full independence costs one extra MECH bone and
one extra CTRL bone versus the fused design — cheap, given what it buys.

**PIP flexion/extension**: actuated on all four — largest ROM, dominant
contributor to curl force per the ROM data below.

**DIP**: not independently actuated on any finger — driven entirely by
PIP, per the coupling mechanism below.

---

## PIP-DIP Coupling Mechanism

Every source found reaches for a flat **2:3 (0.667) DIP:PIP ratio** as
the standard tendon-driven/biomimetic approximation — published
four-bar-linkage hands (ByteDexter V2, DexLink Hand, several compliant
four-bar-linkage patents) use crossed four-bar linkages tuned to roughly
that transmission ratio. Checked against the actual measured ROM figures
this file already needed to source (below), that flat ratio does not
hold up:

| Finger | PIP flex | DIP flex | **Measured flex ratio** | PIP ext | DIP ext | **Measured ext ratio** |
|---|---|---|---|---|---|---|
| Index | 97.2° | 81.6° | **0.84** | 13.7° | 6.0° | 0.44 |
| Middle | 96.2° | ~83° (interp.) | **0.86** | 15.6° | 8.4° | 0.54 |
| Ring | 96.0° | ~84° (interp.) | **0.88** | 16.2° | 8.1° | 0.50 |
| Little | 91.8° | 84.6° | **0.92** | 13.2° | 6.8° | 0.52 |

The generic 2:3 figure is roughly 25 points low against every finger's
actual measured full-flexion ratio in this dataset. Read: that number is
a mechanism-design target for a general-purpose four-bar linkage, not a
fit to this specific 390-hand dataset — a different kind of number, and
reusing it without checking would be exactly the "looks authoritative but
isn't verified against a real source" trap docs/ENGINEERING_RULES.md §17 warns about.

**Decision**: use each finger's own measured flexion-ratio (0.84 / 0.86 /
0.88 / 0.92) as that finger's Copy Rotation `Influence`, instead of one
global constant across all four fingers. Calibrated to the flexion end
deliberately — flexion is the dominant, large-amplitude motion in every
grasp gesture this mascot performs; extension is a small-amplitude
return-to-rest motion where visual error is far less noticeable. The
extension-side ratios (0.44–0.54) genuinely don't match the flexion-side
ratios (0.84–0.92) for any finger — a real, measured nonlinearity in the
coupling (consistent with the biomechanics literature describing a
four-bar-linkage relationship, not a linear one). A single linear
`Influence` scalar cannot satisfy both ends at once; this spec accepts
under-extension at the DIP as the known cost rather than hiding it.
Flagged as this file's weakest approximation, same transparency standard
as every other spec's flagged weak point.

**Mechanism stays a Copy Rotation constraint (not a driver)**, for
consistency with every other joint in this project — every constraint
stack built so far (elbow, wrist, palm, thumb) uses Copy Rotation +
Limit Rotation exclusively. What differs from a naive flat-ratio copy is
the number itself and its derivation, not the constraint type.

**`MECH_{Finger}_DIP` reads from `MECH_{Finger}_PIP`, not from a shared
CTRL input**: a shared-CTRL scheme (one `CTRL_{Finger}_Curl` driving both
PIP and DIP as siblings) was considered and rejected — `MECH_{Finger}_
PIP` carries a hard Limit Rotation clamp; reading DIP from the raw CTRL
value instead of PIP's own clamped output would let DIP keep scaling past
the point where PIP has already hit its stop, breaking the ratio exactly
at the ROM limits — the poses most likely to actually be seen. Reading
DIP from PIP's own post-limit output keeps the ratio correct everywhere,
including both ends of travel.

**No `CTRL_DIP` bone on any finger** — nothing for an animator to
independently control, matching biological reality.

---

## JOINT 1: MCP Flexion/Extension (all four fingers)

### Classification

**Type**: Condyloid (flexion/extension axis only — the joint's second
axis is Joint 2 below, built on three of the four fingers)
**DOF**: 1R per finger
**Axis**: Transverse, through the metacarpal head

### Biological ROM

Source: 390-hand, 4-center goniometer study (Indian Society for Surgery
of the Hand, 2024).

| Finger | Flexion | Extension |
|---|---|---|
| Index | 86.0° | 25.7° |
| Middle | 86.6° | 24.7° |
| Ring | 84.2° | 22.3° |
| Little | 85.0° | 26.3° |

### Mechanical Implementation

**MECH layer** (per finger, `{Finger}` substituted):
- Bone name: `MECH_{Finger}_MCP_Flex`
- Local axis: **X** (hypothesis — verify empirically per §8)
- Locked axes: Y, Z
- Parent: `DEF_Palm_{Finger}`
- Child: `MECH_{Finger}_MCP_Abd` (Index, Ring, Little) or directly
  `DEF_Middle_MCP` (Middle, which has no abduction bone)

**Operational limits** (rounded):

| Finger | Min (ext) | Max (flex) |
|---|---|---|
| Index | –26° | +86° |
| Middle | –25° | +87° |
| Ring | –22° | +84° |
| Little | –26° | +85° |

**Rigify rig type**: None — hand-wired.

### Constraint Stack

```
MECH_{Finger}_MCP_Flex:

Copy Rotation (from CTRL_{Finger}_MCP_Flex)
  Target: CTRL_{Finger}_MCP_Flex
  Space: Local
  Influence: 1.0
Limit Rotation
  Min X / Max X: per finger, table above
  Y/Z: locked (min = max = 0°)
  Space: Local
```

---

## JOINT 2: MCP Abduction/Adduction (Index, Ring, and Little — each independent)

### Classification

**Type**: Condyloid, second axis
**DOF**: 1R per finger — built independently on Index, Ring, and Little;
Middle has no MECH/CTRL bone for this axis at all (not zero-limited —
actually absent).
**Axis**: Roughly perpendicular to MCP flexion, in the plane of the palm

### Operational ROM

No study surfaced a per-finger abduction/adduction split for any of the
three fingers. Every source found gives only a whole-hand figure:
Physiopedia's goniometry reference and the Norkin/White *Measurement of
Joint Motion* text report a full arc of ≈35° with the MCP extended,
collapsing to ≈5° with the MCP flexed, and a separate single-direction
figure of "0°–25°." No source distinguishes Index from Ring from Little.

**Engineering estimate, applied identically to all three** (deliberately
— inventing a finger-specific asymmetry with no citation behind it would
be worse than an honest, uniform estimate): splitting the ~35° extended
arc symmetrically gives ±17.5°, rounded to **±18°** for Index, Ring, and
Little alike. Flagged exactly like `palm.md`'s MC4/MC5 estimates —
revisit once real grasp poses can be tested.

### Mechanical Implementation

**MECH layer** (per finger):
- Bone name: `MECH_{Finger}_MCP_Abd` (Index, Ring, Little only)
- Local axis: **Z** (hypothesis — verify empirically)
- Locked axes: X, Y
- Parent: `MECH_{Finger}_MCP_Flex`
- Child: `DEF_{Finger}_MCP`

**Operational limits**: Z min –18°, Z max +18°; X/Y locked. Same figure
for all three — see estimate note above.

**Rigify rig type**: None — hand-wired.

### Constraint Stack

```
MECH_{Finger}_MCP_Abd:

Copy Rotation (from CTRL_{Finger}_MCP_Abd)
  Target: CTRL_{Finger}_MCP_Abd
  Space: Local
  Influence: 1.0
Limit Rotation
  Min Z: –18°, Max Z: +18°
  X/Y: locked (min = max = 0°)
  Space: Local
```

`DEF_{Finger}_MCP` (Index, Ring, Little) takes a single Pose-space Copy
Rotation from `MECH_{Finger}_MCP_Abd` — the bottom bone of each finger's
own serial two-bone chain already carries both axes' cumulative rotation
in Pose space (same composition pattern validated on `DEF_Thumb_CMC` in
`thumb.md`). `DEF_Middle_MCP` instead takes a single **Local**-space Copy
Rotation directly from `MECH_Middle_MCP_Flex`, since Middle has no second
bone to compose.

Because each finger now has its own independent actuator, there is no
shared-source composition anywhere in this file — every `DEF_{Finger}_MCP`
reads from exactly one MECH chain, not two.

---

## JOINT 3: PIP Flexion/Extension (all four fingers)

### Classification

**Type**: Hinge
**DOF**: 1R
**Axis**: Transverse, through the proximal phalanx's distal end

### Biological ROM

| Finger | Flexion | Extension |
|---|---|---|
| Index | 97.2° | 13.7° |
| Middle | 96.2° | 15.6° |
| Ring | 96.0° | 16.2° |
| Little | 91.8° | 13.2° |

### Mechanical Implementation

**MECH layer**:
- Bone name: `MECH_{Finger}_PIP`
- Local axis: **X** (hypothesis — verify empirically)
- Locked axes: Y, Z
- Parent: `DEF_{Finger}_MCP`
- Child: `MECH_{Finger}_DIP`

**Operational limits** (rounded):

| Finger | Min (ext) | Max (flex) |
|---|---|---|
| Index | –14° | +97° |
| Middle | –16° | +96° |
| Ring | –16° | +96° |
| Little | –13° | +92° |

**Rigify rig type**: None — hand-wired.

### Constraint Stack

```
MECH_{Finger}_PIP:

Copy Rotation (from CTRL_{Finger}_PIP)
  Target: CTRL_{Finger}_PIP
  Space: Local
  Influence: 1.0
Limit Rotation
  Min X / Max X: per finger, table above
  Y/Z: locked (min = max = 0°)
  Space: Local
```

`DEF_{Finger}_MCP` → `DEF_{Finger}_PIP` composition: standard single-axis
Local-space copy from `MECH_{Finger}_PIP` (the `elbow_forearm.md`
pattern).

---

## JOINT 4: DIP Flexion/Extension (all four fingers — driven, not actuated)

### Classification

**Type**: Hinge, passively driven
**DOF**: 0 actuated (no CTRL bone); motion is entirely a function of
Joint 3 (PIP) via the coupling mechanism above
**Axis**: Transverse, through the middle phalanx's distal end

### Biological ROM (for reference — not used directly as a limit)

| Finger | Flexion | Extension |
|---|---|---|
| Index | 81.6° (cited directly) | 6.0° |
| Middle | ~83° (interpolated — study reports DIP flexion increasing index→little but gives exact figures only at the two endpoints) | 8.4° |
| Ring | ~84° (interpolated, same caveat) | 8.1° |
| Little | 84.6° (cited directly) | 6.8° |

### Mechanical Implementation

**MECH layer**:
- Bone name: `MECH_{Finger}_DIP`
- Local axis: **X** (hypothesis — should match Joint 3's axis exactly,
  since this is a driven copy, not an independent axis)
- Locked axes: Y, Z
- Parent: `MECH_{Finger}_PIP`
- Child: `DEF_{Finger}_Tip`

**No CTRL bone. No Limit Rotation constraint** — the Copy Rotation's own
scaled Influence bounds the output as a side effect of tracking PIP's
own already-clamped rotation; a hard limit here would be redundant.

### Constraint Stack — per-finger calibrated Influence

```
MECH_Index_DIP:
  Copy Rotation (from MECH_Index_PIP), Space: Local, Influence: 0.84

MECH_Middle_DIP:
  Copy Rotation (from MECH_Middle_PIP), Space: Local, Influence: 0.86

MECH_Ring_DIP:
  Copy Rotation (from MECH_Ring_PIP), Space: Local, Influence: 0.88

MECH_Little_DIP:
  Copy Rotation (from MECH_Little_PIP), Space: Local, Influence: 0.92
```

Only one constraint per bone — no Limit Rotation step, same structural
exception as every other driven joint in this project. Don't add one by
habit.

---

## Hierarchy

**Index, Ring, Little** (identical shape — independent serial chain):
```
DEF_Palm_{Finger}
└── MECH_{Finger}_MCP_Flex (1R, X)
    └── MECH_{Finger}_MCP_Abd (1R, Z)
        └── DEF_{Finger}_MCP ← Pose-space copy of MECH_{Finger}_MCP_Abd
            └── MECH_{Finger}_PIP (1R, X, actuated)
                └── MECH_{Finger}_DIP (1R, X, driven — ratio per finger)
                    └── DEF_{Finger}_Tip
```
Ratios: Index 0.84, Ring 0.88, Little 0.92.

**Middle** (no abduction actuator at all):
```
DEF_Palm_Middle
└── MECH_Middle_MCP_Flex (1R, X)
    └── DEF_Middle_MCP ← Local-space copy of MECH_Middle_MCP_Flex
        └── MECH_Middle_PIP (1R, X, actuated)
            └── MECH_Middle_DIP (1R, X, driven — 0.86 × PIP)
                └── DEF_Middle_Tip
```

No cross-finger dependencies anywhere in this file — every finger's chain
is fully self-contained.

---

## Metarig / Bone Setup

No Rigify rig type assignment anywhere in this spec.

1. Create `MECH_{Finger}_MCP_Flex` ×4, each a child of its `DEF_Palm_
   {Finger}`
2. Create `MECH_{Finger}_MCP_Abd` ×3 (Index, Ring, Little only), each a
   child of its own `MECH_{Finger}_MCP_Flex` — three separate bones, no
   sharing
3. Create `MECH_{Finger}_PIP` ×4 and `MECH_{Finger}_DIP` ×4, serial per
   finger as shown in the hierarchy
4. Create `DEF_{Finger}_MCP`, `DEF_{Finger}_PIP`, `DEF_{Finger}_Tip` per
   finger
5. Wire Index/Ring/Little's `DEF_{Finger}_MCP` as a single Pose-space Copy
   Rotation from `MECH_{Finger}_MCP_Abd`; wire Middle's `DEF_Middle_MCP`
   as a single Local-space Copy Rotation from `MECH_Middle_MCP_Flex`
6. Wire all `MECH_{Finger}_PIP` stacks (Copy Rotation + Limit Rotation)
7. Wire all `MECH_{Finger}_DIP` single-constraint stacks with the
   per-finger Influence values from Joint 4 — **not** a flat 0.667
8. Create CTRL bones: `CTRL_{Finger}_MCP_Flex` ×4, `CTRL_{Finger}_MCP_
   Abd` ×3 (Index, Ring, Little), `CTRL_{Finger}_PIP` ×4. No `CTRL_
   {Finger}_DIP` on any finger.
9. **Before wiring any constraint**: verify every MECH bone's actual
   local axes via MCP inspection — every X/Z hypothesis above is a
   starting point, not a confirmed fact, same requirement as every joint
   built in this project so far

---

## Animation Workflow

### Test Animation 1: Four-Finger Curl (validates per-finger PIP-DIP ratio)

Frame 0: rest. Frame 15: all four MCP_Flex + PIP at max flexion. Frame
30: hold. Frame 45: back to rest.

**Expected**: at frame 15, each finger's DIP sits at its own calibrated
ratio of its own PIP angle — Index ~0.84×, Middle ~0.86×, Ring ~0.88×,
Little ~0.92× — not a uniform ratio across fingers. Measure exactly, not
by eye.

### Test Animation 2: Independent Spread (validates that Index/Ring/Little are NOT coupled to each other)

Frame 0: rest, all abduction 0°. Frame 15: `CTRL_Little_MCP_Abd` to
+18°, `CTRL_Index_MCP_Abd` and `CTRL_Ring_MCP_Abd` held at 0°. Frame 30:
reverse — `CTRL_Ring_MCP_Abd` to +18°, `CTRL_Little_MCP_Abd` back to 0°.

**Expected**:
- At frame 15, only Little visibly spreads — Index and Ring stay
  completely still. This is the specific case a shared Ring/Little
  actuator (considered and rejected — see Notes) could **not** produce;
  confirming it here is the direct proof the independent-actuator
  decision was worth its extra bone.
- At frame 30, only Ring spreads, Little returns to neutral independently
- Middle shows zero spread at every frame — no abduction bone exists for
  it at all

---

## Validation Checklist

Built and validated in `RobotArmature` following §16's INSPECT → PLAN →
IMPLEMENT → VALIDATE → REPORT workflow, same cycle as elbow/wrist/palm/
thumb.

**Structural**:
- [x] All 15 MECH bones exist (4 Flex + 3 Abd [Index/Ring/Little] + 4
      PIP + 4 DIP), no rigify_type assigned. 12 DEF bones (MCP/PIP/Tip
      ×4) and 11 CTRL bones (Flex ×4 + Abd ×3 + PIP ×4) also confirmed —
      38 new bones total.
- [x] Confirmed Middle has no abduction bone at all — not just locked
      (`MECH_Middle_PIP` parents directly from `MECH_Middle_MCP_Flex`,
      no Abd bone in between; checked live in the armature, not assumed)
- [x] Confirmed no shared bone or cross-finger constraint target exists
      anywhere — every finger's chain parents and constraint targets
      traced individually, all self-contained
- [x] Local axes verified empirically before any constraint was wired:
      local X = world +Y (transverse, matches the flexion hypothesis),
      local Z = world −X (depth, matches the abduction hypothesis) — same
      on Index Flex, Index Abd, Middle Flex, Ring Abd, and Little Abd

**Mechanical**:
- [x] Each finger's MCP Flex and PIP clamp correctly at both ends —
      tested over-max, under-min, and mid-range on Index (Flex: 150°→86°,
      −90°→−26°, 40°→40° pass-through) and Little PIP (120°→92°,
      −30°→−13°)
- [x] `MECH_{Finger}_MCP_Abd` clamps at ±18° independently — tested on
      Ring with combined off-axis input (25,25,25) → X/Y held at 0°, Z
      clamped to 18°
- [x] Cross-axis leak test: `MECH_Index_MCP_Flex` given (40°,30°,20°)
      input → Y/Z stayed exactly 0°, only X passed through
- [x] DIP tracks PIP at each finger's own ratio — measured directly
      (PIP=60° gave DIP: Index 50.4°, Middle 51.6°, Ring 52.8°, Little
      55.2° — exact match to 0.84/0.86/0.88/0.92)
- [x] `CTRL_Ring_MCP_Abd` and `CTRL_Little_MCP_Abd` tested independently
      in both directions — zero cross-talk in either direction, confirmed
      live (this was the specific behavior the shared-actuator design
      could not produce; independence is real, not assumed)
- [x] Constraint order confirmed live: Copy Rotation before Limit
      Rotation on every actuated bone
- [x] `DEF_Middle_MCP`'s distinct Local-space, one-fewer-bone composition
      path checked as its own case, not assumed identical to the other
      three: driving `CTRL_Middle_MCP_Flex` to 50° produced an exact
      (50°,0°,0°) match between `MECH_Middle_MCP_Flex` and
      `DEF_Middle_MCP`. Contrasted directly against Index's two-source
      Pose-space case (Flex 50° + Abd 10° combined): a naive Euler
      component readout looked wrong at first (49.57°, −7.64°, 6.47°) —
      this is expected Euler-decomposition cross-contamination from
      composing two sequential rotations, not a bug (documented in
      `thumb.md` and rediscovered here). Re-verified with the same
      matrix-composition method `thumb.md` validated: `DEF_Index_MCP`'s
      actual rotation matrix matched an independently computed
      `Rotation(X,50°) @ Rotation(Z,10°)` to within 1.5e-7 at two
      different test points. Middle's simpler path needed no such
      correction — its single-source Local copy matches component-wise
      by construction.

**Animation**:
- [x] Four-Finger Curl test (frames 0/15/30/45) plays correctly — each
      DIP's ratio to its own PIP measured exactly, not eyeballed
- [x] Independent Spread test (frames 50/65/80): Little moves alone at
      frame 65, Ring moves alone at frame 80, Middle stays at (0,0,0)
      throughout

**Export**:
- [x] Baked to DEF-only keyframes (208 fcurves across the full rig's DEF
      layer, zero stray CTRL/MECH curves — confirmed by inspecting the
      baked action directly)
- [x] Export/reimport per §5. Exported `blender/robot_test_fingers.glb`.
      Two **new gotchas** found and logged below (Blender's reimport
      misattributed animation data due to a bone-name collision with the
      live rig already in the file, and a baked action's slot can retain
      the wrong object identifier when baking on a duplicated armature).
      Neither is a rig defect — confirmed by parsing the exported `.glb`'s
      raw JSON/accessor data directly, bypassing Blender's importer:
      every sampled frame matched the pre-export values exactly (e.g.
      `DEF_Index_PIP` at frame 15: expected 97.0°, raw glb 97.0°;
      `DEF_Little_MCP` at frame 65: expected 18° Z, raw glb 18.0° Z).

**Status after validation**: `VALIDATED`

### Validation Report

```
VALIDATION REPORT: Fingers (MCP Flex + MCP Abd[Index/Ring/Little] + PIP + DIP, all four)

Status: VALIDATED

Layer 1 (Structural): PASS -- 38 new bones (15 MECH, 12 DEF, 11 CTRL)
                       confirmed correctly named, parented, and collected;
                       Middle's one-fewer-bone hierarchy confirmed live,
                       not assumed; local axes verified empirically before
                       any constraint was wired (X=transverse/flexion,
                       Z=depth/abduction), matching the spec's hypothesis
                       on every sampled bone
Layer 2 (Mechanical):  PASS -- all ROM clamps verified at both ends with
                        over/under/mid-range inputs; cross-axis leak
                        confirmed zero on locked axes; per-finger DIP:PIP
                        ratios measured exactly (0.84/0.86/0.88/0.92);
                        Ring/Little abduction independence confirmed live
                        in both directions (the specific behavior the
                        rejected shared-actuator design could not
                        produce); Middle's distinct composition path
                        checked as its own case and matched exactly;
                        Index's two-source Pose-space composition
                        verified via matrix comparison against an
                        independently computed rotation product (<1.5e-7
                        diff), after an initial naive Euler-component
                        check gave a false read (expected -- documented
                        in thumb.md, rediscovered here, not a defect)
Layer 3 (Animation):   PASS -- Four-Finger Curl (frames 0/15/30/45) and
                        Independent Spread (frames 50/65/80) test
                        animations both play exactly as specified,
                        measured per frame, not eyeballed
Layer 4 (Export):      PASS -- DEF-only baked rig (208 fcurves, zero
                        stray CTRL/MECH curves) exported to
                        blender/robot_test_fingers.glb. Blender's own
                        reimport initially appeared to show every sampled
                        frame flat at zero; traced to a Blender-side
                        reimport artifact (see gotchas below), not an
                        export defect -- confirmed by parsing the .glb's
                        raw JSON/accessor data directly: every sampled
                        frame matched the pre-export baked values exactly
                        (e.g. DEF_Index_PIP frame 15: 97.0 deg vs 97.0 deg;
                        DEF_Little_MCP frame 65: 18.0 deg Z vs 18.0 deg Z)

New gotchas logged for future joints:
1. Reimporting a glTF into the SAME .blend file that already contains an
   armature with identical bone names can cause Blender's importer to
   misattribute the new animation's data to a stale, unrelated
   pre-existing action of the same default name ("Animation") -- the
   reimported action LOOKED like it held the whole rig's historical data
   at every frame flat/wrong, when the actual exported file was correct
   the whole time. When validating an export/reimport round-trip in a
   .blend that already has bones of the same name, don't trust Blender's
   reimported action at face value -- either reimport into a clean/empty
   .blend, or parse the .glb's raw JSON + binary accessor data directly
   (as done here) to verify ground truth.
2. Baking a duplicated armature object's pose (`bpy.ops.nla.bake` with
   `use_current_action=False`) can produce a new Action whose slot
   `identifier`/`name_display` still references the ORIGINAL object's
   name, not the duplicate's -- a latent mismatch between the action's
   declared target and the object it's actually assigned to. Didn't
   turn out to be the root cause of gotcha #1 above, but is a real,
   independently confirmed inconsistency in Blender 5.x's layered-Action
   API worth checking (`obj.animation_data.action_slot.identifier`)
   whenever baking on a duplicated object before export.
3. `bpy.types.Bone.select` does not exist in this Blender version --
   bone selection for operators like `nla.bake` must be set via
   `pose_bone.select`, not `armature.data.bones[name].select`.

Validated by: Claude Code
Timestamp: 2026-09-01T08:36:30Z
```

---

## Notes & Future Work

- **Rejected alternative, kept on record**: a shared Ring+Little
  abduction actuator (`MECH_RingLittle_MCP_Abd`, one bone driving both
  `DEF_Ring_MCP` and `DEF_Little_MCP` via axis-restricted Copy Rotation
  constraints) was drafted and specifically rejected. It was modeled on
  a real published design (SABD Hand, arXiv:2509.13074) and would have
  saved one MECH bone and one CTRL bone, but produces a rigid 1:1 lock
  between the two fingers — no pose exists where one moves and the other
  doesn't. Rejected because this mascot's gesture range needs isolated
  ring- or pinky-only spread poses that a shared actuator makes
  physically unreachable. If actuator count ever becomes a real
  constraint (e.g. a hardware build with limited channels), this is the
  documented fallback to revisit — the mechanism is fully specified in
  this project's history, not lost.
- The DIP:PIP ratios (0.84/0.86/0.88/0.92) are calibrated to each
  finger's own measured full-flexion ROM, not to the generic 2:3 robotics
  convention — deliberate divergence, reasoning kept above rather than
  assumed obvious.
- Middle and Ring's exact DIP flexion figures are interpolated, not
  directly cited by the source study — carried through into this file's
  own ratio calculations for those two fingers.
- The ±18° abduction ROM estimate is applied uniformly to Index, Ring,
  and Little for lack of any per-finger citation — flagged as this file's
  weakest ROM number alongside the DIP extension-ratio mismatch.
- Total actuated DOF is 11, not 10 — one more than either prior draft,
  since giving Ring its own independent actuator (rather than locking it
  or sharing it with Little) genuinely adds one actuator, not a relabel.
- **Correction (2026-09-01), applied project-wide**: every MECH/CTRL bone
  in `RobotArmature` — 46 bones total across elbow, forearm twist, wrist,
  palm, thumb, and fingers — had `use_deform = True` (Blender's default),
  incorrectly making them mesh-skinning-eligible alongside DEF. Per
  docs/ENGINEERING_RULES.md §5, only DEF bones should ever reach the runtime. Corrected to
  `use_deform = False` on all 46; the 23 DEF bones project-wide were left
  untouched (`use_deform = True`, correct). Re-validated per joint: a
  combined test animation (rest + representative max-ROM pose per joint)
  was baked to DEF-only, exported, and its raw `.glb` accessor data
  parsed directly — every DEF bone's exported rotation matched its
  pre-correction value exactly (floating-point-level diffs only, ~1e-6).
  Note this glTF exporter's node inclusion does **not** depend on
  `use_deform` — MECH/CTRL bones still appear as inert nodes in every
  export regardless of this flag; the fix corrects skinning eligibility
  only, not export bloat. Full per-joint breakdown and re-validation
  detail is in the conversation history, not duplicated across every
  spec file — each affected file (`elbow_forearm.md` intended,
  `wrist.md`, `palm.md`, `thumb.md`) carries its own short correction
  note.

---

## References

- The Normal Active Range of Motion of the Index, Middle, Ring, and
  Little Fingers (Thieme-published, 2024, ~390 hands, 4 centers, Indian
  Society for Surgery of the Hand) — MCP and PIP flexion/extension for
  all four fingers, and the two directly-cited DIP flexion endpoints
  (Index 81.6°, Little 84.6°). Verified via independent search, same
  figures returned on a second, differently-worded query.
- SABD Hand (arXiv:2509.13074) — published 2026 robotic-hand design that
  fuses ring/little finger abduction into one shared joint; verified by
  fetching and quoting its abstract directly. Cited here as the
  considered-and-rejected alternative this spec explicitly did not adopt,
  not as a pattern this file follows.
- LEAP Hand (arXiv:2309.06440) — fully-actuated research hand giving
  every finger independent 2-DOF MCP; consistent with this file's final
  choice to make Index/Ring/Little fully independent, though for a
  different reason (gesture range, not manipulation dexterity).
- Physiopedia (Goniometry: Finger MCP Abduction) and Norkin/White,
  *Measurement of Joint Motion: A Guide to Goniometry* — source of the
  whole-hand (not per-finger) abduction/adduction figures used to derive
  this file's ±18° engineering estimate.
- Four-bar-linkage prosthetic/robotic hand literature (ByteDexter V2,
  DexLink Hand, compliant four-bar-linkage patents) — source of the
  generic 2:3 DIP:PIP transmission-ratio convention this file explicitly
  checked against real ROM data and chose not to use.
- `specifications/hand/thumb.md`, `specifications/hand/palm.md` — source
  of every reused pattern in this file (no invented rigify_type, Copy
  Rotation before Limit Rotation, Pose-space composition for a serial
  two-bone combined DEF bone).
