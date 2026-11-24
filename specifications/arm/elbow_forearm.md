# ELBOW & FOREARM — Joint Specification

## Segment Overview

The elbow-forearm unit consists of:
- **Elbow joint**: humerus ↔ radius/ulna (primary articulation)
- **Forearm**: radius + ulna as a rigid composite
- **Pronation/supination axis**: radius rotating around the ulna (secondary)

This spec covers the elbow's primary 1R flexion/extension and the forearm's
1R pronation/supination as two independent 1-DOF joints in series.

---

## JOINT 1: Elbow (Humerus ↔ Radius/Ulna)

### Classification

**Type**: Hinge (modified ginglymus)
**DOF**: 1R (flexion/extension only)
**Biological ROM**: 150° flexion, 0° extension (arm can't hyperextend)
**Functional ROM**: 0° (full extension) to 130° (full flexion)
**Load**: Moderate — carries arm weight + object weight at full extension
**Axis**: Transverse (left-right), running through the trochlear notch

### Mechanical Implementation

**MECH layer**:
- Bone name: `MECH_Elbow`
- Local axis: X (rotation on X = flexion/extension)
- Locked axes: Y, Z
- Rotation mode: XYZ
- Inherit rotation: Yes
- Parent: `MECH_UpperArm` (or `DEF_UpperArm` if no upper-arm MECH layer yet)
- Child: `MECH_Forearm_Twist`

**Limits**:
- X min: 0° (full extension, arm straight)
- X max: 130° (full flexion, forearm near bicep)
- Y min/max: 0° (locked)
- Z min/max: 0° (locked)

**Rigify rig type**: None — plain bone, hand-wired via the Constraint
Stack below (no `rigify_type` assigned). Confirmed against the live
Blender 5.2 LTS Rigify install: no dedicated rig type exists for a
simple single-axis hinge; `super_limb`/`super_finger`/`super_palm` exist
for multi-bone chains, not this.

### Axis Verification

The elbow rotates around a transverse axis (left-right line through the
trochlear notch). In Blender viewport:
- Rotating the bone on its local X axis should flex/extend the arm
- No Y or Z rotation should occur (those are locked)

If the bone's local X does not align with left-right, re-roll the metarig
bone:
- Armature → Recalculate Roll → Y Axis (for a humanoid in standard pose)

### ROM Justification

**Why 0° to 130°, not 150°?**
The maximum biological flexion is 150°, but 130° is the functional ROM for
daily tasks (picked from biomechanics literature). At 130°, the forearm can
grip objects, perform fine manipulation, and still maintain clearance to the
bicep. The extra 20° is passive range, rarely used in real grasping. We
implement the functional 0–130° range; the bone's limit constraints enforce
this.

### Load-Bearing Role

During lifting or reaching:
- The elbow must support the arm's weight (≈3–5 kg for the forearm + hand)
- At full extension, moment arm is maximum → highest torque demand
- Peak torque during lifting: ~75–100 Nm (for a 70 kg body lifting 10 kg at
  arm's length)

**Robotic implication**: The elbow actuator must deliver ≥75 Nm continuous,
with ≥150 Nm peak capacity.

### Constraint Stack

**Order matters.** Copy Rotation must come first (bring the value in
from CTRL); Limit Rotation second (clamp what came in). Reversing this
makes the limit a no-op, because Copy Rotation in REPLACE mode overwrites
whatever Limit Rotation just did.

MECH_Elbow:

Copy Rotation (from CTRL_Elbow)
Target: CTRL_Elbow
Space: Local
Influence: 1.0
Limit Rotation
Min X: 0°, Max X: 130°
Y/Z: locked (min = max = 0°)
Space: Local

---

## JOINT 2: Forearm Pronation/Supination

### Classification

**Type**: Hinge (1R twist)
**DOF**: 1R (pronation/supination around forearm's long axis)
**Biological ROM**: 80° pronation, 80° supination (total range: 160°)
**Functional ROM**: ±50° (enough for most daily tasks: doorknobs, typing, etc.)
**Load**: Low — carries forearm's own weight only
**Axis**: Longitudinal (along the length of the forearm)

### Mechanical Implementation

**MECH layer**:
- Bone name: `MECH_Forearm_Twist`
- Local axis: Y (rotation on Y = pronation/supination around forearm length)
- Locked axes: X, Z
- Rotation mode: XYZ
- Inherit rotation: Yes
- Parent: `MECH_Elbow` (child of the elbow hinge)
- Child: `MECH_Wrist_Flex` (from wrist.md)

**Limits**:
- Y min: –50° (pronation, palm down)
- Y max: +50° (supination, palm up)
- X min/max: 0° (locked)
- Z min/max: 0° (locked)

**Rigify rig type**: None — hand-wired, no rigify_type assigned.

### Axis Verification

The forearm rotates around its own length axis (from elbow to wrist). In
Blender:
- Rotating the `MECH_Forearm_Twist` bone on its local Y axis should
  twist the forearm (palm facing down → up or vice versa)
- No X or Z rotation

If the local Y does not align with the forearm's length, re-roll:
- Armature → Recalculate Roll → Z Axis (for a standard humanoid pose)

### ROM Justification

**Why ±50°, not ±80°?**
Biological supination (palm-up) ROM is 80° from neutral, but the functional
range for daily manipulation is ±50°. This covers:
- Doorknob turning: ±30°
- Pouring from a cup: +30° to +60° supination
- Typing: near neutral (±10°)
- Writing: slight pronation (–10° to –20°)

The additional ±30° is passive/rarely-used range.

### Load-Bearing Role

During a grasping task, pronation/supination is a low-load motion — the
forearm isn't lifting against gravity, only rotating. Torque demand: ~10–20
Nm peak (for 10 kg object rotated in-hand).

### Constraint Stack

MECH_Forearm_Twist:

Copy Rotation (from CTRL_Forearm_Twist)
Target: CTRL_Forearm_Twist
Space: Local
Influence: 1.0
Limit Rotation
Min Y: –50°, Max Y: +50°
X/Z: locked (min = max = 0°)
Space: Local

---

## Serial Hierarchy

The two joints form a serial chain:

DEF_UpperArm (parent, rigid link)
↓
MECH_Elbow (1R hinge, X-axis: flexion/extension)
↓
MECH_Forearm_Twist (1R twist, Y-axis: pronation/supination)
↓
DEF_Wrist_base / MECH_Wrist_Flex (from wrist.md)


**Meaning**:
- Flexing the elbow first rotates on the transverse axis
- Then pronating the forearm twists on the forearm's long axis
- No gimbal locking (two independent 1-DOF hinges in series)

---

## Animation Workflow

### Test Animation: Pick-Up Motion

Frame 0: Rest pose

Elbow: 0° (full extension)
Forearm: 0° (neutral, palm forward)

Frame 10: Reaching for object

Elbow: 90° (mid-flexion)
Forearm: 0° (palm stays forward)

Frame 20: Back to rest

Elbow: 0° (full extension)
Forearm: 0° (neutral)

Playback speed: 30 fps, so total duration is ~0.67 seconds.

### Expected Result

- Motion should be smooth (no jitter)
- At frame 10, the hand (DEF_Hand/wrist chain) should be at mid-reach
- Forearm twist shouldn't interfere with elbow flexion

---

## Validation Checklist

**Structural**:
- [x] `MECH_Elbow` exists, named correctly, no rigify_type assigned
- [x] `MECH_Forearm_Twist` exists, named correctly, no rigify_type assigned
- [x] Both are in the MECH collection
- [x] Both have rotation mode XYZ
- [x] Axes are locked correctly (one free, two at 0°)

**Mechanical**:
- [x] Rotate CTRL_Elbow 0° to 130°, MECH and DEF follow, clamps at both ends
- [x] Rotate CTRL_Forearm_Twist ±50°, MECH and DEF follow, clamps at both ends
- [x] No unwanted axes rotate
- [x] Constraint order confirmed live: Copy Rotation before Limit Rotation
      on both bones

**Animation**:
- [x] Pick-up animation plays smoothly in Blender viewport
- [x] No jitter or constraint conflicts
- [x] All three layers (CTRL, MECH, DEF) stay in sync

**Export**:
- [x] Bake animation to DEF bones only
- [x] Export to .glb
- [x] Reimport into new .blend
- [x] Animation plays identically in reimported file
- [ ] Mesh deforms correctly at all keyframes — deferred, no mesh
      skinned yet

**Status after validation**: `VALIDATED`

---

## Change Log

- **Corrected before build**: removed all references to a `basic.hinge`
  Rigify rig type (confirmed not to exist in the installed Blender 5.2
  LTS Rigify build). Both bones built as plain, untyped, hand-wired
  bones instead.
- **Corrected before build**: constraint stack order swapped from
  [Limit Rotation, Copy Rotation] to [Copy Rotation, Limit Rotation].
  The original order made Limit Rotation a no-op, since Copy Rotation
  (mode REPLACE) fully overwrites the bone's rotation on every
  evaluation, erasing whatever the limit had just clamped. Confirmed
  empirically against the live rig.
- **Corrected during use_deform audit**: `use_deform` set to `False` on
  all four of this file's bones — `MECH_Elbow`, `MECH_Forearm_Twist`,
  `CTRL_Elbow`, and `CTRL_Forearm_Twist` (was `True`, Blender's default,
  on all four) — MECH and CTRL bones should never influence mesh
  deformation per §5; only DEF bones should. (An earlier draft of this
  changelog entry, reconstructed from chat history, only credited the
  two MECH bones; verified against this session's actual audit log and
  corrected here to include the two CTRL bones as well.)

---

## Notes & Future Work

- **Upper arm**: Not specified in this file; assumed to be a rigid link or a
  3-DOF shoulder (from a future shoulder spec)
- **Wrist**: Specified separately in wrist.md, a 2-DOF serial joint
  (flexion + deviation), parented onto `MECH_Forearm_Twist`
- **Pronation/supination split**: Currently a single Y-axis twist. If a future
  spec requires splitting this into radius/ulna articulation, update the
  hierarchy then.
- **Elbows cannot truly hyperextend** (stay at 0° minimum); this is
  enforced by limits. No special case needed.
- **File corruption incident**: this file was found to contain a
  truncated copy of wrist.md's content instead of its own, discovered
  during the project-wide use_deform audit. The live rig was unaffected
  — this document was reconstructed from the original chat history where
  it was authored and corrected, then re-verified against the live
  rig's actual constraints (ROM, axes, hierarchy, rotation mode,
  rigify_type, constraint order) before being treated as restored. One
  discrepancy was found and corrected during that re-verification: see
  the Change Log's use_deform entry above.
- **Corrected in `scripts/rig/arm_ik_solver.py`/`_L.py` (solver, not this
  spec's numbers)**: the twist-zero reference axis was derived from
  `CTRL_Forearm_Twist`'s bone roll relative to `CTRL_Elbow`'s, which is
  degenerate on this rig — both bones share an identical rest matrix (a
  straight-arm rest pose with no distinct roll ever authored between them),
  so the derived reference collapsed onto the elbow's own flex axis and
  produced nonsense raw angles that always clamped to ±50°. Fixed by reusing
  the solver's already-computed `basis_v` (perpendicular to the flex axis and
  the rest forearm direction) as the reference instead — non-degenerate and
  matching this spec's "0° = neutral, palm forward" convention. `IK_Pole.arm.R`
  / `.L` were also recalibrated to the correct twist=0 position (their prior
  placement dated from the abandoned native-Blender-IK pole convention in
  `docs/phase_ik_setup.md`, a different meaning). The ±50°/axis/DOF numbers
  in this spec are unchanged and were never wrong — only the solver's
  reference-axis math was.

---

## References

- HLAS DOF Atlas (arXiv:2511.06796) — elbow 1R, functional ROM 30–130°.
  **Unverified**: this same citation could not be confirmed to exist
  during a later, independent literature search for `fingers.md` in this
  project — flagging here rather than silently trusting it, per §17.
- Biomechanics (Neumann) — elbow hinge mechanics, load distribution
- Rigify docs (Blender 5.2) — confirmed absence of a hinge-specific rig type
