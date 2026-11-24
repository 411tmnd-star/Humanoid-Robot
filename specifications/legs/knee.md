# KNEE — Joint Specification

## Classification (per docs/KINEMATICS.md)

`docs/KINEMATICS.md` classifies the knee under **Compound (Mixed DOF)**, not
simple Hinge — despite the knee also appearing in the Hinge section's
biological-examples list. Both mentions, quoted exactly, not paraphrased:

From `### 1. Hinge (1R)`:
> **Biological examples**: Elbow, knee, PIP/DIP joints, thumb IP joint.

From `### 4. Compound (Mixed DOF)`:
> **Definition**: Multiple joints with different mechanical constraints that
> act together (e.g., ankle = talocrural hinge + subtalar hinge).
>
> **Biological examples**:
> - Ankle complex (1R dorsiflexion + 1R inversion/eversion, with coupling)
> - Knee (primarily 1R hinge + small passive rotation)

**Reconciling the two mentions (my reading, flagged as such — not an
explicit statement in KINEMATICS.md):** the Hinge section's list is the
simplified default (many joints *can* be built as pure hinges); the
Compound section is the more specific entry that actually describes the
knee's mechanism, including the passive component the Hinge list is silent
on. Compound is treated as authoritative here. See Open Items — this
reconciliation itself is not settled by the source document and is called
out rather than silently assumed.

## Actuated DOF — Flexion/Extension

- **Axis**: local X. Quoted from KINEMATICS.md's Hinge section, Blender
  setup: *"MECH bone: one axis unlocked (typically X for elbow/knee), Y and
  Z locked"* — X is explicitly named for both elbow and knee.
- **Empirically verified**, not just cited: `DEF_Shin` was rotated ±15° on
  local X on the live rig. `+15°` moved `DEF_Foot`'s tail by `+0.1048` on
  world Y — toward +Y, which is backward (forward = −Y, confirmed earlier
  this session via the front-facing lens mesh position) — matching
  anatomically correct flexion (shin swings backward). `-15°` produced the
  opposite (foot moved forward). Confirms **positive local-X rotation =
  flexion**, the same sign convention as the elbow (0° = extension,
  positive = flexion).
- **ROM: 0° (full extension) to 120° (full flexion) — decided by project
  owner, 2026-09-02.** Not stated in KINEMATICS.md (the only degree figure
  in the whole document is the elbow's illustrative *"e.g., elbow: 0° to
  130° flexion"*, not a knee number) and not sourced from outside
  literature. Chosen deliberately narrower than human anatomical norm
  (~135°) on the same "functional over full biological" reasoning already
  used for the elbow (130° functional vs 150° biological) and wrist
  (operational range narrower than full AROM): this is a portfolio mascot
  robot that walks and sits, not a general humanoid that needs to squat to
  the floor. 120° covers a seated pose and normal gait clearance with
  margin. Flagged for tuning once animated, same as every other joint's
  ROM in this project.

## Passive DOF — Secondary Rotation

Per KINEMATICS.md's Compound methodology, quoted exactly:

> "Classify which DOF are actuated (the ones the animator controls) vs.
> passive (coupled or omitted)"

and the ankle worked example, given as the template for how a Compound
joint's passive DOF is handled:

> "Passive rotation is omitted (robot can't rotate foot twist
> independently)"

Applying the same treatment to the knee's own passive rotation (per its
classification, *"primarily 1R hinge + small passive rotation"*): **the
passive DOF is omitted entirely — no bone, locked or otherwise — not solved
for, and does not get its own actuated CTRL bone.** There is no
pole-target-driven twist mechanism for the knee, unlike the elbow/forearm-
twist pair where twist was a real second actuated joint. This is the exact
assumption that failed during today's classification check, before this
file was written — noted here as the corrected conclusion, not carried
forward as the original (wrong) assumption.

**Decided by project owner, 2026-09-02:** omit rather than model as a
locked (min=max=0) bone. A locked bone is pure liability for zero
animator-facing value — something to skin around and keep in sync with no
mechanical benefit — and better matches KINEMATICS.md's own "robot can't
rotate foot twist independently" phrasing as "no bone," not "bone, locked."

## Bone Naming

Currently built (placement phase, `docs/phase_leg_bone_placement.md`):
`DEF_Thigh` (root, parentless) → `DEF_Shin` (connected child) → `DEF_Foot`
(connected child). No MECH/CTRL layer exists yet for the knee — that's a
later phase, explicitly out of scope here.

Once that layer is built, naming follows §6's `{LAYER}_{Segment}_{Joint}`
pattern, mirroring the elbow's precedent: `MECH_Knee` (the hinge bone,
positioned in series the same way `MECH_Elbow` sits between `DEF_UpperArm`
and `MECH_Forearm_Twist`), `CTRL_Knee` (free animator handle, undriven at
rest, same role as `CTRL_Elbow`), `DEF_Knee`. Because the knee has no
actuated secondary joint (see Passive DOF above), there is no
`MECH_Shin_Twist` / `CTRL_Shin_Twist` counterpart to `MECH_Forearm_Twist` —
`DEF_Shin` stays a single rigid segment, the same role `DEF_UpperArm` plays.

## Open Items

1. ~~ROM figures~~ — **RESOLVED 2026-09-02**: 0°–120°, project owner
   decision, see Actuated DOF above.
2. **The Hinge-vs-Compound reconciliation above is my reading, not an
   explicit statement in the source doc.** Worth confirming that Compound
   (with Hinge-style single-axis mechanics for the actuated portion) is the
   intended treatment. Still open.
3. ~~Passive DOF: locked bone vs. omitted~~ — **RESOLVED 2026-09-02**:
   omitted entirely, no bone. See Passive DOF above.
4. **Rigify rig-type**: per `docs/RIGGING.md`'s "1-DOF Hinges (no dedicated
   Rigify rig type)" section, elbow and knee are explicitly grouped
   together as hand-built, untyped bones. Stated here for completeness,
   matching `elbow_forearm.md`'s already-resolved history — no
   `rigify_type` should be assigned to `MECH_Knee` once built. Still open
   only in the sense of not yet acted on; not actually ambiguous.

## Notes

**`rotation_mode` bug found and fixed during this session's axis-direction
verification test:** `DEF_Thigh`, `DEF_Shin`, `DEF_Foot` were left at
Blender's default `rotation_mode = 'QUATERNION'` when created during the
placement phase — inconsistent with every other bone in this rig (all use
`'XYZ'`) and with KINEMATICS.md's own Validation Checklist item ("Rotation
mode set correctly (XYZ, not Quaternion)"). This caused a *silent* no-op —
setting `rotation_euler` on a QUATERNION-mode bone does nothing and raises
no error — which initially and misleadingly looked like a zero-DOF joint
during the first flexion-direction test. Fixed to `'XYZ'` on all three
bones; also logged in `docs/VALIDATION.md`'s "Known Export/Validation
Gotchas" list so future joint builds check this explicitly.

## Status

**ASSUMED** — the solver-blocking items (ROM, passive-DOF treatment) are
resolved; axis and sign are VALIDATED (empirically tested against the live
rig, not assumed). Not yet VALIDATED end-to-end because no MECH/CTRL_Knee
bones or solver exist yet — that's the next phase. Open Item 2 (Hinge vs.
Compound reconciliation) remains open but is not solver-blocking. No
solver code written — out of scope for this phase.
