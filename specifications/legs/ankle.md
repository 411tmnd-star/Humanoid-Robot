# ANKLE — Joint Specification (Stub)

Written per `docs/phase_knee_spec_and_ankle_check.md` Part 2: an independent
check of the ankle's own classification in `docs/KINEMATICS.md`, not an
inheritance from its proximity to the knee's passive-DOF description.

## Classification (per docs/KINEMATICS.md)

The ankle has its own dedicated classification entry in KINEMATICS.md's
Compound section — in fact a fuller, more detailed one than the knee's.
Quoted exactly:

From the Compound section's own definition (the ankle is the category's
defining example):
> **Definition**: Multiple joints with different mechanical constraints that
> act together (e.g., ankle = talocrural hinge + subtalar hinge).

From the Biological examples list:
> Ankle complex (1R dorsiflexion + 1R inversion/eversion, with coupling)

From the Blender setup, a full worked example:
> Example for ankle (2R primary, 1R secondary passive):
> - `MECH_Ankle_Dorsi` (1R): X-axis, dorsiflexion/plantarflexion
> - `MECH_Ankle_Inversion` (1R): Z-axis (or Y, per spec), inversion/eversion
> - Passive rotation is omitted (robot can't rotate foot twist
>   independently)

**Confirmed: Compound**, same top-level category as the knee, but a
materially different DOF split — **2R actuated**, not the knee's 1R, plus
1R passive (omitted, same treatment as the knee's passive DOF).

## Actuated DOF

- **Dorsiflexion/plantarflexion**: 1R, local X axis (per KINEMATICS.md's
  worked example) — same role as the knee/elbow single hinge axis.
- **Inversion/eversion**: 1R, axis stated by KINEMATICS.md itself as
  *"Z-axis (or Y, per spec)"* — the source document leaves this genuinely
  ambiguous and defers to a per-joint spec that doesn't exist yet. Not
  resolved here: no MECH/CTRL ankle bones exist yet to verify against
  empirically (unlike the knee's axis, which was checked against a live
  bone), so this is left as-is rather than guessed.

## Passive DOF

Same explicit treatment as the ankle's own line in KINEMATICS.md: passive
rotation is omitted, no third actuated bone.

## Status

**STUB** — classification and actuated/passive DOF breakdown only, per this
phase's scope (not required to be as complete as `knee.md` yet). No ROM
figures. No MECH/CTRL/DEF_Ankle bones exist — currently the ankle joint is
only the boundary point between `DEF_Shin` and `DEF_Foot`, computed during
the leg placement phase.

## Open Items

1. **Inversion/eversion axis (Z vs Y) is ambiguous in KINEMATICS.md
   itself** — not something this document is resolving, since it's the
   source spec that's unsettled, not an omission introduced here.
2. Everything ROM/limit-related, same status as the knee: undocumented,
   not estimated.
