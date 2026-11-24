# Phase Spec: Clear RobotArmature's Baked-In Object Rotation (Pre-IK Cleanup)

> **SUPERSEDED — DO NOT APPLY AS WRITTEN.** This doc predates the IK
> solvers and does not account for them. Applying its fix in isolation
> would break pole-vector tracking (confirmed via Stage 0A audit
> measurements — see `docs/phase_armature_rotation_audit_stage0a.md`).
> A correct fix would need to also recompute `IK_Target.*` world
> positions and pole angles to compensate for removing the rotation —
> that work has not been done and isn't currently planned. Everything
> below this notice describes the original, now-inapplicable plan and
> is kept for historical context only.

## Root cause
RobotArmature's object transform is not identity — it carries a small
baked-in rotation (~2–4°) with zero translation. A rig root should be
identity transform (rotation 0, scale 1, location wherever it needs to be).
A non-zero rotation on the armature object is harmless for static placement
(it was correctly compensated for when converting world-space joint
coordinates to edit-bone local space last phase) but is a liability once IK
is added: IK solvers, pole vector math, and constraint spaces all assume the
armature's own transform is clean. A stray few degrees here is exactly the
kind of thing that produces a subtly wrong pole vector direction or a pose
that looks "almost right" and is hard to diagnose later, because the error
lives one level up from where anyone would think to look.

## Scope
- IN: zeroing RobotArmature's object-level rotation, applying it so the
  bones' actual world-space positions/orientations are preserved exactly as
  currently verified.
- OUT: no bone edits, no re-placement, no IK, no other objects. This should
  be invisible in the renders — same silhouette, same joint locations, only
  the underlying transform data changes.

## The fix
```python
import bpy

arm_obj = bpy.data.objects['RobotArmature']  # confirm exact name first
print(f"BEFORE rotation_euler: {arm_obj.rotation_euler}")
print(f"BEFORE location: {arm_obj.location}")

bpy.context.view_layer.objects.active = arm_obj
bpy.ops.object.select_all(action='DESELECT')
arm_obj.select_set(True)

# Apply the existing rotation into the armature's edit data so the bones
# keep their current world-space pose, then the object transform becomes
# identity.
bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)

print(f"AFTER rotation_euler: {arm_obj.rotation_euler}")
```

Before running: confirm the exact object name for RobotArmature in the
outliner if it differs from the literal string above.

## What to verify afterward
1. Print check: `rotation_euler` should read `(0.0, 0.0, 0.0)` after the
   apply.
2. Re-render `preview_bones_front.png` and `preview_bones_side.png` at the
   same camera angles as the verified placement phase — the bones should
   land in exactly the same visual position as before. If anything visibly
   shifted, stop and report rather than proceeding; that would mean the
   apply didn't preserve world-space placement correctly and the prior
   phase's verified placement may need re-checking.
3. Confirm mesh objects are untouched — this phase only touches the
   armature object's transform, nothing else.

## Stop condition
Stop after this phase and report back with the before/after rotation
values and the two re-renders. Do not proceed into IK setup until this is
confirmed clean — IK is the next phase after this one, not part of it.
