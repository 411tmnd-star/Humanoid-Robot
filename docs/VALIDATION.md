# VALIDATION — Testing Procedures and Pass/Fail Criteria

## Validation Layers

Every joint or mechanism is validated at four levels:

1. **Structural** — bones, hierarchy, constraints exist and are named correctly
2. **Mechanical** — ROM, limits, axes, locking work as specified
3. **Animation** — motion is smooth, no gimbal lock or singularities
4. **Export** — glTF round-trip (export → reimport) preserves motion

All four must pass before a joint is marked VALIDATED.

---

## Layer 1: Structural Validation

Run after every bone/constraint/driver build step.

### Checklist

Bone: MECH_Elbow
☐ Exists in armature
☐ Name matches spec exactly
☐ Parent relationship correct (child of MECH_Forearm, parent of DEF_Elbow)
☐ Rotation mode: 'XYZ'
☐ Inherit rotation: True
☐ Lock rotation: (False, True, True) per spec
☐ Bone roll verified (local X/Y/Z match spec)

Constraint: Copy Rotation (CTRL → MECH)
☐ Exists on MECH_Elbow
☐ Target: CTRL_Elbow
☐ Space: Local or World per spec
☐ Influence: 1.0
☐ Rotation order (if specified): matches local axes

Constraint: Limit Rotation (on MECH_Elbow)
☐ Exists
☐ X min/max: [0°, 130°] per spec (example)
☐ Y, Z: locked (min = max = 0°)
☐ Space: Local


**How to check**:
- Open the live .blend in Blender
- Select the bone via Outliner (not selection in viewport)
- Open the Transform Properties panel (press 'N', find "Transform")
- Verify each field matches the checklist
- For constraints: click the bone, open the Constraints panel (wrench icon),
  expand each constraint, verify target and parameters

**Automated check**: Run `scripts/validation/structural_check.py` (to be
written), which:
- Queries the armature via bpy
- Checks bone existence, hierarchy, locks
- Reports mismatches as FAIL lines

If any field doesn't match: **FAIL**. Stop and fix before proceeding.

---

## Layer 2: Mechanical Validation

Run after all constraints and drivers are in place.

### ROM Testing

1. **Open the .blend in Blender**
2. **Select the CTRL bone** for the joint (e.g., CTRL_Elbow)
3. **Rotate it manually** using the 3D viewport (press 'R', then 'X', then type
   angles)
4. **Observe**:
   - Does the MECH bone follow the CTRL bone? (Verify copy-rotation constraint
     works)
   - Does the MECH bone stay within its locked axes? (Check that Y/Z are still
     zero)
   - Does the DEF bone follow the MECH bone? (Verify DEF constraint works)
   - Do any limits trigger correctly? (Reach the min/max angle and verify the
     bone stops)

5. **Test the full ROM**:
   - Rotate CTRL from min to max (e.g., 0° to 130° for elbow)
   - At each major angle (0°, 30°, 60°, 90°, 130°), pause and check:
     - MECH bone angle matches CTRL
     - DEF bone follows MECH
     - No visual distortion or mesh interpenetration

**Pass criteria**:
- All three layers (CTRL → MECH → DEF) move in sync
- No unwanted axes rotate (Y and Z stay at 0° for a 1-DOF hinge)
- ROM limits enforce correctly (bone stops at min/max)
- No constraint conflicts (bone doesn't jitter or oscillate)

**Fail criteria**:
- CTRL moves but MECH doesn't (constraint not working)
- MECH moves but DEF doesn't (export-stage failure)
- Locked axes rotate anyway (axis lock not applied)
- Motion is jerky or unstable (constraint conflict)

### Gimbal Lock Detection (3-DOF Joints Only)

For 3-DOF joints (shoulder, hip), gimbal lock is expected at certain poses.

1. **Identify the singularity pose** (documented in the joint's spec; e.g.,
   shoulder arm straight up)
2. **Pose the CTRL bone** at the singularity
3. **Attempt small rotations** on the "lost" axis — it won't move (expected)
4. **Move away** from the singularity — the lost axis should return
5. **Document** in the spec that gimbal lock occurs here and is acceptable

**Pass criteria**: Gimbal lock occurs only at specified singularities, not at
rest or mid-range.

**Fail criteria**: Gimbal lock occurs outside the singularity zone, or the
animator can't recover from it by moving away.

### Axis Ownership Verification

For multi-axis joints (2R or 3R):

1. **Create a test animation**:
   - Keyframe 1: CTRL bone at rest (0°, 0°, 0°)
   - Keyframe 2: CTRL bone rotated on axis 1 only (90°, 0°, 0°)
   - Keyframe 3: CTRL bone rotated on axis 2 only (0°, 90°, 0°)
   - Keyframe 4: CTRL bone rotated on both axes (90°, 90°, 0°)

2. **Play the animation** and observe:
   - At each keyframe, does MECH follow CTRL exactly on each axis?
   - Do the axes interact (gimbal coupling)? (Expected for 3R, not for 2R)

3. **For 2-DOF (e.g., wrist)**:
   - Flexion + Deviation should rotate independently
   - No coupling — one axis rotating shouldn't affect the other's motion

4. **For 3-DOF (e.g., shoulder)**:
   - Expected coupling due to gimbal lock — document and accept

**Pass criteria**:
- Each axis rotates as assigned (axis 1 on X, axis 2 on Y, etc.)
- No unexpected cross-axis motion (except gimbal lock at singularities)

**Fail criteria**:
- Rotating axis 1 causes axis 2 to move (coupling outside gimbal lock)
- Axes don't rotate at all (constraint not working)

---

## Layer 3: Animation Validation

Run after Layer 2 passes.

### Smooth Motion Test

1. **Create a simple walk-cycle or reach animation**:
   - 30-frame animation
   - Pose keys at frames 0, 15, 30
   - Frame 0: rest pose
   - Frame 15: mid-motion (joint at a mid-range angle)
   - Frame 30: back to rest

2. **Play the animation** in Blender:
   - Does motion interpolate smoothly? (No jumps or pops)
   - Do all three layers (CTRL, MECH, DEF) stay in sync?
   - Is there any jitter or instability?

**Pass criteria**: Animation plays smoothly without artifacts.

**Fail criteria**: Jitter, discontinuities, or layers desync during playback.

### Constraint Evaluation Order

For complex rigs with multiple constraints:

1. **Check the constraint stack order** on the DEF bone
2. **Expected order** (from top to bottom):
   - Limit Rotation (hard stops first)
   - Copy Rotation (input constraint)
   - (Other constraints as needed per spec)

3. **If the order is wrong**, constraints may fight each other

**Pass criteria**: Constraint order matches documentation, animation is
smooth.

**Fail criteria**: Animation has jitter or unexpected limits trigger.

---

## Layer 4: Export → Reimport Validation

**This is the critical gate.** A rig that looks perfect in Blender can export
incorrectly to glTF.

### Procedure

1. **Bake the animation to DEF bones only**:
```python
   # Pseudo-code (full script in scripts/validation/bake_to_def.py)
   for frame in range(start, end):
     bpy.context.scene.frame_set(frame)
     for def_bone in all_def_bones:
       insert_keyframe(def_bone, 'location', 'rotation_quaternion', 'scale')
```
   This converts all constraints/drivers to static keys on DEF bones only.

2. **Delete all CTRL, MECH, and constraint layers**:
   - Keep only DEF bones
   - Geometry and DEF armature only

3. **Export to .glb**:

File → Export → glTF 2.0 (.glb)
Settings:
- Format: GLB (binary)
- Animation: on (Bake All Actions: on)
- Deformations: on (Armature)
→ Save as robot_test.glb


4. **Create a new test .blend**:
   - File → New
   - Import robot_test.glb
   - Select the armature

5. **Play the animation** in the test .blend:
   - Does it match the Blender viewport?
   - Are bones in the correct positions?
   - Is the mesh deforming correctly?
   - Any unexpected bone rotations or scales?

**Pass criteria**:
- Reimported animation matches original viewport motion
- Mesh deforms the same way
- No bone transform errors

**Fail criteria** (Common glTF issues):
- Bones are rotated/translated incorrectly (axis/order mismatch on export)
- Animation is delayed or plays at a different speed (timing issue)
- Mesh distorts or deforms wrong (skinning weights issue, not MECH-layer issue)
- Some bones are missing or scaled wrong (rig structure issue)

**If export fails**: The problem is usually one of:
1. Rigify's native IK doesn't export cleanly (use FK or bake IK to FK before
   export)
2. Bendy bones create non-linear deformation (disable bendy bones)
3. Constraint stacks aren't being baked correctly (bake script needs fixing)

---

## Known Export/Validation Gotchas

Several joint specs (`palm.md`, `thumb.md`) reference "gotchas already
logged in docs/VALIDATION.md" — this section is that log, consolidated
across every joint built so far so future joints don't rediscover the
same issues from scratch.

1. **`export_animation_mode` defaults to `'ACTIONS'`**, which exports
   every compatible Action in the file, not just the one assigned to the
   object being exported. Set it explicitly to `'ACTIVE_ACTIONS'`.
   (Found on the elbow build.)

2. **Bake only the DEF bones**, not the whole pose. Baking every
   selected pose bone (including CTRL/MECH) leaves stale fcurves in the
   baked Action referencing bones that get deleted in the DEF-only export
   copy — harmless (the exporter skips them with a warning) but noisy and
   worth avoiding by selecting only the DEF bones before baking. (Found
   on the wrist build.)

3. **A reimporting scene's fps must be set *before* `import_scene.gltf`
   runs**, not after. glTF stores animation timing in seconds; Blender
   bakes that into frame numbers at import time using whatever fps is
   current *then*, and does not rescale retroactively if fps changes
   afterward. (Found on the elbow build.)

4. **Scene `frame_end` must cover the full length of whatever animation
   is being baked**, and isn't automatically extended by keyframing past
   it. If a joint's test animation runs longer than a previous joint's
   (e.g. thumb's Opposition Reach needing frame 30 when the scene was
   still set to end at frame 20 from the elbow build), the bake silently
   truncates at the old `frame_end` — no error, no warning — and frames
   past that hold the last-baked frame's value in the reimported file.
   Check and extend `scene.frame_end` before baking, every time. (Found
   on the thumb build.)

5. **Matrix-composition validation for a multi-bone chain must use each
   bone's actual measured rest transform**, not an assumed world-axis
   rotation product. This only matters once a chain bends direction
   partway through (every joint through the wrist and palm builds was a
   straight line from its parent, so this never came up) — composing
   `Rotation('X', a) @ Rotation('Z', b) @ ...` directly gives false
   failures on a bent chain even when the rig itself is correct, because
   each bone's local axes are defined relative to its own rest
   orientation, which differs from its parent's when the chain bends.
   Read each bone's `matrix_local` from the live scene and build the
   expected transform from that. (Found on the thumb build.)

6. **New bones created via `edit_bones.new()` default to
   `rotation_mode = 'QUATERNION'`**, not `'XYZ'` — inconsistent with every
   other bone in this rig and silently failing the Structural checklist's
   "Rotation mode: 'XYZ'" item if not set explicitly. Worse than a
   mismatch: setting `pose_bone.rotation_euler` on a QUATERNION-mode bone
   is a **silent no-op** — no error, no warning, the bone just doesn't
   move — which reads exactly like a zero-DOF joint or a broken parent
   link during testing, not like a rotation-mode mismatch. Set
   `rotation_mode = 'XYZ'` on every new bone immediately after creation,
   before any FK/axis-direction testing. (Found on the leg placement
   build, `DEF_Thigh`/`DEF_Shin`/`DEF_Foot`, during the knee's
   flexion-axis verification.)

---

## Automated Validation Script

`scripts/validation/full_check.py` runs all four layers:

```python
# Pseudocode
def validate_joint(joint_name, spec):
    """
    Args:
        joint_name: 'Elbow', 'Wrist', etc.
        spec: dict with expected DOF, ROM, rig type, etc.
    
    Returns:
        'VALIDATED', 'FAIL', or 'PARTIAL'
    """
    
    # Layer 1: Structural
    structural_pass = check_bone_hierarchy(joint_name, spec)
    
    # Layer 2: Mechanical
    mechanical_pass = check_rom(joint_name, spec)
    
    # Layer 3: Animation
    animation_pass = check_smooth_motion(joint_name)
    
    # Layer 4: Export
    export_pass = check_export_reimport(joint_name)
    
    if all([structural_pass, mechanical_pass, animation_pass, export_pass]):
        return 'VALIDATED'
    else:
        report_failures(...)
        return 'FAIL'
```

Run before shipping any animation to the live site.

---

## Validation Report Format

Every validation produces a report:

VALIDATION REPORT: Elbow

Joint: MECH_Elbow
Status: VALIDATED

Layer 1 (Structural): PASS
Layer 2 (Mechanical): PASS
Layer 3 (Animation): PASS
Layer 4 (Export): PASS

ROM tested: 0° → 130° ✓
Gimbal lock: N/A (1-DOF hinge)
Constraint order: Correct

Export result:

Original frames: 30
Exported frames: 30
Reimport match: YES

Notes: None

Validated by: Claude Code
Timestamp: 2026-08-31T23:45:00Z


**FAIL reports** include:
- Which layer failed
- The specific field that didn't match spec
- Proposed fix
- Timestamp

---

## When Validation is Required

Validate:
- ✓ After building a new joint (before moving to the next)
- ✓ After modifying a joint's constraints or drivers
- ✓ Before any animation is animated on a joint
- ✓ Before export to .glb
- ✓ If a previous validation flagged a failure (re-validate after fix)

Don't validate:
- ✗ While building (too early; constraints may not be in place yet)
- ✗ Inside a saved .blend (run scripts outside of the scene)