# KINEMATICS — Joint Types, DOF Classification, Solver Selection

## Joint Type → DOF → Solver Decision Tree

Before writing any bpy code for a joint, classify it:

### 1. Hinge (1R)

**Definition**: One axis of rotation, two axes locked.

**Biological examples**: Elbow, knee, PIP/DIP joints, thumb IP joint.

**Blender setup**:
- MECH bone: one axis unlocked (typically X for elbow/knee), Y and Z locked
- Limits: hard min/max on that axis (e.g., elbow: 0° to 130° flexion)
- Constraint: Limit Rotation on the unlocked axis only
- Solver: Single revolute — no IK needed, direct FK control

**Validation**: At rest, rotation_euler on locked axes must be [0, 0, 0] or
[value, 0, 0] (one axis only, others zero).

---

### 2. Condyloid / Ellipsoid (2R)

**Definition**: Two orthogonal rotation axes; third axis locked.

**Biological examples**: Wrist (flexion/extension + radial/ulnar deviation),
MCP joints, thumb CMC joint.

**Blender setup**:
- Two serial MECH bones, each owning one axis
- Example for wrist:
  - `MECH_Wrist_Flex`: X-axis unlocked (flexion/extension), Y/Z locked
  - `MECH_Wrist_Dev`: Y-axis unlocked (deviation), X/Z locked
- Limits: hard min/max on each
- Constraint: Copy Rotation from CTRL, then Limit Rotation per axis
- Solver: Two serial revolutes (not a single 2-DOF joint); order matters
  (specified in the joint's spec)

**Why serial, not combined?** Because Blender's rotation_euler uses XYZ
order, and rotating X then Y produces gimbal coupling. Two separate bones
with one axis each gives mechanical clarity and prevents unexpected
cross-axis motion.

**Validation**: Confirm that rotating one MECH axis doesn't visibly affect
the other (they're independent after unlocking).

---

### 3. Ball-and-Socket (3R)

**Definition**: Three orthogonal rotation axes, none locked.

**Biological examples**: Shoulder (glenohumeral), hip joint.

**Blender setup**:
- Three serial MECH bones, each owning one axis (pitch, roll, yaw)
- Rigify's native 3-DOF limb is an option (it generates this automatically)
- OR: three hand-built serial revolutes
- Limits: hard min/max on each axis, but the interaction is more complex
  (some combinations of angles hit singularities)
- Constraint: Copy Rotation from CTRL to each MECH bone per axis
- Solver: Three serial revolutes OR Rigify's built-in spherical IK

**Singularity warning**: At certain poses (e.g., arm straight up), gimbal
lock occurs — two axes align and lose one DOF. Validation checks for this
and reports it.

**Validation**: Confirm all three axes rotate independently at mid-range.
Confirm gimbal lock is understood and acceptable at extreme poses.

---

### 4. Compound (Mixed DOF)

**Definition**: Multiple joints with different mechanical constraints that
act together (e.g., ankle = talocrural hinge + subtalar hinge).

**Biological examples**:
- Ankle complex (1R dorsiflexion + 1R inversion/eversion, with coupling)
- Knee (primarily 1R hinge + small passive rotation)

**Blender setup**:
- Classify which DOF are actuated (the ones the animator controls) vs.
  passive (coupled or omitted)
- Example for ankle (2R primary, 1R secondary passive):
  - `MECH_Ankle_Dorsi` (1R): X-axis, dorsiflexion/plantarflexion
  - `MECH_Ankle_Inversion` (1R): Z-axis (or Y, per spec), inversion/eversion
  - Passive rotation is omitted (robot can't rotate foot twist independently)
- Limits: per-axis
- Constraint: Copy Rotation, Limit Rotation
- Solver: Serial revolutes for the actuated DOF; passive DOF are either
  hardcoded (locked) or mechanically coupled via linkage (if building in
  Blender, this is documented in the joint's spec)

**Validation**: Confirm that the actuated axes move independently; confirm
passive coupling (if any) behaves as expected.

---

### 5. Floating / Deformable (0–1R, often with 2T)

**Definition**: A "joint" that has minimal or no inherent rotation but
significant positional freedom (e.g., scapula, palm cupping).

**Biological examples**: Scapulothoracic "joint" (scapula slides on rib cage),
palm cupping via MC4–MC5 flexion.

**Blender setup**:
- Usually omitted in simplified rigs OR approximated as a single 1-DOF
  rotation (e.g., scapula elevation) with fixed translation
- No MECH_Scapula_Xyz bone; instead, the scapula's local parent position
  is offset from the shoulder
- Constraint: usually just parent/child relationship + hard limits on local
  position
- Solver: None — this is kinematic offset, not a solvable joint

**Validation**: Confirm the scapula/palm stays in its offset position and
doesn't penetrate the torso.

---

## Axis Ownership and Locking

Every MECH bone in a serial chain owns exactly one rotation axis. The other
two are locked:

MECH_Elbow:
rotation_mode = 'XYZ'
lock_rotation = (False, True, True) # X free, Y/Z locked
min_angle[0] = 0°
max_angle[0] = 130°
(min/max for Y and Z don't matter; they're locked)


Before adding any driver or constraint, verify the bone's actual local axes
via Blender's viewport (rotate it, watch which axis spins) or via MCP
inspection. A bone's roll setting determines its local X/Y/Z orientation —
never assume based on a diagram.

---

## IK vs. FK vs. Analytical Solving

**Forward Kinematics (FK)**: Animator poses each joint independently. Works
for simple serial chains (elbow, wrist). No solver needed — just CTRL
bones driving MECH bones.

**Inverse Kinematics (IK)**: Animator positions an end-effector (hand, foot)
and the solver computes joint angles backward. Rigify provides 2-DOF and
3-DOF IK solvers. Use when:
- The limb has 3+ DOF (shoulder, hip, leg)
- The animator wants to place the hand/foot in space without thinking about
  intermediate joint angles

**Analytical Solving**: For specialized joints (2-DOF wrist, 1-DOF knee), a
closed-form mathematical solution exists. Write it as a Python script that
computes MECH bone angles from CTRL inputs. Faster and more reliable than
Blender's generic IK constraint.

**Decision**:
- 1R hinge → FK (no solver)
- 2R condyloid → FK (animator controls both axes) OR analytical solver
  (if authoring complex in-hand tasks)
- 3R ball-socket → IK (using Rigify's 3-DOF limb) OR analytical (if you
  derive the math)

Document the decision in the joint's spec.

---

## Serial Order Matters

For multi-axis joints, the order of bones in the hierarchy determines the
rotation order:

Example wrist (flexion then deviation):

Forearm (DEF parent)
↓
MECH_Wrist_Flex (pitch on X)
↓
MECH_Wrist_Dev (yaw on Y)
↓
DEF_Wrist (follows Dev, which follows Flex)


Reversing the order (Dev then Flex) produces different motion — gimbal
coupling manifests differently. The spec locks this order and never changes
it silently.

---

## Coupling and Cross-Joint Constraints

Some joints are mechanically coupled:

- **PIP-DIP coupling** (fingers): flexing the PIP drives the DIP via
  tendon routing. Implemented as a copy-rotation constraint or (better) a
  four-bar linkage in the MECH layer, so DIP follows PIP without needing
  an actuator.
- **Wrist-finger tenodesis** (hand): wrist extension passively closes the
  fingers. Implemented as a driver that reads the wrist angle and adjusts
  finger positions.

These are explicitly named in the joint's spec and documented in the
constraint stack. They're never added silently.

---

## Validation Checklist

After building any joint:

- [ ] MECH bone exists with correct name
- [ ] Local axes verified (X/Y/Z match the spec)
- [ ] Rotation mode set correctly (XYZ, not Quaternion)
- [ ] Axis locks applied (one free, two locked)
- [ ] Min/max rotation limits set per spec
- [ ] Copy Rotation constraint from CTRL → MECH
- [ ] Copy Rotation constraint from MECH → DEF
- [ ] No other constraints that might conflict
- [ ] Gimbal lock behavior (if 3R) documented
- [ ] ROM tested by hand (pose the CTRL bone, verify MECH and DEF follow)
- [ ] For animated sequences: export → reimport check passes (§5, docs/ENGINEERING_RULES.md)