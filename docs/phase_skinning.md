# Phase Spec: Skinning — Rigid Bone Assignment (All 78 Mesh Objects)

## Approach: Rigid Parenting, Not Weight Painting
This is a hard-surface segmented robot — every mesh piece is a separate
rigid object. The correct skinning method is NOT gradient weight painting.
Instead: each mesh object gets an Armature modifier with exactly ONE vertex
group at 100% weight for exactly ONE bone. No blending, no gradients.
The mesh piece follows that bone completely, like a rigid shell.

This is faster, cleaner, and more appropriate than organic weight painting
for this character. The black joint-cover spheres between segments are also
separate objects and follow one bone rigidly.

## Known pre-existing issue (do not fix in this phase)
411_Thumb_L and 411_Thumb_R use different mesh datablocks and are
genuinely different sizes (~26% width, ~41% height discrepancy). Both will
skin correctly to their respective thumb bones — they'll just be different
sizes. This is a modeling issue predating this session; flag it in the
render output but do not touch the geometry.

## Bone-to-mesh mapping

### Head cluster — all follow DEF_Head
411_Head, and all 411_S_* pieces (lens, cameras, mics, vents, exhaust,
any other sensor/detail pieces centered at Z>1.8). These all move with
the head as one rigid unit. Confirm the full list by checking which
objects have center_z > 1.8 — use that as the cutoff, not a hardcoded
name list, since the S_* naming may have pieces not listed here.

### Neck — DEF_Neck
Any mesh piece sitting in Z[1.63, 1.93] and centered on X≈0. From the
scan this may be part of 411_Torso or a separate collar piece — check
what actually sits in that Z range before assigning. If nothing sits
there, the neck bone drives no mesh (fine for now — it's still a
functional animation bone).

### Chest/torso — DEF_Chest
411_Torso (Z≈1.33, centered).

### Pelvis — DEF_Pelvis
411_Pelvis, 411_Pelvic_WaistRing_New, 411_Pelvic_WaistRing_Backing (or
whatever the WaistRing pieces are actually named — use the names from the
mesh scan, not these approximations).

### Right arm chain
- DEF_UpperArm → 411_UpperArm_R (and any shoulder-socket cover at that Z)
- DEF_Elbow → 411_Elbow_R (the joint cover sphere)
- DEF_Forearm_Twist → 411_Forearm_R
- DEF_Wrist → 411_Wrist_R
- DEF_Palm_Index/Middle/Ring/Little → 411_Palm_R (if one palm piece),
  or split by finger if separate palm pieces exist per finger
- Finger bones (MCP/PIP/DIP/Tip per finger) → matching 411_Finger*_A/B
  pieces by Z position within the finger
- DEF_Thumb_CMC + thumb chain → 411_Thumb_R + 411_ThumbJoint_R

### Left arm chain
Mirror of right: all _R → _L, all bone names → _L equivalents.

### Right leg chain
- DEF_Thigh → 411_Thigh_R
- DEF_Shin → 411_Shin_R + 411_Knee_R (the knee cover sits at the
  shin/thigh boundary — assign to DEF_Shin since the knee cover moves
  with the lower leg during flexion)
- DEF_Foot → 411_Foot_R + 411_Ankle_R

### Left leg chain
Mirror of right: all _R → _L equivalents.

## Method — how to apply rigid skinning

For each mesh object:
1. Add Armature modifier: target = RobotArmature
2. Add a vertex group named exactly after the target bone
3. Select all vertices of the object, assign weight 1.0 to that group
4. No other vertex groups on that object

Do this programmatically, not by hand — 78 objects is too many to do
manually without errors. Write a Python script that:
- Reads the mapping table (hardcoded dict: object_name → bone_name)
- For each entry: adds the Armature modifier if not present, creates the
  vertex group, selects all verts, assigns weight 1.0
- Prints a confirmation line per object: "Skinned {obj} → {bone}"
- Prints a summary at the end: total skinned, any skipped (not found)

Build the mapping dict from the actual mesh names confirmed in the scan,
not from the approximate names in this doc. Read the live scene before
building the dict, the same way every prior phase read live data rather
than trusting spec approximations.

## Ambiguous cases — resolve before scripting, not during
Before writing the skinning script, print the actual names of:
1. All objects with center_z > 1.8 (head cluster)
2. All objects with center_z in [1.63, 1.93] and center_x < 0.05
   (neck region — is anything actually there?)
3. All 411_S_* objects (sensor cluster — confirm full list)
4. All finger-related objects (411_Finger* and 411_Palm*) with their
   center_z and center_x — needed to map each to the correct finger bone
5. The exact names of the WaistRing pieces

Print this resolution pass first, stop, and report back — do not write
the skinning script until I've confirmed the mapping is correct. This is
the same pre-check discipline used on every prior phase.

## Regression test
After skinning, select DEF_Chest and rotate 10°/Z — confirm 411_Torso
visibly rotates with it. Select DEF_Thigh and rotate 20°/X — confirm
411_Thigh_R visibly follows. These are mesh-deformation tests, the first
actual "does the mesh move" confirmation of the entire build. Reset both
to 0° after confirming.

## Renders (explicit filenames)
- `preview_skin_rest.png` — full body, rest pose, confirming nothing
  exploded or jumped to the wrong position
- `preview_skin_torso_rotate.png` — DEF_Chest rotated 10°/Z, confirming
  torso mesh follows
- `preview_skin_arm_bend.png` — arm IK solver called with a bent pose,
  confirming arm mesh segments follow their bones
- `preview_skin_knee_bend.png` — knee bent, confirming leg mesh follows

## Save
Save as `robot_armature_v10.blend` after the regression test, before
renders.

## Stop condition
**STOP FIRST** — run the resolution pass (ambiguous cases above) and
report back before writing the skinning script. Once I confirm the
mapping, proceed with the script, regression test, renders, and save.
Do not proceed into export pipeline or CTRL layer without a new
instruction.
