# QA-010: DEF_Wrist_L local axes do not mirror DEF_Wrist

Status: VALIDATED (measurement), UNVERIFIED (effect on left-arm poses). Opened 2026-10-20 from the QA-003 close-out.
File measured: `blender/robot_armature_v16.blend`, live scene via Blender MCP, rest pose, armature object identity transform.

## Finding
`DEF_Wrist_L` is not the mirror image of `DEF_Wrist`. Comparing each local axis of `DEF_Wrist` reflected across X with the same axis of `DEF_Wrist_L` (world space):

| Local axis | mirrored R . L |
|---|---|
| X | 0.010 |
| Y (long axis) | **-0.998** |
| Z | 0.011 |

A correct mirror gives about +1.0 on the long axis. `DEF_Elbow_L` and `DEF_Forearm_Twist_L` do mirror correctly (Y = 1.0), so the fault is confined to the wrist bones.

Not a metric to use: the raw dot of the two long axes is 0.027, because both arms hang along Z. Always reflect one side before comparing.

## Related facts
- Both `DEF_Wrist` and `DEF_Wrist_L` have world length 0.0 (heads at (+0.341, 0, 1.004) and (-0.341, 0, 1.004), heads mirror exactly). Their orientation exists only in the stored bone matrix. This overlaps QA-004 (near-zero-length bones).
- Stored rolls differ: `DEF_Wrist` 0.1145, `DEF_Wrist_L` -0.8516 (matrix Z-euler, rad).
- Each carries one COPY_ROTATION constraint from `MECH_Wrist_Dev` / `MECH_Wrist_Dev_L`.
- Children are `DEF_Palm_*` and `MECH_Palm_Ring` (same set on both sides).

## Why it matters
A pose authored on the right wrist and mirrored to the left rotates about the wrong axes on `DEF_Wrist_L`. Left-arm animation is blocked until this is fixed. No current action keys a left-arm bone, so nothing shipped is affected.

## Not fixed here
QA-003 covers the left thumb only (docs/ENGINEERING_RULES.md section 14, smallest correct change). Fixing this needs: re-roll `DEF_Wrist_L` (and check `MECH_Wrist_*_L`) from world-space axes as QA-003 did with `align_roll`, record the verified axis letters in `specifications/arm/wrist.md`, and re-validate with the mirrored-axis table above plus a wrist-flex export and reimport test.

## Open question
Whether to give the wrist bones real length first (QA-004) so the long axis is defined by head and tail, not only by the matrix.
