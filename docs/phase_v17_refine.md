# Phase v17 — Wrist frame fix, silhouette refinement, end-effectors, visor

Source `blender/robot_armature_v16.blend` → output `blender/robot_armature_v17.blend`, `blender/export/robot_v17.glb`.
Reproduce: `python3 -I scripts/build/build_v17.py` (headless `bpy` 5.2.3, always rebuilds from v16, never edits it).
Verify: `python3 -I scripts/validation/v17_verify.py blender/robot_armature_v16.blend blender/robot_armature_v17.blend`
(report committed at `blender/inspection/v17/verification.md`). Renders: `scripts/build/render_v17.py`.

Design target (user-locked): enclosed matte-white cylindrical shells, flush dark spherical joints, tapered limbs,
integrated upper yoke / pelvis (UBTECH Walker S1 + Fourier GR-2 + XPeng Iron hybrid). Figure 03 / Optimus rejected.

## Status (docs/ENGINEERING_RULES.md §15)

| Item | Status |
|---|---|
| Wrist rest frames, L = exact world mirror of R; `DEF_Wrist` rest frame == `MECH_Wrist_Dev` | VALIDATED |
| Zero-command hand deviation 113° (R) / 87° (L) → 0.0000° | VALIDATED |
| Mirrored wrist command → mirrored hand motion (residual 0.00°) | VALIDATED |
| 56 DEF bones, 0 non-DEF deform bones, thumb QA-003 mirror preserved | VALIDATED |
| Forearm / shin taper 1.31 / 1.29 → 0.70; neck ×0.525; yoke; pelvis tuck | VALIDATED (numeric) |
| Hands + hand-chain bones ×1.65 about the wrist head (all 22 meshes, both sides) | VALIDATED |
| Feet 0.227 × 0.112 m, flat sole z = 0 | VALIDATED |
| Visor, materials | IMPLEMENTED (visual check only; glTF material round-trip not inspected in a viewer) |
| `robot_v17.glb` DEF-only export + reimport (56 bones, 41 frames, max error 2.8e-6 m) | VALIDATED |
| Finger / thumb zero-command deviation (see Defect 2) | **RESOLVED in v18** (`docs/phase_v18_refine.md`) |
| Three.js / R3F viewer test of the .glb | UNVERIFIED |

## Defects found (root cause → system → correction → validation)

### Defect 1 — `DEF_Wrist{,_L}`: wrong rest frame under an absolute copy (FIXED)

* Root cause: `DEF_Wrist` / `DEF_Wrist_L` were zero-length bones (length 2e-6) with arbitrary roll. Each copies rotation from
  `MECH_Wrist_Dev` in POSE/POSE space, which copies the target's **absolute** orientation. Because the DEF rest frame
  differed from the MECH rest frame, the whole hand (palm, thumb, fingers) sat rotated 113° (R) / 87° (L) about the wrist
  at zero command. The v16 `hello_sequence` keys the wrist from 0, so the exported v16 clip had distorted hands from frame 0.
  This is also the "left wrist reversed axis" logged in `blender/inspection/w0/W0_PLAN.md`; it affected both wrists, not only the left.
* Fix: `DEF_Wrist` gets length 0.05 and the exact rest frame (y and z axes) of `MECH_Wrist_Dev`. The left wrist set
  (`DEF`, `MECH_Wrist_Flex/Dev`, `CTRL_Wrist_Flex/Dev`) is set to the exact world mirror of the right using an explicit
  head/tail/roll calculation (convention verified on the legs: x_L = S·x_R, y_L = S·y_R, z_L = −S·z_R, S = diag(−1,1,1)).
  `bpy.ops.armature.symmetrize` was **not** used: it would also move the skinned left arm chain. Left `MECH_Wrist_Flex/Dev`
  lock flags now match the right (Flex: Y,Z locked; Dev: X,Y locked) per docs/ENGINEERING_RULES.md §8.
* Side effect, stated: the left wrist set is now rolled 3.58° relative to the left forearm (the left arm chain was already
  3.58° off the exact mirror in v16; that roll offset is upstream of the wrist and is left alone). World-space mirror symmetry
  of the wrist is exact; parent-relative frames differ by that 3.58° on the left only.
* Validation: `v17_verify.py` §2 (axes, heads, tails, rolls, locks, zero-command deviation, mirrored-command residual).
  Mirror convention for animators: flex mirrors with the **opposite sign** on L (same as the validated ankle); deviation mirrors with the same sign.

### Defect 2 — finger / thumb roots: same absolute-copy flaw, one level down (RESOLVED in v18)

> **Resolved in v18.** DEF/MECH/CTRL finger frames were aligned and the axis contract restored; zero-command deviation is now 0.0000° on all 56 DEF bones. See `docs/phase_v18_refine.md` Phase 1 and `specifications/hand/fingers.md` "Revision v18". The text below is the original v17 analysis, kept as the record.

* Root cause: `DEF_*_MCP` and `DEF_Thumb_CMC` copy rotation from `MECH_*_MCP_Abd` / `MECH_Thumb_CMC_Abd` in POSE/POSE
  space. All finger MECH/CTRL bones share one common rack orientation that is unrelated to each finger's own direction, so at
  zero command the chain is rotated 103°–178° (index/little) and 17° (thumb) from rest. v16 has the same defect (up to 171°).
* Why not fixed here: the obvious fix (re-seat MECH/CTRL on the DEF bones) would change which local axis is flexion. The DEF
  finger flexion axis is its local Z, while `specifications/hand/fingers.md` wires MECH flexion to local X. A correct fix re-derives
  the finger axes per docs/ENGINEERING_RULES.md §8 and is a change to a locked spec (§14), well outside a wrist / silhouette change.
* Consequence: the REST pose is correct; the **posed** hand at zero command (and therefore the exported `hello_sequence` at frame 0) still shows
  displaced fingers, as in v16. `blender/inspection/v17/zerocmd.png` documents it. Design shots use the REST pose.
* Proposed correction: per finger, define MECH/CTRL rest frames with the spec axes aligned to the DEF finger direction, or replace
  the three root absolute copies with a rest-relative formulation; re-validate with `thumb_curl_export_test.py` and the round-trip check
  at zero command (deviation < 0.01° on all 30 finger/thumb DEF bones).

## Change record for locked specs (§14)

No `specifications/*.md` file is edited. Mechanisms are unchanged: the same MECH → DEF copy-rotation topology, same limits, same DOF.
What changes: (a) rest frames of `DEF_Wrist*` and the left wrist MECH/CTRL bones (Defect 1); (b) hand-chain bone positions are scaled ×1.65
about the wrist head (joint-to-joint lengths scale with the mesh; angles, rolls, parents and constraints unchanged). Dependents: the
`wrist.md` axis hypothesis (X = flexion) is unaffected because MECH/CTRL local axes keep their roles; any animation keyed on the wrist
keeps its meaning. Re-validation: `v17_verify.py` + `glb_roundtrip_check.py` (both pass).

## Geometry and material changes (cage / vertex edits only; no remesh, no deleted objects)

| Part | Change |
|---|---|
| Forearm, shin (2×24-vertex prisms) | distal ring scaled about its own centroid so distal radius = 0.70 × proximal (was 1.31× / 1.29×); proximal ring untouched so the upper arm / thigh / elbow / knee continuity is kept |
| Neck | XY radius ×0.525 (−47.5 %). `N_JointRing` scaled the same; LEDs, service port, cable guard translated toward the axis at constant size |
| Yoke | x ×0.90, y ×0.86, z ×0.65 anchored at the neck base (z = 1.745) |
| Pelvis shell, waist ring, backing | x ×0.90 / ×0.97, y ×0.635 / ×0.73 → inside the torso waist (0.221 × 0.156); smooth shading with sharp edges at 45° / 30° + Weighted Normal modifier (before Armature). Hip sockets smoothed the same way |
| Hands | 11 meshes per side + 114 DEF/MECH/CTRL hand-chain bones scaled ×1.65 about `DEF_Wrist{,_L}` head |
| Feet | each ring remapped to a squared oval, bottom 0.112 × 0.2265 (= 1.4 × the v16 shin ankle-end diameter 0.1618), top 90 %, height 0.158 → 0.085, sole at z = 0, heel 30 % behind the ankle axis |
| Head | 554 front vertices dished 10 mm (smooth falloff); new `411_Visor_Glass` (ray-projected superellipse panel) and `411_Visor_Strip` (cyan emission) skinned 100 % to `DEF_Head`; old camera lens / strip seated 32 mm inward onto the panel |
| Materials | `Helmet_SatinWhite` (rough 0.32, coat 0.25), `Visor_BlackGlass` (rough 0.08), `Visor_CyanEmit` (emission 1.6), `Graphite_Brushed` (rough 0.45, metallic 0.6) replace `BotBlack_Matte` / `BotBlack_Gloss` on all former black parts |

Chosen numbers (not specified by the brief): taper ratio 0.70 (middle of 0.65–0.75), neck ×0.525 (middle of the 45–50 % band),
foot length from the v16 shin ankle-end diameter, visor footprint (half-width 92 mm, half-height 118 mm, superellipse n = 3.5, centred z = 2.115).

## Facts worth knowing

* 54 `411_*` mesh objects carry a baked rotation (quaternion mode, so euler fields read zero). The build edits vertices in **world** space.
* `Object.bound_box` returns the evaluated (posed) box; use base vertices for rest geometry.
* The export adds 2 skinned meshes (80 vs 78 in v16): `411_Visor_Glass`, `411_Visor_Strip`.
