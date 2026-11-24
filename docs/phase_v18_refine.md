# Phase v18 — finger rig fix, neutral hands, panel seams, pelvis / yoke / feet, cyclorama

Source `blender/robot_armature_v17.blend` → output `blender/robot_armature_v18.blend`, `blender/export/robot_v18.glb`.
Reproduce (headless `bpy` 5.2.3, always rebuilds from v17, never edits it):

```bash
python3 -I scripts/build/build_v18.py                      # v17 -> v18
python3 -I scripts/build/export_v18.py                     # v18 -> robot_v18.glb (DEF-only, ~70 s)
python3 -I scripts/build/render_v18.py blender/robot_armature_v18.blend blender/inspection/v18 128
python3 -I scripts/validation/v18_verify.py blender/robot_armature_v17.blend blender/robot_armature_v18.blend \
        blender/inspection/v18/verification.md blender/export/robot_v18.glb <roundtrip.log> blender/inspection/v18
```

Authorised changes: `specifications/hand/{fingers,thumb,palm}.md` (each carries a "Revision v18" block) and `docs/phase_v17_refine.md` (Defect 2 marked resolved).

## Status (docs/ENGINEERING_RULES.md §15)

| Item | Status |
|---|---|
| Zero-command deviation of all 56 DEF bones (incl. 38 finger / thumb / palm and the wrists) = 0.0000° | VALIDATED |
| MECH / CTRL finger frames equal the DEF frames they drive (0 mismatches) | VALIDATED |
| Neutral pose: MCP 12° / PIP 18° / DIP 10°, thumb 25° forward, palms inward, MCP pitch 32 mm | VALIDATED |
| Same flex command curls both hands inward with mirrored motion (4 channels) | VALIDATED |
| Torso waist / chest 0.859, rim / chest 0.921; seams 1.5 mm deep, 0 unweighted vertices on every grooved mesh | VALIDATED |
| Limb split seams 1.2 mm, head coronal seam 1.5 mm, visor pocket 1.5 mm (chamfer measured 1.70 mm) | VALIDATED |
| Yoke −18 % thickness and −4 mm sink; hip sockets 20 % inward; pelvis chassis profile; rubber sole 8 mm; dorsal radius 18 mm | VALIDATED |
| Cyclorama (max normal break 2.8°), 5-light rig, 128 samples + OpenImageDenoise | VALIDATED (settings); look checked visually |
| `robot_v18.glb`: 56 DEF joints, 122 skinned meshes, grooves baked in; export → reimport max error 2.56e-6 m (< 1e-5) | VALIDATED |
| `front.png` identical to `zerocmd.png` | see `blender/inspection/v18/verification.md` §9 |
| `thumb_curl_export_test.py` re-run | VALIDATED on v19 (`docs/phase_v19_refine.md`) |
| Three.js viewer test of the .glb | VALIDATED on v19 (`docs/phase_v19_refine.md`); R3F / the portfolio page itself not tested |

## Phase 1 — finger / thumb zero-command displacement (root cause → fix → validation)

* **Root cause.** `DEF_{F}_MCP` and `DEF_Thumb_CMC` copy rotation from their MECH driver in POSE/POSE space, which copies an *absolute* orientation. Every finger MECH/CTRL bone
  shared one rack frame unrelated to the DEF frames (its local X pointed down the arm), so at zero command the finger chain sat 17°–178° from rest. Fixing only the offset would still have
  curled the fingers about the wrong axis.
* **Fix (option 1 of the brief).** DEF finger / thumb / palm bones were re-posed and **re-rolled**, and each `MECH_*` / `CTRL_*` bone now sits on the exact head / tail / roll of the DEF bone it drives. POSE copies are an
  identity at rest and LOCAL copies act about the right axes. The contract (local X = flexion, positive toward the palm; local Z = abduction = palm normal) is the one `fingers.md` already recorded as verified (X = world +Y,
  Z = world −X, right hand): v16's rotation cleanup had broken it, v18 restores it. Left Z limits are the right limits negated. DOF, limits, locks, constraint topology unchanged; DEF count stays 56.
* **Mirror convention for fingers / thumb / palm:** x_L = −S·x_R, y_L = S·y_R, z_L = S·z_R. The same flex value curls both hands; abduction needs the opposite sign. (Wrist and legs keep x_L = S·x_R.)
* **Neutral pose.** The v17 hands were palm-back with hyperextended fingers and a medial thumb. The whole hand was rotated 90° about the forearm axis (palm inward, thumb forward), then the fingers were curled 12° / 18° / 10° (cumulative 12 / 30 / 40° from the hand axis)
  and the thumb opposed 25° forward with 8° / 8° curl. The inherited v16 MCP mounts overlapped in pairs, so they were re-spaced evenly (32 mm pitch) — **not in the brief, needed for the fingers not to collide**.
* **Phalanges.** Every finger is now white shell – dark cylindrical knuckle ×3 (MCP, PIP, DIP); thumbs have 2 shells and knuckles plus the existing CMC sphere. 38 new skinned meshes; the 16 existing finger shells and 2 thumb shells were re-used (name, group, modifiers kept).
* **Validation.** `v18_verify.py` §1–§2.

## Phase 2 — panel seams (non-destructive grooves)

* **Method.** The torso has 1,056 cage vertices (~20 mm spacing), so 2 mm × 1.5 mm grooves cannot be edge-loop cuts without remeshing. Each grooved mesh gets
  `[existing modifiers] → BOOLEAN (DIFFERENCE, exact, hidden `CUT_*` cutter) → VERTEX_WEIGHT_MIX (every vertex = 1.0 of the mesh's single DEF group) → ARMATURE`.
  The exact boolean creates unweighted vertices (154 on the torso in a first test); the Vertex Weight Mix modifier is what keeps them skinned. Groove faces take material slot 1 (graphite, rough 0.45 / metal 0.6; slot 2 on the head)
  through the cutter's face material index. Cutters are swept along rays projected onto the evaluated surface, so depth and width are exact on curved surfaces.
* **Torso contour.** Pectoral breadth +3 %; waist / chest 0.859 and rim / chest 0.921 (targets 0.86 / 0.92). **Note:** v17 already had waist / chest 0.751, so the brief's numbers make the waist *less* pinched
  (+17 %); the numbers were followed as written. Seams: two curves from the outer upper chest, inward along the ribcage, vertical to the rim (seam x / half-width 0.698 at z = 1.28), plus a horizontal seam 25 mm above the bottom edge.
* **Limbs.** Anterior centre seam 1.5 mm × 1.2 mm on both forearms and shins, t = 0.16 → 0.90 of the length (between the joint cups). **Head.** Coronal seam in the plane through the apex, between the lateral pins, 2 mm × 1.5 mm.
* **Visor.** The v17 10 mm dish was undone (head vertices restored from v16) and replaced by an exact 1.5 mm pocket with a 45° chamfer cut by the same mechanism; glass panel and cyan strip were re-projected onto the pocket floor. Measured chamfer width 1.70 mm (target 1.5; the 45° wall is built from a
  fixed ring offset, not solved per direction).
* **Export.** The exporter bakes the action first and samples it again; both re-evaluate every exact boolean on every frame (over 35 CPU-minutes without finishing). `export_v18.py` therefore evaluates each grooved mesh once, in memory, with the Armature off and swaps the result in through the exporter's `prepare` hook
  (the .blend is reloaded afterwards), cutting the export to ~70 s. The geometry that reaches the .glb is the same as `export_apply=True` would give.

## Phase 3 — pelvis, yoke, feet

* **Pelvis.** The shell and waist ring were refitted to the new torso rim (scale 1.216; it would otherwise have been narrower than the torso by 0.05 m per side), then sculpted: outer hips rise 14 mm, the centre bridge dips 12 mm, lower rings taper 10 %.
  Hip sockets moved inward by exactly 20 % of their radius (0.0144 m).
* **Yoke.** Further −18 % thickness about the 1.745 m anchor, then the lower V-edge sinks 4 mm.
* **Feet.** The foot shell is rebuilt procedurally (96 → 528 vertices): vertical wall, exact 18 mm quarter-circle dorsal edge, flat deck, toe taper 20 %; sits on a new 8 mm dark rubber sole pad (`Graphite_RubberSole`); new contoured graphite ankle cuff per foot.
* **Joint spheres (not in the brief).** The 14 joint spheres are coarse 362-vertex UV spheres; their faceted geometry showed as torn intersection lines in close-ups. A Subdivision modifier (level 2, before the Armature) fixes the cause.

## Phase 4 — cyclorama and verification

* `Studio_Cyc` replaces the two-plane backdrop (old planes **hidden, not deleted**): flat floor → 2.5 m quarter-circle → wall, warm-grey vertical gradient (a Cycles shader; not exported). Lights: Key, Fill, Rim retuned plus two new strip rims `Studio_EdgeL/R`; Cycles 128 samples, OpenImageDenoise.
* `blender/inspection/v18/verification.md` is generated by `v18_verify.py` and is the record of every number above.

## Defect found and fixed during this work

The Phase 2/3 ray-casting helper switched the armature to REST and never restored it, so the first v18 build was saved with `pose_position = REST` and **no animation or pose had any effect** (the .glb would have exported a flat clip). The zero-deviation check passed trivially in REST; the
round-trip check caught it ("bones that move: 0"). Fixed in `build_v18.py` (original mode restored and asserted); `v18_verify.py` now checks the saved mode and that `hello_sequence` moves ≥ 10 DEF bones.

## Not done / limits

* `scripts/validation/thumb_curl_export_test.py` was not re-run: its exporter call has no `prepare` hook, so it would re-evaluate the exact booleans on every frame. Thumb mirror motion is covered by the in-file +40° flex check and the round trip instead.
* No Three.js / R3F viewer test. Cyclorama colour and lighting were judged visually only.
* Seam / pocket geometry is boolean-derived; `CUT_*` objects must stay in the file for the modifiers to evaluate.
* The pre-existing 3.58° world roll of the left arm chain (documented in `docs/phase_v17_refine.md`) is untouched.
