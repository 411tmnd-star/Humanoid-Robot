# Phase v19 — refined finger joints

Source `blender/robot_armature_v18.blend` → `blender/robot_armature_v19.blend`, `blender/export/robot_v19.glb` (v18 files kept).
Reproduce: `python3 -I scripts/build/build_v19.py`, `python3 -I scripts/build/export_v18.py blender/robot_armature_v19.blend blender/export/robot_v19.glb`,
`python3 -I scripts/validation/v19_hand_check.py <v18> <v19> <out.md>`, `python3 -I scripts/validation/v18_verify.py <v17> <v19> <out.md> <glb> <roundtrip.log> <render dir>`.

## Feedback and what was actually in the file

The brief spoke of "14 joint spheres" in the fingers. Inspection of v18 showed otherwise:

| object class | count | geometry | modifiers |
|---|---|---|---|
| finger / thumb knuckles | 28 | **bevelled cylinders**, 29.8 mm across × 28.8 mm long | Bevel, Armature |
| thumb CMC joints | 2 | spheres, 38.4 mm | Subdivision, Armature |
| body joints (shoulder, elbow, wrist, knee, ankle, hip) | 12 | spheres, 115-130 mm | Subdivision, Armature — **not changed** (not what the feedback showed; validated geometry) |

So the 30 hand joint pieces were resized; "14" does not match anything in the hand, and no wrist helper objects exist.

## Changes (hand only)

| quantity | v18 | v19 |
|---|---|---|
| knuckle diameter (mean of 28) | 29.83 mm | **18.64 mm (−37.5 %)** — every knuckle exactly ×0.625 |
| knuckle axial length | 28.8 mm | 25.9 mm (a hinge pin slightly proud of the shell; < 32 mm finger pitch) |
| knuckle sides | 16 | 24 (smoother silhouette at the smaller size) |
| white shell diameter (mean of 28) | 26.40 mm | **22.44 mm (−15.0 %)** |
| knuckle / shell diameter | 1.13 | **0.83** (joint is now secondary to the shell) |
| thumb CMC sphere | 38.4 mm | **24.0 mm (−37.5 %)**, Subdivision modifier kept |

Finger pitch stays 32 mm: the knuckle (26 mm) and shell (22 mm) are both narrower than the pitch, so no re-spacing was needed. Geometry is regenerated on the existing objects (names, vertex groups, materials, modifier order kept);
bones, frames, constraints, limits, the 12°/18°/10° curl and the palm-inward pose are untouched. Nothing outside the hand was changed.

## Verification

* `v19_hand_check.py` (`proof/evidence_v20/v19_hand_check.md`): reductions above; **0 unweighted vertices or weights ≠ 1 across 11,844 evaluated hand vertices (58 meshes)**.
* `v18_verify.py` on v19 (`blender/inspection/v19/verification.md`): every earlier hard check still passes — 0.0000° zero-command deviation on all 56 DEF bones, 12.00/18.00/10.00° curl, mirrored +40° flex on both hands, torso ratios, grooves, visor, pelvis / yoke / feet, cyclorama, 0 unweighted vertices on every grooved mesh.
* `robot_v19.glb`: 56 DEF joints, 122 skinned meshes, 9.09 MB (< 10 MB); export → reimport max error 2.56e-6 m (< 1e-5).
* Renders (`blender/inspection/v19/`, 128 samples, OpenImageDenoise): same cameras as v18; `hand.png` uses the exact camera of the feedback image. `front.png` vs `zerocmd.png` are compared pixel-wise in the verifier.

## Previously open items — now done

* **`thumb_curl_export_test.py` on v19: PASS** (`blender/inspection/v19/thumb_curl_test.log`). The test's exporter call has no hook for the groove pre-bake, which is why it had been skipped; `scripts/build/export_glb_baked.py` is a drop-in exporter that bakes the six grooved meshes once in memory and then runs the unchanged exporter. After a clean reimport the left thumb mirrors the right to ≤ 2.1e-6 m
  (tolerance 1e-4), import-vs-source ≤ 2.1e-6 m, right-thumb motion up to 0.136 m. The wrong-sign control (left abduction key +1) **fails** (mirror error 4.2-5.5 cm), so the test is not vacuous. Re-run: `scripts/validation/thumb_curl_run.py`.
* **Three.js viewer test: PASS** (`blender/inspection/v19/threejs/viewer_test.md`, screenshots `threejs_frame00/10.png`, Blender ground truth `reference.json`). three 0.170.0, `GLTFLoader` + `AnimationMixer` + WebGL (SwiftShader) in headless Chromium: 56 DEF joints and 129 `SkinnedMesh` objects (122 glTF nodes; multi-material meshes split per material),
  149,607 vertices all with unit skin weight and no NaN, one 2.667 s clip on 56 nodes; **bone world positions match Blender to 1.8e-6 m** (2,296 comparisons, all 41 sampled frames); skinned bounding boxes of 12 meshes (grooved torso and head, hands, knuckles, forearm, foot, sole, visor, neck) match Blender to 1.1e-6 m at four frames; robot height 2.335 m; the render changes between frame 0 and the hello peak.
  Negative control: the v17 GLB against the v19 reference fails 3 checks (bones off by up to 0.265 m). Run: `cd scripts/validation/threejs && npm install && node run_viewer_test.mjs <glb> <reference.json> <out dir> [report.md]` after `record_reference.py`.

## Limits

Judging "elegant" is visual; the numbers show the black parts are smaller and secondary, not that the look is final. The viewer test ran in a software-rendered headless browser: it proves loading, skinning, animation and geometry agree with Blender, not frame rate or look on a real GPU, and not the portfolio site's own page (R3F, materials, lighting). Materials were not compared.

*Update (v20): the Three.js test was strengthened (thigh / shin / pelvis boxes and a torso waist band) and v19 was re-run under it: still PASS; see `docs/phase_v20_refine.md`.*
