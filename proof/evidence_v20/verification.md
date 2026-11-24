# v18 verification  (verify script 1.1, Blender 5.2.3 LTS)

## 1. Rig integrity and zero-command pose

| check | v17 | v18 | result |
|---|---|---|---|
| DEF bones | 56 | 56 | PASS |
| non-DEF bones with use_deform | 0 | 0 | PASS |
| armature pose_position as saved in the file | POSE | POSE | PASS |
| max zero-command deviation, all 56 DEF bones (deg) | 178.1636 | 0.0000 (worst: DEF_Pelvis) | PASS |
| max deviation, 38 finger / thumb / palm DEF bones (deg) | | 0.0000 | PASS |
| max deviation, wrist DEF bones (deg) | | 0.0000 | PASS |
| tolerance | | 1e-4 rad = 0.0057 deg | |
| POSE-space DEF roots whose MECH driver has a different rest frame | many (17-178 deg offsets) | 0 | PASS |

| shipped animation `hello_sequence` (action restored for this check) | value | result |
|---|---|---|
| action assigned on the armature in the .blend | hello_sequence | PASS |
| DEF bones that move between frames 0 and 10 | 22 | PASS |

## 2. Neutral hand pose (right and left)

| quantity | right | left | target | result |
|---|---|---|---|---|
| index MCP flexion (deg) | 12.00 | 12.00 | 12 | PASS |
| index PIP flexion (deg) | 18.00 | 18.00 | 18 | PASS |
| index DIP flexion (deg) | 10.00 | 10.00 | 10 | PASS |
| thumb CMC direction vs hand axis (deg) | 25.00 | 25.00 | 25 | PASS |
| thumb direction y (forward = negative) | -0.422 | -0.422 | < 0 | PASS |
| index chain curls toward the body (inward displacement, m) | 0.035 | 0.035 | > 0 | PASS |
| MCP pitch index-mid / mid-ring / ring-little (m) | 0.032, 0.032, 0.032 | 0.032, 0.032, 0.032 | 0.032 each | PASS |

| behaviour: +40 deg flex on the same CTRL channel of both hands | tip displacement right | tip displacement left | mirror check |
|---|---|---|---|
| Index MCP_Flex | (-0.069, -0.000, 0.051) | (0.069, -0.000, 0.051) | PASS |
| Index PIP | (-0.035, -0.000, 0.055) | (0.035, -0.000, 0.055) | PASS |
| Little MCP_Flex | (-0.067, -0.000, 0.047) | (0.067, -0.000, 0.047) | PASS |
| Thumb MCP | (-0.069, 0.017, 0.026) | (0.069, 0.017, 0.026) | PASS |

Hand segments: 28 phalanx bones (24 finger + 4 thumb); each has exactly one white shell and one dark cylindrical knuckle: 28 shells, 28 knuckles -> PASS. Thumbs also keep their CMC joint spheres.

## 3. Torso contour and panel seams

| quantity | v17 | v18 | target | result |
|---|---|---|---|---|
| chest half-width (m) | 0.2730 | 0.2801 | +pectoral breadth | PASS |
| waist / chest | 0.751 | 0.790 | 0.79 | PASS |
| lower iliac rim / chest | 0.810 | 0.830 | 0.83 | PASS |
| horizontal abdomen seam: height above bottom edge (m) | | 0.025 | 0.025 | PASS |
| horizontal seam groove depth at centre / 2.5 mm off-centre (mm) | | 1.53 / 0.00 | 1.5 / 0 | PASS |
| curved seam at z = 1.28: deepest point x (m) / depth (mm) / width at >0.4 mm depth (mm) | | 0.169 / 1.50 / 1.6 | x/W = 0.70 / 1.5 / 2.0 | PASS |
| seam x / half-width at z = 1.28 | | 0.697 | 0.70 | PASS |

| evaluated mesh | vertices | unweighted vertices | material slots used | result |
|---|---|---|---|---|
| 411_Torso | 6270 | 0 | [0, 1] | PASS |
| 411_Forearm_R | 290 | 0 | [0, 1] | PASS |
| 411_Forearm_L | 290 | 0 | [0, 1] | PASS |
| 411_Shin_R | 290 | 0 | [0, 1] | PASS |
| 411_Shin_L | 290 | 0 | [0, 1] | PASS |
| 411_Head | 26841 | 0 | [0, 1, 2] | PASS |

## 4. Limb split seams and head

| quantity | measured | target | result |
|---|---|---|---|
| 411_Forearm_R: front seam depth at mid-length centre / 2 mm off-centre (mm) | 1.22 / -0.00 | 1.2 / 0 | PASS |
| 411_Forearm_L: front seam depth at mid-length centre / 2 mm off-centre (mm) | 1.22 / 0.00 | 1.2 / 0 | PASS |
| 411_Shin_R: front seam depth at mid-length centre / 2 mm off-centre (mm) | 1.21 / -0.00 | 1.2 / 0 | PASS |
| 411_Shin_L: front seam depth at mid-length centre / 2 mm off-centre (mm) | 1.21 / -0.00 | 1.2 / 0 | PASS |
| visor pocket depth at centre (mm) | 1.50 | 1.5 | PASS |
| visor chamfer: horizontal width between floor edge and shell surface (mm) | 1.70 | 1.5 | PASS |
| head coronal seam depth at the apex / 2.5 mm off-centre (mm) | 1.51 / 0.00 | 1.5 / 0 | PASS |

## 5. Pelvis, hips, yoke, feet

| quantity | v17 | v18 | target | result |
|---|---|---|---|---|
| yoke vertical thickness (m) | 0.1521 | 0.1287 | -18 % (+4 mm sink) | PASS |
| yoke lowest point z (m) | 1.5879 | 1.6122 | 1.6122 (compress, then -4 mm) | PASS |
| hip socket L centre x (m), shift as fraction of v17 radius | -0.1524 | -0.1380 | -20 % of radius inward | PASS (shift/r = 0.200) |
| hip socket R centre x (m), shift as fraction of v17 radius | 0.1524 | 0.1380 | -20 % of radius inward | PASS (shift/r = 0.200) |
| pelvis shell: lowest height change at the centre bridge (mm) | 0 | -12.0 | -12 (dip) | PASS |
| pelvis shell: largest height change at the outer hips (mm) | 0 | +6.1 | positive (rise, max +14 at the lowest ring) | PASS |
| pelvis shell half-width / torso rim half-width | 0.936 | 0.944 | <= 1 (tucked in) | PASS |
| foot L: rubber sole height / bottom z (m) | | 0.0080 / 0.0000 | 0.008 / 0 | PASS |
| foot L: dorsal edge radius (wall half-width - deck half-width, m) | | 0.0180 | 0.018 (toe taper excluded: x) | PASS |
| foot L: shell bottom sits on the sole / ankle cuff present | | 0.0080 / True | 0.008 / True | PASS |
| foot R: rubber sole height / bottom z (m) | | 0.0080 / 0.0000 | 0.008 / 0 | PASS |
| foot R: dorsal edge radius (wall half-width - deck half-width, m) | | 0.0180 | 0.018 (toe taper excluded: x) | PASS |
| foot R: shell bottom sits on the sole / ankle cuff present | | 0.0080 / True | 0.008 / True | PASS |

## 6. Cyclorama and lighting

| item | value | target | result |
|---|---|---|---|
| Studio_Cyc exists, old two-plane backdrop hidden from render | True, hidden [True, True] | cyc + both hidden | PASS |
| max normal break between adjacent cyc faces (deg) | 2.81 | smooth (< 5; a plane-plane corner is 90) | PASS |
| floor-to-wall bevel radius (m) | 2.5 (profile: first rise at z=0.003, wall starts at z=2.5) | 2.5 | PASS |
| studio lights | ['Studio_EdgeL', 'Studio_EdgeR', 'Studio_Fill', 'Studio_Key', 'Studio_Rim'] | key, fill, rim + 2 edge rims | PASS |
| Cycles samples / denoiser | 128 / OPENIMAGEDENOISE | 128 / OPENIMAGEDENOISE | PASS |

## 7. glTF export

| check | value | target | result |
|---|---|---|---|
| skin joints | 56 | 56 | PASS |
| skinned meshes | 122 | v17 81 + new objects | PASS |
| 411_Torso vertices in the .glb (grooves baked in) | 5580 | > 3000 (cage has 1056) | PASS |
| file size | 9.09 MB | | |

## 8. Export -> reimport trajectory check

`RESULT: PASS (tolerance 0.0001 m, at least 3 moving bones, max error 2.560e-06 m)`

max error 2.560e-06 m against the < 1e-5 m requirement: PASS

## 9. front.png vs zerocmd.png

mean absolute difference 0.0006 / 255; pixels differing by more than 24/255: 0 of 1170000 -> PASS

## Overall

**ALL HARD CHECKS PASSED**
