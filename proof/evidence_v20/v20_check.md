# v20 check (v20 vs v19)

## 1. What changed, by how much

| quantity | v19 | v20 | target | result |
|---|---|---|---|---|
| waist / chest at z 1.11 | 0.859 | 0.790 | 0.78-0.80 | PASS |
| rim / chest at z 0.975 | 0.921 | 0.830 | below v19 (hips not widened) | PASS |
| z of the narrowest torso width in 0.97-1.36 (m) | 1.22 | 1.11 | at the waist, 1.11 | PASS |
| width is monotone: narrowing 0.98 -> 1.11 and widening 1.11 -> 1.30 (no second bulge; hem bevel below z 0.98 excluded) | no | yes | yes | PASS |
| abdomen depth at waist / chest depth | 0.867 | 0.758 | flatter | PASS |
| chest half-width (m) | 0.2805 | 0.2801 | unchanged (+-0.5 %) | PASS |
| pelvis shell half-width x / y (m) | 0.2439 / 0.1720 | 0.2196 / 0.1529 | not wider | PASS |
| 411_Thigh_R diameter top / bottom (mm) | 161.8 / 120.4 | 154.5 / 115.0 | -4 to -5 % | PASS (-4.5 %, -4.5 %) |
| 411_Thigh_L diameter top / bottom (mm) | 161.8 / 120.4 | 154.5 / 115.0 | -4 to -5 % | PASS (-4.5 %, -4.5 %) |
| 411_Shin_R diameter upper (top) / ankle end (mm) | 125.8 / 88.1 | 120.2 / 88.1 | upper -4..-5 %, ankle unchanged | PASS (-4.5 %) |
| 411_Shin_L diameter upper (top) / ankle end (mm) | 125.8 / 88.1 | 120.2 / 88.1 | upper -4..-5 %, ankle unchanged | PASS (-4.5 %) |
| shin ankle / upper diameter (v17 taper rule 0.65-0.75) | 0.700 | 0.733 | 0.65-0.75 | PASS |
| robot height / bbox min / max | 2.353 / [-0.392, -0.202, 0.0] / [0.392, 0.199, 2.353] | 2.353 / [-0.392, -0.202, 0.0] / [0.392, 0.199, 2.353] | height exact, extents within 0.5 mm | PASS (largest extent change 0.19 mm) |
| hip socket centre x L / R (m) | -0.1380 / 0.1380 | -0.1380 / 0.1380 | unchanged | PASS |

## 2. What must NOT change (identical to v19)

| set | compared | max difference | result |
|---|---|---|---|
| all 184 bones (DEF/MECH/CTRL): head, tail, rest-frame axes | 184 | 0.00e+00 | PASS |
| constraints, limits, influences, lock flags, all pose bones | 184 | 0 differences | PASS |
| base vertices of the other 115 meshes (hands, fingers, head, neck, feet, visor, yoke, hip sockets, arms, ...) | 20787 | 1.49e-08 (411_AnkleCuff_L) | PASS |
| modifier stacks of all meshes | 123 | 0 differences | PASS |

**V20 CHECKS PASSED**
