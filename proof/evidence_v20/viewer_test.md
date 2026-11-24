# Three.js viewer test

three 0.170.0, headless Chromium, renderer: ANGLE (Google, Vulkan 1.3.0 (SwiftShader Device (Subzero) (0x0000C0DE)), SwiftShader driver)

| check | result | detail |
|---|---|---|
| GLB loads in GLTFLoader without page errors | PASS | no errors |
| WebGL2 context available | PASS | ANGLE (Google, Vulkan 1.3.0 (SwiftShader Device (Subzero) (0x0000C0DE)), SwiftShader driver) |
| 56 skeleton joints, all DEF_ | PASS | 56 joints, non-DEF 0 |
| every mesh is skinned (SkinnedMesh) with skin attributes | PASS | 129 skinned meshes |
| no NaN vertex positions; every vertex has unit skin weight | PASS | 149655 vertices, 0 not unit, 0 NaN |
| one animation clip driving 56 nodes | PASS | Animation: 168 tracks, 56 nodes, 2.667 s |
| clip duration equals the Blender action length | PASS | 2.667 s vs 2.667 s |
| bone world positions match Blender (all DEF bones, every 2nd frame) | PASS | 2296 comparisons, worst 1.79e-6 m at 10:DEF_Little_Tip |
| skinned mesh bounding boxes match Blender (torso, head, hands, thigh, shin, pelvis, feet, visor) | PASS | 60 boxes, worst 1.10e-6 m at 10:411_Index_Knuckle2, missing 0 |
| torso waist band (z 1.05-1.17) matches Blender: skinned vertex extents | PASS | 1590 vertices, worst 3.19e-7 m (waist half-width 0.2248 m) |
| robot height is plausible (2.2-2.5 m) | PASS | 2.335 m |
| WebGL render shows the robot | PASS | 5.2 % foreground pixels |
| pose changes between frame 0 and frame 10 in the rendered image | PASS | 2576 pixels changed |

Screenshots: threejs_frame00.png (rest), threejs_frame10.png (hello_sequence peak).

**ALL VIEWER CHECKS PASSED**
