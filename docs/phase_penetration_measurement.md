# Phase: solid overlap measurement (humanoid_qa, solid_v01)

Status of this document: method definitions and audit. Numbers come from the run
recorded in `blender/inspection/humanoid_qa_v01_v15_*/penetration.json`.

Scope: body meshes (`411_*`) only. Environment and presentation meshes are excluded.
Source: `blender/robot_armature_v15.blend` (read-only; SHA-256
`0946c7d469e13dbff0c8b207e565f3cd78c608dd434863e68e6d5f3c1e0a2cfa`).

---

## 1. What the existing `1e-6 m` depth guard measures

Code: `scripts/humanoid_qa/core/intersect.py` (`crossing_depth`, `_side_depth`,
`classify_triangle_pair`, lines ~280-358).

For two triangles T1, T2 whose edge crosses the interior of the other triangle:

- `d2_i` = signed distance of T2's vertices from T1's plane (normal n1).
- `d1_j` = signed distance of T1's vertices from T2's plane (normal n2).
- `_side_depth(ds)` = `min(max(ds), -min(ds))` if the vertices straddle the plane
  (`min < 0 < max`), else 0. It is how far the *lesser side* of one triangle reaches
  past the other triangle's plane.
- `crossing_depth = min(_side_depth(d2), _side_depth(d1))`.
- The pair is `intersect` when `crossing_depth > tri_contact (1e-6 m)`, otherwise `contact`.

**What the value is:** a triangle-pair geometric proxy: how far one triangle's
vertices extend through the other triangle's plane, at the first proper crossing.
Units m. It is a *numerical classification guard*: `1e-6 m` is `tri_contact`, equal to
`coincident_vertex` and to float32 precision at ~1 m (`quality_rules.json`,
`tri_contact`, status JUSTIFIED).

**What it is not:**

- Not a numerical tolerance on the *solid* (it is a triangle-pair quantity).
- Not a penetration depth. It does not measure how far one closed solid is inside
  another. Its value depends on the triangles' sizes and dihedral angle, not on the
  volume of overlap. Two overlaps with the same volume can give different values, and
  a large triangle count can come from a small volume (fine tessellation of a shallow
  overlap).
- Not monotone in overlap severity. A deep overlap with a shallow crossing angle can
  report `contact`.

**Decision:** the value stays as the contact/intersect boundary. It is not reused as a
depth measurement, and `tri_contact` is not changed. The new measurements below are
separate quantities with their own tolerances.

---

## 2. Methods adopted

Pure-Python core (`core/solid.py`, version `solid_v01.0`) and Boolean-kernel backend
(`blender/volume.py`, `volume_v01.0`). Only the Blender kernel handles non-convex shells.

### 2.1 Gating (refusal, never a value)

A pair is refused as `UNABLE_TO_DETERMINE` with reasons if either operand has:
`OPEN_BOUNDARY`, `NON_MANIFOLD`, `DEGENERATE_FACE` (area <= 1e-12 m^2),
`WINDING_INCONSISTENT` (an edge shared by two faces traversed in the same direction),
`EMPTY`, or a component with |signed volume| <= `penetration_null_volume`.

### 2.2 Orientation (in-memory copies only)

Each connected component of an operand is reversed when its own signed volume
(divergence theorem, `geom.signed_volume`) is negative. Reversal changes which side is
outside. It does not change the enclosed region, so the volume is unchanged. The
source triangles and the .blend are never modified.

Evidence for this step: `411_PelvicShell` is one closed component with signed volume
-0.01667 m^3 (inverted). `411_WaistRing_New` is an inverted 512-triangle shell plus 40
positive 12-triangle closed parts. These are the shells named by the existing MEDIUM
finding QA-009, which is not changed.

### 2.3 Overlap volume: `M-SOL-VOLUME`

`V(A n B)` by Boolean INTERSECT on the oriented copies (units m^3), read back from the
evaluated result with the divergence theorem. Validated when all of these hold:

| check | definition | tolerance |
|---|---|---|
| kernel agreement | `|V_MANIFOLD - V_EXACT| / max(V)` | 1e-4 (`volume_solver_agreement_rel`) |
| operand symmetry | `|V(A n B) - V(B n A)| / max(V)` | 1e-4 (same) |
| identity | `|V(A n B) + V(A minus B) - V(A)| / |V(A)|` | 1e-4 (`volume_identity_rel`) |
| zero overlap | `V <= 1e-12 m^3` (`penetration_null_volume`, PROVISIONAL) | reported as `NULL_OVERLAP` |

Pure convex backend (`ConvexBackend`): exact polyhedron clipping (Sutherland-Hodgman
with cap construction). Used as an oracle on convex fixtures; it refuses non-convex
operands rather than approximating them.

### 2.4 Axial depth bound: `M-SOL-DEPTH-AXIAL`

For each world axis direction `d` in {+X, -X, +Y, -Y, +Z, -Z}:

- `T_d = max_B(d.x) - min_A(d.x)` (translating A by T_d along d leaves zero overlap).
- Bisection on `t in [0, T_d]` for the overlap volume of `A + t d`, 18 halvings
  (resolution `T_d / 2^18`, <= ~4 um for bodies below 1 m). Every returned `t` is a
  *verified* separating translation (its volume was evaluated and is <= null).
- `axial_depth_bound = min over d of t*(d)`, with its direction.

**Claim, stated precisely:** a verified separating translation has length >= the
minimum translation distance (MTD). Therefore `axial_depth_bound >= MTD`, an **upper
bound**. For axis-aligned boxes the minimum over the six directions equals MTD exactly
(separating-axis theorem). For general shapes it can exceed MTD; the direction giving
the least resistance may be diagonal. It is **not** an exact penetration depth.

The minimising translation is cross-checked against the other kernel. The depth has its
own status, separate from the volume verdict:

- `CONFIRMED_BY_ALL_KERNELS`: the other kernel's volume at the returned translation is
  <= `penetration_null_volume`.
- `PRIMARY_KERNEL_ONLY`: it is not (the bound is verified by MANIFOLD alone).
- `NOT_COMPUTED`: no direction verified (no bound is reported).

---

## 3. Analytic validation (ground truth from fixture geometry)

Pure fixtures: `tests/test_solid.py` (U-SOL-01..10, U-SOL-CFG).
Blender fixtures: `tests/blender_fixture_tests.py` (B-VOL-01..09).

| case | expected | source of expected value |
|---|---|---|
| cubes offset (0.5, 0.25, 0.125) | V = 0.328125 m^3 | 0.5 x 0.75 x 0.875 box |
| same cubes, six axial bounds | +X 1.5, -X 0.5, +Y 1.25, -Y 0.75, +Z 1.125, -Z 0.875 m | face-to-face distances |
| same cubes, depth bound | 0.5 m (-X) | min overlap extent = box MTD |
| containment (0.25..0.75 in 0..1) | V = 0.125 m^3, all six bounds 0.75 m | cube geometry |
| cube vs same cube rotated 45 deg about z | V = 2 sqrt(2) - 2 = 0.8284271247 m^3 | octagon area |
| reversed cube | V = 0.125 m^3, reversal reported | cube geometry |
| L-prism (volume 3) vs cube | V = 0.5625 m^3 (Blender); refused (pure) | 0.75 m^2 x 0.75 m |
| two components, one inverted | V = 0.125 m^3, component 1 reversed | cube geometry |
| face / edge / vertex / separated contact | V = 0, status NULL_OVERLAP | touching sets |
| open, non-manifold, degenerate, winding-inconsistent | UNABLE_TO_DETERMINE with reason | construction |

Accuracy: the pure backend is exact to double precision (fixture errors <= 1e-12 m^3).
The Boolean kernel is float32; the rotated fixture error is 2.6e-8 relative (2.2e-8
m^3), so it uses a 1e-6 m^3 tolerance justified by float32 position error (~1.2e-7 m
over ~1 m^2 of surface). Exactly representable fixtures keep 1e-9 m^3.

Limits found during validation (and fixed):

- Clipping counted a face lying exactly in a clipping plane twice (kept and capped).
  Fixed: coplanar faces are replaced by the cap, and a touching-only clip is empty.
  Regression: U-SOL-02, U-SOL-03.
- The default box builder produced inward faces; its fixtures were corrected by
  stating the winding convention in the builder.

---

## 4. Limitations (recorded per pair in `penetration.json`)

- Axial bounds are upper bounds, not exact depths, for non-box shapes.
- Multi-component operands are measured by the Boolean kernel as a union; components
  are assumed not to overlap each other (body self-intersection is zero in the
  existing analysis, which supports this for the measured meshes).
- The null volume (1e-12 m^3) is PROVISIONAL.
- Volumes use base mesh data; modifiers (evaluated geometry) are not applied.
- Coordinates are float32 in Blender; results carry ~1e-7 m^3 uncertainty.

---

## 5. Results (run `humanoid_qa_v01_v15_20261013T002429Z`)

Source: `blender/robot_armature_v15.blend`, SHA-256 unchanged. Full data:
`penetration.json`; readable summary: `penetration_summary.md` in the same run folder.

- 113 definite triangle-intersecting body pairs (QA-017, unchanged): **109 volume-validated**,
  **4 inconclusive** (refused: DEGENERATE_FACE on `411_Pelvis`, which has four zero-length
  edges that produce zero-area triangles; it is not repaired).
- Validation of the 109: kernel relative difference <= 3.4e-5 (tolerance 1e-4); operand
  symmetry <= 3.9e-5; identity error <= 3.0e-6.
- Depth: 108 `CONFIRMED_BY_ALL_KERNELS`; 1 `PRIMARY_KERNEL_ONLY`
  (`411_Finger2_L_A` / `411_Palm_L`: overlap 2.27e-8 m^3; the second kernel gives
  1.00004e-12 m^3 at the bound, sitting on the null threshold).
- Largest validated overlap volume: `411_PelvicShell` / `411_Thigh_L` 1.515e-3 m^3,
  axial bound 0.0873 m (+Z).
- Joint and housing pairs (intentional construction is possible; no threshold applied):
  `411_Neck`/`411_Yoke` 3.47e-4 m^3, bound 0.1045 m; `411_Shoulder_L`/`411_UpperArm_L`
  9.50e-4 m^3, bound 0.0907 m; `411_Elbow_L`/`411_Torso` 4.30e-5 m^3, bound 0.0194 m.
- `411_PelvicShell`/`411_WaistRing_New`: validated after reversing both inverted shells on
  copies; 1.084e-5 m^3; bound 1.69 mm (-Z), matching the overlap box thickness of 1.7 mm.
- Near-contact (no intersection): `411_Forearm_L`/`411_Torso` 0.220 mm,
  `411_Forearm_R`/`411_Torso` 0.222 mm (both reported; no defect implied).

## 6. Precision note (found and fixed during the run)

`evidence.clean()` rounds to 7 decimal places, which wrote volumes of order 1e-9 m^3 as
zero in an intermediate run. The penetration output now uses `evidence.write_json_sig`
(9 significant figures). The intermediate runs stay on disk as recorded; the authoritative
run is `humanoid_qa_v01_v15_20261013T002429Z`.
