# humanoid_qa — 411 Humanoid measurement & QA framework (v01)

Read-only. Never saves a `.blend`. Builds on, and does not replace,
`scripts/validation/humanoid_inspect_v01.py` (the baseline inspector).

## Run

```powershell
powershell -ExecutionPolicy Bypass -File scripts\humanoid_qa\run_qa.ps1
# options: -Blender <exe> -Source <file.blend> -Python <python> -NoRender -BaselineRun <run id>
```

Steps: pure unit tests (system Python, stdlib only) → Blender synthetic-fixture
tests (factory startup, no project file) → `run_qa.py` on the source → regression
compare against the previous QA run (`blender/inspection/comparisons/`) → optional
`-Experiments` (`run_experiments.py`: pose-response and glTF-export probes on a
disposable COPY; the source is only read). Exit code: 1 = a step failed,
2 = the comparison found regressions, 0 = clean. Detected design anomalies never
change the exit code. Output:
`blender/inspection/humanoid_qa_v01_<tag>_<UTC stamp>/` plus one appended line
in `blender/inspection/evidence_ledger.jsonl`. Earlier runs are never touched.

Tests alone:

```
python scripts/humanoid_qa/tests/test_core.py [results.json]
blender --background --factory-startup --python-exit-code 1 --python scripts/humanoid_qa/tests/blender_fixture_tests.py -- [results.json]
```

## Layout

| path | role | needs bpy |
|---|---|---|
| `core/geom.py` | vectors, transforms, bounds, exact k-d tree NN, symmetry, topology | no |
| `core/rig.py` | hierarchy, side parsing/pairing, pair metrics, thumb adjacency, skinning conditions, constraint refs | no |
| `core/classify.py` | object classification, finger-mesh name canonicalisation | no |
| `core/rules.py` | config loading, finding schema, method qualification from test results | no |
| `core/evidence.py` | SHA-256, file identity, JSON output, append-only ledger | no |
| `core/gate.py` | side-dependent answers return `UNRESOLVED` until the owner resolves the convention | no |
| `core/compare.py` + `compare_runs.py` | regression diff of two run directories | no |
| `core/glb.py` | GLB JSON reader / skin-joint summary | no |
| `core/intersect.py` | triangle-pair classifier, TriBVH, mesh self-/inter-mesh intersection, exact proximity, exact rational oracle | no |
| `blender/experiments.py` + `run_experiments.py` | isolated copy, pose response, export probe | yes |
| `blender/extract.py` | scene → plain dicts (base and evaluated geometry kept separate) | yes |
| `blender/diagnostics.py` | ortho front/side/top renders with axis markers (in memory) | yes |
| `config/quality_rules.json` | classification rules, tolerances, coordinate declaration, severities, method registry | — |
| `tests/` | `test_core.py` (U-*), `blender_fixture_tests.py` (B-*) | B-* only |
| `run_qa.py` / `run_qa.ps1` | Blender entry point / Windows pipeline | yes / — |

## Run directory contents

`summary.md`, `findings.json`, `coordinate_convention.json` (incl. decision
package), `side_mapping.json`, `measurement_methods.json`, `test_results.json`
and `tests/*`, `geometry.json`, `rig.json`, `unresolved_questions.json`,
`diagnostic_views.json` + `diag_*.png`, `run_manifest.json` (source identity
before/after, script and output hashes).

## Vocabulary

Evidence classes: `SOURCE_FACT`, `MEASURED_FACT`, `VERIFIED_RESULT`,
`HYPOTHESIS`, `PROPOSED_RULE`, `OWNER_DECISION_REQUIRED`, `UNKNOWN_NOT_MEASURED`.

Rule categories: `A_HARD_INVARIANT` (contract or mathematical), `B_SPEC_CONSTRAINT`
(cites a spec line), `C_ENGINEERING_DIAGNOSTIC` (possible problem, not proof),
`D_PROPOSED_DESIGN_RULE`, `E_HUMAN_ACCEPTANCE`. No combined quality score.

Severity (impact and urgency, not size of a number) — defined in
`config/quality_rules.json` → `severity_definitions`: BLOCKER, HIGH, MEDIUM,
LOW, INFO.

Method status is **derived per run** from the tests that actually ran
(`rules.qualify_methods`): a declared-QUALIFIED method whose test failed becomes
`FAILED_VALIDATION`; one whose test did not run becomes `EXPERIMENTAL`.
`rules.validate_finding` refuses a `MEASURED_FACT`/`VERIFIED_RESULT` finding
that rests on a method not (conditionally) qualified.

Tolerances always carry `value`, `unit`, `kind` (numerical / geometric_matching
/ engineering_diagnostic / algorithm_parameter), `status` (JUSTIFIED /
PROVISIONAL) and `basis`. PROVISIONAL tolerances never support a definitive
defect claim.

## Mesh intersection and proximity (`mesh_analysis.json`)

Base (unmodified) body-mesh triangles in world space (Blender loop-triangle tessellation), metres.
Triangle relation: `degenerate` / `coplanar_overlap` / `intersect` (proper crossing deeper than
`tri_contact`) / `contact` (touching or shallower than the tolerance) / `disjoint`. Self-intersection
filters ordinary adjacency using welded vertex ids. Pair status: `DEFINITE_INTERSECTION`,
`POSSIBLE_REQUIRES_INSPECTION`, `NONE_DETECTED`, `UNABLE_TO_DETERMINE`. A run with no findings
means none were *detected* by these methods; it is never a clean bill of health. Proximity is a raw
distance (m) with the triangles and world points involved; the 0.02 m search radius is an
EXPERIMENTAL reporting scope, not a requirement. Blender's `BVHTree.overlap` applies its own float32
triangle test (pinned by B-INT-04), so it is used as an independent implementation; every
disagreement is adjudicated by the exact rational oracle (`exact_proper_crossing`).

## Isolated experiments

`run_experiments.py` copies the source to a temp directory, opens the COPY (never
saves), unassigns the action there, poses each CTRL flex-type bone, measures the
evaluated mesh displacement, restores and verifies the restore, then exports the
copy twice (all bones / deform-only) and reads the GLB skin joints. Responses are
reported by physical side only; anatomical side goes through `core/gate.py`.

## Coordinate convention gate

`coordinate_convention.side_dependent_adjudication_allowed` is `false` until
the owner decides how side labels are meant (see the latest
`coordinate_convention.json`). Label-independent measurements run; renames,
"which side is correct" calls and anatomical deformation tests do not.

## Adding a method

1. Pure function in `core/` with an analytic unit test (and a fixture test if
   it touches bpy behaviour).
2. Register it in `config/quality_rules.json → methods` with definition, units,
   frame, limits and `validated_by` test IDs.
3. Use it from `run_qa.py`; findings cite its method ID.
