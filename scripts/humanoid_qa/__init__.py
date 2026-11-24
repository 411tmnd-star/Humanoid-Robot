"""humanoid_qa — read-only measurement, qualification and QA framework for the
411 Humanoid.

Layout (see README.md):
  core/     pure Python (stdlib only) — runs under system Python 3.11 and
            Blender's bundled 3.13; no bpy import anywhere in core/
  blender/  thin bpy adapters that turn a scene into plain dicts
  config/   versioned rules, tolerances and the coordinate declaration
  tests/    unittest suite (pure) + Blender synthetic-fixture tests
  run_qa.py Blender entry point that writes a run directory + ledger event
"""

QA_VERSION = "humanoid_qa_v01.0"
SCHEMA_VERSION = "hqa-schema-1"
