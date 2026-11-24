"""export_glb_baked.py -- SCRIPT_VERSION 1.0

Purpose: drop-in replacement for export_glb_def_only.py (same `export_def_only(out, prepare=None, reload=True)` signature) for v18+
files whose grooved meshes carry exact BOOLEAN modifiers. It bakes those meshes once, in memory, before the caller's own `prepare`
runs, so the per-frame bake / sampling never re-evaluates a boolean (see scripts/build/export_v18.py for the reasoning).
Used as the `export_script` argument of thumb_curl_export_test.run_export_side. Exec it like export_glb_def_only.py.
"""
import importlib.util
import os

import bpy

SCRIPT_VERSION = "1.0"
# exec'd (no __file__): the repo root is the parent of the folder holding the saved .blend (<root>/blender/*.blend)
_ROOT = os.path.dirname(os.path.dirname(bpy.data.filepath)) if not globals().get("__file__") else \
    os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
_EXPORTER = os.path.join(_ROOT, "scripts", "build", "export_glb_def_only.py")
_V18 = os.path.join(_ROOT, "scripts", "build", "export_v18.py")

_ns = {"__name__": "export_glb_def_only"}
exec(compile(open(_EXPORTER).read(), _EXPORTER, "exec"), _ns)
_spec = importlib.util.spec_from_file_location("export_v18_mod", _V18)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

_orig = _ns["export_def_only"]
validate_glb = _ns["validate_glb"]
read_glb_json = _ns["read_glb_json"]


def export_def_only(out_path, prepare=None, reload=True):
    def both(arm):
        _mod.bake_grooved_meshes(arm)
        if prepare is not None:
            prepare(arm)
    return _orig(out_path, prepare=both, reload=reload)
