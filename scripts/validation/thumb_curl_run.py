"""thumb_curl_run.py -- SCRIPT_VERSION 1.0
Purpose: run thumb_curl_export_test.py on a v18+ .blend (exports through export_glb_baked.py so the exact groove booleans are baked once).
Usage (three headless processes, exit code 0 = pass; the control run must FAIL):
    python3 -I scripts/validation/thumb_curl_run.py export BLEND GLB REF.json [LEFT_ABD_SIGN=-1]
    python3 -I scripts/validation/thumb_curl_run.py check  GLB REF.json
    # control: export with LEFT_ABD_SIGN=1, then check -> must report FAIL
"""
import json
import os
import sys

import bpy

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
T = os.path.join(ROOT, "scripts", "validation", "thumb_curl_export_test.py")
ns = {"__name__": "thumb_curl_export_test"}
exec(compile(open(T).read(), T, "exec"), ns)
a = [x for x in sys.argv[1:] if not x.endswith(".py")]
if a[0] == "export":
    bpy.ops.wm.open_mainfile(filepath=a[1])
    r = ns["run_export_side"](os.path.join(ROOT, "scripts", "build", "export_glb_baked.py"), a[2], a[3], abd_left_sign=int(a[4]) if len(a) > 4 else -1)
    print("THUMBEXPORT", json.dumps({"blender_mirror_error_m": r["blender_mirror_error_m"], "abd_left_sign": r["abd_left_sign"]}))
elif a[0] == "check":
    sys.exit(0 if ns["check"](a[1], a[2]) else 1)
else:
    raise SystemExit(__doc__)
