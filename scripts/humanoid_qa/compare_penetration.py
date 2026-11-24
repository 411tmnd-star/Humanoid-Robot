"""compare_penetration.py (compare_pen_v01.0)

  python scripts/humanoid_qa/compare_penetration.py <old_run_dir> <new_run_dir> <out_dir>

Reads penetration.json from two QA run directories and writes penetration_regression.json
and penetration_regression.md into a NEW output directory (an existing non-empty directory is
refused). Neither input is modified. Exit code 0 always when the comparison ran; the verdict
is in the document (volume_regressions, unexplained_separation_changes).
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from humanoid_qa.core import compare_pen, evidence  # noqa: E402


def main(argv):
    if len(argv) != 4:
        print(__doc__)
        return 2
    old_dir, new_dir, out = argv[1:]
    if os.path.exists(out) and os.listdir(out):
        print("output directory exists and is not empty:", out)
        return 1
    os.makedirs(out, exist_ok=True)
    old = json.load(open(os.path.join(old_dir, "penetration.json"), encoding="utf-8"))
    new = json.load(open(os.path.join(new_dir, "penetration.json"), encoding="utf-8"))
    c = compare_pen.compare(old, new)
    c["old_run"], c["new_run"] = os.path.basename(os.path.normpath(old_dir)), os.path.basename(os.path.normpath(new_dir))
    c["volume_regressions"] = len(c["volume"]["over_tol"])
    c["unexplained_separation_changes"] = len(c["separation"]["unexplained"])
    c["inputs"] = {"old_penetration_sha256": evidence.sha256_file(os.path.join(old_dir, "penetration.json")),
                   "new_penetration_sha256": evidence.sha256_file(os.path.join(new_dir, "penetration.json"))}
    evidence.write_json_sig(os.path.join(out, "penetration_regression.json"), c)
    with open(os.path.join(out, "penetration_regression.md"), "w", encoding="utf-8") as f:
        f.write(compare_pen.to_markdown(c, c["old_run"], c["new_run"]))
    print("volume_regressions", c["volume_regressions"], "unexplained_separation_changes", c["unexplained_separation_changes"],
          "changed", len(c["separation"]["changed"]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
