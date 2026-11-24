"""Compare two QA run directories (pure Python, no Blender).

  python scripts/humanoid_qa/compare_runs.py <earlier_run> <later_run> [out.json]

Exit code 0 = no regressions, 2 = regressions, 1 = usage/IO error.
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from humanoid_qa.core import compare, evidence  # noqa: E402


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 1
    a, b = compare.load_run(argv[1]), compare.load_run(argv[2])
    res = compare.compare(a, b)
    if len(argv) > 3:
        evidence.write_json(argv[3], res)
    print(json.dumps({k: res[k] for k in ("a", "b", "same_source_hash", "verdict")}, indent=1))
    for r in res["regressions"]:
        print("REGRESSION:", r)
    return 2 if res["regressions"] else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
