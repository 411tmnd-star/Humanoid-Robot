"""finalize_proof.py -- check proof/ against proof/PROOF_MANIFEST.json and fill in measured values. Stdlib only.

  python scripts/showcase/finalize_proof.py --write   after the renders are copied into proof/
  python scripts/showcase/finalize_proof.py --check   fails if anything is missing, changed, too big or still pending
"""
import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PROOF = ROOT / "proof"
MANIFEST = PROOF / "PROOF_MANIFEST.json"
LIMIT = 2_000_000


def sha256(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def load(name):
    p = PROOF / name
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    man = json.loads(MANIFEST.read_text(encoding="utf-8"))
    render, compose = load("showcase_report.json"), load("compose_report.json")
    problems = []
    for e in man["files"]:
        p = PROOF / e["file"]
        if not p.exists():
            problems.append(f"missing: {e['file']}")
            e["status"] = "PENDING_RENDER"
            continue
        size, digest = p.stat().st_size, sha256(p)
        if size >= LIMIT:
            problems.append(f"over 2 MB: {e['file']} ({size})")
        if a.write:
            e.update(status="PRESENT", bytes=size, sha256=digest)
        elif e.get("sha256") != digest:
            problems.append(f"hash differs from the manifest: {e['file']}")
    for p in PROOF.rglob("*"):
        if p.is_file() and p.stat().st_size >= LIMIT:
            problems.append(f"over 2 MB: {p.relative_to(PROOF).as_posix()}")
    gates = render.get("gates", {})
    if gates:
        hero = gates.get("hero_framing_mask", {}).get("fill_height")
        man["measured"] = {
            "blender": render.get("blender"), "device": render.get("device"),
            "hero_fill_height": hero,
            "wave_fill_height": round(gates["wave_framing_geometry"]["fill_height"], 4),
            "turntable_fill_height": {k: round(v["fill_height"], 4) for k, v in gates["turntable_framing_geometry"]["panels"].items()},
            "wave_hand_travel_m": gates["wave_hand_travel"]["metres"],
            "all_gates_pass": bool(render.get("pass")) and bool(compose.get("pass")),
            "timing_s": render.get("timing_s"),
        }
        if not man["measured"]["all_gates_pass"]:
            problems.append("a render or compose gate failed")
    else:
        problems.append("proof/showcase_report.json is missing")
    refs = re.findall(r"\((proof/[^)]+)\)", (ROOT / "README.md").read_text(encoding="utf-8"))
    problems += [f"README links a missing file: {r}" for r in refs if not (ROOT / r).exists()]
    if a.write:
        MANIFEST.write_text(json.dumps(man, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    print("\n".join(problems) if problems else "PROOF OK")
    if a.check and problems:
        sys.exit(1)


if __name__ == "__main__":
    main()
