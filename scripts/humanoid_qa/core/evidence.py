"""Hashing, provenance, JSON output and the append-only evidence ledger."""

import datetime
import hashlib
import json
import math
import os


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def file_identity(path):
    st = os.stat(path)
    return {
        "path": os.path.abspath(path),
        "bytes": st.st_size,
        "mtime_utc": datetime.datetime.fromtimestamp(
            st.st_mtime, datetime.timezone.utc).isoformat(),
        "sha256": sha256_file(path),
    }


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def clean(x, digits=7):
    """Round floats, convert tuples, and make inf/nan JSON-safe."""
    if isinstance(x, bool) or x is None or isinstance(x, (int, str)):
        return x
    if isinstance(x, float):
        if math.isinf(x) or math.isnan(x):
            return str(x)
        return round(x, digits)
    if isinstance(x, dict):
        return {str(k): clean(v, digits) for k, v in x.items()}
    if isinstance(x, (list, tuple, set)):
        return [clean(v, digits) for v in x]
    return str(x)


def write_json(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(clean(data), f, indent=2, sort_keys=True)
        f.write("\n")


def sig(x, digits=9):
    """Round floats to `digits` significant figures (tiny values keep their magnitude)."""
    if isinstance(x, bool) or x is None or isinstance(x, (int, str)):
        return x
    if isinstance(x, float):
        if math.isinf(x) or math.isnan(x) or x == 0.0:
            return x if not (math.isinf(x) or math.isnan(x)) else str(x)
        return float(f"{x:.{digits}g}")
    if isinstance(x, dict):
        return {str(k): sig(v, digits) for k, v in x.items()}
    if isinstance(x, (list, tuple, set)):
        return [sig(v, digits) for v in x]
    return str(x)


def write_json_sig(path, data, digits=9):
    """JSON writer for quantities spanning many decades (volumes in m^3). Keeps `digits`
    significant figures instead of a fixed number of decimal places."""
    with open(path, "w", encoding="utf-8") as f:
        json.dump(sig(data, digits), f, indent=2, sort_keys=True)
        f.write("\n")


def append_ledger(ledger_path, entry):
    """Append exactly one line. Never rewrites existing lines."""
    os.makedirs(os.path.dirname(os.path.abspath(ledger_path)), exist_ok=True)
    with open(ledger_path, "a", encoding="utf-8") as f:
        f.write(json.dumps(clean(entry), sort_keys=True) + "\n")


def hash_outputs(run_dir, skip=()):
    out = {}
    for root, _, files in os.walk(run_dir):
        for name in sorted(files):
            p = os.path.join(root, name)
            rel = os.path.relpath(p, run_dir).replace("\\", "/")
            if rel in skip:
                continue
            out[rel] = {"sha256": sha256_file(p), "bytes": os.path.getsize(p)}
    return dict(sorted(out.items()))
