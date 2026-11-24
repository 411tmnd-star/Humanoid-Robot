"""Minimal GLB (glTF 2.0 binary) reader: JSON chunk only. Stdlib."""

import json
import struct


def read_glb_json(data):
    if len(data) < 20 or data[:4] != b"glTF":
        raise ValueError("not a GLB (bad magic)")
    version, total = struct.unpack_from("<II", data, 4)
    if version != 2:
        raise ValueError(f"unsupported GLB version {version}")
    if total != len(data):
        raise ValueError(f"GLB length field {total} != actual {len(data)}")
    clen, ctype = struct.unpack_from("<II", data, 12)
    if ctype != 0x4E4F534A:
        raise ValueError("first chunk is not JSON")
    return json.loads(data[20:20 + clen].decode("utf-8"))


def build_glb(gltf):
    """Build a GLB with only a JSON chunk (used by tests)."""
    raw = json.dumps(gltf).encode("utf-8")
    raw += b" " * ((4 - len(raw) % 4) % 4)
    return b"glTF" + struct.pack("<II", 2, 12 + 8 + len(raw)) + struct.pack("<II", len(raw), 0x4E4F534A) + raw


def skin_summary(gltf):
    """Joint names per skin, grouped by the project's layer prefix."""
    nodes = gltf.get("nodes", [])
    out = []
    for i, sk in enumerate(gltf.get("skins", [])):
        names = [nodes[j].get("name", f"<node {j}>") for j in sk.get("joints", [])]
        layers = {}
        for n in names:
            head = n.split("_", 1)[0]
            layers[head if head in ("DEF", "MECH", "CTRL") else "OTHER"] = \
                layers.get(head if head in ("DEF", "MECH", "CTRL") else "OTHER", 0) + 1
        out.append({"skin": i, "joint_count": len(names), "layers": layers, "joints": names})
    return {"skin_count": len(out), "skins": out, "node_count": len(nodes),
            "mesh_count": len(gltf.get("meshes", [])),
            "animation_count": len(gltf.get("animations", []))}
