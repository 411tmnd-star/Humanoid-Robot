"""Boolean-kernel volume backend for core/solid.py (requires bpy).

Creates IN-MEMORY copies of two oriented closed shells inside a temporary collection,
evaluates Boolean modifiers on them and removes every temporary object, mesh and
collection afterwards. The source .blend is never modified or saved.

Kernels: "MANIFOLD" (fast; primary, used for the axial bisection) and "EXACT"
(independent cross-check). Kinds: "AB" = A n B, "BA" = B n A, "AMB" = A minus B.
Volumes are computed from the evaluated result with the divergence theorem
(core/geom.signed_volume), so they are in cubic metres when the scene is METRIC
with scale_length 1.0 (checked by run_qa before this backend is used).
"""

import bpy

from ..core import geom

VERSION = "volume_v01.0"
SOLVERS = ("MANIFOLD", "EXACT")
_KIND = {"AB": ("A", "B", "INTERSECT"), "BA": ("B", "A", "INTERSECT"), "AMB": ("A", "B", "DIFFERENCE")}


class BooleanBackend:
    """Context-managed pair of temporary Blender objects. Use as a backend factory:

        solid.measure_pair(ca, ta, cb, tb, BooleanBackend.factory(tag), params)
    """

    solvers = SOLVERS

    def __init__(self, coords_a, tris_a, coords_b, tris_b, tag):
        scene = bpy.context.scene
        self._tag = tag
        self._coll = bpy.data.collections.new(f"__hqa_vol_{tag}")
        scene.collection.children.link(self._coll)
        self._objs = {}
        self._meshes = {}
        self._obj = {"A": self._make("A", coords_a, tris_a), "B": self._make("B", coords_b, tris_b)}

    @classmethod
    def factory(cls, tag):
        def make(coords_a, tris_a, coords_b, tris_b):
            return cls(coords_a, tris_a, coords_b, tris_b, tag)
        return make

    def _make(self, label, coords, tris):
        name = f"__hqa_{label}_{self._tag}"
        me = bpy.data.meshes.new(name)
        me.from_pydata([tuple(c) for c in coords], [], [tuple(t) for t in tris])
        me.update()
        ob = bpy.data.objects.new(name, me)
        self._coll.objects.link(ob)
        self._objs[label] = ob
        self._meshes[label] = me
        return ob

    def volume(self, kind, solver, shift=(0.0, 0.0, 0.0)):
        """Overlap volume for `kind` with A translated by `shift` (metres)."""
        host_k, operand_k, op = _KIND[kind]
        if solver not in SOLVERS:
            raise ValueError(f"unknown solver {solver}")
        self._obj["A"].location = tuple(shift)
        host, operand = self._obj[host_k], self._obj[operand_k]
        mod = host.modifiers.new("hqa_bool", "BOOLEAN")
        try:
            mod.operation = op
            mod.operand_type = "OBJECT"
            mod.object = operand
            mod.solver = solver
            bpy.context.view_layer.update()
            ev = host.evaluated_get(bpy.context.evaluated_depsgraph_get())
            me = ev.to_mesh()
            try:
                co = [tuple(v.co) for v in me.vertices]
                faces = [tuple(p.vertices) for p in me.polygons]
                return geom.signed_volume(co, faces) if faces else 0.0
            finally:
                ev.to_mesh_clear()
        finally:
            host.modifiers.remove(mod)
            self._obj["A"].location = (0.0, 0.0, 0.0)

    def close(self):
        """Remove every temporary object, mesh and collection created here."""
        for ob in list(self._objs.values()):
            bpy.data.objects.remove(ob)
        for me in list(self._meshes.values()):
            bpy.data.meshes.remove(me)
        if self._coll.name in bpy.data.collections:
            bpy.context.scene.collection.children.unlink(self._coll)
            bpy.data.collections.remove(self._coll)
        self._objs, self._meshes = {}, {}
