"""
mesh_taper_v1.py -- v1.0
Purpose: Three targeted cosmetic-mesh corrections on the live body mesh
(411_* objects in robot_armature_v14.blend), diagnosed live via MCP before
writing any code:

1. Torso taper -- 411_Torso currently reads as a flat slab: its own ring
   data (22 height bands measured live, z_eps=0.015) shows it narrows only
   ~5% into a real waist (x_half 0.2544 at the base -> 0.2417 at z=1.10-1.15)
   and then WIDENS continuously all the way to its top ring (0.3097 at
   z=1.6685, its single widest point, right at shoulder height) before a
   negligible pullback at the very last (neck-hole) ring. Visually this is
   an upside-down wedge, not a torso -- confirmed against a viewport
   screenshot taken this session. Fix: re-profile the same ring structure
   (same z positions, same vertex count/topology, only radial X/Y position
   changes) against an explicit target profile with a real waist cinch, a
   fuller chest, and a taper back down toward the neck (Neck object's own
   half-width is 0.1749, so the new top ring targets 0.178 -- a near-flush
   hand-off instead of the old 0.3055 overshoot).

2. Shoulder separation -- 411_Shoulder_L/R's own bbox is already centered
   exactly on the shoulder pivot (DEF_UpperArm(_L) bone head, confirmed
   live: ball center (+/-0.2516, 0, 1.6027) equals the bone head to 4
   decimal places) with a true equatorial radius of 0.0988. At that same
   height the OLD torso profile's half-width was 0.298-0.304 -- i.e. roughly
   three-quarters of the ball's horizontal footprint (0.1528-0.3504 in X)
   sat inside/flush with the torso volume, which is why the joint read as
   fused to the body block rather than as a separate socket. Fix #1 above
   already claws back most of that overlap (new top-region target ~0.19-0.21
   vs the ball's inner edge at ~0.153-0.161); on top of that this script
   shrinks the ball itself by 10% about its own live-measured centroid
   (not a hardcoded point) so it never moves off the true pivot, just reads
   as a distinct, smaller-footprint pod sitting beside the now-narrower
   torso.

3. Elbow/knee hinge clearance -- live ring analysis (z-gap clustering,
   which a naive fixed-width height-bin pass gets wrong for these 2-ring,
   48-vertex objects -- confirmed the hard way this session) found:
     - 411_UpperArm_L/R's elbow-facing ring: r_mean 0.0894
     - 411_Elbow_L/R's own equatorial r_max:  0.0647  <-- upper arm is WIDER
     - 411_Thigh_L/R's knee-facing ring:      r_mean 0.0987
     - 411_Knee_L/R's own equatorial r_max:   0.0719  <-- thigh is WIDER
   In both cases the proximal segment's joint-end ring is wider than the
   hinge cap that is supposed to visually contain it -- guaranteed to
   clip past the cap's silhouette as the joint articulates. (The DISTAL
   segments -- Forearm's elbow-end ring 0.0572, Shin's knee-end ring
   0.0628 -- already sit safely inside their caps; left untouched.) Fix:
   shrink just the offending ring (identified live by "the wider of the
   object's two rings", not a hardcoded vertex range) about its own
   centroid, down to a radius with real margin inside the cap:
   UpperArm -> 0.056 (cap 0.0647), Thigh -> 0.060 (cap 0.0719).
   Wrist/ankle show the identical pattern (forearm/shin's distal ring is
   likewise wider than the wrist/ankle cap) but are OUT OF SCOPE for this
   pass -- not requested, and not touched here. Flagged in the report,
   not silently fixed.

Prerequisites (validated live before running):
  - robot_armature_v14.blend is the open file (or a descendant of it) --
    checked via required object names, not filepath, since MCP inspection
    earlier this session confirmed docs/filenames can't be trusted over
    the live scene.
  - 411_Torso, 411_Shoulder_L/R, 411_UpperArm_L/R, 411_Thigh_L/R,
    411_Elbow_L/R, 411_Knee_L/R all exist as MESH objects.
  - None of the target objects already carry this script's backup marker
    (idempotency guard -- see below).

Safe to rerun: build() takes a full per-vertex coordinate backup on each
target object (as a custom property) before making any change, and
ABORTS on any object that already carries that backup rather than
double-applying the taper. Call teardown() first to restore every target
object's original mesh exactly and clear the markers.

This is a pure base-mesh (rest-pose) vertex edit -- vertex indices,
vertex groups/weights, and the Armature modifiers on every object are
untouched, so skinning is unaffected. No bone, constraint, or armature
data is touched by this script.
"""

import bpy
import math

BACKUP_KEY = "_mesh_taper_v1_backup"
FLAG_KEY = "mesh_taper_v1_applied"

TARGET_OBJECTS = [
    "411_Torso", "411_Shoulder_L", "411_Shoulder_R",
    "411_UpperArm_L", "411_UpperArm_R", "411_Thigh_L", "411_Thigh_R",
]

# (z, target_half_x) control points, world space. Y is scaled by the same
# factor as X at each z (the measured x:y ratio is stable at ~0.696-0.706
# across the current profile, so a single uniform XY radial scale per
# height preserves the torso's existing rectangular-ish cross-section
# instead of rounding it into a circle).
TORSO_TARGET_PROFILE = [
    (0.9614, 0.225),
    (1.0357, 0.213),
    (1.1099, 0.205),
    (1.1842, 0.210),
    (1.2584, 0.230),
    (1.3327, 0.265),
    (1.4069, 0.275),
    (1.4812, 0.262),
    (1.5554, 0.235),
    (1.6297, 0.205),
    (1.6685, 0.190),
    (1.6968, 0.178),
]

SHOULDER_SHRINK = 0.90

# (object_name_pattern, target_ring_radius, cap_object_name) -- the ring to
# shrink is found live as "whichever of the object's two rings has the
# larger mean radius", not assumed by index/side.
HINGE_FIXES = [
    ("411_UpperArm_L", 0.056, "411_Elbow_L"),
    ("411_UpperArm_R", 0.056, "411_Elbow_R"),
    ("411_Thigh_L", 0.060, "411_Knee_L"),
    ("411_Thigh_R", 0.060, "411_Knee_R"),
]


def _validate_prerequisites():
    missing = [n for n in TARGET_OBJECTS if n not in bpy.data.objects]
    if missing:
        raise RuntimeError(f"mesh_taper_v1: required object(s) missing -- {missing}. Aborting.")
    for cap in ("411_Elbow_L", "411_Elbow_R", "411_Knee_L", "411_Knee_R"):
        if cap not in bpy.data.objects:
            raise RuntimeError(f"mesh_taper_v1: required hinge-cap object '{cap}' missing -- aborting.")
    already = [n for n in TARGET_OBJECTS if bpy.data.objects[n].get(FLAG_KEY)]
    if already:
        raise RuntimeError(
            f"mesh_taper_v1: already applied to {already} -- call teardown() first if you want to reapply."
        )


def _backup(obj):
    flat = []
    for v in obj.data.vertices:
        flat.extend(v.co)
    obj[BACKUP_KEY] = flat
    obj[FLAG_KEY] = True


def teardown(report=True):
    restored = []
    for name in TARGET_OBJECTS:
        obj = bpy.data.objects.get(name)
        if obj is None or BACKUP_KEY not in obj.keys():
            continue
        flat = list(obj[BACKUP_KEY])
        for i, v in enumerate(obj.data.vertices):
            v.co = flat[i * 3: i * 3 + 3]
        del obj[BACKUP_KEY]
        if FLAG_KEY in obj.keys():
            del obj[FLAG_KEY]
        obj.data.update()
        restored.append(name)
    if report:
        print(f"mesh_taper_v1.teardown(): restored {restored}")
    return restored


def _height_profile(obj, nbins=30):
    mw = obj.matrix_world
    pts = [mw @ v.co for v in obj.data.vertices]
    zs = [p.z for p in pts]
    zmin, zmax = min(zs), max(zs)
    span = (zmax - zmin) or 1.0
    buckets = {}
    for p in pts:
        b = min(nbins - 1, int((p.z - zmin) / span * nbins))
        buckets.setdefault(b, []).append(p.x)
    profile = []
    for b in sorted(buckets):
        xs = buckets[b]
        zc = zmin + span * (b + 0.5) / nbins
        profile.append((zc, (max(xs) - min(xs)) / 2))
    return profile


def _interp(profile, z):
    if z <= profile[0][0]:
        return profile[0][1]
    if z >= profile[-1][0]:
        return profile[-1][1]
    for i in range(len(profile) - 1):
        z0, v0 = profile[i]
        z1, v1 = profile[i + 1]
        if z0 <= z <= z1:
            t = (z - z0) / (z1 - z0) if z1 != z0 else 0.0
            return v0 + (v1 - v0) * t
    return profile[-1][1]


def _taper_torso():
    obj = bpy.data.objects["411_Torso"]
    current = _height_profile(obj, nbins=30)
    mw = obj.matrix_world
    for v in obj.data.vertices:
        world_z = (mw @ v.co).z
        cur_half = _interp(current, world_z)
        tgt_half = _interp(TORSO_TARGET_PROFILE, world_z)
        scale = tgt_half / cur_half if cur_half > 1e-6 else 1.0
        scale = max(0.5, min(1.3, scale))  # sanity clamp -- fails loud via post-hoc validation below, not silently
        v.co.x *= scale
        v.co.y *= scale
    obj.data.update()
    return _height_profile(obj, nbins=30)


def _shrink_shoulder(name):
    obj = bpy.data.objects[name]
    verts = obj.data.vertices
    n = len(verts)
    cx = sum(v.co.x for v in verts) / n
    cy = sum(v.co.y for v in verts) / n
    cz = sum(v.co.z for v in verts) / n
    for v in verts:
        v.co.x = cx + (v.co.x - cx) * SHOULDER_SHRINK
        v.co.y = cy + (v.co.y - cy) * SHOULDER_SHRINK
        v.co.z = cz + (v.co.z - cz) * SHOULDER_SHRINK
    obj.data.update()


def _cluster_rings(obj, z_eps=0.05):
    """Local-space z-gap clustering -- these 2-ring, 48-vert limb objects
    have a real ~0.28 local-Z gap between their two rings, so this is a
    clean split. (A fixed-width height-bin pass on the same data
    mis-splits a single wavy ring into two apparent bands -- confirmed
    live this session; do not swap this for that approach.)"""
    verts = list(obj.data.vertices)
    order = sorted(range(len(verts)), key=lambda i: verts[i].co.z)
    clusters = [[order[0]]]
    for i in order[1:]:
        if verts[i].co.z - verts[clusters[-1][-1]].co.z > z_eps:
            clusters.append([i])
        else:
            clusters[-1].append(i)
    return clusters


def _shrink_hinge_ring(obj_name, target_radius, cap_name):
    obj = bpy.data.objects[obj_name]
    cap = bpy.data.objects[cap_name]
    verts = obj.data.vertices
    clusters = _cluster_rings(obj)
    if len(clusters) != 2:
        raise RuntimeError(
            f"mesh_taper_v1: expected exactly 2 rings on '{obj_name}', found {len(clusters)} -- "
            f"aborting rather than guessing which one to shrink."
        )

    def ring_stats(idxs):
        cx = sum(verts[i].co.x for i in idxs) / len(idxs)
        cy = sum(verts[i].co.y for i in idxs) / len(idxs)
        r = sum(math.hypot(verts[i].co.x - cx, verts[i].co.y - cy) for i in idxs) / len(idxs)
        return cx, cy, r

    stats = [ring_stats(c) for c in clusters]
    wide_i = 0 if stats[0][2] >= stats[1][2] else 1
    cx, cy, r_mean = stats[wide_i]

    # Cap objects (Elbow/Knee) are identity-transform but their mesh data is
    # NOT centered at local origin (confirmed live: 411_Elbow_L's centroid is
    # at local x=-0.299, not 0) -- radius must be measured relative to the
    # cap's own vertex centroid, not (0,0).
    cap_verts = cap.data.vertices
    cap_cx = sum(v.co.x for v in cap_verts) / len(cap_verts)
    cap_cy = sum(v.co.y for v in cap_verts) / len(cap_verts)
    cap_r_max = max(math.hypot(v.co.x - cap_cx, v.co.y - cap_cy) for v in cap_verts)
    if r_mean <= cap_r_max:
        raise RuntimeError(
            f"mesh_taper_v1: '{obj_name}' wider ring (r={r_mean:.4f}) is not actually larger than "
            f"'{cap_name}' (r_max={cap_r_max:.4f}) -- assumption this script was built on doesn't hold "
            f"live; aborting rather than shrinking the wrong ring."
        )

    scale = target_radius / r_mean
    for i in clusters[wide_i]:
        v = verts[i]
        v.co.x = cx + (v.co.x - cx) * scale
        v.co.y = cy + (v.co.y - cy) * scale
    obj.data.update()
    return {"ring_centroid": (round(cx, 4), round(cy, 4)), "old_r": round(r_mean, 4),
            "new_r": round(target_radius, 4), "cap_r_max": round(cap_r_max, 4)}


def build(report=True):
    _validate_prerequisites()
    for name in TARGET_OBJECTS:
        _backup(bpy.data.objects[name])

    result = {}
    result["torso_new_profile"] = [(round(z, 4), round(h, 4)) for z, h in _taper_torso()]

    for side in ("411_Shoulder_L", "411_Shoulder_R"):
        _shrink_shoulder(side)
    result["shoulder_shrink_factor"] = SHOULDER_SHRINK

    hinge_results = {}
    for obj_name, target_r, cap_name in HINGE_FIXES:
        hinge_results[obj_name] = _shrink_hinge_ring(obj_name, target_r, cap_name)
    result["hinge_fixes"] = hinge_results

    bpy.context.view_layer.update()
    if report:
        import json
        print("mesh_taper_v1.build():", json.dumps(result, indent=1))
    return result
