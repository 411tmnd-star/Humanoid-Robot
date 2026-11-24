"""build_v18.py -- SCRIPT_VERSION 1.0

Purpose: derive blender/robot_armature_v18.blend from robot_armature_v17.blend (v17 is never modified).
Headless:  python3 -I scripts/build/build_v18.py [SRC.blend] [DST.blend] [--phases 1,2,3,4]

Phase 1  finger / thumb rig fix + neutral hand pose   (scripts/build/v18_hand_rig.py, v18_hand_mesh.py)
Phase 2  torso contour, panel seams (boolean groove modifiers), limb split seams, head seam, visor chamfer
Phase 3  pelvis chassis, hip sockets, yoke, ankle cuff, toe box, rubber sole
Phase 4  infinity cyclorama + lighting rig

Fails loudly on any unmet prerequisite or failed post-condition; always rebuilds from v17.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import bpy
from mathutils import Vector

import v18_hand_rig as HR
import v18_hand_mesh as HM
import v18_body as BD

SCRIPT_VERSION = "1.0"
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SRC_DEFAULT = os.path.join(ROOT, "blender", "robot_armature_v17.blend")
DST_DEFAULT = os.path.join(ROOT, "blender", "robot_armature_v18.blend")
ARM = "RobotArmature"


def require(c, m):
    if not c:
        raise RuntimeError("v18 build: " + m)


def obj(n):
    o = bpy.data.objects.get(n)
    require(o is not None, f"missing object {n}")
    return o


# =========================================================================================================
# Phase 1
# =========================================================================================================
def phase1(arm):
    B = arm.data.bones
    require(bpy.context.scene.get("v18_build_version") is None, "source already is v18")
    require(bpy.context.scene.get("v17_build_version") is not None, "source is not a v17 file")
    planR, info = HR.plan_right_hand(arm)
    planL = HR.mirrored(planR)
    plan = HR.Frames()
    plan.d.update(planR.d)
    plan.d.update(planL.d)
    old = HR.snapshot_mats(arm, ["DEF_Thumb_CMC", "DEF_Thumb_CMC_L"])
    HR.apply_plan(arm, plan)
    limits = HR.fix_limits_and_locks(arm)

    mover_R = info["mover"]
    mover_L = lambda p: HR.reflect(mover_R(HR.reflect(p)))
    # palm meshes are skinned to DEF_Wrist (which does not move): rotate them with the hand
    for side, mover in (("R", mover_R), ("L", mover_L)):
        o = obj(f"411_Palm_{side}")
        M = o.matrix_world
        Mi = M.inverted()
        for v in o.data.vertices:
            v.co = Mi @ mover(M @ v.co)
        o.data.update()
    # CMC knuckle sphere follows its bone
    for side, bn in (("R", "DEF_Thumb_CMC"), ("L", "DEF_Thumb_CMC_L")):
        HR.rigid_follow(obj(f"411_ThumbJoint_{side}"), old[bn], HR.bone_world_matrix(arm, bn))

    white = bpy.data.materials["BotWhite"]
    graphite = bpy.data.materials["Graphite_Brushed"]
    by_vg = {}
    for o in bpy.data.objects:
        if o.type == "MESH" and o.name.startswith("411_Finger"):
            by_vg[o.vertex_groups[0].name] = o
    made = 0
    for s in ("", "_L"):
        for f in HR.FINGERS:
            bones = [(f"DEF_{f}_MCP{s}", by_vg[f"DEF_{f}_MCP{s}"], HM.R_PHAL[0]),
                     (f"DEF_{f}_PIP{s}", by_vg[f"DEF_{f}_PIP{s}"], HM.R_PHAL[1]),
                     (f"DEF_{f}_Tip{s}", None, HM.R_PHAL[2])]
            made += len(HM.build_chain(arm, by_vg[f"DEF_{f}_MCP{s}"], white, graphite, f"{f}{s.replace('_', '')}", bones,
                                        0.92 if f == "Little" else 1.0))
        side = "L" if s else "R"
        tb = [(f"DEF_Thumb_MCP{s}", obj(f"411_Thumb_{side}"), HM.R_PHAL[0]),
              (f"DEF_Thumb_IP{s}", None, HM.R_PHAL[1])]
        made += len(HM.build_chain(arm, obj(f"411_Thumb_{side}"), white, graphite, f"Thumb{s.replace('_', '')}", tb, 1.08))
    return {"rotation_deg": info["rot_deg"], "limits_rewritten": len(limits), "new_or_rebuilt_meshes": made,
            "hand_axis": tuple(round(x, 3) for x in info["hand_axis"]), "palm_dir": tuple(round(x, 3) for x in info["palm_dir"])}


# =========================================================================================================
# Phase 2
# =========================================================================================================
OPT = {}          # optional overrides from the command line (used by build_v20.py); empty = the validated v18 build


def phase2(arm):
    graphite = bpy.data.materials["Graphite_Brushed"]
    rep = {}
    rep["torso_contour"] = BD.torso_contour(arm, **OPT.get("contour", {}))
    if "thin" in OPT:
        rep["thin_legs"] = BD.thin_legs(OPT["thin"], OPT["thin"])
    rep["torso_seams"] = BD.torso_seams(arm, graphite)
    rep["limb_seam_points"] = {n: BD.limb_seam(arm, n, graphite) for n in
                               ("411_Forearm_R", "411_Forearm_L", "411_Shin_R", "411_Shin_L")}
    BD_V16 = os.path.join(ROOT, "blender", "robot_armature_v16.blend")
    BD.restore_head_from(BD_V16)
    helmet = bpy.data.materials["Helmet_SatinWhite"]
    glass = bpy.data.materials["Visor_BlackGlass"]
    cyan = bpy.data.materials["Visor_CyanEmit"]
    rep["head"] = BD.head_features(arm, helmet, glass, graphite, cyan)
    return rep


# =========================================================================================================
# Phase 3 + 4
# =========================================================================================================
def phase3(arm):
    rep = {}
    white = bpy.data.materials["BotWhite"]
    graphite = bpy.data.materials["Graphite_Brushed"]
    rubber = bpy.data.materials.get("Graphite_RubberSole") or bpy.data.materials.new("Graphite_RubberSole")
    rubber.use_nodes = True
    pb = next(n for n in rubber.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    pb.inputs["Base Color"].default_value = (0.028, 0.030, 0.034, 1.0)
    pb.inputs["Roughness"].default_value = 0.78
    pb.inputs["Metallic"].default_value = 0.0
    rep["smoothed_joint_spheres"] = len(BD.smooth_joint_spheres())
    rep["pelvis"] = BD.pelvis_chassis(arm)
    rep["hip_sockets"] = BD.hip_sockets()
    rep["yoke"] = BD.yoke()
    rep["feet"] = BD.feet(arm, white, rubber, graphite, foot_len=0.22652, foot_width=0.112, heel_y=0.30 * 0.22652)
    return rep


def phase4(arm):
    rep = {"cyc": BD.cyclorama()}
    rep["lights"] = BD.lighting_rig()
    return rep


def main(src, dst, phases):
    require(os.path.exists(src), f"source not found: {src}")
    bpy.ops.wm.open_mainfile(filepath=src)
    arm = obj(ARM)
    require(len([b for b in arm.data.bones if b.name.startswith("DEF_")]) == 56, "expected 56 DEF bones")
    pose_mode_in = arm.data.pose_position      # ray-casting helpers switch the armature to REST; it must be restored
    require(pose_mode_in == "POSE", f"source armature is in {pose_mode_in}, expected POSE")
    rep = {}
    if 1 in phases:
        rep["P1"] = phase1(arm)
    if 2 in phases:
        rep["P2"] = phase2(arm)
    if 3 in phases:
        rep["P3"] = phase3(arm)
    if 4 in phases:
        rep["P4"] = phase4(arm)
    require(len([b for b in arm.data.bones if b.name.startswith("DEF_")]) == 56, "DEF count changed")
    require(not [b for b in arm.data.bones if not b.name.startswith("DEF_") and b.use_deform], "non-DEF deform bone")
    arm.data.pose_position = pose_mode_in
    require(arm.data.pose_position == "POSE", "armature pose_position not restored")
    bpy.context.scene["v18_build_version"] = SCRIPT_VERSION
    bpy.ops.wm.save_as_mainfile(filepath=dst)
    for k, v in rep.items():
        print(k, v)
    print("saved", dst)


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if x.endswith(".blend")]
    ph = (1, 2, 3, 4)
    for flag, key in (("--waist", "waist_ratio"), ("--rim", "rim_ratio"), ("--mid", "mid_ratio"), ("--abdomen", "abdomen_depth")):
        if flag in sys.argv:
            OPT.setdefault("contour", {})[key] = float(sys.argv[sys.argv.index(flag) + 1])
    if "--thin" in sys.argv:
        OPT["thin"] = float(sys.argv[sys.argv.index("--thin") + 1])
    if "--phases" in sys.argv:
        ph = tuple(int(x) for x in sys.argv[sys.argv.index("--phases") + 1].split(","))
    main(a[0] if a else SRC_DEFAULT, a[1] if len(a) > 1 else DST_DEFAULT, ph)
