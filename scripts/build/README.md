# Build chain

Scene files are not stored in git. The published robot is v20 and is rebuilt from two source scenes by script:

    python -I scripts/build/build_v17.py  blender/robot_armature_v16.blend blender/robot_armature_v17.blend
    python -I scripts/build/build_v20.py  blender/robot_armature_v20.blend
    python -I scripts/build/export_v18.py blender/robot_armature_v20.blend blender/export/robot_v20.glb

`build_v20.py` runs `build_v18.py` and `build_v19.py` internally. The starting scene, `robot_armature_v16.blend`, was modelled
by hand and then rigged by the scripts in `scripts/rig/`. It is available on request. Needs Blender 5.2 or the `bpy` 5.2 wheel.

The published v20 scene has SHA-256 `d02f01723636f2365ffeaa05fdcd4f731bc4effd11676f8b8186e87e43798a4a`; every render in `proof/` comes from it.
