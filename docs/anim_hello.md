# Animation spec: hello sequence

## Objective
A 4-5 second character animation showing the robot noticing the camera,
waving, bowing slightly, and returning to rest. Tells a story, shows off
the rig, renders as a video clip to send directly.

## Technical setup
- **Frame range**: 1–120 (30fps × 4 seconds)
- **Action name**: `hello_sequence`
- **Render**: EEVEE, 720x1080, MP4 via system ffmpeg
- **Camera**: fixed front-facing, framed from knee to just above head

## Scene restore first
Before any keyframes: confirm fps=30, frame_start=1, frame_end=120.
Set all pose bones to rest (zero rotation) as the baseline.

## Keyframe sequence

### Phase 1 — Breathing rest (frames 1–20)
Character just stands. Subtle life before the action.
- Frame 1: DEF_Chest X = 0.0° (rest)
- Frame 10: DEF_Chest X = 1.5° (inhale)
- Frame 20: DEF_Chest X = 0.0° (exhale, ready)

### Phase 2 — Head turn to camera (frames 20–35)
The robot notices. Slow, deliberate head turn.
- Frame 20: DEF_Head Z = 0.0° (looking forward/neutral)
- Frame 35: DEF_Head Z = 0.0° (already facing camera — skip if
  already forward-facing. Instead do a subtle tilt:)
- Frame 20: DEF_Neck Z = 0.0°
- Frame 28: DEF_Neck Z = 3.0° (slight right tilt — "I see you")
- Frame 35: DEF_Neck Z = 1.5° (settle, attentive)

### Phase 3 — Right arm raises to wave (frames 35–60)
Smooth arm lift using the IK target. Move IK_Target.arm.R upward
and slightly out, then call arm_ik_solver.py each frame to solve.

Rather than per-frame solver calls, set these IK target keyframes
and let the solver handle the pose:
- Frame 35: IK_Target.arm.R at rest position (wrist-level)
- Frame 50: IK_Target.arm.R raised to (0.45, 0.05, 1.35) —
  hand at shoulder height, slightly out from body
- Frame 60: IK_Target.arm.R at (0.40, 0.05, 1.40) —
  hand slightly higher, wave peak

Call arm_ik_solver.py at frames 50 and 60 to bake the arm pose,
then keyframe CTRL_Elbow and CTRL_Forearm_Twist at those frames.

### Phase 4 — Chest bow (frames 60–75)
The robot acknowledges. Chest leans forward.
- Frame 60: DEF_Chest X = 0.0°
- Frame 68: DEF_Chest X = 8.0° (forward lean — a small bow)
- Frame 75: DEF_Chest X = 3.0° (hold, respectful pause)

### Phase 5 — Head tilt during bow (frames 65–75)
While bowing, head tilts slightly — "at your service"
- Frame 65: DEF_Head X = 0.0°
- Frame 72: DEF_Head X = 5.0° (slight forward nod)
- Frame 75: DEF_Head X = 2.0° (hold)

### Phase 6 — Return to rest (frames 75–110)
Everything eases back smoothly. Not snapping — controlled return.
- Frame 90: DEF_Chest X = 0.0° (torso back upright)
- Frame 90: DEF_Head X = 0.0° (head level)
- Frame 95: DEF_Neck Z = 0.0° (neck straight)
- Frame 100: IK_Target.arm.R back to rest position
  (call arm_ik_solver.py to solve, keyframe the bones)
- Frame 105: DEF_Chest X = 1.0° (one last breath)
- Frame 110: DEF_Chest X = 0.0° (settle)

### Phase 7 — Hold at rest (frames 110–120)
Character back to neutral. Calm, ready.
- No new keyframes — hold last pose through frame 120.

## Interpolation
- All keyframes: BEZIER interpolation
- Phase transitions (e.g. frame 60 bow start): ease-in
- Return to rest (frames 75–110): ease-out, slower than the raise
  — the return should feel deliberate, not rushed
- Loop points (1 and 120): flat handles if looping, otherwise free

## IK arm baking method
Since the arm uses a Python solver (not Blender's native IK):
1. Move IK_Target.arm.R to the target position in world space
2. Call arm_ik_solver.py to compute and apply the pose
3. Keyframe CTRL_Elbow.rotation_euler and
   CTRL_Forearm_Twist.rotation_euler at that frame
4. Repeat for each arm keyframe position
5. Do NOT keyframe the IK target itself — keyframe the CTRL bones
   that the solver wrote to, so the animation is self-contained

## Camera setup
- Position: (0, -2.8, 1.1) — front-facing, character centered
- Rotation: (90°, 0°, 0°) — looking straight at character
- No camera animation — fixed shot, character moves toward camera
  emotionally via the bow and head turn

## Render settings
- Engine: BLENDER_EEVEE
- Resolution: 720 x 1080 (portrait — looks great on a phone)
- FPS: 30
- Frame range: 1–120
- Output: render PNG sequence to blender/hello_frames/
- Then mux with system ffmpeg:
  ffmpeg -framerate 30 -i blender/hello_frames/%04d.png
  -c:v libx264 -pix_fmt yuv420p
  blender/hello_animation.mp4

## Save
Save as `robot_armature_v12.blend` after all keyframes are set and
before rendering.

## Stop condition
Set all keyframes, bake the arm poses, save, then render and mux.
Report back with the MP4 path when done.
The goal is a file you can send directly to someone — it should look
good, not just technically correct. If the 8° bow looks too subtle
in the render, increase to 12°. If the arm raise looks stiff,
soften the interpolation handles. Tune for impact, not just accuracy.
