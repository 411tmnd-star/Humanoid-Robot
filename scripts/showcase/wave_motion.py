"""wave_motion.py -- the hello-wave loop as a pure function of the frame number. No Blender needed.

Every angle is a control value in degrees and stays inside the limits written in specifications/arm/*.md.
The palm faces the camera because the forearm twist sits at its documented -50 degree stop.
"""
import math

FPS = 30
WAVE_FRAMES = 90  # 3.0 s loop: frame 90 equals frame 0 and is not rendered
HERO_FRAME = 31

# MECH joint -> (local axis, minimum, maximum) in degrees
LIMITS = {
    "MECH_Shoulder_FlexExt": ("x", -40, 140), "MECH_Shoulder_AbdAdd": ("z", -20, 140), "MECH_Shoulder_Rot": ("y", -60, 60),
    "MECH_Elbow": ("x", 0, 130), "MECH_Forearm_Twist": ("y", -50, 50), "MECH_Wrist_Flex": ("x", -60, 55),
    "MECH_Wrist_Dev": ("z", -20, 15),
}
# control bone -> (euler axis index, MECH joint that clamps it)
CHANNELS = {
    "CTRL_Shoulder_FlexExt": (0, "MECH_Shoulder_FlexExt"), "CTRL_Shoulder_AbdAdd": (2, "MECH_Shoulder_AbdAdd"),
    "CTRL_Shoulder_Rot": (1, "MECH_Shoulder_Rot"), "CTRL_Elbow": (0, "MECH_Elbow"),
    "CTRL_Forearm_Twist": (1, "MECH_Forearm_Twist"), "CTRL_Wrist_Flex": (0, "MECH_Wrist_Flex"),
    "CTRL_Wrist_Dev": (2, "MECH_Wrist_Dev"),
}
WAVE_POSE = {"CTRL_Shoulder_FlexExt": 40, "CTRL_Shoulder_AbdAdd": 0, "CTRL_Shoulder_Rot": -30, "CTRL_Elbow": 112,
             "CTRL_Forearm_Twist": -50, "CTRL_Wrist_Flex": 0, "CTRL_Wrist_Dev": 0}
SWING_DEG, WRIST_DEG, CYCLES, RAISE_END, LOWER_START = 17.0, 9.0, 3, 20, 70


def smooth(t):
    """Quintic ease with zero velocity and acceleration at both ends."""
    t = min(1.0, max(0.0, t))
    return t * t * t * (t * (6 * t - 15) + 10)


def wave_angles(frame):
    """Control angles in degrees at a loop frame."""
    f = frame % WAVE_FRAMES
    if f < RAISE_END:
        amount = smooth(f / RAISE_END)
    elif f <= LOWER_START:
        amount = 1.0
    else:
        amount = 1.0 - smooth((f - LOWER_START) / (WAVE_FRAMES - LOWER_START))
    out = {k: v * amount for k, v in WAVE_POSE.items()}
    if RAISE_END <= f <= LOWER_START:
        u = (f - RAISE_END) / (LOWER_START - RAISE_END)
        s = math.sin(2 * math.pi * CYCLES * u) * smooth(min(1.0, 4 * min(u, 1 - u)))
        out["CTRL_Elbow"] += SWING_DEG * s
        out["CTRL_Wrist_Dev"] += WRIST_DEG * s
    return out


def angle_ranges():
    """Lowest and highest value each control reaches over the loop."""
    frames = [wave_angles(f) for f in range(WAVE_FRAMES)]
    return {k: (min(a[k] for a in frames), max(a[k] for a in frames)) for k in WAVE_POSE}


def outside_limits():
    """Controls that leave their documented limits (empty list = safe)."""
    bad = []
    for k, (lo, hi) in angle_ranges().items():
        _, low, high = LIMITS[CHANNELS[k][1]]
        if lo < low - 1e-9 or hi > high + 1e-9:
            bad.append(f"{k}: {lo:.2f}..{hi:.2f} outside {low}..{high}")
    return bad
