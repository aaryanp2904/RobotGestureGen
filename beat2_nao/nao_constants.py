"""Shared NAO joint metadata for BEAT2 gesture mapping and playback."""

NAO_JOINTS = [
    "HeadYaw", "HeadPitch",
    "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
    "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
]

NAO_LIMITS = {
    "HeadYaw": (-2.08, 2.08),
    "HeadPitch": (-0.67, 0.51),
    "RShoulderPitch": (-2.08, 2.08),
    "RShoulderRoll": (-1.32, 0.31),
    "RElbowYaw": (-2.08, 2.08),
    "RElbowRoll": (0.03, 1.54),
    "LShoulderPitch": (-2.08, 2.08),
    "LShoulderRoll": (-0.31, 1.32),
    "LElbowYaw": (-2.08, 2.08),
    "LElbowRoll": (-1.54, -0.03),
}

NAO_MAX_VEL = {
    "HeadYaw": 8.27 * 0.8,
    "HeadPitch": 7.19 * 0.8,
    "RShoulderPitch": 8.27 * 0.8,
    "RShoulderRoll": 7.19 * 0.8,
    "RElbowYaw": 8.27 * 0.8,
    "RElbowRoll": 8.27 * 0.8,
    "LShoulderPitch": 8.27 * 0.8,
    "LShoulderRoll": 7.19 * 0.8,
    "LElbowYaw": 8.27 * 0.8,
    "LElbowRoll": 8.27 * 0.8,
}
