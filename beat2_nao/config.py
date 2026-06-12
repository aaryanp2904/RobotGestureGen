"""
Phase 1 configuration — paths, joint definitions, and constants for BEAT2 preprocessing.

Path values can be overridden with environment variables, which is useful when
BEAT2 is mounted/extracted with slightly different folder names.
"""
import os
from pathlib import Path

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
def _env_path(name: str, default) -> Path:
    return Path(os.environ.get(name, str(default))).expanduser()


def _first_existing_dir(root: Path, candidates: list[str]) -> Path:
    """Return the first existing candidate directory, or the first candidate."""
    for name in candidates:
        candidate = root / name
        if candidate.is_dir():
            return candidate
    return root / candidates[0]


BEAT2_ROOT = _env_path("BEAT2_ROOT", "/vol/bitbucket/ap1922/BEAT2")
BEAT2_EN = _env_path("BEAT2_EN", BEAT2_ROOT / "beat_english_v2.0.0")

MOTION_DIR = _env_path(
    "BEAT2_MOTION_DIR",
    _first_existing_dir(BEAT2_EN, [
        "smplxflame_30", "smplx_30", "smplxflame", "motion", "motions",
    ]),
)
AUDIO_DIR = _env_path(
    "BEAT2_AUDIO_DIR",
    _first_existing_dir(BEAT2_EN, [
        "wave16k", "wav16k", "wav", "wave", "audio",
    ]),
)
SEM_DIR = _env_path("BEAT2_SEM_DIR", BEAT2_EN / "sem")
TEXTGRID_DIR = _env_path("BEAT2_TEXTGRID_DIR", BEAT2_EN / "textgrid")
SPLIT_CSV = _env_path("BEAT2_SPLIT_CSV", BEAT2_EN / "train_test_split.csv")

# Output directory for cleaned Phase 1 data
OUTPUT_DIR = _env_path("BEAT2_OUTPUT_DIR", "/vol/bitbucket/ap1922/BEAT2_cleaned")
NAO_OUTPUT_DIR = _env_path("BEAT2_NAO_OUTPUT_DIR", "/vol/bitbucket/ap1922/BEAT2_NAO_Preprocessed")

# ---------------------------------------------------------------------------
# Motion constants
# ---------------------------------------------------------------------------
MOCAP_FPS = 30
AUDIO_SR  = 16_000

# ---------------------------------------------------------------------------
# SMPL-X pose layout (165 dims total)
#   [0:3]     global orientation (pelvis)
#   [3:66]    21 body joints × 3 axis-angle  (joints 1–21 in SMPL-X order)
#   [66:111]  15 left-hand joints × 3
#   [111:156] 15 right-hand joints × 3
#   [156:165] 3 jaw/eye joints × 3
#
# Body joint ordering (index within the 21-joint body block):
#   0  left_hip       7  right_ankle    14 head
#   1  right_hip      8  spine3         15 left_shoulder
#   2  spine1         9  left_foot      16 right_shoulder
#   3  left_knee     10  right_foot     17 left_elbow
#   4  right_knee    11  neck           18 right_elbow
#   5  spine2        12  left_collar    19 left_wrist
#   6  left_ankle    13  right_collar   20 right_wrist
#
# NOTE: The SMPL-X body block [3:66] covers joints 1..21 (21 joints × 3 = 63).
#       Joint 0 (pelvis) is the global orient [0:3].
#       Joint 22 (jaw) begins the jaw/eye block at index 156.
# ---------------------------------------------------------------------------

# Upper-body joint indices within the 21-joint body block.
# Body joint i → poses[:, 3 + i*3 : 3 + (i+1)*3]
UPPER_BODY_JOINT_INDICES = [
    2,   # spine1
    5,   # spine2
    8,   # spine3
    11,  # neck
    12,  # left_collar
    13,  # right_collar
    14,  # head
    15,  # left_shoulder
    16,  # right_shoulder
    17,  # left_elbow
    18,  # right_elbow
    19,  # left_wrist
    20,  # right_wrist
]

UPPER_BODY_JOINT_NAMES = [
    "spine1", "spine2", "spine3",
    "neck", "left_collar", "right_collar", "head",
    "left_shoulder", "right_shoulder",
    "left_elbow", "right_elbow",
    "left_wrist", "right_wrist",
]

# Total extracted dims: 3 (global orient) + 13 joints × 3 = 42
EXTRACTED_POSE_DIM = 3 + len(UPPER_BODY_JOINT_INDICES) * 3  # 42

# ---------------------------------------------------------------------------
# Known missing clips (from beat_english_v2.0.0/readme.md)
# These have audio/annotations but NO motion .npz file.
# ---------------------------------------------------------------------------
KNOWN_MISSING = {
    "9_miranda_0_1_8",
    "15_carlos_0_6_6",
    "15_carlos_0_12_12",
    "21_ayana_0_1_8",
    "6_carla_0_1_48",
    "23_hailing_0_73_74",
    "25_goto_0_1_1",
    "25_goto_0_5_5",
}

# ---------------------------------------------------------------------------
# Speaker LUT (from readme.md)
# ---------------------------------------------------------------------------
SPEAKER_LUT = {
    1: "wayne",   2: "scott",    3: "solomon",  4: "lawrence",
    5: "stewart",  6: "carla",    7: "sophie",   9: "miranda",
    10: "kieks",  11: "nidal",   12: "zhao",    13: "lu",
    15: "carlos", 16: "jorge",   17: "itoi",    18: "daiki",
    20: "li",     21: "ayana",   22: "luqi",    23: "hailing",
    24: "kexin",  25: "goto",    27: "yingqing", 28: "tiffnay",
    30: "katya",
}
