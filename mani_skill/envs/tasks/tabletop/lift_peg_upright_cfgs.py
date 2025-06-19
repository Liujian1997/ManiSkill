# lift_peg_upright_cfgs.py

"""
LiftPegUpright-v1 task configurations for different robots.
Each entry specifies:
- peg_half_width, peg_half_length: dimensions of the peg
- spawn_half_size, spawn_center: area on the table where the peg is spawned
- sensor_cam_eye_pos, sensor_cam_target_pos: for visual observations
- human_cam_eye_pos, human_cam_target_pos: for human‐view renderings
"""

LIFT_PEG_UPRIGHT_CONFIGS = {
    "panda": {
        "peg_half_width": 0.025,
        "peg_half_length": 0.12,
        "spawn_half_size": 0.1,
        "spawn_center": (0.0, 0.0),
        "sensor_cam_eye_pos": [0.3, 0.0, 0.6],
        "sensor_cam_target_pos": [-0.1, 0.0, 0.1],
        "human_cam_eye_pos": [0.6, 0.7, 0.6],
        "human_cam_target_pos": [0.0, 0.0, 0.35],
    },
    "fetch": {
        "peg_half_width": 0.025,
        "peg_half_length": 0.12,
        "spawn_half_size": 0.1,
        "spawn_center": (0.0, 0.0),
        "sensor_cam_eye_pos": [0.3, -0.2, 0.5],
        "sensor_cam_target_pos": [0.0, 0.0, 0.1],
        "human_cam_eye_pos": [0.5, 0.8, 0.6],
        "human_cam_target_pos": [0.0, 0.0, 0.35],
    },
    "xarm6_robotiq": {
        "peg_half_width": 0.025,
        "peg_half_length": 0.12,
        "spawn_half_size": 0.1,
        "spawn_center": (0.0, 0.0),
        "sensor_cam_eye_pos": [0.4, 0.0, 0.55],
        "sensor_cam_target_pos": [0.0, 0.0, 0.1],
        "human_cam_eye_pos": [0.7, 0.5, 0.6],
        "human_cam_target_pos": [0.0, 0.0, 0.35],
    },
}
