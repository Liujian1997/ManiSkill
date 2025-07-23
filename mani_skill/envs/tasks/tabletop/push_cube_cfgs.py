"""
PullCube-v1 is a basic/common task which defaults to using the panda robot. It is also used as a testing task to check whether a robot with manipulation
capabilities can be simulated and trained properly. The configs below set the pick cube task differently to ensure the cube is within reach of the robot tested
and the camera angles are reasonable.
"""
PULL_CUBE_CONFIGS = {
    "panda": {
        "cube_half_size": 0.02,
        "goal_radius": 0.1,
        "sensor_cam_eye_pos": [-0.5,0.0,0.25],
        "sensor_cam_target_pos": [0.2,0.0,-0.5],
        "human_cam_eye_pos": [0.6, 0.7, 0.6],
        "human_cam_target_pos": [0.0, 0.0, 0.35],
        "spawn_range": 0.2,
        "spawn_offset": 0.0,
        "goal_offset_x": 0.1,
    },
    "fetch": {
        "cube_half_size": 0.02,
        "goal_radius": 0.1,
        "sensor_cam_eye_pos": [-0.5,0.0,0.25],
        "sensor_cam_target_pos": [0.2,0.0,-0.5],
        "human_cam_eye_pos": [0.6, 0.7, 0.6],
        "human_cam_target_pos": [0.0, 0.0, 0.35],
        "spawn_range": 0.2,
        "spawn_offset": 0.0,
        "goal_offset_x": 0.1,
    },
    "xarm6_robotiq": {
        "cube_half_size": 0.02,
        "goal_radius": 0.1,
        "sensor_cam_eye_pos": [-0.5,0.0,0.25],
        "sensor_cam_target_pos": [0.2,0.0,-0.5],
        "human_cam_eye_pos": [0.6, 0.7, 0.6],
        "human_cam_target_pos": [0.0, 0.0, 0.35],
        "spawn_range": 0.2,
        "spawn_offset": 0.0,
        "goal_offset_x": 0.1,
    },  
    "widowxai_wristcam": {
        "cube_half_size": 0.02,
        "goal_radius": 0.1,
        "sensor_cam_eye_pos": [-0.5,0.0,0.25],
        "sensor_cam_target_pos": [0.2,0.0,-0.5],
        "human_cam_eye_pos": [0.6, 0.7, 0.6],
        "human_cam_target_pos": [0.0, 0.0, 0.35],
        "spawn_range": 0.2,
        "spawn_offset": -0.4,
        "goal_offset_x": 0.1,
    },
    "xarm7_robotiq_wristcam": {
        "cube_half_size": 0.02,
        "goal_radius": 0.1,
        "sensor_cam_eye_pos": [-0.5,0.0,0.25],
        "sensor_cam_target_pos": [0.2,0.0,-0.5],
        "human_cam_eye_pos": [0.6, 0.7, 0.6],
        "human_cam_target_pos": [0.0, 0.0, 0.35],
        "spawn_range": 0.2,
        "spawn_offset": -0.1,
        "goal_offset_x": 0.1,
    },  
}
