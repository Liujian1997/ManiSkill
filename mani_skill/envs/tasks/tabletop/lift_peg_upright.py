from typing import Any, Dict, Union

import numpy as np
import sapien
import torch
import torch.random
from transforms3d.euler import euler2quat

from mani_skill.agents.robots import Fetch, Panda, XArm6Robotiq
from mani_skill.envs.tasks.tabletop.lift_peg_upright_cfgs import LIFT_PEG_UPRIGHT_CONFIGS
from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.sensors.camera import CameraConfig
from mani_skill.utils.building import actors
from mani_skill.utils.geometry import rotation_conversions
from mani_skill.utils.registration import register_env
from mani_skill.utils.sapien_utils import look_at
from mani_skill.utils.scene_builder.table import TableSceneBuilder
from mani_skill.utils.structs.pose import Pose
from mani_skill.utils.structs.types import Array


@register_env("LiftPegUpright-v1", max_episode_steps=50)
class LiftPegUprightEnv(BaseEnv):
    """
    **Task Description:**
    A simple task where the objective is to move a peg laying on the table to any upright position on the table

    **Randomizations:**
    - the peg's xy position is randomized on top of a table in the region [0.1, 0.1] x [-0.1, -0.1]. It is placed flat along it's length on the table

    **Success Conditions:**
    - the absolute value of the peg's y euler angle is within 0.08 of $\pi$/2 and the z position of the peg is within 0.005 of its half-length (0.12).
    """

    _sample_video_link = "https://github.com/haosulab/ManiSkill/raw/main/figures/environment_demos/LiftPegUpright-v1_rt.mp4"
    SUPPORTED_ROBOTS = ["panda", "fetch", "xarm6_robotiq"]
    agent: Union[Panda, Fetch, XArm6Robotiq]

    # these will be overwritten per‐robot by cfg
    peg_half_width: float
    peg_half_length: float
    spawn_half_size: float
    spawn_center: tuple
    sensor_cam_eye_pos: list
    sensor_cam_target_pos: list
    human_cam_eye_pos: list
    human_cam_target_pos: list

    def __init__(self, *args, robot_uids="panda", robot_init_qpos_noise=0.02, **kwargs):
        # load robot‐specific configs
        self.robot_init_qpos_noise = robot_init_qpos_noise
        cfg = LIFT_PEG_UPRIGHT_CONFIGS.get(robot_uids, LIFT_PEG_UPRIGHT_CONFIGS["panda"])
        self.peg_half_width = cfg["peg_half_width"]
        self.peg_half_length = cfg["peg_half_length"]
        self.spawn_half_size = cfg["spawn_half_size"]
        self.spawn_center = cfg["spawn_center"]
        self.sensor_cam_eye_pos = cfg["sensor_cam_eye_pos"]
        self.sensor_cam_target_pos = cfg["sensor_cam_target_pos"]
        self.human_cam_eye_pos = cfg["human_cam_eye_pos"]
        self.human_cam_target_pos = cfg["human_cam_target_pos"]
        self.spawn_offset = cfg["spawn_offset"]

        super().__init__(*args, robot_uids=robot_uids, **kwargs)

    @property
    def _default_sensor_configs(self):
        pose = look_at(eye=self.sensor_cam_eye_pos, target=self.sensor_cam_target_pos)
        return [CameraConfig("base_camera", pose, 128, 128, np.pi / 2, 0.01, 100)]

    @property
    def _default_human_render_camera_configs(self):
        pose = look_at(self.human_cam_eye_pos, self.human_cam_target_pos)
        return CameraConfig("render_camera", pose, 512, 512, 1, 0.01, 100)

    def _load_agent(self, options: dict):
        super()._load_agent(options, sapien.Pose(p=[-0.615, 0, 0]))

    def _load_scene(self, options: dict):
        self.table_scene = TableSceneBuilder(
            env=self, robot_init_qpos_noise=self.robot_init_qpos_noise
        )
        self.table_scene.build()

        # the peg that we want to manipulate
        self.peg = actors.build_twocolor_peg(
            self.scene,
            length=self.peg_half_length,
            width=self.peg_half_width,
            color_1=np.array([176, 14, 14, 255]) / 255,
            color_2=np.array([12, 42, 160, 255]) / 255,
            name="peg",
            body_type="dynamic",
            initial_pose=sapien.Pose(p=[0, 0, 0.1]),
        )

    def _initialize_episode(self, env_idx: torch.Tensor, options: dict):
        with torch.device(self.device):
            b = len(env_idx)
            self.table_scene.initialize(env_idx)

            # randomize peg spawn within configured region
            xyz = torch.zeros((b, 3), device=self.device)
            half = self.spawn_half_size
            center = self.spawn_center
            vars2 = torch.rand((b, 2), device=self.device) * (2 * half) - half
            xyz[..., :2] = vars2 + torch.tensor(center, device=self.device)
            xyz[..., 2] = self.peg_half_width
            xyz[..., 0] += self.spawn_offset
            q = euler2quat(np.pi / 2, 0, 0)

            obj_pose = Pose.create_from_pq(p=xyz, q=q)
            self.peg.set_pose(obj_pose)

    def evaluate(self):
        q = self.peg.pose.q
        qmat = rotation_conversions.quaternion_to_matrix(q)
        euler = rotation_conversions.matrix_to_euler_angles(qmat, "XYZ")
        is_peg_upright = (
            torch.abs(torch.abs(euler[:, 2]) - np.pi / 2) < 0.08
        )  # 0.08 radians of difference permitted
        close_to_table = torch.abs(self.peg.pose.p[:, 2] - self.peg_half_length) < 0.005
        return {
            "success": is_peg_upright & close_to_table,
        }

    def _get_obs_extra(self, info: Dict):
        obs = dict(
            tcp_pose=self.agent.tcp.pose.raw_pose,
        )
        if self.obs_mode_struct.use_state:
            obs.update(
                obj_pose=self.peg.pose.raw_pose,
            )
        return obs

    def compute_dense_reward(self, obs: Any, action: Array, info: Dict):
        # rotation reward as cosine similarity between peg direction vectors
        # peg center of mass to end of peg, (1,0,0), rotated by peg pose rotation
        # dot product with its goal orientation: (0,0,1) or (0,0,-1)
        qmats = rotation_conversions.quaternion_to_matrix(self.peg.pose.q)
        vec = torch.tensor([1.0, 0, 0], device=self.device)
        goal_vec = torch.tensor([0, 0, 1.0], device=self.device)
        rot_vec = (qmats @ vec).view(-1, 3)
        # abs since (0,0,-1) is also valid, values in [0,1]
        rot_rew = (rot_vec @ goal_vec).view(-1).abs()
        reward = rot_rew

        # position reward using common maniskill distance reward pattern
        # giving reward in [0,1] for moving center of mass toward half length above table
        z_dist = torch.abs(self.peg.pose.p[:, 2] - self.peg_half_length)
        reward += 1 - torch.tanh(5 * z_dist)

        # small reward to motivate initial reaching
        # initially, we want to reach and grip peg
        to_grip_vec = self.peg.pose.p - self.agent.tcp.pose.p
        to_grip_dist = torch.linalg.norm(to_grip_vec, axis=1)
        reaching_rew = 1 - torch.tanh(5 * to_grip_dist)
        # reaching reward granted if gripping block
        reaching_rew[self.agent.is_grasping(self.peg)] = 1
        # weight reaching reward less
        reaching_rew = reaching_rew / 5
        reward += reaching_rew

        reward[info["success"]] = 3
        return reward

    def compute_normalized_dense_reward(self, obs: Any, action: Array, info: Dict):
        max_reward = 3.0
        return self.compute_dense_reward(obs=obs, action=action, info=info) / max_reward
