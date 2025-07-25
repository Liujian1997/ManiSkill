import numpy as np
import sapien
from mani_skill.envs.tasks import DrawSVGEnv
from mani_skill.examples.motionplanning.xarm7.motionplanner import XArm7RobotiqMotionPlanningSolver


def solve(env: DrawSVGEnv, seed=None, debug=False, vis=False):
    env.reset(seed=seed)
    planner = XArm7RobotiqMotionPlanningSolver(
        env,
        debug=debug,
        vis=vis,
        base_pose=env.unwrapped.agent.robot.pose,
        visualize_target_grasp_pose=vis,
        print_env_info=False,
    )

    FINGER_LENGTH = 0.025
    env = env.unwrapped

    rot = list(env.agent.tcp.pose.get_q()[0].cpu().numpy())
    res = None

    # -------------------------------------------------------------------------- #
    # Move to just a little above the first vertex
    # -------------------------------------------------------------------------- #

    reach_pose = sapien.Pose(p=list(env.points[0, 0].numpy()), q=rot)
    offset = sapien.Pose([0, 0, -0.05])
    reach_pose = reach_pose * offset
    res = planner.move_to_pose_with_screw(reach_pose)

    for i, point in enumerate(env.points[0]):
        reach_pose = sapien.Pose(p=list(point.cpu().numpy()), q=rot)
        res = planner.move_to_pose_with_screw(reach_pose)

        if not env.continuous and i - 1 in env.disconts:
            res = planner.move_to_pose_with_screw(
                sapien.Pose(p=list(point.cpu().numpy() + np.array([0, 0, 0.1])), q=rot)
            )

    planner.close()
    return res
