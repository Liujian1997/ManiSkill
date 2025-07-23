import numpy as np
import sapien

from mani_skill.envs.tasks import PullCubeToolEnv
from mani_skill.examples.motionplanning.widowxai.motionplanner import WidowXArmMotionPlanningSolver
from mani_skill.examples.motionplanning.panda.utils import compute_grasp_info_by_obb, get_actor_obb

def solve(env: PullCubeToolEnv, seed=None, debug=False, vis=False):
    env.reset(seed=seed)

    planner = WidowXArmMotionPlanningSolver(
        env,
        debug=debug,
        vis=vis,
        base_pose=env.unwrapped.agent.robot.pose,
        visualize_target_grasp_pose=vis,
        print_env_info=False,
    )
    env = env.unwrapped

    # Get tool OBB and compute grasp pose
    tool_obb = get_actor_obb(env.l_shape_tool)
    approaching = np.array([0, 0, -1])
    target_closing = env.agent.tcp.pose.to_transformation_matrix()[0, :3, 1].cpu().numpy()
    
    grasp_info = compute_grasp_info_by_obb(
        tool_obb,
        approaching=approaching,
        target_closing=target_closing,
        depth=0.03,
    )
    closing, center = grasp_info["closing"], grasp_info["center"]
    grasp_pose = env.agent.build_grasp_pose(approaching, closing, env.l_shape_tool.pose.sp.p)
    offset = sapien.Pose([0.02, 0, 0])
    grasp_pose = grasp_pose * (offset)

    # -------------------------------------------------------------------------- #
    # Reach
    # -------------------------------------------------------------------------- #
    reach_pose = grasp_pose * sapien.Pose([0, 0, -0.05])
    res = planner.move_to_pose_with_RRTStar(reach_pose)
    if res == -1: return res

    # -------------------------------------------------------------------------- #
    # Grasp
    # -------------------------------------------------------------------------- #
    res = planner.move_to_pose_with_RRTStar(grasp_pose)
    if res == -1: return res
    # planner.close_gripper(gripper_state=0.95)
    planner.close_gripper()

    # -------------------------------------------------------------------------- #
    # Lift tool to safe height
    # -------------------------------------------------------------------------- #
    lift_height = 0.11
    lift_pose = sapien.Pose(grasp_pose.p + np.array([0, 0, lift_height]))
    lift_pose.set_q(grasp_pose.q)  # Maintain grasp orientation
    res = planner.move_to_pose_with_RRTStar(lift_pose)
    if res == -1: return res

    cube_pos = env.cube.pose.sp.p
    approach_offset = sapien.Pose(
        [-(env.hook_length + env.cube_half_size + 0.08),  
        -0.067,   # -0.05
        lift_height - 0.05]  
    )
    approach_pose = sapien.Pose(cube_pos) * approach_offset
    approach_pose.set_q(grasp_pose.q)
    
    res = planner.move_to_pose_with_RRTStar(approach_pose)
    if res == -1: return res

    # -------------------------------------------------------------------------- #
    # Lower tool behind cube
    # -------------------------------------------------------------------------- #
    behind_offset = sapien.Pose(
        [-(env.hook_length + env.cube_half_size),  
        -0.067,  #  -0.067
        0] 
    )
    hook_pose = sapien.Pose(cube_pos) * behind_offset
    hook_pose.set_q(grasp_pose.q)
    
    res = planner.move_to_pose_with_RRTStar(hook_pose)
    if res == -1: return res

    # -------------------------------------------------------------------------- #
    # Pull cube
    # -------------------------------------------------------------------------- #
    # pull_offset = sapien.Pose([-0.05, 0, 0])
    pull_offset = sapien.Pose([0, 0, -0.22]) # z, y, x
    target_pose = hook_pose * pull_offset
    res = planner.move_to_pose_with_RRTStar(target_pose)
    if res == -1: return res

    planner.close()
    return res