# ManiSkill with Panda, XArm6

## 借鉴Panda扩充XArm6的任务

借鉴基于Panda机械臂的数据生成流程，为XArm6机械臂编写代码使其可以完成和Panda一样多的任务
包括：PickCube-v1, PushCube-v1, StackCube-v1, PullCube-v1, PullCubeTool-v1, PlaceSphere-v1, LiftPegUpright-v1, PegInsertionSide-v1, DrawTriangle-v1, DrawSVG-v1
由于成功率太低，PulgCharger-v1被抛弃

涉及集成文件urdf、机器人代码、运动规划代码、任务环境代码、任务解决方案代码的撰写

### 任务列表

#### ✅ pull_cube

> **urdf**
> 

/

> **机器人代码**
> 

/

> **运动规划代码**
> 

/

> **任务环境代码**
> 

/

> **任务解决方案代码**
> 
- 将panda的motionplanner修改为xarm6
- 最终点需要稍微上移，否则xarm的夹爪会触底然后崩开
- 最开始运动到立方体后部前加一个动作移动到立方体后部上方，否则夹爪可能对立方体有碰撞

#### ✅ place_sphere

> **urdf**
> 

/

> **机器人代码**
> 

/

> **运动规划代码**
> 

/

> **任务环境代码**
> 

/

> **任务解决方案代码**
> 

将panda的motionplanner修改为xarm6，“Reach”部分改为`move_to_pose_with_RRTStar`

#### ✅ pull_cube_tool

> **urdf**
> 

/

> **机器人代码**
> 

将xarm6_robotiq.py中的
`gripper_force_limit` 修改为`10.0`

> **运动规划代码**
> 

/

> **任务环境代码**
> 

/

> **任务解决方案代码**
> 
- 将panda的motionplanner修改为xarm6，“Reach”部分改为`move_to_pose_with_RRTStar`
- 将`planner.close_gripper()`的`gripper_state` 设为`0.95`

#### ✅ pick_cube

> **urdf**
> 

/

> **机器人代码**
> 

/

> **运动规划代码**
> 

/

> **任务环境代码**
> 

修改任务成功的条件，机械臂只需要将物体提起即可，而非提起并移动到绿色目标点

> **任务解决方案代码**
> 
- 由于修改了xarm6_robotiq.py中的
`gripper_force_limit` ，我们需要将`planner.close_gripper()`的`gripper_state` 设为`0.5`

#### ✅ stack_cube

> **urdf**
> 

/

> **机器人代码**
> 

/

> **运动规划代码**
> 

/

> **任务环境代码**
> 

/

> **任务解决方案代码**
> 
- 由于修改了xarm6_robotiq.py中的
`gripper_force_limit` ，我们需要将`planner.close_gripper()`的`gripper_state` 设为`0.5`

#### ✅ draw_triangle

- 注意在这个任务下，`mani_skill/examples/motionplanning/xarm6/run.py`中`—-reward mode`要改成`sparse`

> **urdf**
> 

借鉴panda_stick.urdf与xarm6_robotiq.urdf编写xarm6_stick.urdf，创建一个带stick的xarm6机械臂集成文件

> **机器人代码**
> 
- 借鉴panda_stick.py与xarm6_robotiq.py编写xarm6_stick.py，创建一个带stick的xarm6机械臂代码
- 由于创建了新的机器人，所以需要在`mani_skill/utils/scene_builder/table/scene_builder.py` 中添加robot的初始化pose

> **运动规划代码**
> 

借鉴panda_stick.py与xarm6_robotiq.py编写xarm6_stick.py，创建一个带stick的xarm6机械臂动作规划代码

> **任务环境代码**
> 
- xarm6需要的迭代步数更多，将`max_episode_steps` （与变量`MAX_DOTS`）从`300`修改为`500`，否则会报错

> **任务解决方案代码**
> 
- 将panda的motionplanner修改为编写好的xarm6_stick
- 加了一个动作，先移动到三角形第一个点的正上方位置，然后下降开始画图，而非直接移动到三角形的第一个点

#### ✅ draw_svg

- 注意在这个任务下，`mani_skill/examples/motionplanning/xarm6/run.py`中`—-reward mode`要改成`sparse`

> **urdf**
> 

使用编写好的xarm6_stick.urdf

> **机器人代码**
> 

使用编写好的xarm6_stick.py

> **运动规划代码**
> 

使用编写好的xarm6_stick.py

> **任务环境代码**
> 
- xarm6需要的迭代步数更多，将`max_episode_steps` （与变量`MAX_DOTS`）从`500`修改为`1000`，否则会报错

> **任务解决方案代码**
> 
- 将panda的motionplanner修改为编写好的xarm6_stick
- 加了一个动作，先移动到svg第一个点的正上方位置，然后下降开始画图，而非直接移动到svg的第一个点

#### ✅ lift_peg_upright

> **urdf**
> 

/

> **机器人代码**
> 

将xarm6_robotiq.py中的
`gripper_force_limit` 修改为`10.0`

> **运动规划代码**
> 

/

> **任务环境代码**
> 

/

> **任务解决方案代码**
> 
- 将panda的motionplanner修改为xarm6，“Reach”部分改为`move_to_pose_with_RRTStar`
- 抓手`gripper_state`改为`0.4`
- 机械旋转角度需要微调，`theta` 从`np.pi/10` 调至`np.pi/15`
- 机械臂下降时高度需要低一点，`lower_pose` 从`sapien.Pose([0, 0, -0.10])` 调至`sapien.Pose([0, 0, -0.12])`

#### ✅ peg_insertion_side

> **urdf**
> 

/

> **机器人代码**
> 

将xarm6_robotiq.py中的
`gripper_force_limit` 修改为`10.0`

> **运动规划代码**
> 

/

> **任务环境代码**
> 
- xarm6与panda有不同的qpos初始化

> **任务解决方案代码**
> 
- 将panda的motionplanner修改为xarm6，“Reach”部分改为`move_to_pose_with_RRTStar`

#### ✅ push_cube

> **urdf**
> 

/

> **机器人代码**
> 

/

> **运动规划代码**
> 

/

> **任务环境代码**
> 

原始任务的目标点对于xarm6的机械臂会存在太远的情况导致夹爪无法到达目标点，需要缩小范围

> **任务解决方案代码**
> 
- 将panda的motionplanner修改为xarm6
- 最终点需要稍微上移，否则xarm的夹爪会触底然后崩开
- 最开始运动到立方体后部前加一个动作移动到立方体后部上方，否则夹爪可能对立方体有碰撞

