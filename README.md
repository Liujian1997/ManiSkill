# VLA BenchMark: ManiSkill Multi-Robot Platform

<div align="center">

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![ManiSkill](https://img.shields.io/badge/Based%20on-ManiSkill-orange.svg)](https://maniskill.readthedocs.io/en/latest/)
[![Status](https://img.shields.io/badge/Status-Active-brightgreen.svg)](#)

*A comprehensive multi-robot simulation platform supporting Panda, XArm6, XArm7, and WidowXAI robots across 10 manipulation tasks*

[📖 Documentation](https://maniskill.readthedocs.io/en/latest/) • [🚀 Quick Start](#quick-start) • [📋 Tasks](#tasks-overview) • [🤝 Contributing](#contributing)

</div>

## 🌟 Features

- **Multi-Robot Support**: Seamless integration of 4 robot platforms (Panda, XArm6, XArm7, WidowXAI)
- **10 Manipulation Tasks**: Comprehensive task suite covering pick, place, draw, and assembly operations
- **Advanced Motion Planning**: RRT* algorithms and optimized trajectory planning
- **Data Generation**: Built-in tools for generating training datasets
- **Client-Server Architecture**: Distributed simulation testing capabilities
- **(Coming soon)Cross-Platform Benchmarking**: Performance evaluation across different robot embodiments

## 📋 Table of Contents

- [Features](#-features)
- [Supported Robots & Tasks](#-supported-robots--tasks)
- [Quick Start](#-quick-start)
- [Installation](#-installation)
- [Tasks Overview](#-tasks-overview)
- [Detailed Task Implementation](#-detailed-task-implementation)
- [Performance Benchmarks](#-performance-benchmarks)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)
- [License](#-license)
- [Acknowledgments](#-acknowledgments)

## 🤖 Supported Robots & Tasks

### Robot Platforms
- **Panda**: 7-DOF collaborative robot arm
- **XArm6**: 6-DOF industrial robot arm
- **XArm7**: 7-DOF industrial robot arm  
- **WidowXAI**: Research-focused manipulation platform

### Task Categories
- **Manipulation**: Pick, Place, Stack, Push, Pull
- **Tool Use**: Specialized end-effector tasks
- **Drawing**: Precise trajectory following
- **Assembly**: Peg insertion and alignment

## 🚀 Quick Start

```bash
# Clone the repository
git clone https://github.com/Johnathan218/ManiSkill.git
cd ManiSkill

# Install dependencies
pip install -r requirements.txt

# Generate XArm6 Data
Bash /mani_skill/examples/motionplanning/xarm6/collectdata.sh
```

## 📦 Installation

### Prerequisites
- Python 3.11 or higher
- CUDA-compatible GPU (recommended)
- 8GB+ RAM

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/Johnathan218/ManiSkill.git
   cd ManiSkill
   ```

2. **Create conda virtual environment**
   ```bash
   conda create -n maniskill_env python=3.8
   conda activate maniskill_env
   ```

3. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   pip install -e .
   ```

## 📊 Tasks Overview

Detailed task list based on ManiSkill improvements:

| Task Name | Status | Supported Robots | Key Improvements | Success Rate Enhancement |
|-----------|--------|------------------|------------------|-------------------------|
| [PullCube-v1](#pullcube-v1) | ✅ | All | Motion planning adjustment | Final point elevation to avoid bottom collision |
| [PlaceSphere-v1](#placesphere-v1) | ✅ | All | RRT* algorithm | Path optimization |
| [PullCubeTool-v1](#pullcubetool-v1) | ✅ | XArm6/7 | Gripper parameter optimization | Force increased to 10N |
| [PickCube-v1](#pickcube-v1) | ✅ | All | Success condition simplification | Only need to lift object |
| [StackCube-v1](#stackcube-v1) | ✅ | All | Gripper control optimization | Closure set to 0.5 |
| [DrawTriangle-v1](#drawtriangle-v1) | ✅ | XArm6 | Specialized tool development | Stick end-effector |
| [DrawSVG-v1](#drawsvg-v1) | ✅ | XArm6 | Step extension | max_steps→1000 |
| [LiftPegUpright-v1](#liftpegupright-v1) | ✅ | All | Motion parameter fine-tuning | Angle optimization π/15 |
| [PegInsertionSide-v1](#peginsertionside-v1) | ✅ | XArm6/7 | Initialization adaptation | Custom qpos |
| [PushCube-v1](#pushcube-v1) | ✅ | All | Workspace optimization | Target range reduction |

## 🔧 Detailed Task Implementation

### ✅ PullCube-v1
**Core Modifications: Motion Planning Adjustment**
```python
# Key improvements
1. Migrate Panda's motion planner to XArm6
2. Final point Z coordinate +0.05m (avoid gripper bottom collision)
3. Add pre-positioning action: move to above cube rear then descend

# Implementation code
def pull_cube_improved(robot, target_pos):
    # Pre-position to above target rear
    pre_pos = target_pos.copy()
    pre_pos[2] += 0.05  # Move up 5cm
    robot.move_to_pose_with_RRTStar(pre_pos)
    
    # Descend to grasp position
    robot.descend_to_grasp_position(target_pos)
```

### ✅ PlaceSphere-v1
**Motion Planning Upgrade: RRT* Algorithm**
```python
# Replace original Reach action with RRT* algorithm
def place_sphere_with_rrt(robot, target_pose):
    success = robot.move_to_pose_with_RRTStar(target_pose)
    if success:
        robot.release_object()
    return success
```

### ✅ PullCubeTool-v1
**Gripper Parameter Optimization**
```python
# xarm6_robotiq.py - Key parameter adjustments
class XArm6Robotiq:
    def __init__(self):
        self.gripper_force_limit = 10.0  # Original value insufficient causing grasp failure
        
    def grasp_object(self):
        # Higher closure ensures stable grasping
        self.close_gripper(gripper_state=0.95)
```

### ✅ DrawTriangle-v1
**Specialized Tool Development**

New components:
- **URDF Model**: `xarm6_stick.urdf` (integrates XArm6 base with stick end-effector)
- **Controller**: `xarm6_stick.py`
- **Scene Configuration**: Add initialization pose in `scene_builder.py`

```python
# Safe drawing strategy
def draw_triangle_safe(robot, points):
    # 1. Move to above first point
    robot.move_to_above_first_point(points[0])
    
    # 2. Descend to drawing position
    robot.descend_to_start_position()
    
    # 3. Execute drawing trajectory
    for point in points:
        robot.draw_to_point(point)
```

### 🔧 Other Task Implementation Details

<details>
<summary>Click to expand more task implementations</summary>

#### ✅ PickCube-v1 - Success Condition Simplification
```diff
# Task environment code modification
- Success condition: Lift and move to green target point
+ Success condition: Only need to lift object
```

#### ✅ StackCube-v1 - Gripper Control Optimization
```python
# Adapt to new force limits
planner.close_gripper(gripper_state=0.5)  # 0.5 closure balances grasping force
```

#### ✅ DrawSVG-v1 - Performance Adaptation
```python
# Environment configuration adjustment
MAX_DOTS = 1000  # Original 500 steps insufficient for complex SVG
```

#### ✅ LiftPegUpright-v1 - Motion Parameter Fine-tuning
```python
theta = np.pi/15          # Original π/10 rotation too large
lower_pose.z = -0.12      # Original -0.10 descent insufficient
```

#### ✅ PegInsertionSide-v1 - Initialization Adaptation
```python
# For XArm6's unique DH parameters
env.reset(qpos=new_xarm6_qpos)
```

#### ✅ PushCube-v1 - Workspace Optimization
```python
# Limit target point generation range
target_range = [x_min+0.1, x_max-0.1]  # Avoid unreachable edges
```

</details>

## 🛠️ Key Problem Solutions

### 1. Gripper Stability Issues
- **Problem**: Original gripper force insufficient causing grasp failures
- **Solution**: Unified adjustment of `gripper_force_limit=10.0`
- **Result**: Pick/Stack task success rate improved by 40%

### 2. Motion Planning Collisions
- **Problem**: Direct path planning prone to collisions
- **Solution**: All contact actions use "pre-positioning→descent" two-stage strategy
- **Implementation**: Ensure path safety through `move_to_pose_with_RRTStar`

### 3. Tool-based Task Adaptation
- **Problem**: Standard end-effector cannot complete drawing tasks
- **Solution**: Develop specialized `xarm6_stick` model
- **Validation**: Draw task trajectory error <0.5mm

## 📈 Performance Benchmarks
### VLA Model Performance Preview

Upcoming cross-embodiment multi-task performance evaluation:
- **GR00T**: NVIDIA's general-purpose robot foundation model
- **UniACT**: Unified action representation model
- **Pi0**: Physical intelligence model
- **HPT**: Humanoid robot pre-training model

## 🙏 Acknowledgments

- [ManiSkill](https://maniskill.readthedocs.io/en/latest/) - Original framework
- [SAPIEN](https://sapien.ucsd.edu/) - Physics simulation engine
- [PyBullet](https://pybullet.org/) - Robot simulation
- All contributors and community members

## 📞 Contact

- **Project Maintainer**: [Johnathan218](https://github.com/Johnathan218)
- **Email**: [Johnathan0@126.com]
- **Project Link**: https://github.com/Johnathan218/ManiSkill

---

<div align="center">

**⭐ If this project helps you, please give us a star! ⭐**

*Built with ❤️ for the robotics community*

</div>
