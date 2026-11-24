# OSCBF ROS2 Workspace

[![Paper](http://img.shields.io/badge/arXiv-2503.06736-B31B1B.svg)](https://arxiv.org/abs/2503.06736)

Fast, safe manipulator teleoperation with [OSCBF](https://github.com/StanfordASL/oscbf)

Currently supported hardware platforms:
- Franka Emika Panda
- AgileX Nero (seven-joint flange model, ROS 2 `agx_arm_ctrl` MIT interface)

## AgileX Nero

本机使用 ROS 2 Humble、Python 3.10、配套的 OSCBF 源码以及已构建的 AgileX 消息包。以下路径对应当前项目布局。从 `oscbf_hardware_ws` 目录构建：

```bash
source /opt/ros/humble/setup.bash
source ~/demo_ws/install/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

运行时使用安装了本项目的 Python 环境。本机已有 `oscbf` 环境，新环境可用 `python -m pip install -e ../oscbf-for-Agilex-Nero` 安装。新终端从工作区目录运行：

```bash
conda activate oscbf
source install/setup.bash
ros2 launch oscbf_control nero_sim.launch.py
```

这个 launch 在 `/nero_sim` 下启动 PyBullet 仿真和 OSCBF 控制器。仿真发布带时间戳的七轴反馈与 2 cm 法兰轨迹，每条 MIT 指令推进 10 ms 仿真时间。可在相同环境的另一终端查看 `ros2 topic hz /nero_sim/control/move_mit`。

### 实机接口

采用竖直固定底座与空载法兰。先通过匹配固件的 AgileX 驱动完成连接、使能及初姿设置。驱动需要提供 `feedback/joint_states`、`control/move_mit` 和 `emergency_stop` 服务。零位接近奇异位形；仿真使用的屈肘姿态是 `[0, 0.6, 0.3, 1.0, -0.4, 0.35, 0.1]` rad。控制器会校验当前姿态，并以实际关节位置作为零空间目标。

已有驱动在默认命名空间运行时，下列命令会在收到有效反馈后向实机发力，保持启动时测得的法兰位姿：

```bash
ros2 run oscbf_control nero_control_node.py --ros-args -p hold_current_pose:=true
```

跟踪外部目标时，使用 `ros2 run oscbf_control nero_control_node.py`，持续向 `ee_state` 发布 `oscbf_control_msgs/msg/EEState`。目标位姿和线、角速度均在基座坐标系表达，四元数顺序为 xyzw，时间戳与反馈使用同一 ROS 时钟。若驱动位于命名空间，例如 `/nero`，给节点追加 ROS 参数 `-r __ns:=/nero`，并向 `/nero/ee_state` 发布目标。

节点按名称读取 `joint1` 到 `joint7`，以最高 100 Hz 向 MIT 关节索引 1 到 7 发布含重力补偿的总力矩，`kp` 和 `kd` 均为零。模型假设 SDK 反馈的弧度、弧度每秒与前馈 N·m 的方向和 URDF 一致。QP 对 J1–J4 采用 16 N·m、J5–J7 采用 8 N·m 的对称指令上限，取本机 SDK 默认协议与 v1.11+ 协议的共同范围。实机方向、负载、软限位和停机动作仍需在对应机械臂上验证。

运行中若所需输入超过 100 ms、数据无效或求解失败，节点停止发布力矩并请求驱动的 `emergency_stop` 服务，重启节点后才会恢复。正常退出也会向驱动请求停机。该请求依赖 ROS 链路及驱动可用。

## Installation

### Libfranka

For newer robots, you can probably follow the standard setup details on the [libfranka Github](https://github.com/frankarobotics/libfranka). However, our lab has an older Panda, which requires libfranka 0.8.0. To get this to work, I had to make a minor change to libfranka which is available [here](https://github.com/danielpmorton/libfranka_08_patch).

### Fetch the code

Clone the original oscbf code and the ROS2 workspace. See the README on the [OSCBF Github](https://github.com/StanfordASL/oscbf) for additional details
```
git clone https://github.com/StanfordASL/oscbf
git clone https://github.com/StanfordASL/oscbf_hardware_ws/
```

### Set up your virtual environment

If using a virtual environment, keep in mind that while the OSCBF code runs on multiple python versions (tested: 3.10, 3.11, 3.12), to get it to work with ROS2, you'll need the python version to match with your ROS2 python version. For ROS2 Humble, this means 3.10.x, and for ROS2 Jazzy, this means 3.12.x. 

I recommend `uv` to manage the virtual environment. If you don't have `uv` already installed, run the following:
```
curl -LsSf https://astral.sh/uv/install.sh | sh
# Optional, but recommended:
# echo 'eval "$(uv generate-shell-completion bash)"' >> ~/.bashrc
```
Then create the virtual environment for the project and install dependencies
```
cd oscbf_hardware_ws
uv venv --python 3.12.3 --system-site-packages
# Or, feel free to select a different python version.
# If you intend to use ROS2 Humble, I recommend 3.10.12
source .venv/bin/activate
# Note: this assumes you cloned oscbf and oscbf_hardware_ws in the same directory
cd ../oscbf
uv pip install -e .
```

### Oculus Reader

If using the Oculus, follow the steps as found on the [oculus_reader github page](https://github.com/rail-berkeley/oculus_reader). Then, install the package in your environment with
```
source oscbf_hardware_ws/.venv/bin/activate
cd oculus_reader # Wherever you cloned it
uv pip install -e .
```

When working with the Oculus/Quest hardware, the following tips might be useful:
- Go into settings and turn all of the automatic sleep times to the maximum value (4 hours)
- Add a sticker on top of the proximity sensor on the inside of the headset
- The Meta Quest 3 sometimes has some issues where it loses track of the controller, and then when it regains tracking, it "snaps" to the new location, leading to unstable robot control. The Quest 2 seems to be more stable here.

### Build + Setup

To build, run
```
./scripts/build.sh
```
To configure your terminal/environment, run
```
source scripts/setup.sh
```

## Demo (ROS-free)

There are many interactive demos available in the original OSCBF repo! Give these a try and make sure that things work before trying to run the ROS2 nodes.

## ROS Demo

For a ROS2 end-effector trajectory tracking demo in sim,
```
cd oscbf_hardware_ws
source scripts/setup.sh
ros2 launch oscbf_control traj_sim.launch.py
```

## Overview

This contains two packages:

1. `oscbf_control`: Control nodes (Python and C++)
3. `oscbf_control_msgs`: Custom ROS2 message definitions

## Terminal setup -- Hardware teleoperation example

This is how to configure your terminals if you are running individual nodes. But, a launch file can also be used to run multiple nodes at once.

**Terminal 1 (Franka node)**: Publishes joint states, subscribes to joint torques
```
cd oscbf_hardware_ws
source scripts/setup.sh
ros2 run oscbf_control franka_impedance_controller
```

**Terminal 2 (OSCBF node)**: Publishes joint torques, subscribes to joint states and desired EE state
```
cd oscbf_hardware_ws
source scripts/setup.sh
ros2 run oscbf_control franka_control_node.py
```

**Terminal 3 (Oculus node)**: Publishes desired EE state, subscribes to joint states
```
cd oscbf_hardware_ws
source scripts/setup.sh
ros2 run oscbf_control oculus_node.py
```

### Alternative terminal setup options

#### Trajectory following

Terminal 3 can be replaced with a trajectory node, which publishes just the desired EE state from a predefined EE trajectory
```
cd oscbf_hardware_ws
source scripts/setup.sh
ros2 run oscbf_control traj_node.py
```

#### Testing in simulation

Terminal 1 can be replaced with a simulated pybullet environment which does not require a connection to the robot, and can be used to debug the controller prior to testing on hardware
```
cd oscbf_hardware_ws
source scripts/setup.sh
ros2 run oscbf_control pybullet_sim_node.py
```

## Assorted notes

- Connect to the Franka control box via ethernet
- Make sure that the ethernet profile is configured to Franka (see the Setting Up the Network section of [Franka FCI documentation](https://frankarobotics.github.io/docs/getting_started.html) if this is not already configured)
- Make sure that the robot joints are unlocked (accessed via the [Franka Desk](https://172.16.0.2/desk/)) and that the emergency stop button is not depressed. The robot should be in the blue light mode to begin accepting commands over FCI. This code also currently assumes that the Franka hand is attached.
- If the Franka node (Terminal 1) reports an error like `Move command rejected: command not possible in the current mode!`, depress and release the emergency stop to reset the mode. The robot should return to the blue light state and you can re-run the command

## Citation
```
@inproceedings{morton2025oscbf,
  author={Morton, Daniel and Pavone, Marco},
  booktitle={2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Safe, Task-Consistent Manipulation with Operational Space Control Barrier Functions}, 
  year={2025},
  pages={187-194},
  doi={10.1109/IROS60139.2025.11246389}
}
```
