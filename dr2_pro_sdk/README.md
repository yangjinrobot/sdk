# DR2 Pro SDK

DR2 Pro 的 ROS 2 运动控制 SDK，包含状态机、DDS 消息、键盘/手柄控制、
MuJoCo 仿真和 ROS 2 话题示例。当前代码支持上半身与全身控制，并加入了
RL 站立、AMP 行走、深度行走、倒地起身以及仿真手柄速度输入。

> 安全提示：真机运行前应确认急停、控制模式、关节空间和周边环境均安全。
> 策略文件缺失或输入异常时，相关状态会回退到关节阻尼，但这不能替代现场安全措施。

## 仓库结构

- `drdds/`：ROS 2 消息与服务定义。
- `dr2_pro_sdk/state_machine/`：运动控制状态机和策略运行器。
- `dr2_pro_sdk/example/`：独立的 ROS 2 话题示例。
- `dr2_pro_sdk/simulation/`：MuJoCo 仿真脚本和机器人描述文件。
- `dr2_pro_sdk/运控.md`：按源码整理的详细状态切换与操作说明。

## 环境与依赖

推荐 Ubuntu 22.04、ROS 2 Humble、CMake 3.10+ 和支持 C++17 的编译器。
真机 ARM 构建可使用 ROS 2 Jazzy 与 AArch64 工具链。主要依赖包括：

- ROS 2：`ament_cmake`、`rclcpp`、`sensor_msgs`、`std_msgs`；
- C++：OpenCV、Eigen、ONNX Runtime 1.22.0；
- Python 仿真：Python 3、`numpy<2.0`、`mujoco`；
- 可选调试工具：Python OpenCV、Linux joystick API。

Ubuntu/ROS 2 Humble 的常用安装命令：

```bash
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions \
  libopencv-dev ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-std-msgs
python3 -m pip install "numpy<2.0" mujoco opencv-python
```

## 运行资源说明

仓库已包含 `state_machine/policy/` 下的 ONNX 策略权重。当前代码直接使用的
模型如下，路径必须保持不变：

| 用途 | 仓库内路径 |
| --- | --- |
| RL 站立 | `state_machine/policy/model_0610_rl_moveboxes.onnx` |
| AMP 行走 | `state_machine/policy/policy_0610_walking.onnx` |
| WholeBody / Run | `state_machine/policy/policy_0731.onnx` |
| Arm | `state_machine/policy/policy_0709_1.onnx` |
| 倒地起身 | `state_machine/policy/policy_stand_0907_dr02_2.onnx` |
| 真机深度行走 | `state_machine/policy/actor_0911.onnx`、`context_encoder_0911.onnx` |
| 仿真深度行走 | `state_machine/policy/actor_0902.onnx`、`context_encoder_0902.onnx` |

平台动态库、机器人网格、APK、压缩归档和生成目录仍不提交。请从有权使用的
原始 DR2 Pro SDK 发布包中恢复以下资源：

| 用途 | 需要恢复的路径 |
| --- | --- |
| x86 ONNX Runtime | `state_machine/third_party/onnxruntime/x86/lib/*.so*` |
| ARM ONNX Runtime | `state_machine/third_party/onnxruntime/arm/lib/*.so*` |
| MuJoCo 可视化网格 | `simulation/DR2-B2-PRO-URDF/meshes/` |

仓库内的 ONNX Runtime 头文件版本为 1.22.0；也可以从 ONNX Runtime 官方
1.22.0 发布包提取与目标架构匹配的共享库。网格没有公共下载地址时，请向原
SDK 提供方索取。`controlapp.apk` 仅为可选手柄应用，不影响源码编译。

## 控制模式

在 `state_machine/main.cpp` 中选择模式：

```cpp
const ControlMode mode = ControlMode::kWholeBody;
```

可改为 `ControlMode::kUpperBody`。仿真脚本与真机 SDK 服务模式必须和这里一致。

全身模式的主要流程为：

```text
Idle --Z--> ZeroPos --C--> WholeBodyControl
                   \--U--> RLStand --M--> AmpWalking
                   \--N--> DepthWalk
                   \--P--> GroundStand
```

详细按键、状态可达性和安全逻辑见 [`运控.md`](./%E8%BF%90%E6%8E%A7.md)。

## 构建

在包含 `drdds/` 与 `dr2_pro_sdk/` 的工作空间根目录执行。

### x86 仿真

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to drdds dr2_pro_sdk --cmake-args \
  -DBUILD_PLATFORM=x86 -DBUILD_SIM=on -DPython3_EXECUTABLE=/usr/bin/python3
```

### ARM 真机

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drdds dr2_pro_sdk --cmake-args \
  -DBUILD_PLATFORM=arm -DBUILD_SIM=off
```

## 运行仿真

终端 1：

```bash
export ROS_DOMAIN_ID=1
source install/setup.bash
ros2 run dr2_pro_sdk state_machine
```

终端 2（全身模式）：

```bash
export ROS_DOMAIN_ID=1
source install/setup.bash
python3 dr2_pro_sdk/simulation/mujoco_simulation_whole.py
```

使用倒地初始姿态时添加 `--ground`，然后在状态机终端按 `P` 进入起身策略。
上半身模式改用 `mujoco_simulation_upper.py`。

可选的仿真手柄速度节点：

```bash
source install/setup.bash
ros2 run dr2_pro_sdk joystick_velocity_control.py --device /dev/input/js0
```

默认轴为前后 `1`、侧向 `0`、偏航 `2`；可用 `--forward-axis`、
`--lateral-axis` 和 `--yaw-axis` 调整。该节点只发布 `/SIM_STEER`，不会切换状态。

## 真机运行

1. 使用厂商界面选择与编译配置一致的 Whole Body 或 Upper Body SDK 模式。
2. 通过组织批准的安全传输方式部署工作空间和未入库资源；主机地址、账号与
   凭据应通过私密渠道配置，不应写入仓库。
3. 在机器人工作空间中执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run dr2_pro_sdk state_machine
```

退出时优先使用手柄红色停止键，或在厂商界面关闭 SDK 模式。

## 话题示例与深度调试

每个 `example/*.cpp` 会生成同名可执行程序，例如：

```bash
source install/setup.bash
ros2 run dr2_pro_sdk lidar_data_example
```

根目录下的 `depth_tcp_client.py` 可将 TCP 深度帧转发为 ROS 2 `Image`，
`depth_ros_viewer.py` 可显示原始深度图和策略预处理结果。服务地址通过
`DEPTH_SERVER_HOST`、`DEPTH_SERVER_PORT` 环境变量配置。

## 来源与许可证

本项目基于 Deep Robotics（深度机器人）提供的 DR2 Pro SDK 源码整理，
保留了原有包信息和第三方来源说明。`dr2_pro_sdk/package.xml` 将主包声明为
BSD 3-Clause；Eigen、ONNX Runtime、fmt/gamepad 等第三方组件仍分别受其目录内
的 `LICENSE`、`COPYING*` 和通知文件约束。使用或再分发模型、网格、机器人固件
及厂商资源前，请确认已获得相应授权。
