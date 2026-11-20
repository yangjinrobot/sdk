# motion_master（RL 精简版）

CR1 人形机器人运动控制程序，包含 RL 行走、跑步、站立、坐下/起立、待机蹲姿准备，以及准备与安全停机状态。旧 Run1、Run2、Run3、Run4 已停用。

## 状态与模型

| 状态 | 内部码 | 用途 |
| --- | --- | --- |
| Idle | `0x0` | 开机默认，零力矩 |
| JointDamping | `0x2` | 软急停/阻尼保护 |
| Restand | `0x1100B` | motion 1：RL Stand 原地站立 |
| Run | `0x1100D` | motion 3：原 NirenWalk，`amp/1028_run/policy_run_2.onnx` |
| Stand2SitMimic | `0x1100E` | motion 4：CR1STANDARD 站立到坐下 |
| Sit2StandMimic | `0x1100F` | motion 5：CR1STANDARD 坐下到站立 |
| Run1 | `0x11014` | motion 10：旧 Run，已停用 |
| Run2 | `0x11015` | motion 11：已停用 |
| Run3 | `0x11016` | motion 12：已停用 |
| Run4 | `0x11017` | motion 13：已停用 |
| RLControlMode | `0x11` | `amp/1118/nirenwalk.onnx` |
| ZeroPos | `0x20001` | 回零位、进入 RL 的准备状态 |
| ZeroCalibration | `0x20004` | 零位校准 |
| SuspendedStand | `0x20006` | 悬架起立、进入 RL 的准备状态 |
| SquatPreparation | `0x20010` | CR1 Standard 待机进入蹲姿准备，稳定后确认起立 |

正常流程：`Idle → SuspendedStand（或 ZeroPos）→ RLControlMode`。RL 可以请求回到 `ZeroPos` 或 `Idle`；急停/安全故障进入 `JointDamping`，阻尼状态约 10 秒后返回 `Idle`。除表中状态外，旧独立策略状态码不再受理。

手柄的「蹲姿起立」从待机发送 `0x20010`，控制器从实测关节位置用 1 秒插值到 21 轴蹲姿。每个关节的位置误差和速度在容差内保持 150 ms 后，才接受「开始站立」的 `0x1100F`。起立使用 `mimic/sit2stand_policy`，播完全部参考帧后可发送「开始运动」`0x11`。行走中仍可用 motion 4 的 `mimic/stand2sit_policy` 进入坐姿，再用 motion 5 起立。

蹲姿按 CR1PRO SDK 关节名称映射到 CR1 Standard，保留腰 yaw、双臂四关节和双腿六关节。DDS `/CR1_HANDSET_STATUS` 使用 `0x40000000` 标识准备到位或起立完成。当前 Android 通过 RobotServer 的运动状态反馈切换按钮，机器人端按实测姿态强制检查到位。

RLControlMode 从 `/opt/robot/share/motion_master/policy/amp/1118/nirenwalk.onnx` 加载模型，观测为 78×6＝468 维，包含步态相位。控制器默认双肘为 +1.2 rad，腿部默认姿态为髋 `-0.1`、膝 `0.2`、踝 `-0.1` rad，速度范围 X [-0.6, 1.0] m/s、Y ±0.6 m/s、Yaw ±1 rad/s。

`run_policy/cr1/policy/` 及其安装目录按策略类型组织：

- `amp/` 存放 `nirenwalk_18000.onnx`、1117/1118 行走和 1028 Run。RLControlMode 使用 `AmpWalkPolicyRunner` 加载 `amp/1118/nirenwalk.onnx`。
- `rl/` 存放 Restand 使用的 `RL_stand.onnx`。
- `mimic/` 存放坐下和起立的模型与动作清单。这些策略使用参考动作数据，当前导出已把参考帧嵌入 ONNX。

旧 Run1–Run4 模型归档在工作区根目录的 `policy/disabled_run_policies/`。

Run（键盘 V / Retroid B / Skydroid E / DDS `0x1100D`）是原 NirenWalk 的改名状态，继续使用 `/opt/robot/share/motion_master/policy/amp/1028_run/policy_run_2.onnx` 和 `Run1027` 参数，速度范围 X [-1, 2] m/s、Y ±0.6 m/s、Yaw ±1 rad/s。

旧 Run1（motion 10 / `0x11014`）、Run2（motion 11 / `0x11015`）、Run3（motion 12 / `0x11016`）和 Run4（motion 13 / `0x11017`）不注册控制器、不加载模型；DDS 请求会被拒绝，统一状态转换保护会保持当前状态。

RL Stand（Restand，DDS `0x1100B`）进入时双肘默认 −0.4 rad（抬起）。肩部保持站立策略默认角，站立期间持续保持选定姿态；从上一状态最后下发的手臂五列指令用 1 秒 smoothstep 过渡。每次切入时重置双肘目标。离线验证：`python -B tests/test_restand.py`。

## 控制接口

默认使用 DDS 指令接口；可通过 `MOTION_MASTER_CTRL_MODE=keyboard`、`skydroid` 或 `retroid` 选择其他接口。DDS 订阅 `/MOTION_STATE`、`/GAIT`、`/STEER`、`/REAL_STEER` 等话题。RL 行走步态为 `0x21000`，急停请求为 `0x2`。

键盘模式支持 `Z`（回零位）、`X`（悬架起立）、`C`（RL）、`V`（Run）、`R`（阻尼），以及行走中的 `W/S/A/D/Q/E` 速度控制。

CR1STANDARD 坐下/起立流程：从 RL 或 Run 按 `T` 进入坐下；按 `Y` 请求起立；起立后按 `C` 回 RL 或 `V` 回 Run。DDS 使用 `0x1100E`、`0x1100F`、`0x11`、`0x1100D`。坐下/起立各播放一次，末帧继续接受反馈并运行策略；起立完成前 200 帧后即可响应 RL、Run、Restand 或坐下请求。`Z`、`R` 和安全故障优先退出。

两个动作各有独立的 BeyondMimic runner，均保留 21 维原始动作历史，按训练顺序映射关节和增益，以 50 Hz 推理，每次进入从第零帧重置。坐下使用 `model_stand2sit_1020_21dof.onnx`，观测为 111 维，其中 `projected_gravity = Rᵀ × [0, 0, -1]` 是实际 base_link 坐标系中的 3 维单位重力矢量，对纯偏航旋转不敏感；起立使用 114 维观测，包含进入时航向对齐的 6 维参考相对姿态。支持 CR1STANDARD（21 DOF）；CR1PRO 不注册这两个控制器。模型路径依次为环境变量 `STAND2SIT_MIMIC_POLICY_DIR` / `SIT2STAND_MIMIC_POLICY_DIR`、编译时的本项目模型目录、安装目录 `/opt/robot/share/motion_master/policy/mimic/{stand2sit,sit2stand}_policy`。显式路径或已选模型目录缺少文件时加载失败，避免静默换模型。

手柄控制：Retroid 在 RL/Run 中按 `X` 坐下，按 `Y` 请求起立，起立后按 `A` 回 RL 或 `B` 回 Run；Skydroid 对应 `C` 坐下、`D` 起立、`right` 回 RL、`E` 回 Run。坐下/起立反馈码为 `0x1100E` / `0x1100F`；旧 DDS 输入 `0x18` / `0x19` 仍兼容。

离线检查命令：`python3 -B tests/test_sit_stand.py`，需要系统 g++ 和项目内的 x86 ONNX Runtime。测试独立读取 ONNX 常量，与实际推理的观测、动作、关节顺序及每帧参考对比，并验证状态周期、末帧保持、重入、错误反馈和安全退出。测试使用模拟接口，不执行机器人动作；物理动作及目标 ARM 上的 20 ms 推理时限仍需实机验证。

DDS 运动地址包括 Restand（`0x1100B`）、Run（`0x1100D`）、坐下（`0x1100E`）和起立（`0x1100F`）。RLControlMode 使用 `0x11`，站立内的双肘指令使用 `0x11012` 与 `0x11013`。

先进入 Restand（motion 1 / `0x1100B`），再通过 `/MOTION_STATE` 发送 motion 8（`0x11012`，双侧 elbow_pitch → −0.4 rad）或 motion 9（`0x11013`，双侧 elbow_pitch → +1.2 rad）。指令仅在 RL Stand 内生效，状态反馈始终为 `0x1100B`；其他状态忽略且不缓存这些命令。每次从上一次下发的双肘位置以 1 秒 smoothstep 过渡，完成后保持目标。

## 构建与部署

```bash
cmake -S . -B build/arm -DBUILD_PLATFORM=arm -DBUILD_SIM=OFF -DSEND_REMOTE=OFF -DROBOT=cr1
cmake --build build/arm -j2
cmake --build build/arm --target normal_package
```

`normal_package` 在构建目录生成 deb。`run_policy/cr1/policy/` 中的模型随包安装。更多配置和 DDS 流程见 `docs/USAGE.md`。

策略头文件位于 `run_policy/cr1/include/`，实现位于 `src/`，新增源文件在 `run_policy/cr1/sources.cmake` 登记。各 runner 实现自己的模型加载、观测、推理和动作转换。状态机通过 `PolicyRunnerBase` 接口调用具体策略。

`state_machine/humanoid/` 存放基础状态和公共状态机。CR1 Standard 专用的 Run、Restand、坐下/起立和待机蹲姿准备状态位于同级的 `state_machine/humanoid_std/`，由该目录中的 `humanoid_std_state_machine.hpp` 注册。

## x86 离线回归

测试需要 g++、Python 3、NumPy、ONNX、ONNX Runtime 和 PyYAML。可用 `-DCR1_TEST_PYTHON=/path/to/python3` 指定已安装这些依赖的解释器。

```bash
cmake -S . -B build/offline-tests -DBUILD_TESTS_ONLY=ON
cmake --build build/offline-tests -j2
ctest --test-dir build/offline-tests --output-on-failure -j2
```
