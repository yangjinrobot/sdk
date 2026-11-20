# AMP MoE 部署链路检查

检查日期：2026-10-23。问题：真机使用 UDP 14 关节目标包驱动手臂时，下肢稳定性差于 `simtosim.py`。没有真机日志，也没有当时的 simtosim 启动参数。

## 结论与证据边界

当前未发现 MoE runner 的观测排列、历史拼接、动作映射或导出 PD 数值错误。已经确认：实际 UDP 手臂规划不遵守训练的手臂范围与固定腕部约束；其换向方式和仿真随机手臂也不同。这是应优先修正、对照验证的部署差异，但尚不能证明它就是此次真机失稳的唯一原因。

本次仅检查代码、执行离线 ONNX 对照和本机回环 UDP 复现，没有连接真机、没有修改控制源码或 PD 参数。完整复现文件在 [审计附件](amp_moe_audit_20261023/)。

## 1. 已核对正确的模型接口

参考 `../train_document/b2_dwaq_env(1).py`、`walk_dwaq_cfg(1).py`、`arm_randomization.py` 和策略目录的导出接口。

| 项目 | 检查结果 |
| --- | --- |
| 模型及配置版本 | 模型、env(10).yaml、agent(10).yaml、isaac_interface.json 的 SHA256 与 metadata.json 一致 |
| 观测 | 角速度 3、重力投影 3、速度命令 3、29 轴相对位置、29 轴速度、29 轴上一动作，共 96 维 |
| 关节排列 | 观测排列与 Isaac 导出顺序一致；15 动作独立映射到腰 3 轴和双腿 12 轴 |
| 历史 | 当前帧先进入五帧历史，再拼接当前帧，共 576；首次重复当前帧初始化 |
| 缩放 | 关节速度 ×0.05、动作 ×0.25、观测/动作裁剪 ±100；外部手臂的上一策略动作槽为零 |
| 默认角度、增益 | 默认角度最大浮点差 1.19e-8 rad，Kp/Kd 差为零 |
| 周期设置 | 训练 0.005 s ×4 =0.020 s；runner 固定 dt=0.020 s，状态机每 20 tick 调用 |
| 手柄限速 | x 最大 ±0.8 m/s，保留用户此前指定的范围 |

相关实现：[runner](../run_policy/cr1pro/include/amp_moe_15dof_policy_runner.hpp)。训练使用当前手臂**实际反馈**作为观测；没有把下一帧外部手臂目标加入观测，因此不能简单将 UDP 目标填入手臂 action 槽，这会改变模型输入语义。

重新构建并运行现有 `amp_moe_15dof` CTest 通过：480 次 ONNX 数值对照和 900 次 RL Stand 手臂规划对照。这些测试没有覆盖 UDP 手臂分支或真实执行器。

## 2. 优先问题：UDP 允许训练之外的手臂目标

训练 `ArmRandCfg` 与 `ArmRandomizationMixin` 共同约束手臂：

- `fix_wrist_joints=True`：六个腕关节目标固定为默认角度（本模型均为零）。配置中虽有腕部 safe range，最终仍被固定腕部逻辑覆盖。
- 肩 yaw 限于 [-1.4, 1.4] rad；肘限于 [0.3, 1.6] rad。
- 左肩 roll [0, 2.5]、右肩 roll [-2.9, 0] rad，并与资产关节软限位留 0.1 rad 后的范围取交集。
- 站立与行走使用不同的幅度分布；还有静止、保持姿态、波动、动作库、随机摆动多种模式。

[共享规划器的 UDP 分支](../run_policy/cr1pro/include/rl_arm_motion_planner.hpp) 只使用更宽的机械限位，允许腕部运动、肩 yaw 超过 ±1.4、肘低于 0.3；未区分 MoE 模型的训练约束。其他使用该共享规划器的状态可能有不同需求，后续应给 MoE 单独配置，避免直接更改全部状态的行为。

本机实际 UDP 复现使用真实 `UdpNode`、MoE runner 和 ONNX，但反馈是固定的测试数据，无物理仿真。发送一个 14×float 包后，运行 100 个控制步：

```text
shoulder_yaw_goal_rad = 2.0
elbow_goal_rad        = 0.0
wrist_yaw_goal_rad    = 0.6
```

上述目标均进入 runner 输出，分别超出训练的 yaw、肘、腕部约束。这个试验证明代码可输出这些目标，并不表示用户实际发送的目标就是这些值。必须记录真实 UDP 轨迹才能判断此次故障是否触发了这些差异。

## 3. 速度限制不能保证换向平滑

UDP 每轴使用位置增量限速：肩部 1.5 rad/s，肘腕 2 rad/s；没有加速度限制。在 20 ms 离散周期内，肩部目标速度可以直接从 -1.5 变成 +1.5 rad/s。本机复现的目标有限差分加速度为 **150 rad/s²**。这是目标轨迹的离散变化，不是真实关节测得的加速度。

训练也不是所有模式都严格加速度连续：wave 加噪声、模式切换等会产生变化。因此不能断言所有快速变化都超出训练；但 UDP 换向/接管与训练控制器不是同一套轨迹生成过程。建议在 MoE 专用手臂入口中平滑接管，并显式约束速度和加速度，再与同一条输入轨迹做仿真对照。

另外，该分支具有这些行为：

- 收到一次目标后持续接管，直到规划器 Reset 或 UDP 节点为空；本机复现停包约 2.12 s 后仍保持该目标。没有目标包年龄判断。应明确停包时是保持还是平滑退出，而不是未经设计就切换目标。
- UDP 分支在 `disable_arm_motion_planning` 处理之后执行，接管后仍覆盖该标志处理的结果；预设动作也会被覆盖。
- 每轴独立调用 `getTargetPosition` 加锁读取，一包更新可能发生在两个轴读取之间。可改用已有的 `getTargetPositionsArray` 一次获取整包，避免同一控制步混用两帧。
- `sendCurrentStates()` 同步位于策略线程内。不能由源码断定它造成超时，但需要测量其耗时，不应忽略它对策略周期的影响。

## 4. 既有仿真没有覆盖完整的 UDP 链路

`simtosim.py` 的默认值是 `--pd-mode position`、`--arm-mode fixed`。没有原启动参数，不能认定用户当时确实用了默认值。position 模式采用 MuJoCo 隐式位置伺服；torque 模式才逐物理子步计算 `Kp*(qdes-q)-Kd*dq`。二者均没有真实总线、固件、IMU 滤波与数据传输延迟。

`simtosim.py --arm-mode random` 使用训练随机控制器；此前 SDK 仿真的 `--moe-arm-mode random` 则使用独立的 Python 五次插值随机器，通过仿真 adapter 覆盖手臂目标。bridge 将 UDP 节点设为 nullptr，**不经过真实 UDP 接收和规划分支**。

所以之前“大幅摆臂不倒”的结果，验证的是该仿真模型和那组平滑手臂轨迹下的控制表现，不能当作真机 UDP 链路已验证。后续对照应将同一份 14 轴 UDP 数据经过同一个 C++ 规划器，再送入 MuJoCo，而不是更换随机种子后直接比较。

## 5. 执行器能力存在静态差异，尚无饱和证据

| 关节 | simtosim 导出 effort limit | CR1PRO ControlParameters 声明值 |
| --- | ---: | ---: |
| hip roll | 363 Nm | 107 Nm |
| ankle roll | 137 Nm | 31 Nm |
| hip yaw / ankle pitch | 137 Nm | 107 Nm |

来源：[导出接口](../run_policy/cr1pro/policy/amp_moe_policy/isaac_interface.json)、[硬件参数](../state_machine/parameters/cr1_pro_control_parameters.cpp)。横向恢复能力值得重点核查。不过右列是 SDK 声明值，尚未证明等于电机固件的实际输出上限；`JointProtection` 对命令第五列的检查针对前馈力矩，runner 的前馈为零，不等于它在软件中限制了最终 PD 总力矩。

此前 seed7 大幅摆臂 CSV 的 20 ms 采样请求力矩峰值：左右 hip roll 约 38.8/36.9 Nm，左右 ankle roll 约 5.8/6.5 Nm，均低于上表 SDK 声明值。因此静态限值差异本身不足以解释该组仿真结果，也不能据此断言真机已饱和。需要真实力矩/电流和固件限值，且 20 ms 采样不能排除子步峰值。

B2 仿真资产与实际 CR1PRO 的手臂质量、质心、惯量和末端负载也尚未完成标定对照。大幅摆臂会放大这些模型差异。

## 6. 必须测量的时序、IMU 与固件接口

- [状态机](../state_machine/humanoid_pro/amp_moe_15dof_state.hpp) 按 tick 数推理，而 [TimeTool](../utils/basic_function.hpp) 读取 timerfd 后没有按过期次数补计。若发生调度延迟，20 tick 可能超过 20 ms，runner 的固定 dt 和观测历史间隔随之不再一致。目前没有策略周期和输入年龄的诊断，不能证明已发生超时。
- 硬件分别获取 q/dq/tau 和 IMU，没有在该 runner 入口校验统一时间戳与反馈新鲜度。仿真状态则同步可得。
- [IMU 接口](../interface/robot/hardware/ecan_hardware_interface.hpp) 将腿部/骨盆 IMU 的 RPY 从度转弧度，角速度直接透传。训练需要基座坐标系的 rad/s。上游发布单位与安装轴向未在所检查文件中明确，不能直接乘/除 57.3，应先核实单位、方向和延迟。
- 同一硬件接口在协议封装时将 Kp/Kd 乘 10；底层结构使用整数。可能是定点编码，缺少固件协议依据时不能认定物理增益多了十倍，也不能直接删除乘 10。
- [模型路径解析](../run_policy/cr1pro/include/policy_paths.hpp) 优先使用 `MOTION_MASTER_POLICY_DIR`，其次 `/opt/robot/share/motion_master/policy/cr1pro`，然后源码目录。检查源码通过不等于正在运行的机器人加载了相同模型和二进制。当前模型 SHA256：`4aa1c8c60c4920c4f353c59cfc994b35fdef1448a8671210122bf5b594570c9f`。

## 后续处理顺序

1. 为 MoE 配置独立的 UDP 手臂约束：腕部默认角、训练肩肘范围、整包快照、平滑接管、速度/加速度限制和明确的停包行为；不要通过全局修改共享规划器影响其他状态。
2. 记录同一条 UDP 轨迹，走实际 C++ 规划器在 MuJoCo 回放，对照固定手臂/受约束手臂/原始目标；逐步加入执行器实测限值和反馈延迟，使用 torque PD 对照。
3. 真机诊断至少记录单调时钟、状态、模型路径/哈希、实际推理间隔和耗时、UDP 包序号/年龄、原始及规划后手臂目标、全身 q/dq/实际力矩、下发 qdes/Kp/Kd、IMU 角速度/姿态及年龄、保护事件。用这些数据区分目标轨迹、反馈延迟、扭矩饱和和标定问题。

本次证据支持先处理 UDP 与训练的差异，并补齐真实执行时序测量；尚不支持用增大腿部 PD 或放宽电机力矩上限来解决。
