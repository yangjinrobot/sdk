# Tangxia / Zhanli 策略接入

`0x11015` 和 `0x11016` 为 SDK 保留地址。当前版本拒绝这两个 DDS 命令，运行时未注册两个控制器。以下记录保留策略启用时的实现。

```text
AmpWalking(0x11) -> Tangxia(0x11015) -> Zhanli(0x11016) -> AmpWalking(0x11)
```

`Tangxia` 只接受来自 `AmpControl_Walk_Mode` 的请求，`Zhanli` 只接受当前处于 `Tangxia` 的请求，并只能返回 `AmpControl_Walk_Mode`。直接向其他状态发送这两个编号不会改变当前状态。显式阻尼及除姿态超限外的安全故障仍可随时中止策略。

## AmpWalking 到 Tangxia 的入口过渡

从 `AmpControl_Walk_Mode 0x11` 进入 `MimicControl_LieDown_Mode 0x11015` 时，不再直接跳到 Tangxia 第一帧输出：

1. 进入 Tangxia 前读取 AMP Walking 最后下发的完整 `31×5` 指令，包含 `kp/位置/kd/目标速度/前馈力矩`。
2. Tangxia 策略从第 0 帧开始正常推理并推进帧号。
3. 使用 1 秒 smoothstep 混合：

```text
s = clamp(t / 1.0, 0, 1)
w = 3s² - 2s³
output = (1-w) × AMP Walking旧指令 + w × Tangxia当前策略指令
```

4. 混合覆盖全部 31 行，包括腰、双臂、双腿和脖子指令。
5. 主循环有效控制周期内持续重新计算并发布混合结果，不是只在 50 Hz 推理点更新。
6. 第一秒内 Tangxia 参考帧继续推进，过渡结束后直接使用当前 Tangxia 策略输出。

`Zhanli` 当前没有新增入口全身混合，因为它只能从已经完成的 `Tangxia` 进入；后续如需优化躺姿交接，应单独加入 Zhanli 入口过渡。

## 策略接口

| 项目 | Tangxia | Zhanli |
|---|---|---|
| 模型 | `recovery_policy/tangxia/model_11000.onnx` | `recovery_policy/zhanli/model_8000.onnx` |
| 帧数 | 234，末帧 233 | 317，末帧 316 |
| 观测 | `151` | `151` |
| 动作 | `29` | `29` |
| 推理周期 | 20 ms | 20 ms |
| 环境覆盖 | `DR2_TANGXIA_MODEL_PATH` | `DR2_ZHANLI_MODEL_PATH` |

观测顺序严格为：

```text
reference_joint_pos[29]
reference_joint_vel[29]
projected_gravity[3]
base_angular_velocity[3]
actual_joint_pos-default_joint_pos[29]
actual_joint_vel[29]
last_action[29]
```

参考角和实际反馈均按 ONNX `joint_names` 排列；输出目标及 PD 按显式策略到 SDK 的关节映射写回。默认角、动作尺度、KP、KD 和 `clip_actions` 全部从各自 ONNX metadata 读取，两个 runner 不共享这些配置。

## 安全处理

`SafeController` 在 `MimicControl_LieDown_Mode` 或 `MimicControl_GetUp_Mode` 期间只屏蔽 `kSafePostureNormal` 引起的软急停。IMU NaN/超时、关节更新/位置/NaN、驱动状态、温度和通信故障仍按原逻辑进入阻尼。禁止通过全局 `report_only=true` 或把姿态阈值改成 180° 来实现此功能。

## 验证状态

- 归档全部 SHA-256 校验通过。
- 两个完整 ONNX 可按 151 维观测和 50 Hz 加载，234/317 帧全程推理、末帧保持、29 轴 PD 映射、NaN 故障和重入测试通过。
- 25 个状态注册、DDS 白名单和 `AmpWalking -> Tangxia -> Zhanli -> AmpWalking` 入口约束通过。
- `AmpWalking -> Tangxia` 已验证入口首帧与 AMP Walking 最后一条 `31×5` 指令完全一致，腰部、双臂和双腿均参与 1 秒 smoothstep。
- 当前仓库的 B2 MuJoCo 资产未复现策略包记录的自由底座最终高度约 `0.896 m`；直接按 NPZ 参考角做 PD 跟踪也同样无法稳定。因此不能用该结果判断状态机接线，但在匹配的 DR02 PRO 模型和真机上仍必须重新验证接触、关节限位和入口姿态。
