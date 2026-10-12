# motion_master（RL 精简版）

CR1 人形机器人运动控制程序。当前只保留普通 RL 运动策略，以及进入 RL 和安全停机必需的基础状态；不再提供鲁棒行走、射击、AMP 持枪、New/Old 独立状态、模仿起立或上车状态。

## 状态与模型

| 状态 | 内部码 | 用途 |
| --- | --- | --- |
| Idle | `0x0` | 开机默认，零力矩 |
| JointDamping | `0x2` | 软急停/阻尼保护 |
| RLControlMode | `0x11` | 普通 RL 运动 |
| ZeroPos | `0x20001` | 回零位、进入 RL 的准备状态 |
| ZeroCalibration | `0x20004` | 零位校准 |
| SuspendedStand | `0x20006` | 悬架起立、进入 RL 的准备状态 |

正常流程：`Idle → SuspendedStand（或 ZeroPos）→ RLControlMode`。RL 可以请求回到 `ZeroPos` 或 `Idle`；急停/安全故障进入 `JointDamping`，阻尼状态约 10 秒后返回 `Idle`。旧独立策略状态码不再受理。

普通 RL 默认从 `/opt/robot/share/motion_master/policy/new.onnx` 加载模型。`run_policy/cr1/policy/old.onnx` 按要求保留，但当前没有状态加载它；不要删除 RL 所需的 `new.onnx`、`CommonPolicyRunner` 或 ONNX Runtime。

## 控制接口

默认使用 DDS 指令接口；可通过 `MOTION_MASTER_CTRL_MODE=keyboard`、`skydroid` 或 `retroid` 选择其他接口。DDS 订阅 `/MOTION_STATE`、`/GAIT`、`/STEER`、`/REAL_STEER` 等话题。RL 行走步态为 `0x21000`，急停请求为 `0x2`。

键盘模式保留 `Z`（回零位）、`X`（悬架起立）、`C`（RL）、`R`（阻尼），以及 RL 中的 `W/S/A/D/Q/E` 速度控制。

## 构建与部署

```bash
cmake -S . -B build -DBUILD_PLATFORM=arm -DBUILD_SIM=OFF -DSEND_REMOTE=OFF -DROBOT=cr1
cmake --build build -j2
```

可在确认目标机器人与部署配置后再打包或推送；编译不会自动部署。`run_policy/cr1/policy/` 目前仅保留 `new.onnx` 和 `old.onnx`，打包规则会安装这两个文件。更多配置和 DDS 流程见 `docs/USAGE.md`。
