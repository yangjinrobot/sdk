# motion_master：CR1PRO 状态机

当前启用 18 个 CR1PRO 状态，其中 `0x1100c..0x11012` 依次对应 AMP MoE、AMP WAQ、AMP、Mimic12 坐下、Mimic13 起立、站到跪和跪到站。启用列表位于 `types/custom_types.h` 的 `kRobotMotionStates`。敬礼复用 RL 站立 ONNX 模型。原搬箱／蹲起状态的协议编号 `0x9..0xc` 继续作废。
主程序默认 **CR1PRO + DDS手柄控制 + WholeBody**，通过现有 CR1PRO 硬件接口输出31轴命令。

**手柄状态协议：WaitingForStand/Idle=`0x0`、JointDamping=`0x2`、AmpZeroPos=`0x20006`、AmpWalkingControl=`0x11`、RLStandControlMode=`0x1100b`、AmpMoe15Dof=`0x1100c`、AmpWalkingWAQ=`0x1100d`、RLControlMode=`0x1100e`、Mimic12=`0x1100f`、Mimic13=`0x11010`、Stand2Knee=`0x11011`、Knee2Stand=`0x11012`、RLStandSalute=`0x11013`。独立ZeroPos使用`0x20002`，避免与阻尼冲突。**

- 策略位于 `run_policy/cr1pro/`，模型路径及 SHA-256 见 [模型清单](run_policy/cr1pro/policy/manifest.json)。旧 `run_policy/cr1` 和旧 `model_inference` 实现已移除。
- 默认DDS接收 `/MOTION_STATE`、`/STEER`、`/REAL_STEER`、`/GAIT`、`/NECK_STEER`，也保留 `/GAMEPAD_KEY` 和GUI／手臂UDP入口。摇杆数据超过2秒未更新时清零速度。
- 单关节标零：在 Idle（`0x0`）向 DDS domain 0 的 `/JOINTS_MARK_ZERO` 发送 `std_msgs::msg::UInt64`，第 `i` 位对应 SDK 从 0 开始的第 `i` 个关节（31轴有效位为 0～30），例如第0轴为 `1`、第6轴为 `64`。Idle 执行驱动标零，等待5ms，再重置所选关节偏置并进行整圈修正；连续待处理请求按位合并，非Idle期间的请求丢弃。该操作与 ZeroPos／AMP ZeroPos 准备姿态不同。
- `MOTION_MASTER_CTRL_MODE=keyboard` 可切回键盘；还支持 `dds`、`retroid`、`skydroid`，运行参数快照记录实际选择。
- 所有指令模式均订阅 `/MOTION_STATE`，因此手柄 App 的 motion_1（`0x1100b`）可从 AMP Walking（`0x11`）请求 RL Stand；物理 R1 或键盘 `V` 使用同一目标状态。接收结果以 `[DDS] motion request current=... target=... accepted/rejected` 同时输出到终端和 `/var/opt/robot/log/` 下的 `motion_master` 日志，拒绝时标明状态已停用或急停尚未确认。
- 全身模式下，手柄 `L1` 请求 AMP ZeroPos，`L2` 从 AMP 准备态、RL Stand、AMP、MoE、WAQ 或手臂动作状态进入 AMP Walking。`R1` 从 AMP Walking、AMP、MoE 或 WAQ 进入 RL Stand，`R2` 从 RL Stand 进入 AMP MoE。键盘 `R` 请求阻尼。
- 新增坐下／起立流程：从RL站立或AMP行走按 `B → N → M → B`，依次进入AMP、坐下、起立并返回AMP；[接入说明](docs/amp_mimic_integration.md)包含模型覆盖变量和状态编号。
- 跪下／起立流程中，按 `Z` 准备3秒，再按 `C → T → Y → C` 依次进入AMP Walking、Stand2Knee（`0x11011`）、Knee2Stand（`0x11012`）并返回AMP Walking。跪下和起立分别预留至少8秒和7秒。[接入记录](docs/knee_mimic_20261028.md)
- 站立敬礼可通过手柄DDS发送 `0x11013`（`rl_stand_salute_state`）。右臂抬到敬礼姿态并保持。收到 `0x11` 或 L2 请求后先用2.4秒放下右臂，反馈到位才切换AMP Walking。[接入和验证](docs/rl_stand_salute_20261028.md)
- UDP 默认 `10.21.33.103:8888 → 10.21.33.7:9999`，在 `conf/config.toml` 的 `[pro_sdk]` 中配置或关闭。
- 31轴SDK命令由29轴策略输出和2轴脖子命令组成；4套RL网络和AMP MoE网络本身输出15维动作，再补成29轴。

## 构建与离线回归

```bash
cmake -S . -B build/pro-tests -DBUILD_PRO_TESTS_ONLY=ON
cmake --build build/pro-tests -j2
ctest --test-dir build/pro-tests --output-on-failure
```

ARM硬件程序：

```bash
cmake -S . -B build/arm -DBUILD_PLATFORM=arm -DSEND_REMOTE=OFF
cmake --build build/arm -j2
```

x86开发机交叉编译需要 `aarch64-linux-gnu-gcc/g++`。`SEND_REMOTE=OFF` 不自动推送。

模型部署目录为 `/opt/robot/share/motion_master/policy/cr1pro/`；可通过 `MOTION_MASTER_POLICY_DIR` 显式覆盖。
完整状态图、所有按键、模型维度、部署与配置见 [使用说明](docs/USAGE.md)。

本轮离线源算法对照中，7策略共64,960个浮点指令逐字节一致。源策略基于B2训练物理资源；
代码和离线推理验证尚不能证明其在CR1PRO上的物理等价性或动作稳定性，本轮未启动实际机器人。

## AMP MoE

AMP MoE 键盘流程为 `Z → C → V → P`；`V` 返回 RL Stand，`M` 返回 AMP Walking。
AMP MoE 默认策略为 `moe_policy_1026.onnx`，模型哈希和接口见 [1026验证记录](docs/amp_moe_1026_verification.md)。
MoE观测为29维交错关节顺序，动作独立使用“腰3轴、左腿6轴、右腿6轴”，上次动作按此映射回观测。
