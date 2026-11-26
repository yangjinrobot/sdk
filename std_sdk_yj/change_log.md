# Change Log [+]=新增功能 [!]=Bug 修复 [*]=变更/更新/优化/重构
## v.1.3.0
[*]初始化版本
[+]新增 `AmpControl_NiRenWalk_Mode`（`0x11`）State/PolicyRunner，加载 `amp/nirenxingzou_fuzai/policy_15500.onnx`。
[*]GunShootWalk 改为 `0x1100B`，RL Stand 至跪姿起立顺延到 `0x1100C`～`0x11011`，双肘命令改为 `0x11012`/`0x11013`。
[*]SuspendedStand 的 21 DOF 初始默认姿态改为双肘 +1.2 rad；ZeroPos 保持 -0.2 rad。
[*]NiRenWalk/GunShootWalk、GunShootWalk/Walk2Run 双向切换及 ChairUp/Walk2Run 切换自动经过 RLControl_Stand_Mode，复用原 1 秒手臂平滑后自动进入目标状态。
[*]按 STD 动作表统一 RobotMotionState、StateName、State/PolicyRunner 类及对应 `.cpp/.hpp/.h` 文件名。
[*]AmpControl_GunShootWalk_Mode 改为通过 AmpControlGunShootWalkModePolicyRunner 加载 amp/std_amp_2.onnx。
[*]移除 ChiqiangWalk 状态和手柄显示；动作地址恢复为 RLControl_Stand_Mode、AmpControl_Walk2Run_Mode、坐下、起立、蹲下、站起、手臂抬起、手臂放下。
[*]RLControl_Stand_Mode 按来源选择双肘目标：RL/MimicControl_Kneel_StandUp_Mode +1.2 rad，AmpControl_Walk2Run_Mode/MimicControl_ChairUp_Mode -0.4 rad。
[*]进入坐下、起立、蹲下和蹲起站立 mimic 时统一只混合 8 个手臂关节，腰腿直接使用策略输出。
[*]MimicControl_Kneel_Mode 跪姿与 MimicControl_Kneel_StandUp_Mode 跪姿起立切换为 `1125_mimic` 的 `shoot_down.onnx`、`shoot_up.onnx` 及对应 `*_groundfix_kneel.npz`；原 `dunxia.onnx`、`model_19999.onnx`、`qili.onnx` 和旧 YAML 保留用于回退。
[!]mjrun 的 DDS 指令和状态反馈改为 `0x11018`。策略预热在启动时完成，首帧关节指令下发后确认状态。全身过渡完成后，摇杆回中才接收移动指令。
[+]AmpControl_GunShootWalk_Mode 切换为 1118/nirenwalk.onnx。
[+]新增 ChiqiangWalk（0x1100C），复用 std_amp_2.onnx。
[*]活动跑步状态统一命名为 AmpControl_Walk2Run_Mode（0x1100C）；RL 站立、椅子动作、跪姿动作和肘部命令地址保持对应关系。
[*]移除旧 Run1-Run4 状态类、测试及对应枚举定义。
[*]AmpControl_GunShootWalk_Mode 默认加载 amp/nirenwalk_18000.onnx。
[*]AmpControl_GunShootWalk_Mode 默认模型恢复为 amp/1118/nirenwalk.onnx。
[*]AmpControl_GunShootWalk_Mode 按 1117 交接版恢复默认模型 amp/1117/nirenwalk.onnx。
[*]AmpControl_GunShootWalk_Mode 默认模型切换为 amp/1118/nirenwalk.onnx。
