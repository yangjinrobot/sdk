# Change Log [+]=新增功能 [!]=Bug 修复 [*]=变更/更新/优化/重构
## v.1.3.0
[*]初始化版本
[*]RLControlMode 改为通过 ChiqiangWalkPolicyRunner 加载 amp/std_amp_2.onnx。
[*]移除 ChiqiangWalk 状态和手柄显示；动作地址恢复为 Restand、Run、坐下、起立、蹲下、站起、手臂抬起、手臂放下。
[*]Restand 按来源选择双肘目标：RL/ZhanLi +1.2 rad，Run/Sit2Stand -0.4 rad。
[*]进入坐下、起立、蹲下和蹲起站立 mimic 时统一只混合 8 个手臂关节，腰腿直接使用策略输出。
[!]mjrun 的 DDS 指令和状态反馈改为 `0x11018`。策略预热在启动时完成，首帧关节指令下发后确认状态。全身过渡完成后，摇杆回中才接收移动指令。
[+]RLControlMode 切换为 1118/nirenwalk.onnx。
[+]新增 ChiqiangWalk（0x1100C），复用 std_amp_2.onnx。
[*]原 NirenWalk 改名为 Run（0x1100D）；Stand、坐下/起立、蹲下/站立、肘部命令及旧 Run1-Run4 地址整体顺延。
[*]停用旧 Run1-Run4 并将其策略目录归档到 policy/disabled_run_policies。
[*]RLControlMode 默认加载 amp/nirenwalk_18000.onnx。
[*]RLControlMode 默认模型恢复为 amp/1118/nirenwalk.onnx。
[*]RLControlMode 按 1117 交接版恢复默认模型 amp/1117/nirenwalk.onnx。
[*]RLControlMode 默认模型切换为 amp/1118/nirenwalk.onnx。
