# Stand2Knee / Knee2Stand 接入（2026-10-28）

> 历史记录。跪下和跪姿起立状态机已移除。原地址 `0x11011`、`0x11012` 现分别用于欢迎2和椅子坐姿。 当前映射见 [使用说明](USAGE.md#状态编号与切换)。

WholeBody 键盘操作流程：

| 当前状态 | 按键 | 目标状态 | 状态编号 |
|---|---|---|---|
| AmpZeroPos（完成3秒准备） | C | amp_walking_state | `0x11` |
| amp_walking_state | T | stand2knee_mimic_state | `0x11011` |
| stand2knee_mimic_state | Y | knee2stand_mimic_state | `0x11012` |
| knee2stand_mimic_state | C | amp_walking_state | `0x11` |

新状态也已加入32位DDS状态白名单；控制器仍检查合法后继。
`R`／安全故障／推理失败均可进入JointDamping。跪下阶段的 `C`、行走阶段的 `Y` 不切换状态。
进入新动作及返回行走时清零速度。动作切换由按键触发，不自动跳转；请分别为跪下和起立预留至少8秒和7秒。

## 模型与观测

默认模型为 `model_stand2knee_1028.onnx` 和 `model_knee2stand_1028.onnx`。
`MOTION_MASTER_POLICY_DIR` 可覆盖策略根目录；构造runner时也支持显式模型路径。
模型哈希见 [manifest.json](../run_policy/cr1pro/policy/manifest.json)。

- 两个模型均为151维观测、29维动作，50 Hz（主循环1 ms，decimation=20）。
- 观测依次为参考关节位置29、参考速度29、机体坐标重力方向3、机体角速度3、实际关节相对默认位置29、实际速度29、上一原始动作29。
- 重力方向为 `R_world_from_base.transpose() * (0,0,-1)`；不使用旧起立模型的6维相对姿态观测。
- 默认关节位置、action scale、kp、kd读取ONNX元数据，并校验观测／关节顺序、频率和输入输出schema。归一化由ONNX图内部完成。
- Isaac交错关节顺序与SDK腰3／左臂7／右臂7／左腿6／右腿6分别映射，两个虚拟neck命令保持零。
- 每次进入从参考帧0和零历史动作开始。Stand2Knee共384帧，7.66秒到末帧；Knee2Stand共313帧，6.24秒到末帧。之后继续推理和执行物理，参考帧保持不变。
- 复用现有 `CopiedPolicyControlState` 线程生命周期及 `BoundedPolicyCommand`，输出31×5指令后继续经过关节保护。

实现位于 [公共 runner](../run_policy/cr1pro/src/knee_mimic_policy_runner.cpp)。
