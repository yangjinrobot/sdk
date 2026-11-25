# Stand2Knee / Knee2Stand 接入与 MuJoCo 验证（2026-10-28）

后续末帧独立手臂控制改动及新的稳定性结果见 [末帧手臂控制](knee_terminal_arms_20261028.md)。下文为最初接入验证记录。

WholeBody 键盘操作流程如下：

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

模型直接使用提供目录中的 `model_stand2knee_1027.onnx`、`model_knee2stand_1027.onnx`。
`MOTION_MASTER_POLICY_DIR` 可覆盖策略根目录；构造runner时也支持显式模型路径。
模型哈希已加入 [manifest.json](../run_policy/cr1pro/policy/manifest.json)。

- 两个模型均为151维观测、29维动作，50 Hz（主循环1 ms，decimation=20）。
- 观测依次为参考关节位置29、参考速度29、机体坐标重力方向3、机体角速度3、实际关节相对默认位置29、实际速度29、上一原始动作29。
- 重力方向为 `R_world_from_base.transpose() * (0,0,-1)`；不使用旧起立模型的6维相对姿态观测。
- 默认关节位置、action scale、kp、kd读取ONNX元数据，并校验观测／关节顺序、频率和输入输出schema。归一化由ONNX图内部完成。
- Isaac交错关节顺序与SDK腰3／左臂7／右臂7／左腿6／右腿6分别映射，两个虚拟neck命令保持零。
- 每次进入从参考帧0和零历史动作开始。Stand2Knee共384帧，7.66秒到末帧；Knee2Stand共313帧，6.24秒到末帧。之后继续推理和执行物理，参考帧保持不变。
- `Stand2KneeMimicState` 和 `Knee2StandMimicState` 各自管理策略线程和状态切换，通过 `BoundedPolicyCommand` 生成31×5指令，再交给关节保护。仿真没有关闭保护或放宽0.45 m／1 rad倒地阈值；日志中的L3限幅记录仍可能出现。

实现位于 [公共runner](../run_policy/cr1pro/src/knee_mimic_policy_runner.cpp)、
[跪下状态](../state_machine/humanoid_pro/stand2knee_mimic_state.hpp)、
[起立状态](../state_machine/humanoid_pro/knee2stand_mimic_state.hpp)。

## 历史验证结果

26秒无窗口流程已通过，日志 `fault_latched=false`、`fall_event=null`、`failures=[]`。
状态依次为Idle、AmpZeroPos、AmpWalking、Stand2Knee、Knee2Stand、AmpWalking。
跪下阶段最低base高度0.4703 m、最大绝对roll/pitch为0.1295 rad；起立后base恢复约0.897 m。

45秒连续两轮流程也通过，第二轮使用同一状态实例重新进入。两轮最低base高度0.4633 m，最大绝对roll/pitch为0.1443 rad，最后返回AMP Walking。

数值回归 `knee_mimic` 使用独立ONNX会话按参考公式构造观测，对照两个模型的全部参考帧及额外100帧保持，
检查动作、PD映射、重入、虚拟neck隔离、错误维度模型和NaN故障。
状态回归覆盖16状态合法后继、首帧输出、worker失败和退出线程同步；MuJoCo回归另外检查新流程的实际跪姿、起立高度、无支撑状态、保护拒绝计数及急停。

```bash
ctest --test-dir build/pro-tests --output-on-failure
```

最终结果：C++回归9/9通过，MuJoCo CTest 4/4通过（其中状态／策略／物理测试14项）。
最终154维错误模型拒绝测试也单独复验通过。

验证范围是项目现有B2 MuJoCo模型和实际C++状态／策略／保护路径，未运行真机。
