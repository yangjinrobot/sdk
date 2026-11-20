# AMP / Mimic12 / Mimic13 接入

已接入 CR1PRO 的 `MakeProControllers`。控制器总数由 11 个增加到 14 个，原有状态编号保持不变，退役编号 9～12 不复用。

| 控制器 | DDS `target_mode` | 默认模型（相对策略根目录） |
| --- | --- | --- |
| `RLControlState` | `0x1100e` | `amp_policy/policy_1022_75k.onnx` |
| `RLMimic12ControlState` | `0x11016` | `mimic_policy/model_stand2sit_1020.onnx` |
| `RLMimic13ControlState` | `0x11017` | `mimic_policy/sit2stand.onnx` |
| `RLMimic1ControlState` | `0x11018` | `mimic_policy/policy_disco8_10k.onnx` |
| `RLMimic2ControlState` | `0x11019` | `mimic_policy/policy_simpson2_20k.onnx` |

## 操作方式

全身模式下，先按现有流程进入 RL 站立或 AMP 行走，再使用：

- `B`：RL 站立 / AMP 行走 → 新接入的 AMP。
- `N`：AMP → Mimic12 坐下。
- `M`：Mimic12 → Mimic13 起立。
- `B`：Mimic13 → AMP。
- `V` / `M`：AMP → RL 站立 / 原 AMP 行走；全身模式手柄 `L2` 也可从 AMP 直接切入原 AMP 行走（`0x1100e → 0x11`）。
- `G`：AMP 行走 → Mimic1 舞蹈；再按 `G` 从 Mimic1 返回 AMP 行走。
- `K`：AMP 行走 → Mimic2 舞蹈；再按 `K` 从 Mimic2 返回 AMP 行走。
- `R`：急停至关节阻尼；安全故障和推理失败也进入阻尼。

两支舞蹈的入口只在 `AmpWalkingControl`（`0x11`），出口只有 `AmpWalkingControl` 与阻尼。它们之间不能直接互切，也不能从 RL 站立、AMP、Mimic12/13 等状态直接进入；DDS 发送 `0x11018`/`0x11019` 时，同样只有 AMP 行走会接受。舞蹈策略沿用复制代码的末帧保持：Mimic1 计数上限 520，Mimic2 为 390，每 20 个控制周期推理一次；观测 158 维，动作为 29 维。

键盘和 UDP 的按键入口共用映射。DDS 可发送表中的编号，状态机仍按上述有向切换关系执行。坐下与起立之间由命令切换；策略继续保留复制代码中的末帧保持逻辑，不自动跳转。坐下策略计数上限为 430，起立为 360，每 20 个控制周期推理一次。

## 模型路径

默认使用项目现有 `ResolveModelPath`：`MOTION_MASTER_POLICY_DIR` 指定的策略根目录、安装目录 `/opt/robot/share/motion_master/policy/cr1pro`，或源码模型目录。安装规则已包含整个策略目录。

兼容原文件中的独立模型覆盖变量：

- AMP：`DR2_MODEL_PATH`
- Mimic12：`DR2_MIMIC12_MODEL_PATH`
- Mimic13：`DR2_MIMIC13_MODEL_PATH`
- Mimic1：`DR2_MIMIC1_MODEL_PATH`
- Mimic2：`DR2_MIMIC2_MODEL_PATH`

独立覆盖变量优先。加载失败会保存错误并拒绝进入策略，不退出主程序。模型清单已增加这三个模型的 SHA-256 和大小。

## 实现与验证范围

移除了旧项目中不存在的 `CRXModelInferenceExternal` / `ModelInferenceBase` 依赖，改用当前 `PolicyRunnerBase` 的观测、输出校验及错误回退。保留原模型的 PD 参数、关节排列、观测排列、动作缩放和参考轨迹逻辑。策略输出前 29 个关节，颈部两行补零；底层硬件保护继续作用于完整指令。状态退出时先停止并等待推理线程，再清理策略缓冲区。

离线测试覆盖真实 ONNX 推理、完整坐下/起立时间步及末帧保持、29/31 关节映射、重复进入、缺失/错误模型、异常和 NaN 回退、14 状态切换及线程退出。这些测试覆盖软件控制链路。

```bash
cmake -S . -B build/pro_logic_tests -DBUILD_PRO_TESTS_ONLY=ON
cmake --build build/pro_logic_tests -j2
ctest --test-dir build/pro_logic_tests --output-on-failure
```
