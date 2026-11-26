# AMP / Mimic12 / Mimic13 接入

相关控制器由 `ProControllers` 注册。协议编号 `0x9` 至 `0xc` 和 `0x11017` 至 `0x1101b` 已退役。

| 控制器 | DDS `target_mode` | 默认模型（相对策略根目录） |
| --- | --- | --- |
| `RLControlState` | `0x1100c` | `amp_policy/policy_1022_75k.onnx` |
| `RLMimic12ControlState` | `0x11012` | `mimic_policy/model_stand2sit_1020.onnx` |
| `RLMimic13ControlState` | `0x11013` | `mimic_policy/sit2stand.onnx` |
| `RLMimic2ControlState`（`MimicControl_BoxFight_Mode`） | `0x11014` | `mimic_policy/box_fight.onnx` |

## 操作方式

全身模式下，先按现有流程进入 RL 站立或 AMP 行走，再使用：

- `B`：RL 站立 / AMP 行走 → 新接入的 AMP。
- `N`：AMP → Mimic12 坐下。
- `M`：Mimic12 → Mimic13 起立。
- `M` 在起立完成后请求返回普通 AMP 行走。
- `V` / `M`：AMP → RL 站立 / 原 AMP 行走；全身模式手柄 `L2` 也可从 AMP 直接切入原 AMP 行走（`0x1100c → 0x11`）。
- `K` 在 AMP 行走与低 KP 拳击之间切换。
- `R`：急停至关节阻尼；安全故障和推理失败也进入阻尼。

低 KP 拳击状态（`0x11014`）仅从 `AmpControl_Walk_Mode`（`0x11`）进入，出口为 AMP 行走和关节阻尼。DDS 拒绝已退役的 `0x11019`。

键盘和 UDP 的按键入口共用映射。DDS 可发送表中的编号，状态机仍按上述有向切换关系执行。坐下与起立之间由命令切换；策略继续保留复制代码中的末帧保持逻辑，不自动跳转。坐下策略计数上限为 430，起立为 360，每 20 个控制周期推理一次。

## 模型路径

默认使用项目现有 `ResolveModelPath`：`MOTION_MASTER_POLICY_DIR` 指定的策略根目录、安装目录 `/opt/robot/share/motion_master/policy/cr1pro`，或源码模型目录。安装规则已包含整个策略目录。

兼容原文件中的独立模型覆盖变量：

- AMP：`DR2_MODEL_PATH`
- Mimic12：`DR2_MIMIC12_MODEL_PATH`
- Mimic13：`DR2_MIMIC13_MODEL_PATH`
- 低 KP 拳击使用 `DR2_MIMIC2_MODEL_PATH`。

独立覆盖变量优先。加载失败会保存错误并拒绝进入策略，不退出主程序。模型清单已增加这三个模型的 SHA-256 和大小。

## 实现与验证范围

移除了旧项目中不存在的 `CRXModelInferenceExternal` / `ModelInferenceBase` 依赖，改用当前 `PolicyRunnerBase` 的观测、输出校验及错误回退。保留原模型的 PD 参数、关节排列、观测排列、动作缩放和参考轨迹逻辑。策略输出前 29 个关节，颈部两行补零；底层硬件保护继续作用于完整指令。状态退出时先停止并等待推理线程，再清理策略缓冲区。

离线测试覆盖真实 ONNX 推理、完整坐下/起立时间步及末帧保持、29/31 关节映射、重复进入、缺失/错误模型、异常和 NaN 回退、状态切换及线程退出。

```bash
cmake -S . -B build/pro_logic_tests -DBUILD_PRO_TESTS_ONLY=ON
cmake --build build/pro_logic_tests -j2
ctest --test-dir build/pro_logic_tests --output-on-failure
```
