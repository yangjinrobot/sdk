# AMP MoE 1026 替换与 MuJoCo 验证

验证日期：2026-10-26。实际 SDK 状态机、C++ ONNX runner、关节保护与 MuJoCo 3.3.7 共用部署链路，未启动硬件通信。

## 默认策略与接口

- 默认模型：`run_policy/cr1pro/policy/amp_moe_policy/moe_policy_1026.onnx`。
- SHA-256：`6a0dada806be542737a971c39a7aec161e7f50eca263e542e8988e0c22d35c96`；大小 5,765,914 字节。
- 输入 `observations[batch,576]`、输出 `actions[batch,15]`，均为 float32，ONNX Runtime 加载和接口检查通过。
- 相比1022，88个计算节点及60个权重张量的名称／形状／类型一致，60个张量内容均发生变化。默认加载路径、独立数值回归模型和部署清单同步更新，启动打印实际模型路径。
- 保留当前96维观测、五帧历史加当前帧、首帧重复历史、独立15维动作映射、0.25动作缩放、20 ms策略周期，以及默认关节角和PD参数。
- 1026未附带新的训练配置或checkpoint说明，不能从文件名推断checkpoint。现有 `metadata.json` / `isaac_interface.json` 对应1022来源，保持原记录；本次以实际1026模型的接口、独立推理对照和物理回归验证现有适配。
- 1022模型保留为历史资产，默认runner只选择1026，未添加静默回退到旧策略的逻辑。

## 结果

| 验证项 | 实测结果 |
| --- | --- |
| 离线 CTest | 7/7通过，assert已启用 |
| 数值与手臂规划 | 480次1026 ONNX独立观测／映射对照、900次RL Stand手臂规划对照通过 |
| MuJoCo CTest | 4/4通过，含12个仿真用例、5个椅子用例、4个地形用例及硬件指令保护检查 |
| 自由站立 | MoE阶段10.5秒无支撑，最低base高度0.816 m，最大绝对roll/pitch为0.069 rad |
| 随机摆臂种子42／退出重入 | 最低高度0.805 m，最大绝对roll/pitch为0.098 rad，最大平移漂移0.033 m；急停与重入通过 |
| 36秒前进／六组手臂预设／退出重入 | MoE阶段28.5秒均无支撑，最低高度0.8031 m，最大绝对roll/pitch为0.0765 rad；正常返回RL Stand并重新进入MoE |
| 32秒随机摆臂种子7 | MoE阶段26.5秒均无支撑，随机目标6组；最低高度0.8153 m，最大绝对roll/pitch为0.0920 rad，最大漂移0.0290 m |
| 两项额外完整流程 | `fault_latched=false`、`fall_event=null`、`failures=[]`；所有CSV值有限，虚拟头部命令为零，无JointProtection告警 |

物理检查保持原门限：MoE高度大于0.75 m、绝对roll/pitch小于0.25 rad，随机摆臂漂移小于0.20 m；未放宽门限或固定底座来通过回归。仅前1秒有启动支撑，MoE在5.5秒进入。

1026在本模型中站立高度约0.82 m，较1022记录约0.89 m更低；上述新模型回归仍满足原有门限。验证结论限于这些场景和现有B2物理模型，不代表CR1PRO实机已通过。

## 离线回归

在仓库根目录运行：

```bash
cmake -S . -B build/pro_tests -DBUILD_PRO_TESTS_ONLY=ON \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE=-O2
cmake --build build/pro_tests -j2
ctest --test-dir build/pro_tests --output-on-failure
```
