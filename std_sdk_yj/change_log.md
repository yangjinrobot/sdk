# Change Log [+]=新增功能 [!]=Bug 修复 [*]=变更/更新/优化/重构
## v.1.3.0
[*]初始化版本
[+]RLControlMode 切换为 1118/nirenwalk.onnx。
[*]原 NirenWalk 改名为 Run（0x1100D）；Stand、坐下/起立、肘部命令及旧 Run1-Run4 地址整体顺延。
[*]停用旧 Run1-Run4 并将其策略目录归档到 policy/disabled_run_policies。
[*]RLControlMode 默认加载 amp/nirenwalk_18000.onnx。
[*]RLControlMode 默认模型恢复为 amp/1118/nirenwalk.onnx。
[*]RLControlMode 按 1117 交接版恢复默认模型 amp/1117/nirenwalk.onnx。
[*]RLControlMode 默认模型切换为 amp/1118/nirenwalk.onnx。
