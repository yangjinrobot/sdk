# AmpControl_Walk2Run_Mode 1026 run

AmpControl_Walk2Run_Mode 默认加载 `/opt/robot/share/motion_master/policy/1026/run_2.onnx`，
控制参数沿用同目录 `16500.yaml` 对应的现有设置；未提供 run_2 专属 YAML。
`run_2.onnx` 的输入输出为 float 438 → 21，现有递归打包规则会一并安装。
入口保持键盘 V / DDS `0x1100D`。AmpControl_NiRenWalk_Mode（键盘 C / DDS `0x11`）
使用 `nirenxingzou_fuzai/policy_15500.onnx`（78×6＝468 维）；AmpControl_GunShootWalk_Mode
使用 `std_amp_2.onnx`，DDS `0x1100B`。
同目录 `model_15500.onnx` 和 `15500.yaml` 保留，不再默认加载；
离线验证仍可使用 `python -B tests/test_1026_policy.py --mode walk`。

单帧依次为角速度 3、投影重力 3、物理速度指令 3、moving_flag 1、关节角偏差 21、
关节速度×0.05 共 21、上一动作 21，共 73 维。moving_flag 在平移速度模长 >0.15 或
转向速度绝对值 >0.15 时为 1，不输入步态相位。5 帧历史从旧到新，再接当前帧，
共 438 维；首帧观测填满历史，重入清空上一动作与速度缓存。

双肘默认角 +1.2 rad，其余默认角、PD 增益与 YAML 一致。动作缩放 0.25、动作裁剪 ±10、
观测裁剪 ±100。归一化指令限幅 ±1，X 实际范围 [-1, 3] m/s，Y ±0.6 m/s，Yaw ±1 rad/s；
沿用每帧 [0.02, 0.02, 0.04] 的归一化速度变化限制。
训练周期 0.005×4＝0.02 秒，对应控制器 1 kHz / decimation 20。
保留 1 秒手臂五列指令过渡、状态切换与阻尼保护。

离线验证：使用安装 numpy、PyYAML、onnxruntime 的 Python 执行
`python -B tests/test_1026_policy.py`。检查 YAML 默认角/PD/速度/缩放、280 帧独立
Python 观测与 C++ ONNX 动作对齐、历史重入、关节映射、moving_flag 边界，
以及真实 AmpControl_Walk2Run_Mode 状态入场、手臂过渡和无效指令保护。离线验证不执行机器人动作。
