# restand（CR1STANDARD）

DDS `/MOTION_STATE` 的 `data.state = 0x2000C` 选择独立的 `restand` 状态。
可从 ZeroPos、SuspendedStand、RLControlMode、NirenWalk 进入；Sit2StandMimic
收到该目标后，会等起立动作完成再进入。待机时先使用原有回零或悬架起立流程。

模型为本目录的 `RL_stand.onnx`，随现有 CMake policy 目录安装到
`/opt/robot/share/motion_master/policy/rl/`。PD、默认姿态及观测参数直接写在
`run_policy/cr1/src/rl_stand_policy_runner.cpp` 的构造函数中。修改参数后需重新编译部署。
调试时可用 `RESTAND_POLICY_PATH` 指定模型文件。

- 1 kHz 控制，每 20 个周期推理一次（50 Hz）。
- 66 维单帧观测，5 帧从旧到新堆叠，共 330 维；初始历史为零。
- 观测关节顺序：左腿 6、右腿 6、腰 1、左臂 4、右臂 4。
- 输出 13 维：左腿、右腿、腰；动作裁剪到 ±10，再乘 0.25 加默认角度。
- 双臂以 C++ 中设置的 PD 保持默认姿态，进入时从实测位置平滑过渡 2 秒。
- `/STEER` 或 `/REAL_STEER` 的前后速度经现有 DDS 接口转换后，归一化到
  ±1 并乘 0.5，送入策略的实际范围为 ±0.5 m/s；横移和转向始终为零。
- 平移命令绝对值超过 0.1 m/s 时相位按 20 ms 累加，否则相位时间复位；周期 0.8 秒。
- 可切回 RL、NirenWalk、Stand2SitMimic、ZeroPos、SuspendedStand 或待机；
  急停、安全故障或推理异常优先进入 JointDamping。

观测、PD、相位和历史语义来自 `sim2sim_dr02_std_stand.py` 及原始训练参数；
结构参考 Pro SDK 的 `RLStandPolicyRunner`，没有移植 Pro 的 UDP 手臂控制。

离线检查：使用含 NumPy、ONNX Runtime 的 Python 环境执行
`python -B tests/test_restand.py`。该检查逐帧对比 Python 与 C++ 的模型输入输出，
并验证状态时序、双臂过渡、重入、退出和异常处理，不连接机器人。
