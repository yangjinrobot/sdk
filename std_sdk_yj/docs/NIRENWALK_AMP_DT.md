# NirenWalk 使用 amp_dt（2026-10-21）

> 2026-10-22：已恢复本页描述的有步态 amp_dt 模型，取消 2 秒速度等待的修改继续保留。

现有 `NirenWalk` 状态（键盘 V / DDS 0x1100C）加载：

`/opt/robot/share/motion_master/policy/amp_dt/amp_dt.onnx`

模型及原始训练导出 YAML 保存在项目根目录 `policy/amp_dt/`，不再随 deb 打包。
旧 `nirenxingzou.onnx` 保留作历史文件，当前状态不再加载它。

## 部署参数

`RunPolicyRunner` 的常量取自配套 `amp_dt.yaml`；程序不在运行时解析训练 YAML。
修改 YAML 后需同步 C++ 参数并重新运行离线校验、编译。测试会核对默认角、PD、缩放、步态和速度范围。

- 21 动作、78 维单帧观测，5 帧历史加当前帧，共 468 维；首次有效观测重复填满历史。
- 双肘默认角 -0.2 rad；其余默认角及 PD 对齐 YAML，目标角为默认角 + 0.25 × 动作。
- 动作裁剪 ±10，观测裁剪 ±100；非有限输入在裁剪前拒绝。
- 50 Hz 推理，1 kHz 状态控制循环每 20 次更新一次策略。
- 步态周期 0.85 s，左右相位偏置 0.38 / 0.88，air ratio 0.38 / 0.38。
- 依据本次确认：实际输入模型的 vx、vy、yaw 绝对值均严格小于 0.1 时，sin/cos 清零；任一达到 0.1 即保留相位。phase_ratio 保留；复位首帧相位仍全零。
- 输入速度是归一化量：X 正向 ×1.5、负向 ×0.5，Y ×0.6，Yaw ×1；限幅后范围为 X [-0.5,1.5] m/s、Y ±0.6 m/s、Yaw ±1 rad/s。键盘或 DDS 的 0.1 归一化前进量对应 0.15 m/s。
- 已取消进入状态后约 2 秒的强制零速等待，首帧即可响应速度；归一化速度增量限制 [0.02,0.02,0.04] 保留，安全控制期间仍以零速为目标。
- 原有 1 秒实测手臂位置到实时策略目标的过渡保留；真实关节位置不被重置为默认角。
- 状态切换、坐下/起立、急停及推理错误转阻尼逻辑沿用现有实现。
- 公共 runner 新参数默认值保持旧策略行为，amp_dt 单独启用自己的反向速度、裁剪和相位边界。

双枪视觉姿态未在此次移植中改写：沿用 YAML 的 -0.2 肘角及现有机器人坐标约定。
之前 sim2sim 发现当前骨架肘部有固定 -90° 旋转，因此默认枪口接近水平。
实际训练 USD 与当前骨架的姿态一致性仍待核对；离线测试及交叉编译不等同于实机步态验证。

## 离线检查

在工程根目录运行（需要 g++、numpy、PyYAML、onnxruntime）：

```bash
python tests/test_nirenwalk_amp_dt.py --report /tmp/nirenwalk_amp_dt_check.json
```

测试使用项目内 x86 ONNX Runtime 编译 C++ 检查，不连接机器人。
校验 242 个含历史的策略输入及对应动作，覆盖重新进入、无效观测、三轴相位边界、正反向范围、观测裁剪，并回归普通 RL。

## 编译

```bash
cmake -S . -B build -DBUILD_PLATFORM=arm -DBUILD_SIM=OFF -DSEND_REMOTE=OFF -DROBOT=cr1
cmake --build build -j2
```

`normal_package` 会递归包含新的 amp_dt 模型目录。编译及离线验证不会安装程序或向机器人发送命令。
