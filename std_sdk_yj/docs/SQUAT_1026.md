# 1026 蹲下与站立迁移

| 手柄 motion | RobotMotionState | DDS /MOTION_STATE 地址 |
| --- | --- | --- |
| 原有 RL | RLControlMode | `0x11` |
| 1 | Restand | `0x1100B` |
| 2 | Run（原 NirenWalk） | `0x1100C` |
| 3 | Stand2SitMimic | `0x1100D` |
| 4 | Sit2StandMimic | `0x1100E` |
| 5 | DunXia | `0x1100F` |
| 6 | ZhanLi | `0x11010` |

连续地址中，`0x1100C` 为改名后的 Run；Stand 内双肘命令为 motion 7 / `0x11011` 和 motion 8 / `0x11012`。旧 Run1–Run4 为 `0x11014`～`0x11017` 且均已停用。旧 `0x2000B` 兼容为 Run，旧 `0x2000C` 为 Restand，旧 `0x2000D`/`0x18` 为坐下，旧 `0x2000E`/`0x19` 为起立。

DunXia 与 ZhanLi 各有完整的 State 类和 PolicyRunner 类，直接继承 BeyondMimicPolicyRunner；不调用、嵌套或派生自 Sit2StandMimic 状态/策略。

DunXia 从 RLControlMode、Run、Restand 或完成的 ZhanLi 进入。轨迹 267 帧、50 Hz，共 5.34 秒。最终动作必须执行完整 20 ms 才接受 ZhanLi；蹲姿不能直接切到行走或 Stand。ZhanLi 只能从 DunXia 进入，轨迹 350 帧、50 Hz，共 7 秒。最终动作执行完整 20 ms 后才响应 RL、Restand、Run、坐下或蹲下请求。

进入时只读取实测反馈，不改实测姿态；清零速度指令，清空上一动作与轨迹索引，按进入时实际偏航对齐参考航向。控制周期为 1 ms，推理 decimation=20，持续检查反馈、动作、目标角的有限性及推理耗时；无效数据或单次推理超过 20 ms 进入阻尼。离线测试用锁步模式关闭机器性能相关的超时检查。

进入 DunXia、ZhanLi、Stand2SitMimic 和 Sit2StandMimic 时，都只对 SDK 索引 1–8 的 8 个手臂关节做 1 秒 smoothstep 入场混合。混合覆盖手臂的 kp、目标位置、kd、目标速度和前馈力矩五列，起点为上一状态最后实际下发的命令。腰、腿不参与该入场混合，直接使用对应 mimic 策略输出。

观测依次为参考关节位置 21、参考速度 21、相对 anchor 朝向 6（旋转矩阵前两列按行展开）、实际角速度 3、实际关节位置减训练默认角 21、实际关节速度 21、上一原始动作 21，总计 114 维。不做 AMP 动作裁剪、缩放或历史堆叠。输出目标角为训练默认角 + YAML 精确 action_scale × ONNX 原始动作，参考取自模型内嵌常量。

两个模型的 ONNX PD 元数据含训练随机化，故本次使用 YAML 标称 PD，保持与给定 sim2sim 一致；默认角和动作缩放同时与模型元数据核对。精确缩放为 `0.25 × effort_limit_sim / stiffness`，足踝 roll 的 effort 为 50，不能沿用旧坐下策略的 137。标称 PD、effort、缩放及 SDK 关节顺序写入各自 `motion.json`，运行不依赖 Python 或外部 NPZ。模型、YAML 和清单由现有递归打包规则安装到 `/opt/robot/share/motion_master/policy/mimic/{dunxia,zhanli}_policy`。

两个独立模型目录分别支持 `DUNXIA_POLICY_DIR`、`ZHANLI_POLICY_DIR` 环境变量覆盖；然后选择编译时项目模型目录，再选择安装目录。显式选择的目录缺文件或参数不匹配时失败，不静默换模型。

ZhanLi → Restand 时双肘目标为 +1.2 rad，肩部使用 Stand 默认角，手臂五列指令从上一状态最后下发值以 1 秒 smoothstep 过渡。RLControlMode/ZhanLi → Restand 为 +1.2 rad；Run/Sit2Stand → Restand 为 −0.4 rad。

验证命令：`python -B tests/test_squat.py`、`python -B tests/test_sit_stand.py`、`python -B tests/test_restand.py`。需要 g++、NumPy、PyYAML、onnxruntime。覆盖全部内嵌参考通道、逐帧观测及原始动作、标称 YAML 增益、末帧额外推理、重入、完整周期门槛、非法来源、安全退出、手柄新编码与旧编码兼容，以及 ZhanLi 进入 Stand 的手臂目标。测试不连接机器人；物理蹲起动作仍需目标设备验证。

## Stand 内双肘控制

先进入 motion 1 的 RL Stand，然后在 `/MOTION_STATE` 发送 motion 7（`0x11011`）使双肘到 −0.4 rad，或 motion 8（`0x11012`）使双肘到 +1.2 rad。这两个地址属于 RobotMotionCommand，不注册独立 State；仅当前状态为 Restand 时接收，反馈仍为 `0x1100B`。其他状态忽略这些命令，也不留待下次进入 Stand 执行。待处理急停、回零及安全故障不会被肘关节指令覆盖。

每次目标改变从最近下发的 elbow_pitch 位置开始，1 秒 smoothstep 过渡并持续保持。连续重复同一指令不会重启计时；过渡中反向切换会捕获当时的双肘指令，避免位置阶跃。只覆盖双肘目标位置，不重置站立策略、历史、肩部过渡或腿部输出。重新进入 Stand 时清除此过渡，仍执行原有来源姿态规则。
