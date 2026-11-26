# 跪姿与跪姿起立迁移

| 手柄 motion | RobotMotionState | DDS /MOTION_STATE 地址 |
| --- | --- | --- |
| 原有 RL / 开始运动 | AmpControl_NiRenWalk_Mode | `0x11` |
| 1 | AmpControl_GunShootWalk_Mode | `0x1100B` |
| 2 | RLControl_Stand_Mode | `0x1100C` |
| 3 | AmpControl_Walk2Run_Mode | `0x1100D` |
| 4 | MimicControl_ChairDown_Mode | `0x1100E` |
| 5 | MimicControl_ChairUp_Mode | `0x1100F` |
| 6 | MimicControl_Kneel_Mode | `0x11010` |
| 7 | MimicControl_Kneel_StandUp_Mode | `0x11011` |

连续地址中，`0x1100D` 为 AmpControl_Walk2Run_Mode；Stand 内双肘命令为 motion 8 / `0x11012` 和 motion 9 / `0x11013`。

MimicControl_Kneel_Mode 与 MimicControl_Kneel_StandUp_Mode 各有完整的 State 类和 PolicyRunner 类，直接继承 BeyondMimicPolicyRunner；不调用、嵌套或派生自 MimicControl_ChairUp_Mode 状态/策略。

MimicControl_Kneel_Mode 从 AmpControl_NiRenWalk_Mode、AmpControl_GunShootWalk_Mode、AmpControl_Walk2Run_Mode、RLControl_Stand_Mode 或完成的 MimicControl_Kneel_StandUp_Mode 进入，加载 `1125_mimic` 的 `mimic/dunxia_policy/shoot_down.onnx` 和 `shoot_down_dual_gun_groundfix_kneel.npz`。MimicControl_Kneel_StandUp_Mode 加载 `mimic/zhanli_policy/shoot_up.onnx` 和 `shoot_up_dual_gun_groundfix_kneel.npz`。

进入时只读取实测反馈，不改实测姿态；清零速度指令，清空上一动作与轨迹索引，按进入时实际偏航对齐参考航向。控制周期为 1 ms，推理 decimation=20，持续检查反馈、动作、目标角的有限性及推理耗时；无效数据或单次推理超过 20 ms 进入阻尼。离线测试用锁步模式关闭机器性能相关的超时检查。

进入 MimicControl_Kneel_Mode、MimicControl_Kneel_StandUp_Mode、MimicControl_ChairDown_Mode 和 MimicControl_ChairUp_Mode 时，都只对 SDK 索引 1–8 的 8 个手臂关节做 1 秒 smoothstep 入场混合。混合覆盖手臂的 kp、目标位置、kd、目标速度和前馈力矩五列，起点为上一状态最后实际下发的命令。腰、腿不参与该入场混合，直接使用对应 mimic 策略输出。

观测依次为参考关节位置 21、参考速度 21、相对 anchor 朝向 6（旋转矩阵前两列按行展开）、实际角速度 3、实际关节位置减训练默认角 21、实际关节速度 21、上一原始动作 21，总计 114 维。不做 AMP 动作裁剪、缩放或历史堆叠。输出目标角为训练默认角 + YAML 精确 action_scale × ONNX 原始动作，参考取自模型内嵌常量。

两个模型的 ONNX PD 元数据含训练随机化，故本次使用 `1125_mimic/env.yaml` 的标称 PD，保持与给定 sim2sim 一致；默认角和动作缩放同时与模型元数据核对。精确缩放为 `0.25 × effort_limit_sim / stiffness`，足踝 roll 的 effort 为 50，不能沿用旧坐下策略的 137。标称 PD、effort、缩放及 SDK 关节顺序写入各自 `motion.json`。ONNX 与对应 NPZ 成对安装，离线测试逐帧核对 NPZ 和 ONNX 内嵌参考；机器人运行仍直接读取 ONNX 内嵌参考，不依赖 Python 或 NumPy。模型、NPZ、YAML 和清单由现有递归打包规则安装到 `/opt/robot/share/motion_master/policy/mimic/{dunxia,zhanli}_policy`，旧版模型文件不覆盖。

两个独立模型目录分别支持 `MIMIC_CONTROL_KNEEL_MODE_POLICY_DIR`、`MIMIC_CONTROL_KNEEL_STAND_UP_MODE_POLICY_DIR` 环境变量覆盖；然后选择编译时项目模型目录，再选择安装目录。显式选择的目录缺文件或参数不匹配时失败，不静默换模型。

MimicControl_Kneel_StandUp_Mode → RLControl_Stand_Mode 时双肘目标为 +1.2 rad，肩部使用 Stand 默认角，手臂五列指令从上一状态最后下发值以 1 秒 smoothstep 过渡。AmpControl_GunShootWalk_Mode/MimicControl_Kneel_StandUp_Mode → RLControl_Stand_Mode 为 +1.2 rad；AmpControl_Walk2Run_Mode/MimicControl_ChairUp_Mode → RLControl_Stand_Mode 为 −0.4 rad。

验证命令：`python -B tests/test_squat.py`、`python -B tests/test_sit_stand.py`、`python -B tests/test_restand.py`。需要 g++、NumPy、PyYAML、onnxruntime。覆盖全部内嵌参考通道、逐帧观测及原始动作、标称 YAML 增益、末帧额外推理、重入、完整周期门槛、非法来源、安全退出、手柄新编码与旧编码兼容，以及 MimicControl_Kneel_StandUp_Mode 进入 Stand 的手臂目标。测试不连接机器人；物理蹲起动作仍需目标设备验证。

## Stand 内双肘控制

先进入 motion 2 的 RL Stand，然后在 `/MOTION_STATE` 发送 motion 8（`0x11012`）使双肘到 −0.4 rad，或 motion 9（`0x11013`）使双肘到 +1.2 rad。反馈仍为 `0x1100C`。

每次目标改变从最近下发的 elbow_pitch 位置开始，1 秒 smoothstep 过渡并持续保持。连续重复同一指令不会重启计时；过渡中反向切换会捕获当时的双肘指令，避免位置阶跃。只覆盖双肘目标位置，不重置站立策略、历史、肩部过渡或腿部输出。重新进入 Stand 时清除此过渡，仍执行原有来源姿态规则。
