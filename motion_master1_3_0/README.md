# motion_master

CR1 人形机器人运动控制主程序（CR1PRO / CR1STANDARD），基于 DRDDS 通讯中间件，通过 RL 策略推理输出关节指令，并由上位机经 DDS 指令接口（kDDS）进行控制。



## 编译选项

| 选项 | 取值 | 说明 |
|------|------|------|
| `BUILD_PLATFORM` | `arm` / `x86` | 目标平台，不指定则自动检测当前架构 |
| `BUILD_SIM` | `ON` / `OFF` | 是否编译仿真模式（Pybullet） |
| `SEND_REMOTE` | `ON` / `OFF` | 编译后自动 sftp 推送至远程主机 |
| `ROBOT` | `cr1` | 机型选择，默认 `cr1`，链接对应策略库 |

### 打包

```bash
make -j4 package              # 普通 deb 包
```

---

## 各模块介绍

### state_machine — 状态机

```mermaid
graph TD
    Idle -->|起立准备（X）| SuspendedStand
    Idle -->|悬架起立（Z）| ZeroPos
    Idle -->|ZeroCalibration| ZeroCalibration

    ZeroPos -->|开始运动（C）| RL
    ZeroPos -->|起立准备（X）| SuspendedStand
    ZeroPos -->|蹲姿射击（1）| ShootDown
    ZeroPos -->|鲁棒行走（V）| ReStand

    SuspendedStand -->|开始运动（C）| RL
    SuspendedStand -->|鲁棒行走（V）| ReStand
    SuspendedStand -->|悬架起立（Z）| ZeroPos

    RL -->|AMP 持枪（仅 CR1STANDARD）| AmpGunWalking
    RL -->|鲁棒行走（V）| ReStand
    RL -->|站立射击（L）| ZhanliShooting
    RL -->|蹲姿射击（1）| ShootDown
    RL -->|悬架起立（Z）| ZeroPos
    RL -->|WaitingForStand| Idle

    AmpGunWalking -->|开始运动| RL
    AmpGunWalking -->|悬架起立（Z）| ZeroPos
    ReStand -->|开始运动（C）| RL
    ReStand -->|站立射击（L）| ZhanliShooting
    ReStand -->|悬架起立（Z）| ZeroPos
    ReStand -->|WaitingForStand| Idle

    ZhanliShooting -->|开始运动（C）| RL
    ZhanliShooting -->|鲁棒行走（V）| ReStand
    ShootDown -->|恢复站姿（2）| ShootUp
    ShootUp -->|开始运动（C）| RL
    ShootUp -->|鲁棒行走（V）| ReStand
    ZeroCalibration -->|完成| Idle
    ZeroCalibration -->|悬架起立（Z）| ZeroPos
    ZeroCalibration -->|起立准备（X）| SuspendedStand

    ZeroPos -->|保护/阻尼请求| JointDamping
    SuspendedStand -->|保护/阻尼请求| JointDamping
    RL -->|保护/阻尼请求| JointDamping
    ReStand -->|保护/阻尼请求| JointDamping
    ZhanliShooting -->|保护/阻尼请求| JointDamping
    ShootDown -->|保护/阻尼请求| JointDamping
    ShootUp -->|保护/阻尼请求| JointDamping
    AmpGunWalking -->|保护/推理失败/阻尼请求| JointDamping
    ZeroCalibration -->|阻尼请求| JointDamping
    JointDamping -->|保持 10 秒后| Idle
```

主要状态说明：

| 状态 | 功能 |
|------|------|
| **Idle** | 空闲，关节不发力（开机默认） |
| **ZeroPos** | 悬架起立（App 术语；内部状态名为 `ZeroPos`） |
| **ZeroCalibration** | 打限位校准零位 |
| **SuspendedStand** | 起立准备（悬架预站立 → 起立） |
| **RL** | 开始运动：通用 RL 策略控制（`RLControlState`） |
| **AmpGunWalking** | AMP 持枪，仅 CR1STANDARD；仅可由通用 RL 进入，并返回通用 RL |
| **ReStand** | 鲁棒行走策略，仅 CR1STANDARD |
| **ZhanliShooting** | 站立射击策略，仅 CR1STANDARD |
| **ShootDown / ShootUp** | 蹲姿射击 / 恢复站姿，仅 CR1STANDARD；下蹲后可切入恢复站姿 |
| **JointDamping** | 软急停，关节阻尼 |

除图中显式列出的切换外，状态会保持不变。`JointDamping` 保持 10 秒后自动回到 `Idle`。CR1PRO 不创建 AMP、ReStand、站立射击与射击上下状态；若请求这些状态，会回退到 `JointDamping`。

#### 手柄 App 控制术语

| App 按钮名称 | 内部目标状态 | 说明 |
|------|------|------|
| 悬架起立 | `ZeroPos` | 回到零位/悬架起立姿态 |
| 开始运动 | `RLControlMode` | 进入普通 RL 控制 |
| 鲁棒行走 | `ReStand` | 进入重新站立策略，仅 CR1STANDARD |
| 站立射击 | `ZhanliShooting` | 进入站立射击策略，仅 CR1STANDARD |
| 蹲姿射击 | `ShootDown` | 进入下蹲射击动作，仅 CR1STANDARD |
| 恢复站姿 | `ShootUp` | 进入起身恢复动作，仅 CR1STANDARD |
| AMP 持枪 | `AmpGunWalking` | 进入 AMP 持枪行走，仅 CR1STANDARD |

手柄/键盘控制的有效入口如下；不在列出的源状态时，当前状态保持不变：

| 按键 | App 术语 | 有效流转 |
|------|------|------|
| `Z` | 悬架起立 | 待机、零位校准、悬架起立、普通 RL 或 ReStand → `ZeroPos` |
| `X` | 起立准备 | 待机或零位校准 → `SuspendedStand`；已在悬架起立状态时保持该状态 |
| `L` | 站立射击 | 零位、普通 RL 或 ReStand → `ZhanliShooting`（仅 CR1STANDARD） |
| `1` | 蹲姿射击 | 零位或普通 RL → `ShootDown` |
| `2` | 恢复站姿 | `ShootDown` → `ShootUp` |
| `V` | 鲁棒行走 | 零位、悬架起立、普通 RL、站立射击或恢复站姿 → `ReStand`（仅 CR1STANDARD） |
| `C` | 开始运动 | 零位、悬架起立、ReStand 或站立射击 → 普通 RL 控制 |

#### 策略状态切换与指令过渡

`RLControlState` 承载使用 `CommonPolicyRunner` 的通用 RL 策略。进入策略后的前 100 个策略周期，`CommonPolicyRunner` 仅对左右手臂的目标位置做过渡：以当前关节观测为起点，每周期最多向策略默认位姿移动 `0.08 rad`。`kp`、`kd`、目标速度和前馈力矩，以及腿、腰和颈部的全部命令，均直接使用策略输出。

`AmpGunWalkingState` 为独立状态机，采用相同的 0.5 秒完整命令混合机制。普通 RL 可请求切入 AMP 持枪行走；AMP 请求 `RLControlMode` 后返回普通 RL。AMP 仅支持 CR1STANDARD（21 DoF）。

`CommonPolicyRunner` 负责 ONNX 推理、观测构造、策略动作到关节目标的换算，以及通用 RL 的手臂位置过渡。

### interface — 接口层

```
Robot Interface              User Command Interface
├── hardware                 ├── DdsInterface (默认)
│   ├── CR1Std              ├── SkydroidGamepadInterface
│   ├── CR1Pro              └── RetroidGamepadInterface
│   └── ecan 基类
└── simulation
    └── PybulletInterface
```

- **Robot**：实物通过 Ecan 总线 + DRDDS 与硬件通讯；仿真通过 Pybullet
- **User Command**：支持 DDS 话题、键盘、多协议手柄，默认使用 DDS

### run_policy — 策略库

按机型拆分，各自独立维护：

```
run_policy/
└── cr1/                     # CR1 系列（include + lib + change_log）
```

通过 `-DROBOT=cr1` 选择，编译时链接对应 `libRunPolicy.a`。

### model_inference — 模型推理

模型推理封装，基于 ONNX Runtime 加载策略网络，实时输出关节 action。

### conf — 配置文件

| 文件 | 说明 |
|------|------|
| `config.toml` | 主配置（数据流/关节保护/地形策略/HeightMap/SafeController 等） |
| `log.properties` | 日志级别配置 |
| `runtime_params.toml` | 运行期参数实时快照（程序自动写入，禁止手动编辑） |

### utils — 工具集

| 模块 | 说明 |
|------|------|
| `SafeController` | 安全控制器：故障检测/分组/分级，软急停，姿态超限检测 |
| `JointProtection` | 关节指令三级保护（L1 硬限位 / L2 软限位 / L3 边界日志） |
| `RuntimeParams` | 运行期参数实时快照，1Hz 后台回写 TOML |
| `DataStreaming` | 数据流：在线监测 + 轮转 CSV 记录 |
| `NeckGamepadOverride` | 脖子关节手柄接管 |
| `MotionStateFeedback` | 运动状态反馈与遥测 |

---

## 部署

### 1. 交叉编译部署（x86 → arm）

```bash
# 安装交叉编译工具
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# 编译
mkdir build && cd build
cmake .. -DBUILD_PLATFORM=arm -DBUILD_SIM=off -DSEND_REMOTE=ON -DROBOT=cr1
make -j4

# 推送至远程主机（需先配置 scripts/sftp_to_remote.sh 中的 IP/用户名）
make push_to_robot
```

### 2. 运动主机本地编译

```bash
mkdir build && cd build
cmake .. -DBUILD_PLATFORM=arm -DBUILD_SIM=off -DSEND_REMOTE=OFF -DROBOT=cr1
make -j4
./motion_master
```

### 3. 服务模式

```bash
sudo journalctl -u motion_master.service -f    # 查看服务日志
```
