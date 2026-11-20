# motion_master：CR1PRO 状态机

本项目包含26个CR1PRO状态、17套策略runner，包括AMP行走、AMP全身、RL站立、AMP MoE、AMP WAQ、AMP、Mimic12坐下、蹲姿准备（`0x20010`）、Mimic13起立、Tangxia躺下、Zhanli站起、Mimic1／Mimic2舞蹈、Stand2Knee／Knee2Stand及RL站立敬礼。原搬箱／蹲起状态的协议编号 `0x9..0xc` 继续作废。
主程序默认 **CR1PRO + DDS手柄控制 + WholeBody**，通过现有 CR1PRO 硬件接口输出31轴命令。

**手柄状态协议：WaitingForStand/Idle=`0x0`、JointDamping=`0x2`、AmpZeroPos=`0x20006`、AmpWalkingControl=`0x11`、RLStandControlMode=`0x1100b`、AmpMoe15Dof=`0x1100c`、AmpWalkingWAQ=`0x1100d`、RLControlMode=`0x1100e`、Mimic12=`0x11016`、Mimic13=`0x11017`、Stand2Knee=`0x11011`、Knee2Stand=`0x11012`、RLStandSalute=`0x11013`、Mimic1舞蹈=`0x11018`、Mimic2舞蹈=`0x11019`、Tangxia躺下=`0x1101a`、Zhanli站起=`0x1101b`。独立ZeroPos使用`0x20002`，避免与阻尼冲突。**

- 策略位于 `run_policy/cr1pro/`，模型路径及 SHA-256 见 [模型清单](run_policy/cr1pro/policy/manifest.json)。旧 `run_policy/cr1` 和旧 `model_inference` 实现已移除。
- 默认DDS接收 `/MOTION_STATE`、`/STEER`、`/REAL_STEER`、`/GAIT`、`/NECK_STEER`，也保留 `/GAMEPAD_KEY` 和GUI／手臂UDP入口。摇杆数据超过2秒未更新时清零速度。
- `MOTION_MASTER_CTRL_MODE=keyboard` 可切回键盘；还支持 `dds`、`retroid`、`skydroid`，运行参数快照记录实际选择。
- 全身模式手柄：`L1` 进入 AMP ZeroPos，`L2` 从准备态／RL Stand／AMP MoE／新增AMP进入 AMP Walking，`R1` 从 AMP Walking／AMP MoE 进入 RL Stand，`R2` 仅从 RL Stand 进入 AMP MoE 15 DOF。键盘 `R` 请求阻尼。
- 坐下／起立流程：从AMP行走按 `N → M → M`，依次进入Mimic12坐下、Mimic13起立并返回AMP行走；[接入说明](docs/amp_mimic_integration.md)包含模型覆盖变量和状态编号。
- 新增跪下／起立流程：`Z` 准备3秒后，按 `C → T → Y → C` 进入AMP Walking、Stand2Knee（`0x11011`）、Knee2Stand（`0x11012`）并返回AMP Walking。跪下、起立分别预留至少8秒、7秒；[接入说明](docs/knee_mimic_20261028.md)。
- 新增两支舞蹈：仅在AMP Walking时可用 `G` 进入Mimic1（`0x11018`，disco8末帧保持520步）、`K` 进入Mimic2（`0x11019`，simpson2末帧保持390步）；再按同一按键回到AMP Walking。两支舞蹈之间不能直接互切，安全故障／急停仍直接进入阻尼。
- 新增恢复链：DDS `0x1101a` 只能从AMP Walking `0x11` 进入躺下，DDS `0x1101b` 只能从躺下进入站起，站起后返回AMP Walking `0x11`。进入Tangxia时使用完整31轴、1秒smoothstep入口混合；这两个状态仅屏蔽姿态超限软急停，其他安全保护保持有效。
- 站立敬礼：手柄DDS发送 `0x11013`（`rl_stand_salute_state`），右臂从当前角度用2.4秒五次插值抬到敬礼姿态并保持；收到 `0x11`／L2 后先在敬礼状态中用2.4秒放下右臂，反馈到位才切换AMP Walking。键盘可用 `U` 进入、`M/C` 请求返回；[接入说明](docs/rl_stand_salute_20261028.md)。
- UDP 默认 `10.21.33.103:8888 → 10.21.33.7:9999`，在 `conf/config.toml` 的 `[pro_sdk]` 中配置或关闭。
- 31轴SDK命令由29轴策略输出和2轴脖子命令组成；4套RL网络和AMP MoE网络本身输出15维动作，再补成29轴。

## 构建与离线回归

```bash
cmake -S . -B build/pro-tests -DBUILD_PRO_TESTS_ONLY=ON
cmake --build build/pro-tests -j2
ctest --test-dir build/pro-tests --output-on-failure
```

ARM硬件程序：

```bash
cmake -S . -B build/arm -DBUILD_PLATFORM=arm -DSEND_REMOTE=OFF
cmake --build build/arm -j2
```

x86开发机交叉编译需要 `aarch64-linux-gnu-gcc/g++`。`SEND_REMOTE=OFF` 不自动推送。

模型部署目录为 `/opt/robot/share/motion_master/policy/cr1pro/`；可通过 `MOTION_MASTER_POLICY_DIR` 显式覆盖。
完整状态图、所有按键、模型维度、部署与配置见 [使用说明](docs/USAGE.md)。

本轮离线源算法对照中，7策略共64,960个浮点指令逐字节一致。源策略基于B2训练物理资源；
代码和离线推理验证尚不能证明其在CR1PRO上的物理等价性或动作稳定性，本轮未启动实际机器人。

## AMP MoE

AMP MoE 键盘流程为 `Z → C → V → P`；`V` 返回 RL Stand，`M` 返回 AMP Walking。
AMP MoE 默认策略为 `moe_policy_1026.onnx`。模型哈希及接口见 [1026验证记录](docs/amp_moe_1026_verification.md)。
MoE观测为29维交错关节顺序，动作独立使用“腰3轴、左腿6轴、右腿6轴”，上次动作按此映射回观测。

‘






mkdir -p build
cd build
cmake .. -DBUILD_PLATFORM=arm
make -j4



scp -O motion_master.v1.3.0+unknown.arm64.2611181536.deb user@10.21.33.103:~/yj_deb

ssh user@10.21.33.103
sudo dpkg -i motion_master.v1.3.0+unknown.arm64.2611181536.deb


sudo systemctl stop motion_master.service
sudo systemctl start imusense.service
/opt/robot/share/motion_master/scripts/run.sh
