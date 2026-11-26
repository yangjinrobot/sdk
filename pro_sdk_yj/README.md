# motion_master：CR1PRO 状态机

本项目注册 19 个可运行的 CR1PRO 状态。MoE、WAQ、跪下、跪姿起立和高 KP 拳击已退出状态机。
主程序默认 **CR1PRO + DDS手柄控制 + WholeBody**，通过现有 CR1PRO 硬件接口输出31轴命令。

DDS 地址与手柄按钮映射见 [展示流程](../展示流程.md#cr1pro-sdk)。`motion_1` 至 `motion_10` 使用 `0x1100b` 至 `0x11014`。`0x11015` 和 `0x11016` 保留给 `motion_11` 和 `motion_12`，当前版本拒绝这两条命令。

- 策略位于 `run_policy/cr1pro/`，模型路径及 SHA-256 见 [模型清单](run_policy/cr1pro/policy/manifest.json)。旧 `run_policy/cr1` 和旧 `model_inference` 实现已移除。
- 默认DDS接收 `/MOTION_STATE`、`/STEER`、`/REAL_STEER`、`/GAIT`、`/NECK_STEER`，也保留 `/GAMEPAD_KEY` 和GUI／手臂UDP入口。摇杆数据超过2秒未更新时清零速度。
- `MOTION_MASTER_CTRL_MODE=keyboard` 可切回键盘；还支持 `dds`、`retroid`、`skydroid`，运行参数快照记录实际选择。
- 全身模式手柄常用 `L1 → L2 → R1`，依次请求 AMP 准备、普通 AMP 行走和 RL 站立。键盘 `R` 请求阻尼。
- 坐下／起立流程：从 AMP 行走按 `B → N → M → M`，依次切换跑步、坐下、起立并返回 AMP 行走；[接入说明](docs/amp_mimic_integration.md)包含模型覆盖变量和状态编号。
- 在 AMP Walking 状态下，键盘 `K` 在行走与低 KP 拳击（`0x11014`）之间切换。安全故障或急停时进入关节阻尼。
- SDK 保留躺下和躺姿起立的状态定义及模型代码。当前版本未注册这两个控制器。
- 站立敬礼：手柄DDS发送 `0x1100f`（`rl_stand_salute_state`），右臂从当前角度用2.4秒五次插值抬到敬礼姿态并保持；收到 `0x11`／L2 后先在敬礼状态中用2.4秒放下右臂，反馈到位才切换AMP Walking。键盘可用 `U` 进入、`M/C` 请求返回；[接入说明](docs/rl_stand_salute_20261028.md)。
- UDP 默认 `10.21.33.103:8888 → 10.21.33.7:9999`，在 `conf/config.toml` 的 `[pro_sdk]` 中配置或关闭。
- 31轴SDK命令由29轴策略输出和2轴脖子命令组成；RL 站立策略输出 15 维动作，由手臂目标补成 29 轴。

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
