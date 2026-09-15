# Motion Master：编译、部署与启动

本文记录当前键盘控制版本在 ARM 机器人上的标准启动流程。

## 1. 编译

在项目根目录执行：

```bash
mkdir -p build
cd build
cmake .. -DBUILD_PLATFORM=arm
make -j4
```


## 2. 打包

```bash
make package
ls -lh *.deb
```

> Deb 文件名包含版本号和构建时间。部署时请以 `ls` 实际输出的文件名为准。

## 3. 上传并安装

将生成的 Deb 包上传至机器人：

```bash
scp -O motion_master.v1.3.0+unknown.arm64.2609150936.deb user@10.21.33.103:~/
    '
```

登录机器人后安装：

```bash
sudo dpkg -i ~/motion_master.v1.3.0+unknown.arm64.2609150936.deb
```

## 4. 启动机器人控制


```bash
sudo systemctl stop motion_master.service
sudo systemctl start imusense.service
/opt/robot/share/motion_master/scripts/run.sh
```




## 5. 键盘状态切换

| 按键 | 功能 |
| --- | --- |
| `Z` | 进入零位；在 RL 或 ReStand 状态时返回零位 |
| `X` | 悬架起立 |
| `M` | 地面起立策略（仅 CR1STANDARD） |
| `P` | 从零位进入站立射击策略（仅 CR1STANDARD） |
| `1` / `2` | 站立射击状态：抬起右手 / 放下右手 |
| `O` | 从零位或 RL 进入 Mofang 动作模仿（仅 CR1STANDARD） |
| `V` | 从零位、悬架起立、RL 或 Mofang 进入 ReStand（仅 CR1STANDARD） |
| `C` | 从零位、悬架起立、地面起立、Mofang 或 ReStand 进入普通 RL 控制 |
| `R` | 进入关节阻尼 |
| `W` / `S` | 前进 / 后退；每次按键增加 / 减少 `0.1` 归一化速度 |
| `A` / `D` | 左移 / 右移；每次按键增加 / 减少 `0.1` 归一化速度 |
| `Q` / `E` | 逆时针 / 顺时针转向；每次按键增加 / 减少 `0.1` 归一化速度 |

地面起立流程：`M` → 等待机器人完成起立 → `C` 进入 RL 控制。出现异常时，按 `R` 进入关节阻尼。

站立射击流程：`Z` → `P` → `1` 抬起右手或 `2` 放下右手；按 `C` 切入常规 RL，按 `R` 进入关节阻尼。

Mofang 流程：在零位或 RL 中按 `O`，机器人直接从 `shooting_2.npz` 第 0 帧开始，以 50 Hz 播放 467 帧动作；播放结束后保持末帧。按 `C` 返回普通 RL，按 `V` 切入 ReStand，按 `R` 进入关节阻尼。

ReStand 流程：在零位、悬架起立、普通 RL 或 Mofang 中按 `V`，切入使用 `restand.onnx` 的独立 ReStand 状态。ReStand 中可以继续使用 `W/S/A/D/Q/E` 控制速度；按 `C` 切回普通 RL，按 `Z` 返回零位，按 `R` 进入关节阻尼。

普通 RL 和 ReStand 的速度上限均为前进 `vx=+1.0 m/s`、后退 `vx=-0.5 m/s`、横移 `vy=±0.5 m/s`、转向 `wz=±1.0 rad/s`。后退的 `0.5` 缩放仅在步态策略层处理；各方向速度会保持，直到反向按键抵消、退出当前 RL 状态或进入阻尼。
