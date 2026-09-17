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
scp -O motion_master.v1.3.0+cc48b0.arm64.2609171417.deb user@10.21.33.103:~/

```

登录机器人后安装：

```bash
'
ssh user@10.21.33.103
sudo dpkg -i ~/motion_master.v1.3.0+cc48b0.arm64.2609171417.deb
```

## 4. 启动机器人控制


```bash
sudo systemctl stop motion_master.service
sudo systemctl start imusense.service
/opt/robot/share/motion_master/scripts/run.sh
```


rm -f motion_master.v1.3.0+*.arm64.*.deb

## 5. 键盘状态切换

| 按键 | 功能 |
| --- | --- |
| `Z` | 从待机、零位校准、悬架起立、普通 RL 或 ReStand 进入/返回零位 |
| `X` | 从待机或零位校准进入悬架起立；在悬架起立状态保持该状态 |
| `L` | 仅从零位、普通 RL 或 ReStand 进入射击步行策略 `zhanli_shooting`（`policy_shoot_walk.onnx`，仅 CR1STANDARD） |
| `1` | 仅从零位或普通 RL 进入 `shoot_down`（`model_43000.onnx` + `shoot_down.npz`） |
| `2` | 仅从 `shoot_down` 进入 `shoot_up`（`model_28000.onnx` + `shoot_up.npz`） |
| `V` | 从零位、悬架起立、普通 RL、`zhanli_shooting` 或 `shoot_up` 进入 ReStand（仅 CR1STANDARD） |
| `C` | 从零位、悬架起立、ReStand 或 `zhanli_shooting` 进入普通 RL 控制 |
| `R` | 在运动状态中请求进入关节阻尼 |
| `W` / `S` | 前进 / 后退；每次按键增加 / 减少 `0.1` 归一化速度 |
| `A` / `D` | 左移 / 右移；每次按键增加 / 减少 `0.1` 归一化速度 |
| `Q` / `E` | 逆时针 / 顺时针转向；每次按键增加 / 减少 `0.1` 归一化速度 |

射击步行流程：在零位、普通 RL 或 ReStand 中按 `L` 进入 `zhanli_shooting`（`policy_shoot_walk.onnx`）。该状态可使用 `W/S/A/D/Q/E` 控制速度；按 `C` 切入普通 RL、按 `V` 切入 ReStand、按 `R` 进入关节阻尼。

下蹲/起立射击流程：在零位或普通 RL 中按 `1` 进入 `shoot_down`。该状态以 50 Hz 使用 `shoot_down.npz` 参考轨迹及 `model_43000.onnx` 动作输出，结束后保持末帧；`shoot_down` 只能按 `2` 切入 `shoot_up`，或按 `R` 进入阻尼。`shoot_up` 使用 `shoot_up.npz` 与 `model_28000.onnx`，结束后同样保持末帧；键盘可按 `V` 进入 ReStand 或按 `R` 进入阻尼。

ReStand 流程：在零位、悬架起立、普通 RL、`zhanli_shooting` 或 `shoot_up` 中按 `V`，切入使用 `restand.onnx` 的独立 ReStand 状态。ReStand 中可以继续使用 `W/S/A/D/Q/E` 控制速度；按 `C` 切回普通 RL，按 `Z` 返回零位，按 `L` 切入射击步行策略，按 `R` 进入阻尼。

普通 RL、ReStand 和射击步行状态均支持速度控制，速度上限为前进 `vx=+1.0 m/s`、后退 `vx=-0.5 m/s`、横移 `vy=±0.5 m/s`、转向 `wz=±1.0 rad/s`。后退的 `0.5` 缩放仅在步态策略层处理；各方向速度会保持，直到反向按键抵消、退出当前速度控制状态或进入阻尼。

## 6. GitHub：提交并推送本地更新

完成代码修改后，在项目根目录执行。多人协作时先获取远程更新，避免基于旧版本提交：

```bash
git pull --rebase origin main
```

查看本次修改的文件，确认编译缓存（如 `build/`、`install/`、`log/`）没有出现在待提交列表中：

```bash
git status
```

确认无误后，提交并推送：

```bash
git add .
git commit -m "简要说明本次修改"
git push origin main
```

例如：

```bash
git commit -m "Update keyboard control documentation"
```

若 `git pull --rebase` 出现冲突，先处理标记为冲突的文件，再执行 `git add <文件名>` 和 `git rebase --continue`；完成后再运行 `git push origin main`。如果只想取消这次变基，可运行 `git rebase --abort`。




L zhanli shooting amp
1 蹲下设计 2 站起来

C：RL 控制
V：restand 用来从mimic控制恢复到初始姿态 经历过大干扰
L：站立shooting _wlak
C(1)：跪姿射击，C(2)：跪姿起来
R:阻尼
