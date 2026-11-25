# Stand2Knee / Knee2Stand 1028模型更新

默认模型更新为 `mimic_policy/stand2knee/model_stand2knee_1028.onnx`，配套训练配置为同目录下的 `agent(13).yaml` 和 `env(13).yaml`。
模型SHA-256为 `68aebd1ab813fcc4699208713eec20da1a447c700d1b695ec502dd87356b74d7`，已加入 [模型清单](../run_policy/cr1pro/policy/manifest.json)。

Knee2Stand默认模型也更新为 `mimic_policy/knee2stand/model_knee2stand_1028.onnx`，配套同目录下的 `agent(14).yaml`／`env(14).yaml`。
其SHA-256为 `e7cb28e88229fef262231673e44dde249b447577c6825c88b14edb5c9a019a09`，同样已加入模型清单。

Stand2Knee的新旧agent／env配置仅训练设备由 `cuda:1` 改为 `cuda:0`；Knee2Stand仅由 `cuda:0` 改为 `cuda:1`。
两套新ONNX的关节顺序、观测顺序、默认关节角、动作尺度、kp、kd及fps元数据均与各自1027模型一致：
151维观测、29维动作、50 Hz；Stand2Knee共384帧，末帧383，7.66秒到末帧；Knee2Stand共313帧，末帧312，6.24秒到末帧。
运行时直接读取新ONNX中的参数和归一化，YAML作为对应训练配置记录。

新模型的嵌入参考动作有变化：末帧左右肩roll由±0.2 rad变为约±0.436332 rad。
新Knee2Stand 1028首帧肩roll也为±0.436332 rad。两套1028衔接帧的参考关节位置、速度、机身位置及姿态完全一致，最大绝对差均为0。

末帧手臂观测冻结、手臂动作屏蔽、独立预设／UDP规划及Knee2Stand控制逻辑保持现有实现。
测试中的ONNX数值对照路径也更新为1028；旧模型与历史验证数据保留。

## 历史验证

MuJoCo桥和C++测试均编译成功。
两套1028均通过完整轨迹及额外100个末帧保持的独立ONNX数值对照、观测异常回退和重入检查。

当时完整C++回归为7/9通过，两项失败分别为 `knee_mimic` 和 `amp_moe_15dof` 中的RL Stand手臂规划对照。
原因是当前 `rl_stand_policy_runner.hpp` 的home／preset3肩roll已为±0.25，而公共 `RLArmMotionPlanner` 仍为±0.15。
使用替换前的1027默认路径和原测试代码，在隔离目录重新编译后也复现相同失败，因此这是更换模型前已有的配置差异；本次仅更新模型及对应记录。

双1028的55秒测试中，跪下、末帧保持及六组手臂预设阶段均未触发故障。
43秒切入Knee2Stand 1028后，43.938秒触发高度保护（base高度0.44987 m，roll/pitch约0.37753／-0.27013 rad），进入JointDamping。
因此长时间摆臂后的完整起立流程尚未通过稳定性验证。

不摆臂的标准26秒流程 `0.1:z,3.5:c,5:t,14:y,22:c` 已通过，状态完整返回AMP Walking，
`fault_latched=false`、`fall_event=null`、`failures=[]`。

先前只替换跪下模型、混用1027起立模型的55秒流程中，跪下、末帧保持及全部手臂预设阶段均未触发故障；43秒切入Knee2Stand 1027后，
47.617秒触发高度保护（base高度0.4483 m，roll约-0.8113 rad），进入JointDamping。

进入Stand2Knee约7.66秒后，可按 `1..6` 控制手臂；`Y` 起立，至少7秒后 `C` 返回行走。
