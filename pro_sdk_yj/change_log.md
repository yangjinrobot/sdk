# Change Log [+]=新增功能 [!]=Bug 修复 [*]=变更/更新/优化/重构
## v.1.3.0
[*]初始化版本
[+]新增Mimic1舞蹈状态`0x11018`与Mimic2舞蹈状态`0x11019`，仅与`0x11`AMP行走双向切换，安全故障和急停仍直接进入阻尼
[+]dance目录的mimic1/mimic2 runner迁入`run_policy/cr1pro/include`，模型放入`policy/mimic_policy`，支持`DR2_MIMIC1_MODEL_PATH`／`DR2_MIMIC2_MODEL_PATH`覆盖
[+]新增Tangxia躺下`0x1101a`与Zhanli站起`0x1101b`，状态链固定为`0x11 -> 0x1101a -> 0x1101b -> 0x11`
[+]Tangxia/Zhanli使用独立151维观测、metadata PD和模型参数，仅在这两个状态屏蔽姿态超限软急停，其他安全故障仍进入阻尼
[+]AmpWalking进入Tangxia时保存AMP Walking最后31×5指令，对全身执行1秒smoothstep入口混合；Tangxia帧号在过渡期间继续推进
