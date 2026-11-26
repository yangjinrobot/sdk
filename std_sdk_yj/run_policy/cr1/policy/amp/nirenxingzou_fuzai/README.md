# AmpControl_NiRenWalk_Mode

- DDS /MOTION_STATE: `0x11`
- Handset: 开始运动/RL
- Model: `policy_15500.onnx`
- Training config: `env_nirenxingzou_1020.yaml`
- Input: float `[batch, 468]` (`78 × 6`)
- Output: float `[batch, 21]`

The model source is:

```text
/home/yj/桌面/code/sim2sim/policy/std/nirenxingzou_fuzai/policy_15500 (1).onnx
```

The copied deployment name removes spaces for stable path handling. The model
SHA256 is `7c083756c7cf8f70f5b51f722d5ed60bf1639684b8c3b1fc79043ee040569be9`.

`AmpControlNiRenWalkModePolicyRunner` uses the strict 0.1 m/s command deadband,
five-frame 78-D history plus the current frame, 50 Hz inference, and the
default elbow pitch of +1.2 rad from the training YAML.
