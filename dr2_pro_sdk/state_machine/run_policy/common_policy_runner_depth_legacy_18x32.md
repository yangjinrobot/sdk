# Legacy 18x32 depth runner restore record

`common_policy_runner_depth_origin.hpp` is the exact pre-0911 runner, retained
before switching `common_policy_runner_depth.hpp` to the 0911 full-height
`36x32` depth crop.

Its depth crop is `(up=6, down=12, left=16, right=16)` and it targets the old
`18x32` encoder input.  To restore it, first make a copy of the current runner,
then replace the active file:

```bash
cp state_machine/run_policy/common_policy_runner_depth.hpp \
   state_machine/run_policy/common_policy_runner_depth_0911_36x32.hpp
cp state_machine/run_policy/common_policy_runner_depth_origin.hpp \
   state_machine/run_policy/common_policy_runner_depth.hpp
```

When restoring, also revert `DEPTH_CROP` and the encoder-size display text in
`simulation/mujoco_simulation_whole.py`, plus the crop constants in
`depth_ros_viewer.py`, to avoid inspecting a depth image different from the
one sent to the policy.
