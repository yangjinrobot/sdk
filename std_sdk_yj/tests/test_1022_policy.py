"""Run with the sim2sim Python environment: independent YAML/ONNX parity."""
from collections import deque
import hashlib
from pathlib import Path
import subprocess
import tempfile
import numpy as np
import onnxruntime as ort
import yaml
from test_nirenwalk_amp_dt import ROOT, JOINTS, TrainingLoader, joint_value

assets = ROOT.parent / 'policy/1022'
includes = ['run_policy/cr1/include', 'types', 'utils', 'state_machine',
            'third_party/eigen', 'third_party/onnxruntime/x86/include']
library = ROOT / 'third_party/onnxruntime/x86/lib'
with tempfile.TemporaryDirectory(prefix='1022-policy-') as tmp:
    binary = Path(tmp) / 'check'
    subprocess.run(['g++', '-O2', '-std=c++17', '-DEIGEN_DONT_ALIGN_STATICALLY',
                    str(ROOT / 'tests/test_1022_policy.cpp'),
                    str(ROOT / 'run_policy/cr1/src/amp_control_walk2run_mode_policy_runner.cpp'),
                    str(ROOT / 'run_policy/cr1/src/amp_control_gun_shoot_walk_mode_policy_runner.cpp'),
                    *['-I' + str(ROOT / p) for p in includes], '-L' + str(library),
                    '-Wl,-rpath,' + str(library), '-lonnxruntime', '-pthread', '-o', str(binary)], check=True)
    for mode, model, config in [('gun', 'policy_18000.onnx', 'env.yaml'),
                                 ('humanoid', 'policy_17000.onnx', 'env_nirenxingzou_1020.yaml')]:
        trace = Path(tmp) / (mode + '.txt')
        subprocess.run([str(binary), mode, str(assets / model), str(trace)], check=True)
        cfg = yaml.load((assets / config).read_text(), Loader=TrainingLoader)
        rows, samples = {}, []
        for line in trace.read_text().splitlines():
            key, data = line.split(' ', 1)
            values = np.fromstring(data, sep=' ', dtype=np.float32)
            if key == 'sample': samples.append(values)
            else: rows[key] = values
        robot = cfg['scene']['robot']
        defaults = np.array([joint_value(robot['init_state']['joint_pos'], j) for j in JOINTS], np.float32)
        np.testing.assert_allclose(rows['defaults'], defaults, atol=1e-7)
        for key, param in [('kp', 'stiffness'), ('kd', 'damping')]:
            import re
            expected = []
            for joint in JOINTS:
                groups = [g for g in robot['actuators'].values()
                          if any(re.fullmatch(p, joint) for p in g['joint_names_expr'])]
                assert len(groups) == 1
                expected.append(joint_value(groups[0][param], joint))
            np.testing.assert_allclose(rows[key], expected, atol=1e-7)
        norm = cfg['normalization']
        assert cfg['robot']['actor_obs_history_length'] == 5
        assert cfg['robot']['action_scale'] == .25 and norm['clip_actions'] == 10
        assert norm['clip_observations'] == 100
        assert cfg['sim']['dt'] * cfg['sim']['decimation'] == .02
        for key in ['ang_vel', 'projected_gravity', 'commands', 'joint_pos', 'actions']:
            assert norm['obs_scales'][key] == 1
        assert norm['obs_scales']['joint_vel'] == .05
        limits = cfg['commands']['ranges']
        np.testing.assert_allclose(rows['max_cmd'], [limits[k][1] for k in ['lin_vel_x', 'lin_vel_y', 'ang_vel_z']])
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        session = ort.InferenceSession(str(assets / model), sess_options=options)
        assert session.get_inputs()[0].shape[-1] == 432
        assert session.get_outputs()[0].shape[-1] == 21
        assert len(samples) == 280
        max_error = 0.
        for episode in range(2):
            history = deque(maxlen=5)
            previous = np.zeros(21, np.float32)
            cmd = np.zeros(3, np.float32)
            target = np.array([1 if episode == 0 else limits['lin_vel_x'][0] / limits['lin_vel_x'][1], 1, -1], np.float32)
            for step in range(140):
                cmd += np.clip(target - cmd,
                               [-.02, -.02, -.04], [.02, .02, .04]).astype(np.float32)
                current = np.r_[np.array([.1, -.2, .3], np.float32),
                                [0, -np.sin(.1), -np.cos(.1)], cmd * rows['max_cmd'],
                                (defaults + np.float32(.001) * (np.arange(21, dtype=np.float32) + step % 5)) - defaults,
                                np.float32(.05) * np.float32(.02) * (np.arange(21, dtype=np.float32) - step % 3),
                                previous].astype(np.float32)
                if not history: history.extend(current.copy() for _ in range(5))
                else: history.append(current.copy())
                expected_input = np.concatenate([*history, current])
                actual = samples[episode * 140 + step]
                np.testing.assert_allclose(actual[:432], expected_input, atol=2e-5, rtol=1e-5)
                previous = np.clip(session.run(None, {'obs': expected_input[None]})[0][0], -10, 10)
                max_error = max(max_error, float(np.max(np.abs(previous - actual[432:]))))
                np.testing.assert_allclose(actual[432:], previous, atol=1e-4, rtol=1e-5)
        print(f'PASS {mode}: 280 frames; YAML/observation/action parity; max action error {max_error:.3g}; '
              f'SHA256 {hashlib.sha256((assets / model).read_bytes()).hexdigest()}')
