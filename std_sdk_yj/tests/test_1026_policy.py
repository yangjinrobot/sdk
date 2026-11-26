"""Offline 1026/1027/1101/1102/1103/1103_2/1106/1111/1112 policy checks; no robot connection."""
import argparse
from collections import deque
from pathlib import Path
import subprocess
import tempfile
import re
import yaml
from test_nirenwalk_amp_dt import JOINTS, TrainingLoader, joint_value

import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['walk', 'run'], default='run')
    parser.add_argument('--version', choices=['1026', '1027', '1101', '1102', '1103', '1103_2', '1106', '1111', '1112'],
                        default='1026')
    args = parser.parse_args()
    mode, version = args.mode, args.version
    if version in ('1027', '1101', '1102', '1103', '1103_2', '1106', '1111', '1112') and mode != 'run':
        parser.error(f'{version} supports run only')
    threshold = .1 if version in ('1027', '1101', '1102', '1103', '1103_2', '1106', '1111', '1112') else .15
    checkpoint = '15500' if mode == 'walk' else '16500'
    model_name = 'run_2.onnx' if mode == 'run' else f'model_{checkpoint}.onnx'
    MODEL = ROOT.parent / 'policy/1026' / model_name
    if version == '1027':
        MODEL = ROOT.parent / 'policy/1027_run/run_1.onnx'
    archived = ROOT.parent / 'policy/disabled_run_policies'
    if version == '1101':
        MODEL = archived / '1101/1101_resume_gait_v2_24500.onnx'
    if version == '1102':
        MODEL = archived / '1102/1102_3.onnx'
    if version == '1103':
        MODEL = archived / '1103/1103.onnx'
    if version == '1103_2':
        MODEL = archived / '1103_2/1103_2.onnx'
    if version == '1106':
        MODEL = archived / '1106/run_1106.onnx'
    if version == '1111':
        MODEL = archived / '1111/walk2run_1111.onnx'
    if version == '1112':
        MODEL = archived / '1112/run_1112.onnx'
    includes = ['run_policy/cr1/include', 'types', 'utils', 'state_machine',
                'third_party/eigen', 'third_party/onnxruntime/x86/include']
    library = ROOT / 'third_party/onnxruntime/x86/lib'
    with tempfile.TemporaryDirectory(prefix='1026-policy-') as tmp:
        binary, trace = Path(tmp) / 'check', Path(tmp) / 'trace.txt'
        subprocess.run(['g++', '-O2', '-std=c++17', '-DEIGEN_DONT_ALIGN_STATICALLY',
                        str(ROOT / 'tests/test_1022_policy.cpp'),
                        str(ROOT / 'run_policy/cr1/src/amp_control_walk2run_mode_policy_runner.cpp'),
                        str(ROOT / 'run_policy/cr1/src/amp_control_gun_shoot_walk_mode_policy_runner.cpp'),
                        *['-I' + str(ROOT / p) for p in includes], '-L' + str(library),
                        '-Wl,-rpath,' + str(library), '-lonnxruntime', '-pthread',
                        '-o', str(binary)], check=True)
        subprocess.run([str(binary), '1026_walk' if mode == 'walk' else version, str(MODEL), str(trace)], check=True)
        rows, samples = {}, []
        for line in trace.read_text().splitlines():
            key, data = line.split(' ', 1)
            values = np.fromstring(data, sep=' ', dtype=np.float32)
            if key == 'sample':
                samples.append(values)
            else:
                rows[key] = values
        config_name = 'env.yaml' if version == '1112' else \
                      'run.yaml' if version in ('1027', '1101', '1102', '1103', '1103_2', '1106', '1111') else f'{checkpoint}.yaml'
        cfg = yaml.load((MODEL.parent / config_name).read_text(), Loader=TrainingLoader)
        robot = cfg['scene']['robot']
        defaults = np.array([joint_value(robot['init_state']['joint_pos'], j) for j in JOINTS], np.float32)
        np.testing.assert_allclose(rows['defaults'], defaults, atol=1e-7)
        for key, param in [('kp', 'stiffness'), ('kd', 'damping')]:
            expected = []
            for joint in JOINTS:
                groups = [g for g in robot['actuators'].values()
                          if any(re.fullmatch(p, joint) for p in g['joint_names_expr'])]
                assert len(groups) == 1
                expected.append(joint_value(groups[0][param], joint))
            np.testing.assert_allclose(rows[key], expected, atol=1e-7)
        limits = cfg['commands']['ranges']
        max_cmd = np.array([limits[k][1] for k in ['lin_vel_x', 'lin_vel_y', 'ang_vel_z']], np.float32)
        if version in ('1026', '1027'):
            # These legacy run variants cap forward speed at 2 m/s; their YAML
            # files allow 3 m/s and are not the active deployment contract.
            max_cmd[0] = 2.
        if version == '1112':
            # The 1112 environment starts at 1 m/s and applies curriculum up
            # to 3 m/s; deployment preserves the trained upper bound.
            max_cmd[0] = cfg['commands']['curriculum_max_linear']
        np.testing.assert_allclose(rows['max_cmd'], max_cmd)
        assert cfg['robot']['actor_obs_history_length'] == 5
        assert cfg['robot']['action_scale'] == .25
        norm = cfg['normalization']
        assert norm['clip_actions'] == 10 and norm['clip_observations'] == 100
        assert norm['obs_scales']['joint_vel'] == .05
        assert all(norm['obs_scales'][key] == 1 for key in
                   ['ang_vel', 'projected_gravity', 'commands', 'joint_pos', 'actions'])
        assert cfg['sim']['dt'] * cfg['sim']['decimation'] == .02
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        session = ort.InferenceSession(str(MODEL), sess_options=options,
                                       providers=['CPUExecutionProvider'])
        assert session.get_inputs()[0].name == 'obs'
        assert session.get_inputs()[0].shape[-1] == 438
        assert session.get_outputs()[0].name == 'actions'
        assert session.get_outputs()[0].shape[-1] == 21
        assert len(samples) == 280
        max_error = 0.
        for episode in range(2):
            history = deque(maxlen=5)
            previous = np.zeros(21, np.float32)
            command = np.zeros(3, np.float32)
            target = np.array([1 if episode == 0 else limits['lin_vel_x'][0] / max_cmd[0], 1, -1], np.float32)
            for step in range(140):
                command += np.clip(target - command, [-.02, -.02, -.04],
                                   [.02, .02, .04]).astype(np.float32)
                velocity = command * max_cmd
                moving = np.linalg.norm(velocity[:2]) > np.float32(threshold) or abs(velocity[2]) > np.float32(threshold)
                defaults = rows['defaults']
                current = np.r_[np.array([.1, -.2, .3], np.float32),
                                [0, -np.sin(.1), -np.cos(.1)], velocity, float(moving),
                                (defaults + np.float32(.001) * (np.arange(21, dtype=np.float32) + step % 5)) - defaults,
                                np.float32(.05) * np.float32(.02) * (np.arange(21, dtype=np.float32) - step % 3),
                                previous].astype(np.float32)
                if not history:
                    history.extend(current.copy() for _ in range(5))
                else:
                    history.append(current.copy())
                expected = np.concatenate([*history, current])
                actual = samples[episode * 140 + step]
                np.testing.assert_allclose(actual[:438], expected, atol=2e-5, rtol=1e-5)
                previous = np.clip(session.run(None, {'obs': expected[None]})[0][0], -10, 10)
                max_error = max(max_error, float(np.max(np.abs(previous - actual[438:]))))
                np.testing.assert_allclose(actual[438:], previous, atol=1e-4, rtol=1e-5)
        if mode == 'run':
            entry = Path(tmp) / 'entry'
            entry_source = 'test_run1_state.cpp' if version in ('1101', '1103_2') else \
                           'test_run2_state.cpp' if version in ('1102', '1103') else \
                           'test_run3_state.cpp' if version == '1106' else \
                           'test_run4_state.cpp' if version in ('1111', '1112') else \
                           'test_amp_control_walk2run_mode_entry.cpp'
            subprocess.run(['g++', '-O2', '-std=c++17', '-DEIGEN_DONT_ALIGN_STATICALLY',
                            '-I' + str(ROOT / 'tests/mimic_stubs'),
                            str(ROOT / 'tests' / entry_source),
                            str(ROOT / 'run_policy/cr1/src/amp_control_walk2run_mode_policy_runner.cpp'),
                            str(ROOT / 'state_machine/parameters/cr1_standard_control_parameters.cpp'),
                            str(ROOT / 'state_machine/parameters/cr1_pro_control_parameters.cpp'),
                            *['-I' + str(ROOT / p) for p in includes],
                            '-I' + str(ROOT / 'state_machine/parameters'),
                            '-L' + str(library), '-Wl,-rpath,' + str(library),
                            '-lonnxruntime', '-pthread', '-o', str(entry)], check=True)
            subprocess.run([str(entry), str(MODEL)], check=True)
        else:
            adapter = Path(tmp) / 'rl_adapter'
            logs = ROOT / 'third_party/dr_lib/x86/log4cplus'
            subprocess.run(['g++', '-O2', '-std=c++17', '-DEIGEN_DONT_ALIGN_STATICALLY',
                            '-I' + str(ROOT / 'tests/mimic_stubs'),
                            str(ROOT / 'tests/test_rlcontrol_1026.cpp'),
                            str(ROOT / 'run_policy/cr1/src/amp_walk_policy_runner.cpp'),
                            str(ROOT / 'run_policy/cr1/src/amp_control_gun_shoot_walk_mode_policy_runner.cpp'),
                            str(ROOT / 'state_machine/parameters/cr1_standard_control_parameters.cpp'),
                            str(ROOT / 'state_machine/parameters/cr1_pro_control_parameters.cpp'),
                            *['-I' + str(ROOT / p) for p in includes],
                            '-I' + str(ROOT / 'model_inference'),
                            '-I' + str(ROOT / 'state_machine/parameters'),
                            '-I' + str(logs / 'include'), '-I' + str(logs / 'include/log4cplus_dr'),
                            '-L' + str(logs / 'lib'), '-Wl,-rpath,' + str(logs / 'lib'),
                            '-L' + str(library), '-Wl,-rpath,' + str(library),
                            '-lonnxruntime', '-llog4cplus', '-pthread', '-o', str(adapter)], check=True)
            subprocess.run([str(adapter), str(MODEL)], check=True)
        print(f'PASS {version} {mode}: 280 frames; 438-D observations and ONNX actions; max action error {max_error:.3g}')


if __name__ == '__main__':
    main()
