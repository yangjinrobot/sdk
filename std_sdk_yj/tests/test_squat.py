"""Offline MimicControl_Kneel_Mode/MimicControl_Kneel_StandUp_Mode checks. Needs numpy, PyYAML and onnxruntime.

python -B tests/test_squat.py
No robot/DDS connection. The pair-specific NPZ is checked against the independently
read ONNX constants before the C++ runner is tested.
"""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

import numpy as np
import yaml
from test_nirenwalk_amp_dt import TrainingLoader, joint_value
from test_sit_stand import embedded_reference

ROOT = Path(__file__).resolve().parents[1]


def npz_reference(directory, manifest, payload):
    path = directory / manifest['motion_file']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest['motion_sha256']
    with np.load(path, allow_pickle=False) as motion:
        expected_keys = {
            'fps', 'joint_pos', 'joint_vel', 'body_pos_w', 'body_quat_w',
            'body_lin_vel_w', 'body_ang_vel_w', 'joint_names', 'body_names',
        }
        assert set(motion.files) == expected_keys
        assert motion['fps'].tolist() == [manifest['fps']]
        assert motion['joint_names'].tolist() == payload['joint_names']
        assert motion['joint_pos'].shape == (manifest['num_frames'], manifest['num_joints'])
        assert motion['joint_vel'].shape == motion['joint_pos'].shape
        body_index = {
            name: index for index, name in enumerate(motion['body_names'].tolist())
        }
        selected_bodies = [body_index[name] for name in payload['body_names']]
        for key in ('joint_pos', 'joint_vel'):
            np.testing.assert_array_equal(motion[key], np.asarray(payload[key]))
        for key in ('body_pos_w', 'body_quat_w', 'body_lin_vel_w', 'body_ang_vel_w'):
            width = 4 if key == 'body_quat_w' else 3
            expected = np.asarray(payload[key]).reshape(
                manifest['num_frames'], len(payload['body_names']), width
            )
            np.testing.assert_array_equal(
                motion[key][:, selected_bodies], expected
            )
        result = {key: motion[key] for key in motion.files}
    print(f"NPZ: {path.name}, {len(result['joint_pos'])} frames, matched ONNX constants",
          flush=True)
    return result


def yaml_training(directory, manifest):
    cfg = yaml.load((directory / manifest['training_file']).read_text(), Loader=TrainingLoader)
    robot = cfg['scene']['robot']
    names = manifest['training']['joint_names']
    assert len(names) == len(set(names)) == 21
    expected = dict(default_joint_pos=[], joint_stiffness=[], joint_damping=[],
                    action_scale=[], effort_limit=[])
    action = cfg['actions']['joint_pos']
    assert action['use_default_offset'] and action['clip'] is None and action['joint_names'] == ['.*']
    for joint in names:
        groups = [g for g in robot['actuators'].values()
                  if any(re.fullmatch(p, joint) for p in g['joint_names_expr'])]
        assert len(groups) == 1
        group = groups[0]
        for key, field in [('joint_stiffness', 'stiffness'), ('joint_damping', 'damping'),
                           ('effort_limit', 'effort_limit_sim')]:
            expected[key].append(joint_value(group[field], joint))
        expected['default_joint_pos'].append(joint_value(robot['init_state']['joint_pos'], joint))
        assert joint_value(robot['init_state']['joint_vel'], joint) == 0
        expected['action_scale'].append(joint_value(action['scale'], joint))
    for key, value in expected.items():
        np.testing.assert_allclose(manifest['training'][key], value, rtol=0, atol=1e-7)
    np.testing.assert_allclose(expected['action_scale'],
                               .25 * np.array(expected['effort_limit']) / expected['joint_stiffness'])
    assert cfg['sim']['dt'] * cfg['decimation'] == .02
    assert not cfg['commands']['motion']['use_default_joint_pos_after_motion']
    terms = cfg['observations']['policy']
    assert [k for k, v in terms.items() if isinstance(v, dict) and 'func' in v] == [
        'command', 'motion_anchor_ori_b', 'base_ang_vel', 'joint_pos', 'joint_vel', 'actions']
    assert terms['history_length'] is None
    for key in ['command', 'motion_anchor_ori_b', 'base_ang_vel', 'joint_pos', 'joint_vel', 'actions']:
        assert terms[key]['history_length'] == 0
        assert all(terms[key].get(field) is None for field in ['clip', 'scale', 'modifiers'])
    return expected


def main():
    includes = ['tests/mimic_stubs', 'run_policy/cr1/include', 'types', 'utils',
                'state_machine', 'interface/user_command', 'third_party/eigen',
                'third_party/onnxruntime/x86/include']
    library = ROOT / 'third_party/onnxruntime/x86/lib'
    runner_dir = Path('run_policy/cr1/src')

    def compile_test(binary, sources, flags=()):
        subprocess.run(['g++', '-O2', '-std=c++17', '-DEIGEN_DONT_ALIGN_STATICALLY', *flags,
                        *[str(ROOT / source) for source in sources],
                        *['-I' + str(ROOT / p) for p in includes], '-L' + str(library),
                        '-Wl,-rpath,' + str(library), '-lonnxruntime', '-pthread', '-o', str(binary)], check=True)

    with tempfile.TemporaryDirectory(prefix='squat-check-') as temp:
        temp = Path(temp)
        references = {}
        cases = [
            ('dunxia', 'TEST_MIMIC_CONTROL_KNEEL_MODE',
             'mimic_control_kneel_mode_policy_runner.cpp'),
            ('zhanli', 'TEST_MIMIC_CONTROL_KNEEL_STAND_UP_MODE',
             'mimic_control_kneel_stand_up_mode_policy_runner.cpp'),
        ]
        for kind, macro, runner in cases:
            directory = ROOT / 'run_policy/cr1/policy/mimic' / (kind + '_policy')
            manifest = json.loads((directory / 'motion.json').read_text())
            for filename, field in [(manifest['policy_file'], 'model_sha256'),
                                    (manifest['training_file'], 'training_sha256')]:
                assert hashlib.sha256((directory / filename).read_bytes()).hexdigest() == manifest[field]
            payload = embedded_reference(directory / manifest['policy_file'], manifest)
            references[kind] = npz_reference(directory, manifest, payload)
            assert references[kind]['joint_names'].tolist() == payload['joint_names']
            payload['training'] = yaml_training(directory, manifest)
            binary = temp / kind
            compile_test(binary, ['tests/test_mimic_control_chair_policy.cpp',
                                  runner_dir / runner], ['-D' + macro])
            subprocess.run([str(binary), str(directory)], input=json.dumps(payload), text=True, check=True)
            mismatch = temp / (kind + '_mismatch')
            mismatch.mkdir()
            (mismatch / manifest['policy_file']).symlink_to(directory / manifest['policy_file'])
            manifest['training']['default_joint_pos'][0] += .01
            (mismatch / 'motion.json').write_text(json.dumps(manifest))
            rejected = subprocess.run([str(binary), str(mismatch)], input=json.dumps(payload),
                                      text=True, capture_output=True)
            assert rejected.returncode != 0 and 'mismatch' in rejected.stderr
        for key in ('joint_pos', 'joint_vel', 'body_pos_w', 'body_quat_w',
                    'body_lin_vel_w', 'body_ang_vel_w'):
            np.testing.assert_array_equal(references['dunxia'][key][-1],
                                          references['zhanli'][key][0])
        parameters = ['state_machine/parameters/cr1_standard_control_parameters.cpp',
                      'state_machine/parameters/cr1_pro_control_parameters.cpp']
        states = temp / 'states'
        compile_test(states, ['tests/test_mimic_control_kneel_states.cpp', *parameters,
                              runner_dir / 'mimic_control_kneel_mode_policy_runner.cpp',
                              runner_dir / 'mimic_control_kneel_stand_up_mode_policy_runner.cpp'],
                     ['-DMOTION_MASTER_MUJOCO_LOCKSTEP'])
        subprocess.run([str(states), str(ROOT / 'run_policy/cr1/policy/mimic')], check=True)
        gamepad = temp / 'gamepad'
        compile_test(gamepad, ['tests/test_gamepad_motion.cpp'])
        subprocess.run([str(gamepad)], check=True)
        print('PASS: squat/rise YAML parity, embedded references, raw actions, motion slots, source restrictions')


if __name__ == '__main__':
    main()
