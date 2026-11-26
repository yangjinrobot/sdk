"""Offline reference checks; run with a Python environment containing NumPy/onnxruntime.

    python -B tests/test_restand.py

Uses the real ONNX and independent training parameter fixtures, no robot/DDS connection.
"""
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[1]
POLICY = ROOT / 'run_policy/cr1/policy/rl/RL_stand.onnx'


def reference():
    # Reference values from the original STD training export. No config file is loaded.
    cfg = dict(
        default_angles=[-.2, 0, 0, .4, -.2, 0] * 2 + [0, 0, .15, 0, -.2, 0, -.15, 0, -.2],
        kp=[300, 300, 200, 300, 50, 30] * 2 + [200] + [100] * 8,
        kd=[15, 15, 10, 15, 2, 1] * 2 + [5] + [3] * 8,
        cycle_time=.8, clip_observations=100., clip_actions=10.,
        obs_scales=dict(ang_vel=1., lin_vel=2., dof_pos=1., dof_vel=.05),
    )
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    session = ort.InferenceSession(str(POLICY), options, providers=['CPUExecutionProvider'])
    mapping = np.array([9,10,11,12,13,14,15,16,17,18,19,20,0,1,2,3,4,5,6,7,8])
    rng = np.random.default_rng(200012)
    sequences = []
    for replay in range(2):
        history = [np.zeros(66, np.float32) for _ in range(5)]
        action = np.zeros(13, np.float32)
        phase_time = 0.
        frames = []
        for step in range(200):
            q = np.asarray(cfg['default_angles']) + rng.normal(0, .03, 21)
            dq = rng.normal(0, .3, 21)
            rpy = rng.normal(0, .03, 3)
            omega = rng.normal(0, .05, 3)
            command = [.18, .8, -.7, -.2, 1.5][step // 40]
            vx = np.clip(command, -1, 1) * .5
            if abs(vx) < .1:
                vx = 0.
            roll, pitch, _ = rpy
            gravity = [np.sin(pitch), -np.sin(roll)*np.cos(pitch), -np.cos(roll)*np.cos(pitch)]
            phase = 2*np.pi*phase_time/cfg['cycle_time']
            obs = np.concatenate([omega * cfg['obs_scales']['ang_vel'], gravity,
                                  [vx*cfg['obs_scales']['lin_vel'], 0, 0],
                                  (q-np.asarray(cfg['default_angles']))*cfg['obs_scales']['dof_pos'],
                                  dq*cfg['obs_scales']['dof_vel'], action, [np.sin(phase), np.cos(phase)]]).astype(np.float32)
            obs = np.clip(obs, -cfg['clip_observations'], cfg['clip_observations'])
            history = history[1:] + [obs]
            stacked = np.concatenate(history)
            action = np.clip(session.run(['actions'], {'obs': stacked[None]})[0][0],
                             -cfg['clip_actions'], cfg['clip_actions'])
            sdk_q, sdk_dq = np.empty(21), np.empty(21)
            sdk_q[mapping], sdk_dq[mapping] = q, dq
            frames.append(dict(q=sdk_q.tolist(), dq=sdk_dq.tolist(), rpy=rpy.tolist(), omega=omega.tolist(),
                               command=command, history=stacked.tolist(), action=action.tolist()))
            phase_time = phase_time + .02 if abs(vx) > .1 else 0.
        sequences.append(frames)
    return dict(config=cfg, sequences=sequences)


def main():
    payload = reference()
    library = ROOT / 'third_party/onnxruntime/x86/lib'
    includes = ['tests/mimic_stubs', 'run_policy/cr1/include', 'types', 'utils',
                'state_machine', 'third_party/eigen', 'third_party/onnxruntime/x86/include']
    with tempfile.TemporaryDirectory(prefix='anhui_restand_') as tmp:
        binary = Path(tmp) / 'test_restand'
        subprocess.run(['g++', '-O2', '-std=c++17', '-DEIGEN_DONT_ALIGN_STATICALLY',
                        str(ROOT / 'tests/test_rl_control_stand_mode.cpp'),
                        str(ROOT / 'run_policy/cr1/src/rl_control_stand_mode_policy_runner.cpp'),
                        str(ROOT / 'state_machine/parameters/cr1_standard_control_parameters.cpp'),
                        str(ROOT / 'state_machine/parameters/cr1_pro_control_parameters.cpp'),
                        *['-I' + str(ROOT / p) for p in includes], '-L' + str(library),
                        '-Wl,-rpath,' + str(library), '-lonnxruntime', '-pthread', '-o', str(binary)], check=True)
        # Only the ONNX is present: loading and inference must work without a sidecar file.
        model_only = Path(tmp) / 'RL_stand.onnx'
        model_only.symlink_to(POLICY)
        subprocess.run([str(binary), str(model_only)], input=json.dumps(payload), text=True, check=True)
        mismatch = Path(tmp) / 'mismatch'
        mismatch.mkdir()
        wrong_model = mismatch / 'RL_stand.onnx'
        wrong_model.symlink_to(ROOT.parent / 'policy/std_amp.onnx')
        rejected = subprocess.run([str(binary), str(wrong_model)], text=True, capture_output=True)
        assert rejected.returncode != 0 and 'expected float [batch,330]' in rejected.stderr
        print('PASS: ONNX-only deployment and incompatible model shape rejection')


if __name__ == '__main__':
    main()
