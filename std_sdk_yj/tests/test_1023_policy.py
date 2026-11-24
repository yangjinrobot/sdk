"""Offline 1023 NirenWalk observation/ONNX parity; no robot connection."""
from collections import deque
from pathlib import Path
import subprocess
import tempfile

import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT.parent / 'policy/1023/policy_11500.onnx'


def main():
    includes = ['run_policy/cr1/include', 'types', 'utils', 'state_machine',
                'third_party/eigen', 'third_party/onnxruntime/x86/include']
    library = ROOT / 'third_party/onnxruntime/x86/lib'
    with tempfile.TemporaryDirectory(prefix='1023-policy-') as tmp:
        binary, trace = Path(tmp) / 'check', Path(tmp) / 'trace.txt'
        subprocess.run(['g++', '-O2', '-std=c++17', '-DEIGEN_DONT_ALIGN_STATICALLY',
                        str(ROOT / 'tests/test_1022_policy.cpp'),
                        str(ROOT / 'run_policy/cr1/src/run_policy_runner.cpp'),
                        str(ROOT / 'run_policy/cr1/src/chiqiangwalk_policy_runner.cpp'),
                        *['-I' + str(ROOT / p) for p in includes], '-L' + str(library),
                        '-Wl,-rpath,' + str(library), '-lonnxruntime', '-pthread',
                        '-o', str(binary)], check=True)
        subprocess.run([str(binary), '1023', str(MODEL), str(trace)], check=True)
        rows, samples = {}, []
        for line in trace.read_text().splitlines():
            key, data = line.split(' ', 1)
            values = np.fromstring(data, sep=' ', dtype=np.float32)
            if key == 'sample':
                samples.append(values)
            else:
                rows[key] = values
        np.testing.assert_allclose(rows['defaults'][17:19], [1.2, 1.2], atol=1e-7)
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
            target = np.array([1 if episode == 0 else -1/3, 1, -1], np.float32)
            for step in range(140):
                command += np.clip(target - command, [-.02, -.02, -.04],
                                   [.02, .02, .04]).astype(np.float32)
                velocity = command * np.array([1.5, .6, 1], np.float32)
                moving = np.linalg.norm(velocity[:2]) > np.float32(.15) or abs(velocity[2]) > np.float32(.15)
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
                np.testing.assert_allclose(actual[:438], expected, atol=5e-5, rtol=2e-5)
                previous = np.clip(session.run(None, {'obs': expected[None]})[0][0], -10, 10)
                max_error = max(max_error, float(np.max(np.abs(previous - actual[438:]))))
                np.testing.assert_allclose(actual[438:], previous, atol=1e-4, rtol=1e-5)
        print(f'PASS 1023: 280 frames; 438-D observations and ONNX actions; max action error {max_error:.3g}')


if __name__ == '__main__':
    main()
