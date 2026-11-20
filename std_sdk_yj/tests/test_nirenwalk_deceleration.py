"""Compile/run the offline S-curve test; requires g++ and bundled ONNX Runtime only."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    includes = ['run_policy/cr1/include', 'types', 'utils', 'state_machine',
                'third_party/eigen', 'third_party/onnxruntime/x86/include']
    library = ROOT / 'third_party/onnxruntime/x86/lib'
    with tempfile.TemporaryDirectory(prefix='nirenwalk-scurve-') as temp:
        binary = Path(temp) / 'test'
        subprocess.run([
            'g++', '-O2', '-std=c++17', '-DEIGEN_DONT_ALIGN_STATICALLY',
            str(ROOT / 'tests/test_nirenwalk_deceleration.cpp'),
            str(ROOT / 'run_policy/cr1/src/run_policy_runner.cpp'),
            *['-I' + str(ROOT / path) for path in includes],
            '-L' + str(library), '-Wl,-rpath,' + str(library),
            '-lonnxruntime', '-pthread', '-o', str(binary),
        ], check=True)
        subprocess.run([str(binary),
                        str(ROOT / 'run_policy/cr1/policy/amp/1028_run/policy_run_2.onnx')],
                       check=True)


if __name__ == '__main__':
    main()
