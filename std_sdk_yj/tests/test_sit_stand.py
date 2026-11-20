"""Run offline model and state checks: python3 -B tests/test_sit_stand.py.
Requires g++, bundled x86 ONNX Runtime, and Python standard library only.
"""
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

def varint(data, offset):
    value = 0
    for shift in range(0, 70, 7):
        if offset >= len(data):
            raise ValueError("Truncated protobuf varint")
        byte = data[offset]
        offset += 1
        value |= (byte & 127) << shift
        if byte < 128:
            return value, offset
    raise ValueError("Invalid protobuf varint")


def fields(data):
    result, offset = [], 0
    while offset < len(data):
        tag, offset = varint(data, offset)
        number, wire = tag >> 3, tag & 7
        if not number:
            raise ValueError("Invalid protobuf field")
        if wire == 0:
            value, offset = varint(data, offset)
        else:
            if wire == 2:
                size, offset = varint(data, offset)
            elif wire in (1, 5):
                size = 8 if wire == 1 else 4
            else:
                raise ValueError(f"Unsupported protobuf wire type {wire}")
            value = data[offset:offset+size]
            offset += size
            if len(value) != size:
                raise ValueError("Truncated protobuf field")
        result.append((number, wire, value))
    return result


def values(items, number):
    return [value for field, _, value in items if field == number]


def only(items, number):
    result = values(items, number)
    if len(result) != 1:
        raise ValueError(f"Expected one field {number}, found {len(result)}")
    return result[0]


def embedded_reference(model_path, manifest):
    """Read the exported graph constants independently of the C++ runtime."""
    raw = model_path.read_bytes()
    model = fields(raw)
    graph = fields(only(model, 7))
    metadata = {only(fields(entry), 1).decode(): only(fields(entry), 2).decode()
                for entry in values(model, 14)}
    nodes = [fields(node) for node in values(graph, 1)]
    producers = {output: node for node in nodes for output in values(node, 2)}
    frames = manifest["num_frames"]
    payload = {"frames": frames, "fps": manifest["fps"],
               "joint_names": metadata["joint_names"].split(","),
               "body_names": metadata["body_names"].split(",")}
    assert len(payload["joint_names"]) == manifest["num_joints"] == 21
    for name in ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w",
                 "body_lin_vel_w", "body_ang_vel_w"):
        gather = producers[name.encode()]
        assert only(gather, 4) == b"Gather", name
        constant = producers[values(gather, 1)[0]]
        assert only(constant, 4) == b"Constant", name
        tensors = [only(fields(attribute), 5) for attribute in values(constant, 5)
                   if values(fields(attribute), 5)]
        assert len(tensors) == 1, name
        tensor = fields(tensors[0])
        shape = values(tensor, 1)
        expected = ([frames, 21] if name.startswith("joint_") else
                    [frames, len(payload["body_names"]), 4 if name == "body_quat_w" else 3])
        assert shape == expected and only(tensor, 2) == 1, (name, shape)
        data = [value[0] for value in struct.iter_unpack("<f", only(tensor, 9))]
        assert len(data) == math.prod(shape) and all(map(math.isfinite, data)), name
        width = math.prod(shape[1:])
        payload[name] = [data[i * width:(i + 1) * width] for i in range(frames)]
    anchor = payload["body_names"].index("base_link")
    payload["anchor_quat"] = [q[anchor * 4:(anchor + 1) * 4] for q in payload["body_quat_w"]]
    print(f"Model SHA256: {hashlib.sha256(raw).hexdigest()}", flush=True)
    print(f"Reference source: ONNX constants, {frames} frames; no NPZ required", flush=True)
    return payload


def main():
    includes = ["run_policy/cr1/include", "types", "utils", "state_machine",
                "interface/user_command", "third_party/eigen", "third_party/onnxruntime/x86/include"]
    library = ROOT / "third_party/onnxruntime/x86/lib"
    runner_dir = Path("run_policy/cr1/src")
    # RobotBasicState in this SDK is packed to 2 bytes; use the same Eigen
    # alignment setting as the original project's x86 offline tests.
    def compile_test(binary, sources, flags=()):
        subprocess.run(["g++", "-O2", "-std=c++17", "-DEIGEN_DONT_ALIGN_STATICALLY", *flags,
                        *[str(ROOT / source) for source in sources],
                        *["-I" + str(ROOT / path) for path in includes],
                        "-L" + str(library), "-Wl,-rpath," + str(library),
                        "-lonnxruntime", "-pthread", "-o", str(binary)], check=True)
    with tempfile.TemporaryDirectory(prefix="anhui_mimic_test_") as temp:
        temp = Path(temp)
        gamepad = temp / "gamepad"
        compile_test(gamepad, ["tests/test_gamepad_motion.cpp"])
        subprocess.run([str(gamepad)], check=True)
        for kind in ("stand2sit", "sit2stand"):
            assets = ROOT / "run_policy/cr1/policy/mimic" / (kind + "_policy")
            manifest = json.loads((assets / "motion.json").read_text())
            payload = embedded_reference(assets / manifest["policy_file"], manifest)
            binary = temp / kind
            flags = ["-DTEST_SIT2STAND_MIMIC"] if kind == "sit2stand" else []
            compile_test(binary, ["tests/test_sit_stand_policy.cpp",
                                  runner_dir / f"{kind}_mimic_policy_runner.cpp"], flags)
            subprocess.run([str(binary), str(assets)], input=json.dumps(payload), text=True, check=True)
            mismatched = temp / (kind + "_mismatch")
            mismatched.mkdir()
            (mismatched / manifest["policy_file"]).symlink_to(assets / manifest["policy_file"])
            manifest["training"]["default_joint_pos"][1] += .01
            (mismatched / "motion.json").write_text(json.dumps(manifest))
            rejected = subprocess.run([str(binary), str(mismatched)], input=json.dumps(payload),
                                      text=True, capture_output=True)
            assert rejected.returncode != 0 and "model/training mismatch" in rejected.stderr
            print("PASS: mismatched training manifest rejected", flush=True)
        binary = temp / "states"
        compile_test(binary, ["tests/test_sit_stand_states.cpp",
                             runner_dir / "stand2sit_mimic_policy_runner.cpp",
                             runner_dir / "sit2stand_mimic_policy_runner.cpp",
                             "state_machine/parameters/cr1_standard_control_parameters.cpp",
                             "state_machine/parameters/cr1_pro_control_parameters.cpp"],
                     ["-I" + str(ROOT / "tests/mimic_stubs"), "-DMOTION_MASTER_MUJOCO_LOCKSTEP"])
        subprocess.run([str(binary), str(ROOT / "run_policy/cr1/policy/mimic/sit2stand_policy")], check=True)


if __name__ == "__main__":
    main()
