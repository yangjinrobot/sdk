"""Offline amp_dt YAML/C++/ONNX checks. Run with numpy, PyYAML, onnxruntime installed."""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile
import numpy as np
import onnxruntime as ort
import yaml

ROOT = Path(__file__).resolve().parents[1]
POLICY_ARCHIVE = ROOT.parent / 'policy'
ASSETS = POLICY_ARCHIVE / 'amp_dt'
JOINTS = ['left_hip_pitch_joint','right_hip_pitch_joint','waist_yaw_joint',
          'left_hip_roll_joint','right_hip_roll_joint','left_shoulder_pitch_joint',
          'right_shoulder_pitch_joint','left_hip_yaw_joint','right_hip_yaw_joint',
          'left_shoulder_roll_joint','right_shoulder_roll_joint','left_knee_pitch_joint',
          'right_knee_pitch_joint','left_shoulder_yaw_joint','right_shoulder_yaw_joint',
          'left_ankle_pitch_joint','right_ankle_pitch_joint','left_elbow_pitch_joint',
          'right_elbow_pitch_joint','left_ankle_roll_joint','right_ankle_roll_joint']

class TrainingLoader(yaml.SafeLoader):
    pass
for tag in ('tag:yaml.org,2002:python/tuple','tag:yaml.org,2002:python/object/apply:builtins.slice'):
    TrainingLoader.add_constructor(tag, lambda loader,node: loader.construct_sequence(node))

def joint_value(value, name):
    if not isinstance(value,dict): return float(value)
    matches=[v for pattern,v in value.items() if re.fullmatch(pattern,name)]
    assert len(matches)==1, name
    return float(matches[0])

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--report',type=Path)
    args=parser.parse_args()
    training=yaml.load((ASSETS/'amp_dt.yaml').read_text(),Loader=TrainingLoader)
    includes=['run_policy/cr1/include','types','utils','state_machine','third_party/eigen',
              'third_party/onnxruntime/x86/include']
    library=ROOT/'third_party/onnxruntime/x86/lib'
    def compile_test(source, runner, binary):
        sources = [ROOT / source, ROOT / 'run_policy/cr1/src' / f'{runner}_policy_runner.cpp']
        if runner == 'amp_control_gun_shoot_walk_mode':
            sources.append(ROOT / 'run_policy/cr1/src/common_policy_runner.cpp')
        subprocess.run(['g++','-O2','-std=c++17','-DEIGEN_DONT_ALIGN_STATICALLY',
                        *map(str, sources),
                        *['-I'+str(ROOT/p) for p in includes],'-L'+str(library),
                        '-Wl,-rpath,'+str(library),'-lonnxruntime','-pthread','-o',str(binary)],check=True)
    with tempfile.TemporaryDirectory(prefix='amp_dt_offline_') as temp:
        temp=Path(temp)
        binary=temp/'nirenwalk'; trace=temp/'trace.txt'
        compile_test('tests/test_amp_control_walk2run_mode_policy.cpp', 'amp_control_walk2run_mode', binary)
        subprocess.run([str(binary),str(ASSETS/'amp_dt.onnx'),str(trace)],check=True)
        rows={};samples=[]
        for line in trace.read_text().splitlines():
            name,data=line.split(' ',1)
            values=np.fromstring(data,sep=' ',dtype=np.float32)
            if name=='sample':samples.append(values)
            else:rows[name]=values
        robot=training['scene']['robot']
        defaults=np.array([joint_value(robot['init_state']['joint_pos'],j) for j in JOINTS],np.float32)
        np.testing.assert_allclose(rows['defaults'],defaults,atol=1e-7)
        for kind in ('kp','kd'):
            values=[]
            for joint in JOINTS:
                groups=[g for g in robot['actuators'].values() if any(re.fullmatch(p,joint) for p in g['joint_names_expr'])]
                assert len(groups)==1
                values.append(joint_value(groups[0]['stiffness' if kind=='kp' else 'damping'],joint))
            np.testing.assert_allclose(rows[kind],values,atol=1e-7)
        ranges=training['commands']['ranges']; norm=training['normalization'];scales=norm['obs_scales']
        np.testing.assert_allclose(rows['max_cmd'],[ranges[k][1] for k in ('lin_vel_x','lin_vel_y','ang_vel_z')])
        np.testing.assert_allclose(rows['scales'],[training['robot']['action_scale'],norm['clip_actions'],norm['clip_observations'],scales['ang_vel'],scales['joint_vel']])
        assert ranges['lin_vel_x'][0]==-.5 and ranges['lin_vel_y'][0]==-.6 and ranges['ang_vel_z'][0]==-1
        assert all(scales[k]==1 for k in ('commands','projected_gravity','joint_pos','actions'))
        assert abs(training['sim']['dt']*training['sim']['decimation']-.02)<1e-9
        assert training['robot']['actor_obs_history_length']==5
        gait=training['gait']
        assert gait['gait_cycle']==.85 and gait['gait_phase_offset_l']==.38 and gait['gait_phase_offset_r']==.88
        assert gait['gait_air_ratio_l']==.38 and gait['gait_air_ratio_r']==.38
        options=ort.SessionOptions();options.intra_op_num_threads=1
        session=ort.InferenceSession(str(ASSETS/'amp_dt.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
        assert session.get_inputs()[0].shape[-1]==468 and session.get_outputs()[0].shape[-1]==21
        assert len(samples)==242
        max_obs_error=0.;max_action_error=0.
        for episode in range(2):
            history=deque(maxlen=5); previous=np.zeros(21,np.float32)
            cmd=np.float32(0.)
            for step in range(121):
                sample=samples[episode*121+step]
                actual_input,actual_action=sample[:468],sample[468:]
                current=np.zeros(78,np.float32)
                current[5]=-1
                cmd=min(np.float32(1.), cmd+np.float32(.02))
                forward=np.float32(cmd*np.float32(1.5))
                current[6]=forward if abs(forward)>=np.float32(.1) else np.float32(0)
                # Mirror measured float32 sensor precision, not a precomputed pose.
                q=defaults+np.float32(.001)*np.float32(episode+step%5)
                current[9:30]=q-defaults
                current[51:72]=previous
                if step and abs(current[6])>=.1:
                    phases=(step*.02/gait['gait_cycle']+np.array([gait['gait_phase_offset_l'],gait['gait_phase_offset_r']]))%1
                    current[72:74]=np.sin(2*np.pi*phases)
                    current[74:76]=np.cos(2*np.pi*phases)
                current[76:]=[gait['gait_air_ratio_l'],gait['gait_air_ratio_r']]
                if not history:history.extend(current.copy() for _ in range(5))
                else:history.append(current.copy())
                expected_input=np.concatenate([*history,current])
                max_obs_error=max(max_obs_error,float(np.max(np.abs(expected_input-actual_input))))
                np.testing.assert_allclose(actual_input,expected_input,atol=1e-4,rtol=1e-5)
                expected_action=np.clip(session.run(None,{'obs':expected_input[None]})[0][0],-norm['clip_actions'],norm['clip_actions'])
                max_action_error=max(max_action_error,float(np.max(np.abs(expected_action-actual_action))))
                np.testing.assert_allclose(actual_action,expected_action,atol=1e-4,rtol=1e-5)
                previous=expected_action
        # Shared-runner changes must preserve the existing ordinary RL policy.
        binary=temp/'chiqiangwalk';compile_test(
            'tests/test_amp_control_gun_shoot_walk_legacy_policy.cpp',
            'amp_control_gun_shoot_walk_mode', binary)
        subprocess.run([str(binary), str(POLICY_ARCHIVE / 'std_amp.onnx')], check=True)
        report=dict(status='PASS',model_sha256=hashlib.sha256((ASSETS/'amp_dt.onnx').read_bytes()).hexdigest(),
                    yaml_sha256=hashlib.sha256((ASSETS/'amp_dt.yaml').read_bytes()).hexdigest(),
                    samples_checked=len(samples),max_observation_error=max_obs_error,max_action_error=max_action_error,
                    checked=['YAML defaults/PD/scales/gait/ranges','468-dim history and independent Python observations',
                             'C++ actions vs Python ONNX','strict <0.1 phase mask on all axes',
                             'normalized saturation and asymmetric reverse','immediate entry commands and re-entry',
                             'NaN rejection and observation clipping','ordinary RL regression'])
        print(json.dumps(report,indent=2))
        if args.report:
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(report,indent=2)+'\n')
if __name__=='__main__':main()
