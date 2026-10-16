"""
 * @file mujoco_simulation_pro_upper.py
 * @brief upper-body simulation in MuJoCo (lower body locked)
 * @author chenchangshuo
 * @version 1.4
 * @date 2026-03-30
 *
 * @copyright Copyright (c) 2024  DeepRobotics
"""

import os
import time
import signal
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from drdds.msg import ImuData, ImuDataValue, Joints, JointsCmd, JointValue, MetaType

MODEL_NAME = "DR2PRO"
# Get the directory of the current Python file
CURRENT_DIR = Path(__file__).resolve().parent

# Define the XML path relative to the Python file
XML_PATH = CURRENT_DIR / "DR2-B2-PRO-URDF" / "DR2-B2-pro_simplify_collision.xml"

# Convert to absolute path as string
XML_PATH = str(XML_PATH.resolve())
USE_VIEWER = True
DT = 0.001
STATE_PUBLISH_DECIMATION = 2
RENDER_INTERVAL = 40

URDF_INIT = {
    "DR2PRO": np.array([0, ] * 29, dtype=np.float32)
}

virtual_joint_num = {
    "DR2PRO": 2,
}


class MuJoCoSimulationNode(Node):
    def __init__(self,
                 model_key: str = MODEL_NAME,
                 xml_path: str = XML_PATH):
        super().__init__('mujoco_simulation_pro_upper')
        self.exit_requested = False

        self.robot_name = "DR2PRO"

        # Load MJCF
        if not os.path.isfile(xml_path):
            raise FileNotFoundError(f"Cannot find MJCF: {xml_path}")

        self.model = mujoco.MjModel.from_xml_path(xml_path)

        self.model.opt.timestep = DT
        self.data = mujoco.MjData(self.model)

        # Robot DOF list
        self.dof_num = int(self.model.nu)
        self.get_logger().info(f"model_key: {model_key}, dof = {self.dof_num}")
        if self.dof_num == 0:
            raise RuntimeError(
                f"Loaded model has zero actuator (nu=0): {xml_path}. "
                "Please use MJCF .xml with actuators."
            )

        self.virtual_joint_num = virtual_joint_num[self.robot_name]
        self.total_joint_num = self.dof_num + self.virtual_joint_num
        self.virtual_joint = np.zeros(self.virtual_joint_num, dtype=np.float32) if self.virtual_joint_num > 0 else None

        # Initialize standing pose
        self._set_initial_pose(model_key)

        # Control buffers
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.kd_cmd = np.zeros_like(self.kp_cmd)
        self.pos_cmd = np.zeros_like(self.kp_cmd)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)
        self.input_tq = np.zeros_like(self.kp_cmd)
        self.has_cmd = False

        # IMU
        self.timestamp = 0.0

        # ROS2 DDS communication
        self.joint_pub = self.create_publisher(Joints, '/JOINTS_DATA', 50)
        self.imu_body_pub = self.create_publisher(ImuData, '/IMU_DATA_BODY', 50)
        self.imu_leg_pub = self.create_publisher(ImuData, '/IMU_DATA_LEG', 50)
        self.cmd_sub = self.create_subscription(JointsCmd, '/JOINTS_CMD', self._cmd_callback, 50)

        self.get_logger().info("DDS topics ready: /JOINTS_CMD, /JOINTS_DATA, /IMU_DATA_BODY, /IMU_DATA_LEG")

        # Visualization
        self.viewer = None
        if USE_VIEWER:
            try:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                self.viewer.cam.distance = 3.0
                self.viewer.cam.lookat = [0.0, 0.0, 0.8]
                self.viewer.cam.azimuth = 45.0
                self.viewer.cam.elevation = -30.0
            except Exception as e:
                self.get_logger().warning(f"Failed to launch viewer: {e}")

        dof_list = [0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0]
        self.arr_pos = np.array([0., 0., 0.9], dtype=np.float64)
        self.arr_quat = np.array([0., 0., 0., 1.0], dtype=np.float64)
        self.arr_init_dof = np.array(dof_list, dtype=np.float64)
        self.base_init_pos = self.arr_pos.copy()
        self.base_init_quat = self.arr_quat.copy()

        # Upper-body simulation: lock lower-body joints (waist3 + arms14 + legs12 + virtual2 = total31).
        # For MuJoCo actuators (dof_num=29), lower body is actuator indices [17, 28].
        self.lower_start_idx = min(17, self.dof_num)
        self.lower_end_idx = min(29, self.dof_num)
        self.lower_lock_pos = self.arr_init_dof[self.lower_start_idx:self.lower_end_idx].astype(np.float32).reshape(-1, 1)
        self.lower_lock_vel = np.zeros_like(self.lower_lock_pos)
        self.lower_lock_kp = np.full_like(self.lower_lock_pos, 300.0)
        self.lower_lock_kd = np.full_like(self.lower_lock_pos, 8.0)
        self.get_logger().info(
            f"Upper-body mode enabled: lock lower-body actuator range [{self.lower_start_idx}, {self.lower_end_idx - 1}]"
        )

        # Ensure Ctrl+C / SIGTERM can always stop the loop even when viewer has GLX issues.
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        self.exit_requested = True

    def _set_initial_pose(self, key: str):
        qpos0 = self.data.qpos.copy()
        qpos0[7:7 + self.dof_num] = URDF_INIT[key]
        qpos0[:3] = np.array([0, 0, 0.5])
        qpos0[3:7] = np.array([0.707, 0, -0.707, 0])
        self.data.qpos[:] = qpos0
        mujoco.mj_forward(self.model, self.data)

    def _cmd_callback(self, msg: JointsCmd):
        if len(msg.data) < self.dof_num:
            self.get_logger().warning(
                f"Received JointsCmd size {len(msg.data)} < required {self.dof_num}, ignore"
            )
            return

        # Support command length of either dof or dof+virtual_joint
        for i in range(self.dof_num):
            cmd = msg.data[i]
            # Lower-body joints are intentionally non-controllable in upper-body simulation mode.
            if self.lower_start_idx <= i < self.lower_end_idx:
                continue
            self.kp_cmd[i] = cmd.kp
            self.kd_cmd[i] = cmd.kd
            self.pos_cmd[i] = cmd.position
            self.vel_cmd[i] = cmd.velocity
            self.tau_ff[i] = cmd.torque
        self.has_cmd = True

    def start(self):
        step = 0
        last_time = time.time()
        init_time = time.time()

        while rclpy.ok() and not self.exit_requested:
            if time.time() - last_time >= DT:
                last_time = time.time()
                step += 1

                # Before first /JOINTS_CMD, keep stand pose.
                # After first command, switch to command control.
                if not self.has_cmd:
                    self._set_mimic_init_state()
                    self.input_tq.fill(0.0)
                    self.data.ctrl[:] = 0.0
                else:
                    self._apply_joint_torque()

                # Simulate one step
                self._enforce_upper_only_constraints()
                mujoco.mj_step(self.model, self.data)
                self._enforce_upper_only_constraints()
                self.timestamp = time.time() - init_time
                if step % STATE_PUBLISH_DECIMATION == 0:
                    self._publish_robot_state()

                # Visualize
                if self.viewer and step % RENDER_INTERVAL == 0:
                    if hasattr(self.viewer, "is_running") and not self.viewer.is_running():
                        self.get_logger().info("Viewer closed, exiting simulation loop.")
                        self.exit_requested = True
                        continue
                    try:
                        if hasattr(signal, "pthread_sigmask"):
                            signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})
                        self.viewer.sync()
                    except Exception as e:
                        self.get_logger().warning(f"Viewer sync failed: {e}. Disable viewer and continue.")
                        self.viewer = None
                    finally:
                        if hasattr(signal, "pthread_sigmask"):
                            signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGINT})

            try:
                rclpy.spin_once(self, timeout_sec=0.0)
            except ExternalShutdownException:
                break
            except RuntimeError as e:
                # Workaround for occasional pybind conversion race during Ctrl+C shutdown.
                if "Unable to convert call argument" in str(e):
                    self.get_logger().info("Spin interrupted during shutdown, exiting loop safely.")
                    break
                raise

    def _apply_joint_torque(self):
        # Force lower-body to stay fixed in upper-body simulation mode.
        self.pos_cmd[self.lower_start_idx:self.lower_end_idx] = self.lower_lock_pos
        self.vel_cmd[self.lower_start_idx:self.lower_end_idx] = self.lower_lock_vel
        self.kp_cmd[self.lower_start_idx:self.lower_end_idx] = self.lower_lock_kp
        self.kd_cmd[self.lower_start_idx:self.lower_end_idx] = self.lower_lock_kd

        q = self.data.qpos[7:7 + self.dof_num].reshape(-1, 1)
        dq = self.data.qvel[6:6 + self.dof_num].reshape(-1, 1)
        self.input_tq = (
            self.kp_cmd * (self.pos_cmd - q) +
            self.kd_cmd * (self.vel_cmd - dq)
        )
        self.data.ctrl[:] = self.input_tq.flatten()

    def _enforce_upper_only_constraints(self):
        # Keep the floating base fixed so the robot cannot fall in upper-body-only simulation.
        self.data.qpos[0:3] = self.base_init_pos.astype(np.float64)
        self.data.qpos[3:7] = self.base_init_quat.astype(np.float64)
        self.data.qvel[0:6] = 0.0

        # Hard-lock lower-body joints against any physics drift.
        qpos_start = 7 + self.lower_start_idx
        qpos_end = 7 + self.lower_end_idx
        qvel_start = 6 + self.lower_start_idx
        qvel_end = 6 + self.lower_end_idx
        self.data.qpos[qpos_start:qpos_end] = self.lower_lock_pos.flatten().astype(np.float64)
        self.data.qvel[qvel_start:qvel_end] = 0.0

    def _set_mimic_init_state(self):
        self.data.qpos[-self.dof_num:] = self.arr_init_dof.astype(np.float64)
        self.data.qpos[0:3] = self.arr_pos.astype(np.float64)
        self.data.qpos[3:7] = self.arr_quat.astype(np.float64)
        self.data.qvel[:] = np.zeros((35,), dtype=np.float64)

    def quaternion_to_euler(self, q):
        w, x, y, z = q
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return np.array([roll, pitch, yaw], dtype=np.float32)

    def _build_meta(self) -> MetaType:
        header = MetaType()
        header.frame_id = 0
        # Transport latency is measured against the receiver's realtime clock,
        # so publish a ROS system-time stamp instead of simulation elapsed time.
        header.stamp = self.get_clock().now().to_msg()
        return header

    def _publish_robot_state(self):
        q_world = self.data.sensordata[:4]
        rpy_rad = self.quaternion_to_euler(q_world)
        rpy_deg = rpy_rad * (180.0 / np.pi)

        body_acc = self.data.sensordata[4:7]
        angvel_b = self.data.sensordata[7:10]

        q = self.data.qpos[7:7 + self.dof_num]
        dq = self.data.qvel[6:6 + self.dof_num]
        tau = self.input_tq.flatten()

        # IMU body
        imu_body_msg = ImuData()
        imu_body_msg.header = self._build_meta()
        imu_body_msg.data = ImuDataValue()
        imu_body_msg.data.roll = float(rpy_deg[0])
        imu_body_msg.data.pitch = float(rpy_deg[1])
        imu_body_msg.data.yaw = float(rpy_deg[2])
        imu_body_msg.data.omega_x = float(angvel_b[0])
        imu_body_msg.data.omega_y = float(angvel_b[1])
        imu_body_msg.data.omega_z = float(angvel_b[2])
        imu_body_msg.data.acc_x = float(body_acc[0])
        imu_body_msg.data.acc_y = float(body_acc[1])
        imu_body_msg.data.acc_z = float(body_acc[2])
        self.imu_body_pub.publish(imu_body_msg)

        # IMU leg (Only one IMU is available in simulation; publish identical data)
        imu_leg_msg = ImuData()
        imu_leg_msg.header = imu_body_msg.header
        imu_leg_msg.data = ImuDataValue()
        imu_leg_msg.data.roll = imu_body_msg.data.roll
        imu_leg_msg.data.pitch = imu_body_msg.data.pitch
        imu_leg_msg.data.yaw = imu_body_msg.data.yaw
        imu_leg_msg.data.omega_x = imu_body_msg.data.omega_x
        imu_leg_msg.data.omega_y = imu_body_msg.data.omega_y
        imu_leg_msg.data.omega_z = imu_body_msg.data.omega_z
        imu_leg_msg.data.acc_x = imu_body_msg.data.acc_x
        imu_leg_msg.data.acc_y = imu_body_msg.data.acc_y
        imu_leg_msg.data.acc_z = imu_body_msg.data.acc_z
        self.imu_leg_pub.publish(imu_leg_msg)

        joints_msg = Joints()
        joints_msg.header = self._build_meta()
        joints_msg.data = [JointValue() for _ in range(self.total_joint_num)]

        for i in range(self.dof_num):
            joint = joints_msg.data[i]
            joint.name = [32, 32, 32, 32]
            joint.data_id = 0
            joint.status_word = 1
            joint.position = float(q[i])
            joint.torque = float(tau[i])
            joint.velocity = float(dq[i])
            joint.motion_temp = 40.0
            joint.driver_temp = 45.0

        for i in range(self.dof_num, self.total_joint_num):
            joint = joints_msg.data[i]
            joint.name = [32, 32, 32, 32]
            joint.data_id = 0
            joint.status_word = 1
            joint.position = float(q[0])
            joint.torque = float(tau[0])
            joint.velocity = float(dq[0])
            joint.motion_temp = 40.0
            joint.driver_temp = 45.0

        self.joint_pub.publish(joints_msg)


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    rclpy.init()
    sim_node = None
    try:
        sim_node = MuJoCoSimulationNode()
        sim_node.start()
    except KeyboardInterrupt:
        pass
    finally:
        if sim_node is not None:
            sim_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    os._exit(0)
