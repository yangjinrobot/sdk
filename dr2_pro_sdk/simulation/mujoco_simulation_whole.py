"""
 * @file mujoco_simulation_pro.py
 * @brief simulation in MuJoCo
 * @author chenchangshuo
 * @version 1.4
 * @date 2026-03-30
 *
 * @copyright Copyright (c) 2024  DeepRobotics
"""

import argparse
import os
import time
from pathlib import Path

import cv2
import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from drdds.msg import ImuData, ImuDataValue, Joints, JointsCmd, JointValue, MetaType
from sensor_msgs.msg import Image

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
DEPTH_PUBLISH_DECIMATION = 20
DEPTH_HEIGHT = 36
DEPTH_WIDTH = 64
DEPTH_FOV_Y_DEG = 58.29
DEPTH_FOV_X_DEG = 89.51
DEPTH_MAX_METERS = 2.5
DEPTH_CAMERA_BODY = "body"
DEPTH_CAMERA_OFFSET = np.array([0.16, 0.0, 0.22], dtype=np.float64)
DEPTH_CAMERA_QUAT_WXYZ = np.array([0.843391, 0.0, 0.537300, 0.0], dtype=np.float64)
DEPTH_CROP = (0, 0, 16, 16)  # up, down, left, right; aligned with sim2sim
DEPTH_WINDOW_NAME = "Raw Depth | Processed Depth"
DEPTH_WINDOW_SCALE = 8
SHOW_DEPTH_WINDOW = True
DEPTH_VISUALIZATION_STRIDE = 2  # 50 Hz depth source / 2 = 25 Hz preview

URDF_INIT = {
    "DR2PRO": np.array([0, ] * 29, dtype=np.float32)
}

virtual_joint_num = {
    "DR2PRO": 2,
}


class MuJoCoSimulationNode(Node):
    def __init__(self,
                 model_key: str = MODEL_NAME,
                 xml_path: str = XML_PATH,
                 ground: bool = False):
        super().__init__('mujoco_simulation_pro')

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

        self.ground_start = ground

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
        self.depth_pub = self.create_publisher(Image, '/DEPTH_FRAME', 5)
        self.depth_publish_count = 0
        self._prepare_depth_camera()
        if SHOW_DEPTH_WINDOW:
            cv2.namedWindow(DEPTH_WINDOW_NAME, cv2.WINDOW_NORMAL)

        self.get_logger().info(
            "DDS topics ready: /JOINTS_CMD, /JOINTS_DATA, /IMU_DATA_BODY, "
            "/IMU_DATA_LEG, /DEPTH_FRAME"
        )

        # Visualization
        self.viewer = None
        if USE_VIEWER:
            try:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                self.viewer.cam.distance = 8.0
                self.viewer.cam.lookat = [4.0, 0.0, 0.6]
                self.viewer.cam.azimuth = 45.0
                self.viewer.cam.elevation = -30.0
            except Exception as e:
                self.get_logger().warning(f"Failed to launch viewer: {e}")

        self._configure_initial_pose()
        self._set_mimic_init_state()
        mujoco.mj_forward(self.model, self.data)

    def _configure_initial_pose(self):
        """Select the pinned idle pose before the state machine sends its first command."""
        if self.ground_start:
            # Match dr02_pro_2_config_ground.py / sim2sim(2).py:
            # Isaac Gym pos=[0, 0, .5], quat(xyzw)=[0, -1, 0, 1].
            # MuJoCo free-joint quaternions use wxyz, hence [sqrt(.5), 0, -sqrt(.5), 0].
            self.arr_pos = np.array([0.0, 0.0, 0.50], dtype=np.float64)
            self.arr_quat = np.array([np.sqrt(0.5), 0.0, -np.sqrt(0.5), 0.0], dtype=np.float64)
            self.arr_init_dof = np.array([
                0.0, 0.0, 0.0,                         # waist: yaw, roll, pitch
                0.0, 0.25, 0.0, 1.2, 0.0, 0.0, 0.0,    # left arm
                0.0, -0.25, 0.0, 1.2, 0.0, 0.0, 0.0,   # right arm
                -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,        # left leg
                -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,         # right leg
            ], dtype=np.float64)
            self.get_logger().info("initial pose: ground (sim2sim-compatible fallen pose)")
        else:
            self.arr_pos = np.array([0.0, 0.0, 0.9], dtype=np.float64)
            self.arr_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
            self.arr_init_dof = np.zeros(self.dof_num, dtype=np.float64)
            self.get_logger().info("initial pose: standing zero pose")

        if self.arr_init_dof.size != self.dof_num:
            raise RuntimeError(
                f"Initial pose has {self.arr_init_dof.size} joints, model has {self.dof_num} actuators"
            )

    def _cmd_callback(self, msg: JointsCmd):
        if len(msg.data) < self.dof_num:
            self.get_logger().warning(
                f"Received JointsCmd size {len(msg.data)} < required {self.dof_num}, ignore"
            )
            return

        # Support command length of either dof or dof+virtual_joint
        for i in range(self.dof_num):
            cmd = msg.data[i]
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

        while rclpy.ok():
            if time.time() - last_time >= DT:
                last_time = time.time()
                step += 1

                if not self.has_cmd:
                    self._set_mimic_init_state()
                    self.input_tq.fill(0.0)
                    self.data.ctrl[:] = 0.0
                else:
                    self._apply_joint_torque()

                mujoco.mj_step(self.model, self.data)
                self.timestamp = time.time() - init_time
                if step % STATE_PUBLISH_DECIMATION == 0:
                    self._publish_robot_state()
                if step % DEPTH_PUBLISH_DECIMATION == 0:
                    self._publish_depth_frame()

                if self.viewer and step % RENDER_INTERVAL == 0:
                    try:
                        self.viewer.sync()
                    except Exception as e:
                        self.get_logger().warning(f"Viewer sync failed: {e}")

            rclpy.spin_once(self, timeout_sec=0.0)

    @staticmethod
    def _quat_wxyz_to_matrix(quat):
        w, x, y, z = quat
        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ], dtype=np.float64)

    def _prepare_depth_camera(self):
        self.depth_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, DEPTH_CAMERA_BODY
        )
        if self.depth_body_id < 0:
            raise RuntimeError(f"Depth camera body not found: {DEPTH_CAMERA_BODY}")
        self.depth_offset_rotation = self._quat_wxyz_to_matrix(DEPTH_CAMERA_QUAT_WXYZ)

        u = (np.arange(DEPTH_WIDTH, dtype=np.float64) + 0.5) / DEPTH_WIDTH * 2.0 - 1.0
        v = (np.arange(DEPTH_HEIGHT, dtype=np.float64) + 0.5) / DEPTH_HEIGHT * 2.0 - 1.0
        uu, vv = np.meshgrid(u, v)
        directions = np.stack([
            np.ones_like(uu),
            -uu * np.tan(np.deg2rad(DEPTH_FOV_X_DEG) * 0.5),
            -vv * np.tan(np.deg2rad(DEPTH_FOV_Y_DEG) * 0.5),
        ], axis=-1)
        self.depth_directions_camera = directions / np.linalg.norm(
            directions, axis=-1, keepdims=True
        )

    def _capture_depth_frame(self):
        body_rotation = self.data.xmat[self.depth_body_id].reshape(3, 3)
        # Match the validated sim2sim: camera pose follows the full torso
        # transform, including roll and pitch.
        camera_rotation = body_rotation @ self.depth_offset_rotation
        eye = self.data.xpos[self.depth_body_id] + body_rotation @ DEPTH_CAMERA_OFFSET
        directions_world = self.depth_directions_camera @ camera_rotation.T
        depth = np.full((DEPTH_HEIGHT, DEPTH_WIDTH), DEPTH_MAX_METERS, dtype=np.float32)
        geom_id = np.zeros(1, dtype=np.int32)

        for row in range(DEPTH_HEIGHT):
            for col in range(DEPTH_WIDTH):
                distance = mujoco.mj_ray(
                    self.model, self.data, eye, directions_world[row, col], None, 1, -1, geom_id
                )
                if distance is not None and distance >= 0.0:
                    image_plane_distance = (
                        float(distance) * float(self.depth_directions_camera[row, col, 0])
                    )
                    depth[row, col] = min(image_plane_distance, DEPTH_MAX_METERS)
        return depth

    def _publish_depth_frame(self):
        depth = self._capture_depth_frame()
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = DEPTH_CAMERA_BODY
        msg.height = DEPTH_HEIGHT
        msg.width = DEPTH_WIDTH
        msg.encoding = "32FC1"
        msg.is_bigendian = False
        msg.step = DEPTH_WIDTH * np.dtype(np.float32).itemsize
        msg.data = np.ascontiguousarray(depth, dtype="<f4").tobytes()
        self.depth_pub.publish(msg)
        if (
            SHOW_DEPTH_WINDOW
            and self.depth_publish_count % max(1, DEPTH_VISUALIZATION_STRIDE) == 0
        ):
            self._show_depth_window(depth)
        self.depth_publish_count += 1

    @staticmethod
    def _preprocess_depth_for_policy(raw_depth):
        """Match DepthWalkPolicyRunner: crop -> 3x3 Gaussian -> clip -> normalize."""
        up, down, left, right = DEPTH_CROP
        depth = np.nan_to_num(
            raw_depth, nan=DEPTH_MAX_METERS,
            posinf=DEPTH_MAX_METERS, neginf=0.0
        )
        processed = depth[
            up:DEPTH_HEIGHT - down,
            left:DEPTH_WIDTH - right,
        ].copy()
        processed = cv2.GaussianBlur(processed.astype(np.float32), (3, 3), 1.0)
        processed = np.clip(processed, 0.0, DEPTH_MAX_METERS)
        return (processed / DEPTH_MAX_METERS).astype(np.float32)

    @staticmethod
    def _show_depth_window(raw_depth):
        processed = MuJoCoSimulationNode._preprocess_depth_for_policy(raw_depth)
        scale = max(1, int(DEPTH_WINDOW_SCALE))
        target_size = (DEPTH_WIDTH * scale, DEPTH_HEIGHT * scale)

        raw_normalized = np.clip(raw_depth, 0.0, DEPTH_MAX_METERS) / DEPTH_MAX_METERS
        raw_bgr = cv2.cvtColor(
            (raw_normalized * 255.0).astype(np.uint8), cv2.COLOR_GRAY2BGR
        )
        raw_bgr = cv2.resize(raw_bgr, target_size, interpolation=cv2.INTER_NEAREST)

        up, down, left, right = DEPTH_CROP
        cv2.rectangle(
            raw_bgr,
            (left * scale, up * scale),
            ((DEPTH_WIDTH - right) * scale - 1,
             (DEPTH_HEIGHT - down) * scale - 1),
            (0, 0, 255),
            max(1, scale // 2),
        )

        processed_bgr = cv2.cvtColor(
            (np.clip(processed, 0.0, 1.0) * 255.0).astype(np.uint8),
            cv2.COLOR_GRAY2BGR,
        )
        processed_bgr = cv2.resize(
            processed_bgr, target_size, interpolation=cv2.INTER_NEAREST
        )
        cv2.putText(
            raw_bgr, "RAW 64x36 (meters)", (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA
        )
        cv2.putText(
            processed_bgr, "ENCODER 36x32 ([0,1])", (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA
        )
        cv2.imshow(DEPTH_WINDOW_NAME, np.hstack((raw_bgr, processed_bgr)))
        cv2.waitKey(1)

    def _apply_joint_torque(self):
        q = self.data.qpos[7:7 + self.dof_num].reshape(-1, 1)
        dq = self.data.qvel[6:6 + self.dof_num].reshape(-1, 1)
        self.input_tq = (
            self.kp_cmd * (self.pos_cmd - q) +
            self.kd_cmd * (self.vel_cmd - dq)
        )
        self.data.ctrl[:] = self.input_tq.flatten()

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
    parser = argparse.ArgumentParser(description="DR2 Pro whole-body MuJoCo simulation")
    parser.add_argument(
        "--ground", action="store_true",
        help="start in the sim2sim-compatible fallen ground pose instead of standing idle",
    )
    args = parser.parse_args()
    np.set_printoptions(precision=4, suppress=True)
    rclpy.init()
    sim_node = None
    try:
        sim_node = MuJoCoSimulationNode(ground=args.ground)
        sim_node.start()
    except KeyboardInterrupt:
        pass
    finally:
        if sim_node is not None:
            sim_node.destroy_node()
        if SHOW_DEPTH_WINDOW:
            cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

        os._exit(0)
