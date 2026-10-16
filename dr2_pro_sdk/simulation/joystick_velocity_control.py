#!/usr/bin/python3
"""Simulation-only joystick velocity publisher.

Reads the Linux joystick API (/dev/input/js*) without pygame and publishes
normalized forward/lateral/yaw commands on /SIM_STEER. It intentionally does
not publish mode buttons, /STEER, or /REAL_STEER.
"""

import argparse
import errno
import os
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node

from drdds.msg import MetaType, Steer, SteerValue


JS_EVENT_FORMAT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def apply_deadzone(value: float, deadzone: float) -> float:
    """Remove stick center noise and rescale the remaining range to [0, 1]."""
    magnitude = abs(float(value))
    if magnitude <= deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / max(1.0 - deadzone, 1.0e-6)
    return float(np.copysign(min(scaled, 1.0), value))


class SimulationJoystickVelocity(Node):
    def __init__(
        self,
        device: str,
        forward_axis: int,
        lateral_axis: int,
        yaw_axis: int,
        deadzone: float,
        publish_rate: float,
    ):
        super().__init__("simulation_joystick_velocity")
        self.device = device
        self.forward_axis = forward_axis
        self.lateral_axis = lateral_axis
        self.yaw_axis = yaw_axis
        self.deadzone = float(np.clip(deadzone, 0.0, 0.95))
        self.axes = np.zeros(max(forward_axis, lateral_axis, yaw_axis) + 1, dtype=np.float32)
        self.fd = None
        self.last_open_attempt = 0.0
        self.was_connected = False
        self.last_command_log = 0.0
        self.last_logged_command = np.zeros(3, dtype=np.float32)

        self.publisher = self.create_publisher(Steer, "/SIM_STEER", 10)
        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self._timer_callback)
        self.get_logger().info(
            f"Simulation joystick velocity: device={device}, "
            f"axes forward/lateral/yaw={forward_axis}/{lateral_axis}/{yaw_axis}, "
            f"deadzone={self.deadzone:.2f}, topic=/SIM_STEER"
        )

    def destroy_node(self):
        self._close_device(publish_stop=False)
        super().destroy_node()

    def _open_device(self):
        now = time.monotonic()
        if self.fd is not None or now - self.last_open_attempt < 1.0:
            return
        self.last_open_attempt = now
        try:
            self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
            self.axes.fill(0.0)
            self.was_connected = True
            self.get_logger().info(f"Joystick connected: {self.device}")
        except OSError as exc:
            if exc.errno not in (errno.ENOENT, errno.EACCES):
                self.get_logger().warning(f"Cannot open {self.device}: {exc}")

    def _close_device(self, publish_stop=True):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None
        self.axes.fill(0.0)
        if publish_stop and self.was_connected:
            self._publish_command(0.0, 0.0, 0.0)
            self.get_logger().warning("Joystick disconnected; published zero velocity")
        self.was_connected = False

    def _read_pending_events(self):
        while self.fd is not None:
            try:
                payload = os.read(self.fd, JS_EVENT_SIZE)
                if len(payload) != JS_EVENT_SIZE:
                    self._close_device()
                    return
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    return
                self._close_device()
                return

            _, raw_value, event_type, number = struct.unpack(JS_EVENT_FORMAT, payload)
            event_type &= ~JS_EVENT_INIT
            if event_type == JS_EVENT_AXIS and number < self.axes.size:
                self.axes[number] = float(np.clip(raw_value / 32767.0, -1.0, 1.0))
            # Button events are deliberately ignored: this node controls velocity only.

    def _publish_command(self, forward: float, lateral: float, yaw: float):
        message = Steer()
        message.header = MetaType()
        message.header.frame_id = 0
        message.header.stamp = self.get_clock().now().to_msg()
        message.data = SteerValue()
        message.data.x = float(np.clip(forward, -1.0, 1.0))
        message.data.y = float(np.clip(lateral, -1.0, 1.0))
        message.data.z = 0.0
        message.data.roll = 0.0
        message.data.pitch = 0.0
        message.data.yaw = float(np.clip(yaw, -1.0, 1.0))
        self.publisher.publish(message)

    def _timer_callback(self):
        self._open_device()
        if self.fd is None:
            return
        self._read_pending_events()
        if self.fd is None:
            return

        # Same signs used by sim2sim_b2_parkour: stick up/left produce +forward/+lateral.
        forward = apply_deadzone(-self.axes[self.forward_axis], self.deadzone)
        lateral = apply_deadzone(-self.axes[self.lateral_axis], self.deadzone)
        yaw = apply_deadzone(-self.axes[self.yaw_axis], self.deadzone)
        self._publish_command(forward, lateral, yaw)

        command = np.array([forward, lateral, yaw], dtype=np.float32)
        now = time.monotonic()
        if (
            now - self.last_command_log >= 0.2
            and np.max(np.abs(command - self.last_logged_command)) >= 0.02
        ):
            self.get_logger().info(
                f"velocity forward={forward:+.3f}, lateral={lateral:+.3f}, yaw={yaw:+.3f}"
            )
            self.last_logged_command = command
            self.last_command_log = now


def parse_args():
    parser = argparse.ArgumentParser(description="Simulation-only joystick velocity control")
    parser.add_argument("--device", default="/dev/input/js0", help="Linux joystick device")
    parser.add_argument("--forward-axis", type=int, default=1, help="Left-stick vertical axis")
    parser.add_argument("--lateral-axis", type=int, default=0, help="Left-stick horizontal axis")
    parser.add_argument("--yaw-axis", type=int, default=2, help="Right-stick horizontal axis")
    parser.add_argument("--deadzone", type=float, default=0.08, help="Stick deadzone [0, 0.95]")
    parser.add_argument("--rate", type=float, default=50.0, help="Publish rate in Hz")
    return parser.parse_args()


def main():
    args = parse_args()
    for value, name in (
        (args.forward_axis, "forward-axis"),
        (args.lateral_axis, "lateral-axis"),
        (args.yaw_axis, "yaw-axis"),
    ):
        if value < 0:
            raise ValueError(f"--{name} must be >= 0")

    rclpy.init()
    node = SimulationJoystickVelocity(
        args.device,
        args.forward_axis,
        args.lateral_axis,
        args.yaw_axis,
        args.deadzone,
        args.rate,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
