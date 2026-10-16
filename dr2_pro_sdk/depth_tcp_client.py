#!/usr/bin/env python3
"""Receive 64x36 float depth over TCP and republish it as local ROS 2."""

import os
import socket
import struct
import time

import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


SERVER_HOST = os.environ.get("DEPTH_SERVER_HOST", "10.21.33.106")
SERVER_PORT = int(os.environ.get("DEPTH_SERVER_PORT", "16000"))
WIDTH = 64
HEIGHT = 36
PAYLOAD_BYTES = WIDTH * HEIGHT * 4
HEADER = struct.Struct("!4sI")
MAGIC = b"DPT1"

OUTPUT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def receive_exact(connection: socket.socket, size: int) -> bytes:
    data = bytearray(size)
    view = memoryview(data)
    received = 0
    while received < size:
        count = connection.recv_into(view[received:])
        if count == 0:
            raise ConnectionError("server closed the connection")
        received += count
    return bytes(data)


def publish_connection(node, publisher, connection: socket.socket) -> None:
    received = 0
    started = time.monotonic()
    last_log = started
    while rclpy.ok():
        magic, payload_size = HEADER.unpack(receive_exact(connection, HEADER.size))
        if magic != MAGIC or payload_size != PAYLOAD_BYTES:
            raise ValueError(
                "invalid frame header magic={!r} bytes={}".format(
                    magic, payload_size
                )
            )
        payload = receive_exact(connection, payload_size)

        message = Image()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = "camera_torso_front_depth_optical_frame"
        message.height = HEIGHT
        message.width = WIDTH
        message.encoding = "32FC1"
        message.is_bigendian = False
        message.step = WIDTH * 4
        message.data = payload
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.0)
        received += 1

        now = time.monotonic()
        if now - last_log >= 5.0:
            rate = received / max(now - started, 1.0e-6)
            node.get_logger().info(
                "received={} published={} rate={:.2f} Hz".format(
                    received, received, rate
                )
            )
            last_log = now


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("depth_tcp_client")
    publisher = node.create_publisher(Image, "/DEPTH_FRAME", OUTPUT_QOS)
    node.get_logger().info(
        "TCP source={}:{} output=/DEPTH_FRAME format={}x{}/32FC1/metres".format(
            SERVER_HOST, SERVER_PORT, WIDTH, HEIGHT
        )
    )

    try:
        while rclpy.ok():
            try:
                with socket.create_connection(
                    (SERVER_HOST, SERVER_PORT), timeout=5.0
                ) as connection:
                    connection.settimeout(10.0)
                    node.get_logger().info("connected to depth server")
                    publish_connection(node, publisher, connection)
            except (ConnectionError, OSError, ValueError) as error:
                if rclpy.ok():
                    node.get_logger().warning(
                        "depth server unavailable: {}; retrying in 2s".format(error)
                    )
                    time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
