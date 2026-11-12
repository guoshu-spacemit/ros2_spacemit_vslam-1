#!/usr/bin/env python3
"""Satisfy the replay transport watermark in VO; never integrate IMU samples."""

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node("replay_imu_receipt_only")
    node.declare_parameter("imu_topic", "/replay/camera/imu/data_raw")
    node.declare_parameter("ack_topic", "/replay/vio/imu_ack")
    publisher = node.create_publisher(
        String, node.get_parameter("ack_topic").value,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def receive(message):
        stamp = message.header.stamp
        publisher.publish(String(data=str(stamp.sec * 1000000000 + stamp.nanosec)))

    node.create_subscription(
        Imu, node.get_parameter("imu_topic").value, receive,
        QoSProfile(depth=2048, reliability=ReliabilityPolicy.RELIABLE))
    node.get_logger().info("VO replay: IMU receipt acknowledgement only; no IMU fusion")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
