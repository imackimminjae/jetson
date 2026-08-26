#!/usr/bin/env python3
"""Publish PX4 SIH MAVLink estimator state as ROS 2 topics.

Input:
  MAVProxy UDP output, default udpin:127.0.0.1:14541

Outputs:
  /px4/sih/odom       nav_msgs/msg/Odometry, ENU/FLU convention
  /px4/sih/state_ned  std_msgs/msg/Float64MultiArray, raw MAVLink NED/FRD values
"""

import math

from pymavlink import mavutil

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


def matmul(a, b):
    return [
        [sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def ned_frd_to_enu_flu_quaternion(roll, pitch, yaw):
    """Convert MAVLink aerospace RPY (NED/FRD) to ROS quaternion (ENU/FLU)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # Body FRD -> world NED: Rz(yaw) * Ry(pitch) * Rx(roll).
    r_ned_frd = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]

    ned_to_enu = [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    flu_to_frd = [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    rotation = matmul(matmul(ned_to_enu, r_ned_frd), flu_to_frd)
    return rotation_matrix_to_quaternion(rotation)


def rotation_matrix_to_quaternion(r):
    trace = r[0][0] + r[1][1] + r[2][2]

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (r[2][1] - r[1][2]) / scale
        qy = (r[0][2] - r[2][0]) / scale
        qz = (r[1][0] - r[0][1]) / scale
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        scale = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        qw = (r[2][1] - r[1][2]) / scale
        qx = 0.25 * scale
        qy = (r[0][1] + r[1][0]) / scale
        qz = (r[0][2] + r[2][0]) / scale
    elif r[1][1] > r[2][2]:
        scale = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        qw = (r[0][2] - r[2][0]) / scale
        qx = (r[0][1] + r[1][0]) / scale
        qy = 0.25 * scale
        qz = (r[1][2] + r[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        qw = (r[1][0] - r[0][1]) / scale
        qx = (r[0][2] + r[2][0]) / scale
        qy = (r[1][2] + r[2][1]) / scale
        qz = 0.25 * scale

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / norm, qy / norm, qz / norm, qw / norm


class Px4SihRosBridge(Node):
    def __init__(self):
        super().__init__("px4_sih_ros_bridge")
        self.declare_parameter("mavlink_endpoint", "udpin:127.0.0.1:14541")
        self.declare_parameter("odom_topic", "/px4/sih/odom")
        self.declare_parameter("raw_topic", "/px4/sih/state_ned")
        self.declare_parameter("frame_id", "px4_enu")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("poll_rate_hz", 100.0)

        endpoint = self.get_parameter("mavlink_endpoint").value
        odom_topic = self.get_parameter("odom_topic").value
        raw_topic = self.get_parameter("raw_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        poll_rate_hz = max(10.0, float(self.get_parameter("poll_rate_hz").value))

        self.link = mavutil.mavlink_connection(endpoint, source_system=253)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.raw_pub = self.create_publisher(Float64MultiArray, raw_topic, 10)
        self.latest_position = None
        self.latest_attitude = None
        self.last_published_position_ms = None
        self.px4_announced = False
        self.timer = self.create_timer(1.0 / poll_rate_hz, self.poll_and_publish)

        self.get_logger().info(f"MAVLink input: {endpoint}")
        self.get_logger().info(f"ROS 2 outputs: {odom_topic}, {raw_topic}")

    def poll_and_publish(self):
        for _ in range(100):
            message = self.link.recv_match(blocking=False)

            if message is None:
                break

            message_type = message.get_type()

            if message_type == "BAD_DATA":
                continue

            if message_type == "HEARTBEAT":
                if message.get_srcSystem() == 1 and not self.px4_announced:
                    self.get_logger().info(
                        f"PX4 heartbeat detected: {message.get_srcSystem()}:{message.get_srcComponent()}"
                    )
                    self.px4_announced = True
            elif message_type == "LOCAL_POSITION_NED":
                self.latest_position = message
            elif message_type == "ATTITUDE":
                self.latest_attitude = message

        if self.latest_position is None or self.latest_attitude is None:
            return

        position_ms = int(self.latest_position.time_boot_ms)

        if position_ms == self.last_published_position_ms:
            return

        self.last_published_position_ms = position_ms
        self.publish_state(self.latest_position, self.latest_attitude)

    def publish_state(self, position, attitude):
        stamp = self.get_clock().now().to_msg()

        # MAVLink NED -> ROS ENU.
        east = float(position.y)
        north = float(position.x)
        up = -float(position.z)
        velocity_east = float(position.vy)
        velocity_north = float(position.vx)
        velocity_up = -float(position.vz)

        qx, qy, qz, qw = ned_frd_to_enu_flu_quaternion(
            float(attitude.roll), float(attitude.pitch), float(attitude.yaw)
        )

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = east
        odom.pose.pose.position.y = north
        odom.pose.pose.position.z = up
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = velocity_east
        odom.twist.twist.linear.y = velocity_north
        odom.twist.twist.linear.z = velocity_up
        odom.twist.twist.angular.x = float(attitude.rollspeed)
        odom.twist.twist.angular.y = -float(attitude.pitchspeed)
        odom.twist.twist.angular.z = -float(attitude.yawspeed)
        self.odom_pub.publish(odom)

        raw = Float64MultiArray()
        raw.data = [
            float(position.time_boot_ms),
            float(position.x), float(position.y), float(position.z),
            float(position.vx), float(position.vy), float(position.vz),
            float(attitude.roll), float(attitude.pitch), float(attitude.yaw),
            float(attitude.rollspeed), float(attitude.pitchspeed), float(attitude.yawspeed),
        ]
        self.raw_pub.publish(raw)

    def destroy_node(self):
        self.link.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Px4SihRosBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
