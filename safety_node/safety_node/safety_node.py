#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        self.speed = 0.
        self.ttc_threshold = 1.0

        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'ego_racecar/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges, dtype=np.float64)
        ranges = np.nan_to_num(
            ranges, nan=0.0, posinf=scan_msg.range_max, neginf=0.0)

        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment

        closing_speed = self.speed * np.cos(angles)
        closing_speed = np.maximum(closing_speed, 1e-3)

        ittc = ranges / closing_speed

        if np.any(ittc < self.ttc_threshold):
            self.brake()

    def brake(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive = AckermannDrive()
        drive_msg.drive.speed = 0.0
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
