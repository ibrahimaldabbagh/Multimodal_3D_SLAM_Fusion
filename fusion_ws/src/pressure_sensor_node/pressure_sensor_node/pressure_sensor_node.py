#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import serial
import time

class PressureSensorNode(Node):
    def __init__(self):
        super().__init__('pressure_sensor_node')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_rate', 20)
        self.declare_parameter('altitude_variance', 5.0)
        self.declare_parameter('altitude_smoothing_alpha', 0.5)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value
        self.z_variance = self.get_parameter('altitude_variance').get_parameter_value().double_value
        self.smoothing_alpha = self.get_parameter('altitude_smoothing_alpha').get_parameter_value().double_value

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 'pressure/pose', 10)

        try:
            self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=0.05)  # Shorter timeout
            self.get_logger().info("Serial connection established on /dev/ttyACM0.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        self.median_buffer_size = 5
        self.recent_altitudes = []

        raw_samples = []
        for _ in range(20):
            alt = self.read_altitude_from_serial()
            if alt is not None:
                raw_samples.append(alt)
        if len(raw_samples) > 0:
            self.initial_altitude = sum(raw_samples) / len(raw_samples)
        else:
            self.get_logger().warn("Could not determine initial altitude; defaulting to 0.0 m")
            self.initial_altitude = 0.0

        self.filtered_altitude = self.initial_altitude

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_altitude)
        self.get_logger().info(f"Pressure Sensor Node Started (Publishing {self.publish_rate} Hz)")

    def read_altitude_from_serial(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return None
            return float(line)
        except ValueError:
            self.get_logger().warn(f"Invalid data on serial: '{line}'")
            return None

    def median_filter(self, value):
        self.recent_altitudes.append(value)
        if len(self.recent_altitudes) > self.median_buffer_size:
            self.recent_altitudes.pop(0)
        return sorted(self.recent_altitudes)[len(self.recent_altitudes) // 2]

    def publish_altitude(self):
        # Take timestamp before serial read
        timestamp = self.get_clock().now()
        current_raw_alt = self.read_altitude_from_serial()
        if current_raw_alt is None:
            return

        # Drop outdated samples (e.g., caused by serial lag)
        age_ms = (self.get_clock().now() - timestamp).nanoseconds / 1e6
        if age_ms > 50:
            self.get_logger().warn(f"Dropped outdated pressure reading (age {age_ms:.1f} ms)")
            return

        median_alt = self.median_filter(current_raw_alt)
        alpha = self.smoothing_alpha
        self.filtered_altitude = alpha * self.filtered_altitude + (1.0 - alpha) * median_alt
        z_position = self.filtered_altitude - self.initial_altitude

        floor = 0.0001  # small positive value to avoid zero-only covariance
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = z_position
        msg.pose.covariance = [
            floor, 0.0,   0.0,   0.0, 0.0, 0.0,
            0.0,  floor,  0.0,   0.0, 0.0, 0.0,
            0.0,  0.0, self.z_variance, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, floor,  0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, floor, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, floor
        ]

        self.publisher.publish(msg)
        self.get_logger().info(f"Published Filtered Altitude Z: {z_position:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = PressureSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


