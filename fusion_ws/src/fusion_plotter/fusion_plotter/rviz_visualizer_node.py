#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import tf_transformations
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs  # Needed to transform PoseStamped
import numpy as np

class RVizVisualizer(Node):
    def __init__(self):
        super().__init__('rviz_visualizer_node')

        # Subscribers
        self.create_subscription(PoseWithCovarianceStamped, '/fused/pose', self.fused_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose_estimation', self.radar_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/mvs/pose', self.camera_callback, 10)
        self.create_subscription(PoseStamped, '/step_only/pose', self.step_counter_callback, 10)

        # Publishers
        self.fused_path_pub = self.create_publisher(Path, '/rviz/fused_path', 10)
        self.radar_path_pub = self.create_publisher(Path, '/rviz/radar_path', 10)
        self.camera_path_pub = self.create_publisher(Path, '/rviz/camera_path', 10)
        self.step_path_pub = self.create_publisher(Path, '/rviz/step_counter_path', 10)
        self.covariance_marker_pub = self.create_publisher(MarkerArray, '/rviz/covariance_markers', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Paths
        self.fused_path = Path()
        self.radar_path = Path()
        self.camera_path = Path()
        self.step_counter_path = Path()

        self.fused_path.header.frame_id = "odom"
        self.radar_path.header.frame_id = "odom"
        self.camera_path.header.frame_id = "odom"
        self.step_counter_path.header.frame_id = "odom"

    def fused_callback(self, msg):
        self.publish_path_smoothed(msg, self.fused_path, self.fused_path_pub)
        self.publish_covariance_marker(msg)

    def radar_callback(self, msg):
        self.publish_path_flattened_2d(msg, self.radar_path, self.radar_path_pub)

    def camera_callback(self, msg):
        self.publish_path_flattened_2d(msg, self.camera_path, self.camera_path_pub)

    def step_counter_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header

        # Flip X and Z to match 'odom' frame alignment
        pose.pose.position.x = -msg.pose.position.x
        pose.pose.position.y = msg.pose.position.y
        pose.pose.position.z = -msg.pose.position.z

        # Original orientation from sensor
        q_orig = msg.pose.orientation
        q_input = [q_orig.x, q_orig.y, q_orig.z, q_orig.w]

        # 🔧 Manual yaw offset in degrees for alignment tuning
        yaw_offset_deg = -20.0  # <<<<< Change this value to align better
        yaw_offset_rad = np.deg2rad(yaw_offset_deg)

        # Apply 180° pitch (to flip forward) and yaw correction
        q_fix = tf_transformations.quaternion_from_euler(0, np.pi, yaw_offset_rad)
        q_final = tf_transformations.quaternion_multiply(q_fix, q_input)

        pose.pose.orientation.x = q_final[0]
        pose.pose.orientation.y = q_final[1]
        pose.pose.orientation.z = q_final[2]
        pose.pose.orientation.w = q_final[3]

        self.step_counter_path.header.stamp = self.get_clock().now().to_msg()
        self.step_counter_path.poses.append(pose)
        self.step_path_pub.publish(self.step_counter_path)

    def publish_path_smoothed(self, msg, path_obj, publisher, alpha=0.3):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        if path_obj.poses:
            last_pose = path_obj.poses[-1].pose
            pose_stamped.pose.position.x = (1 - alpha) * last_pose.position.x + alpha * pose_stamped.pose.position.x
            pose_stamped.pose.position.y = (1 - alpha) * last_pose.position.y + alpha * pose_stamped.pose.position.y
            pose_stamped.pose.position.z = (1 - alpha) * last_pose.position.z + alpha * pose_stamped.pose.position.z

        path_obj.header.stamp = self.get_clock().now().to_msg()
        path_obj.poses.append(pose_stamped)
        publisher.publish(path_obj)

    def publish_path_flattened_2d(self, msg, path_obj, publisher):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        pose_stamped.pose.position.z = 0.0

        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        q_flat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose_stamped.pose.orientation.x = q_flat[0]
        pose_stamped.pose.orientation.y = q_flat[1]
        pose_stamped.pose.orientation.z = q_flat[2]
        pose_stamped.pose.orientation.w = q_flat[3]

        path_obj.header.stamp = self.get_clock().now().to_msg()
        path_obj.poses.append(pose_stamped)
        publisher.publish(path_obj)

    def publish_covariance_marker(self, msg):
        marker_array = MarkerArray()

        marker = Marker()
        marker.header = msg.header
        marker.ns = "covariance"
        marker.id = len(self.fused_path.poses)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = msg.pose.pose

        marker.scale.x = max(0.1, msg.pose.covariance[0])
        marker.scale.y = max(0.1, msg.pose.covariance[7])
        marker.scale.z = max(0.1, msg.pose.covariance[14])

        marker.color.a = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.lifetime.sec = 30
        marker_array.markers.append(marker)

        self.covariance_marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = RVizVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

