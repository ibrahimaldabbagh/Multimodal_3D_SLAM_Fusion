import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
import csv
import numpy as np

class UWBPoseAndPathPublisher(Node):
    def __init__(self):
        super().__init__('uwb_pose_and_path_publisher')

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'uwb_pose_cov', 10)
        self.path_pub = self.create_publisher(Path, 'uwb_path', 6)

        self.csv_file = 'locations.csv'
        self.frame_id = 'odom'
        self.data = self.load_and_smooth_csv(window_size=5)
        self.index = 0
        self.initial_epoch = self.data[0]['epoch']
        self.start_time_ros = self.get_clock().now().nanoseconds / 1e6  # ms

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz

    def load_and_smooth_csv(self, window_size=50):
        raw = []
        with open(self.csv_file, newline='') as csvfile:
            reader = csv.DictReader(csvfile, delimiter=';')
            for row in reader:
                raw.append({
                    'epoch': int(row['epoch']),
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'z': float(row['z'])
                })

        smoothed = []
        for i in range(len(raw)):
            if i < window_size:
                smoothed.append(raw[i])
            else:
                avg_x = np.mean([raw[j]['x'] for j in range(i - window_size, i)])
                avg_y = np.mean([raw[j]['y'] for j in range(i - window_size, i)])
                avg_z = np.mean([raw[j]['z'] for j in range(i - window_size, i)])
                smoothed.append({
                    'epoch': raw[i]['epoch'],
                    'x': avg_x,
                    'y': avg_y,
                    'z': avg_z
                })
        return smoothed

    def timer_callback(self):
        if self.index >= len(self.data):
            return

        now_ros = self.get_clock().now().nanoseconds / 1e6  # ms
        elapsed = now_ros - self.start_time_ros
        target_epoch = self.data[self.index]['epoch'] - self.initial_epoch

        if elapsed >= target_epoch:
            row = self.data[self.index]

            # PoseWithCovarianceStamped
            pose_cov = PoseWithCovarianceStamped()
            pose_cov.header.frame_id = self.frame_id
            pose_cov.header.stamp = self.get_clock().now().to_msg()
            pose_cov.pose.pose.position.x = row['x']
            pose_cov.pose.pose.position.y = row['y']
            pose_cov.pose.pose.position.z = row['z']
            pose_cov.pose.pose.orientation.w = 1.0

            cov = np.zeros((6, 6))
            cov[0, 0] = 2.0
            cov[1, 1] = 2.0
            cov[2, 2] = 1.0
            cov[3, 3] = 0.5
            cov[4, 4] = 0.5
            cov[5, 5] = 0.5
            pose_cov.pose.covariance = cov.flatten().tolist()

            self.pose_pub.publish(pose_cov)

            # Path
            pose_stamped = PoseStamped()
            pose_stamped.header = pose_cov.header
            pose_stamped.pose = pose_cov.pose.pose
            self.path_msg.poses.append(pose_stamped)
            self.path_msg.header.stamp = pose_cov.header.stamp
            self.path_pub.publish(self.path_msg)

            self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = UWBPoseAndPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

