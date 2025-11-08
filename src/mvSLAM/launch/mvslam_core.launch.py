from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mvslam',
            executable='mvslam_core_node',
            name='mvslam_core',
            parameters=[{
                'left_image': '/stereo/left/image_rect',
                'right_image': '/stereo/right/image_rect',
                'imu_topic': '/imu/data',
                'fx': 718.856, 'fy': 718.856, 'cx': 607.1928, 'cy': 185.2157,
                'baseline': 0.537,         # meters
                'max_features': 800,
                'gftt_quality': 0.01,
                'gftt_min_dist': 8.0,
                'lk_win': 15,
                'lk_levels': 3,
                'lk_term_eps': 0.02,
                'lk_term_iter': 30,
                'huber_delta': 3.0,
                'min_disparity': 1.0,
                'ema_alpha_translation': 0.3,
                'frame_id': 'odom'
            }]
        )
    ])
