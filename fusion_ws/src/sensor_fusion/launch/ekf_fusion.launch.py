from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'radar_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'pressure_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        # EKF Fusion Node
        Node(
            package='sensor_fusion',
            executable='sensor_fusion_node',
            output='screen',
            parameters=[
                {'use_radar': True},
                {'use_camera': True},
                {'use_pressure': True},
                {'base_frame': 'base_link'},
                {'world_frame': 'odom'}
            ]
        ),
    ])

