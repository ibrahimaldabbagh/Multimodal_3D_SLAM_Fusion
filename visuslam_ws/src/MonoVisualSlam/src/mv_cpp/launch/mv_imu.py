from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mv_cpp',
            executable='mv_publisher',
            name='cam_1',
            namespace='',
            output='screen',
            arguments=['/dev/video2']
        ),
        Node(
            package='mv_cpp',
            executable='mv_publisher',
            name='cam_2',
            namespace='',
            output='screen',
            arguments=['/dev/video6']

 	),

    ])
