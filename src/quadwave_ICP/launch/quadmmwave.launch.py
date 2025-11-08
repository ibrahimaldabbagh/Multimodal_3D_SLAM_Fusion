from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    params_file = LaunchConfiguration('params', default='')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='',
            description='Path to quadmmwave_params.yaml'),
        Node(
            package='quadmmwave_slam',
            executable='quadmmwave_icp_core',
            name='quadmmwave_icp_core',
            output='screen',
            parameters=[params_file] if params_file.perform(None) != '' else []
        ),
        Node(
            package='quadmmwave_slam',
            executable='quadmmwave_mapping',
            name='quadmmwave_mapping',
            output='screen',
            parameters=[params_file] if params_file.perform(None) != '' else []
        )
    ])