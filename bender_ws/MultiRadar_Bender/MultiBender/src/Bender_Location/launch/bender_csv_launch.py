import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription


def generate_launch_description():

    my_package_dir = get_package_share_directory('Bender_Location')
    ld = LaunchDescription()

    bender_csv = Node(
        package="Bender_Location",
        name="bender_csv",
        executable="bender_csv",
        output="screen"
    )

    ld.add_action(bender_csv)
    return ld
