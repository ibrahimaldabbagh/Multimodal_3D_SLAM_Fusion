import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# Hier finden sich die einzelnen Nodes heißt Kombinieren , Filtern ICP usw zuerst werden diese Initialisiert und dann können sie unten entweder Aktiviert werden oder deaktiviert werden


def generate_launch_description():

    my_package_dir = get_package_share_directory(
        'Bender_Location'),

    ld = LaunchDescription()

    bender_processing = Node(
        package="Bender_Location",
        name="bender_radar_processing",
        executable="bender_radar_processing",
        output="screen"
    )
    bender_filter = Node(
        package="Bender_Location",
        name="bender_radar_filter",
        executable="bender_radar_filter",
        output="screen"
    )
    bender_icp = Node(
        package="Bender_Location",
        name="bender_radar_icp",
        executable="bender_radar_icp",
        output="screen"
    )

    bender_visu = Node(
        package="Bender_Location",
        name="bender_radar_visualziation",
        executable="bender_radar_visualziation",
        output="screen"
    )
    
    radar_map = Node(
        package="Bender_Location",
        name="radar_occupancy_node",
        executable="radar_occupancy_node",
        output="screen"
    )

    map = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': '/home/aldabbag/SensorFusion/bender_ws/MultiRadar_Bender/MultiBender/src/Bender_Location/resource/map.yaml'}]
    )
    # Aktivierung oder Deaktivierung der Nodes beim Starten des Launch-Files
    ld.add_action(bender_processing)
    ld.add_action(bender_filter)
    ld.add_action(bender_icp)
    ld.add_action(bender_visu)
    ld.add_action(radar_map)

    # ld.add_action(map)

    return ld
