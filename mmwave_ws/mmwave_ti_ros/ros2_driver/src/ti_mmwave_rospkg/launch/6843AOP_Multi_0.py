import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Enable RViz')

    my_package_dir = get_package_share_directory('ti_mmwave_rospkg')
    path = os.path.join(my_package_dir, 'cfg',
                        'ootb_v6_manual_8_5m_4_2cmres_10kmh_0_34kmhres_70ms_5db_5db.cfg')
    ld = LaunchDescription()

    #     #   ('/ti_mmwave/radar_scan_pcl', '/ti_mmwave/radar_scan_pcl_0'),
    #     # ],
    #     parameters=[

    ConfigParameters = os.path.join(
        my_package_dir,
        'config',
        'global_params.yaml'
    )

    global_param_node = Node(
        package='ti_mmwave_rospkg',
        executable='ConfigParameterServer',
        name='ConfigParameterServer',
        parameters=[ConfigParameters]
    )

    mmWaveCommSrv = Node(
        package="ti_mmwave_rospkg",
        executable="mmWaveCommSrv",
        name="mmWaveCommSrv",
        namespace="radar_0",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"command_port": "/dev/ttyXRUSB0"},
            {"command_rate": "115200"},
            {"data_port": "/dev/ttyXRUSB1"},
            {"data_rate": "921600"},
            {"frame_id": "ti_mmwave_0"},
            {"mmwavecli_name": "/mmWaveCLI"},
            {"mmwavecli_cfg": "ootb_v6_manual_8_5m_4_2cmres_10kmh_0_34kmhres_70ms_5db_5db.cfg"},
            {"device": "6843"},

        ]
    )

    mmWaveQuickConfig = Node(
        package="ti_mmwave_rospkg",
        executable="mmWaveQuickConfig",
        name="mmWaveQuickConfig",
        namespace="radar_0",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"mmwavecli_cfg": path},
            {"frame_id": "ti_mmwave_0"}
        ]
    )

    ParameterParser = Node(
        package="ti_mmwave_rospkg",
        executable="ParameterParser",
        name="ParameterParser",
        output="screen",
        emulate_tty=True,
        parameters=[
             {"device_name": "6843"},
            {"mmwavecli_cfg": path},
            {"frame_id": "ti_mmwave_0"},
            {"namespace": "radar_0"}
        ]
    )

    DataHandlerClass = Node(
        package="ti_mmwave_rospkg",
        executable="DataHandlerClass",
        namespace="radar_0",
        name="DataHandlerClass",
        output="screen",
        remappings=[
            ('/ti_mmwave/radar_scan_pcl', '/ti_mmwave/radar_scan_pcl_0'),
        ],
        emulate_tty=True,
        parameters=[
            {"data_port": "/dev/ttyXRUSB1"},
            {"data_rate": "921600"},
            {"max_allowed_elevation_angle_deg": 90},
            {"max_allowed_azimuth_angle_deg": 90},
            {"frame_id": "ti_mmwave_0"}
        ]

    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        name="radar_baselink_0",
        arguments=["-0.04", "0", "0.08", "-1.0472",
                   "0", "0", "ti_mmwave_pcl", "ti_mmwave_0"]
    )

    Rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', os.path.join(my_package_dir, 'launch', 'rviz.rviz')]
    )

    ld.add_action(global_param_node)
    ld.add_action(mmWaveCommSrv)
    ld.add_action(mmWaveQuickConfig)
    ld.add_action(ParameterParser)
    ld.add_action(DataHandlerClass)
    ld.add_action(static_transform_publisher)
   # ld.add_action(Rviz2)
    return ld
