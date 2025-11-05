import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Paths
    my_package_dir = get_package_share_directory('ti_mmwave_rospkg')
    config_path = os.path.join(my_package_dir, 'cfg', '1843_Standard.cfg')
    yaml_path = os.path.join(my_package_dir, 'config', 'global_params.yaml')
    rviz_config_path = os.path.join(my_package_dir, 'launch', 'rviz.rviz')

    # Device settings
    device = "1843"
    name = "/mmWaveCLI"
    command_port = "/dev/ttyXRUSB0"
    command_rate = "115200"
    data_port = "/dev/ttyXRUSB1"
    data_rate = "921600"

    # Nodes
    global_param_node = Node(
        package='ti_mmwave_rospkg',
        executable='ConfigParameterServer',
        name='ConfigParameterServer',
        parameters=[yaml_path]
    )

    mmWaveCommSrv = Node(
        package="ti_mmwave_rospkg",
        executable="mmWaveCommSrv",
        name="mmWaveCommSrv",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"command_port": command_port},
            {"command_rate": command_rate},
            {"data_port": data_port},
            {"data_rate": data_rate},
            {"max_allowed_elevation_angle_deg": "90"},
            {"max_allowed_azimuth_angle_deg": "90"},
            {"frame_id": "/ti_mmwave_0"},
            {"mmwavecli_name": name},
            {"mmwavecli_cfg": config_path}
        ]
    )

    mmWaveQuickConfig = Node(
        package="ti_mmwave_rospkg",
        executable="mmWaveQuickConfig",
        name="mmWaveQuickConfig",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"mmwavecli_name": name},
            {"mmwavecli_cfg": config_path}
        ]
    )

    ParameterParser = Node(
        package="ti_mmwave_rospkg",
        executable="ParameterParser",
        name="ParameterParser",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"device_name": device},
            {"mmwavecli_name": name},
            {"mmwavecli_cfg": config_path}
        ]
    )

    DataHandlerClass = Node(
        package="ti_mmwave_rospkg",
        executable="DataHandlerClass",
        name="DataHandlerClass",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"mmwavecli_name": name},
            {"mmwavecli_cfg": config_path},
            {"data_port": data_port},
            {"data_rate": data_rate}
        ]
    )

    Rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]
    )

    # Launch description
    ld = LaunchDescription()
    ld.add_action(global_param_node)
    ld.add_action(mmWaveCommSrv)
    ld.add_action(mmWaveQuickConfig)
    ld.add_action(ParameterParser)
    ld.add_action(DataHandlerClass)
    ld.add_action(Rviz2)

    return ld

