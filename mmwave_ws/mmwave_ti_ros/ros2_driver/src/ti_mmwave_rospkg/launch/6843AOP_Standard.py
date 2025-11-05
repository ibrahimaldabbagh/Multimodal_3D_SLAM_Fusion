import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
	
	
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Enable RViz')
    
    # include IWR6843.py
    
    
    package_dir = get_package_share_directory('ti_mmwave_rospkg')
    iwr6843_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir,'launch','IWR6843.py')),
        launch_arguments={
            "namespace":  "radar_0",
            "device" : "6843",
            "name" : "/mmWaveCLI",
            "cfg_file": '6843AOP_Standard.cfg',
             "command_port": "/dev/ttyXRUSB0",
            "command_rate": "115200",
            "data_port": "/dev/ttyXRUSB1",
            "data_rate" : "921600",
             "frame_id" : "ti_mmwave_0",
             
            "rviz": LaunchConfiguration('rviz'),
        }.items()
    )
    
    static_transform_publisher = Node(
    package = "tf2_ros",
    executable= "static_transform_publisher", 
    arguments = " 4 0 0 0 0 0 ti_mmwave_plc ti_mmwave_0".split()
    )
    
 

    ld = LaunchDescription()
    ld.add_action(rviz_arg)
    ld.add_action(iwr6843_include)
    ld.add_action(static_transform_publisher)
    return ld
