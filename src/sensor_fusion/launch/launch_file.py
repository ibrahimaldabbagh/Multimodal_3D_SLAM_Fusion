#!/usr/bin/env python3
"""
ROS2 Launch file for Multimodal Fusion with CA Model
Copyright (c) 2025
License: MIT
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for fusion system."""
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    world_frame_arg = DeclareLaunchArgument(
        'world_frame',
        default_value='odom',
        description='World/map frame ID'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame ID'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to YAML config file'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    
    # Fusion Core Node
    fusion_node = Node(
        package='fusion_stack',
        executable='fusion_core_ca_node',
        name='fusion_core_ca',
        output='screen',
        parameters=[
            {
                # Frames
                'world_frame': LaunchConfiguration('world_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'imu_frame': 'imu_link',
                
                # Topics
                'imu_topic': '/imu/data',
                'visual_pose_topic': '/mvslam/pose',
                'radar_pose_topic': '/radar/pose',
                'baro_topic': '/baro/altitude',
                'wheel_odo_topic': '/wheel/odom',
                'step_topic': '/step_count',
                'gnss_topic': '/gnss/fix',
                'lidar_topic': '/lidar/points',
                'velocity_topic': '/velocity',
                
                # ESKF-CA parameters (process noise)
                'sigma_gyro': 1.5e-3,
                'sigma_accel': 3.0e-2,
                'sigma_gyro_bias': 1.0e-5,
                'sigma_accel_bias': 5.0e-5,
                'sigma_velocity': 1.0e-3,
                'sigma_acceleration': 5.0e-3,
                'sigma_angular_velocity': 1.0e-4,
                'sigma_angular_accel': 1.0e-3,
                
                # Measurement variances
                'visual_pos_variance': 0.25,
                'visual_yaw_variance': 0.01,
                'radar_pos_variance': 0.04,
                'baro_variance': 0.25,
                'gnss_horizontal_variance': 4.0,
                'gnss_vertical_variance': 9.0,
                'wheel_odo_variance': 0.01,
                'step_variance': 0.25,
                'velocity_variance': 0.01,
                
                # Outlier rejection
                'mahalanobis_gate': 16.0,
                
                # Mapping
                'voxel_size': 0.2,
                'max_mapping_range': 50.0,
                
                # Loop closure
                'enable_loop_closure': True,
                'loop_closure_distance': 5.0,
                
                # Step counter
                'step_length': 0.7,
                
                # GNSS
                'use_gnss': False,
                
                # Use simulation time
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            # Add remappings here if needed
        ]
    )
    
    # RViz Node (optional)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('fusion_stack'),
        'rviz',
        'fusion_ca.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=LaunchConfiguration('enable_rviz'),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Static TF publishers (example - adjust for your robot)
    base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_broadcaster',
        arguments=['0', '0', '0.1', '0', '0', '0', 
                  LaunchConfiguration('base_frame'), 'imu_link'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_broadcaster',
        arguments=['0', '0', '0.3', '0', '0', '0',
                  LaunchConfiguration('base_frame'), 'lidar_link'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    base_to_radar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_radar_broadcaster',
        arguments=['0.2', '0', '0.2', '0', '0', '0',
                  LaunchConfiguration('base_frame'), 'radar_link'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        world_frame_arg,
        base_frame_arg,
        config_file_arg,
        enable_rviz_arg,
        
        # Nodes
        fusion_node,
        rviz_node,
        
        # Static TFs
        base_to_imu_tf,
        base_to_lidar_tf,
        base_to_radar_tf,
    ])
