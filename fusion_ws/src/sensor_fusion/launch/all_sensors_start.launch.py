#!/usr/bin/env python3

import os
import time

from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # ----------------------------------------------------------------
    #  Helper function to add a "TimerAction" that starts a process
    #  after some delay, to ensure sequential startup.
    # ----------------------------------------------------------------
    def delayed_action(delay_seconds, cmd_list, cwd=None, desc=""):
        return TimerAction(
            period=delay_seconds,
            actions=[
                ExecuteProcess(
                    cmd=cmd_list,
                    shell=True,
                    output='screen',
                    cwd=cwd,
                )
            ]
        )

    # 1) Launch mmWave Radars (each delayed by 4s):
    mmwave_0 = delayed_action(
        4.0,
        [
            'ros2', 'launch',
            os.path.expanduser('~/SensorFusion/mmwave_ws/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg/launch/6843AOP_Multi_0.py')
        ]
    )
    mmwave_1 = delayed_action(
        4.0,
        [
            'ros2', 'launch',
            os.path.expanduser('~/SensorFusion/mmwave_ws/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg/launch/6843AOP_Multi_1.py')
        ]
    )
    mmwave_2 = delayed_action(
        4.0,
        [
            'ros2', 'launch',
            os.path.expanduser('~/SensorFusion/mmwave_ws/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg/launch/6843AOP_Multi_2.py')
        ]
    )
    mmwave_3 = delayed_action(
        4.0,
        [
            'ros2', 'launch',
            os.path.expanduser('~/SensorFusion/mmwave_ws/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg/launch/6843AOP_Multi_3.py')
        ]
    )

    # 2) Pressure Sensor: Start-Stop Twice, then Final Launch
    pressure_node_1 = delayed_action(
        1.0,
        [
            'bash', '-c',
            'ros2 run pressure_sensor_node pressure_sensor_node & PID=$!; sleep 2; kill -INT $PID'
        ],
        cwd=os.path.expanduser('~/SensorFusion/fusion_ws/src/pressure_sensor_node/pressure_sensor_node')
    )
    pressure_node_2 = delayed_action(
        1.0,
        [
            'bash', '-c',
            'ros2 run pressure_sensor_node pressure_sensor_node & PID=$!; sleep 2; kill -INT $PID'
        ],
        cwd=os.path.expanduser('~/SensorFusion/fusion_ws/src/pressure_sensor_node/pressure_sensor_node')
    )
    pressure_node_real = delayed_action(
        1.0,
        [
            'ros2', 'run', 'pressure_sensor_node', 'pressure_sensor_node'
        ],
        cwd=os.path.expanduser('~/SensorFusion/fusion_ws/src/pressure_sensor_node/pressure_sensor_node')
    )

    # 3) Camera Publisher (mv_cpp mv_imu.py)
    camera_imu_pub = delayed_action(
        2.0,
        [
            'ros2', 'launch', 'mv_cpp', 'mv_imu.py'
        ],
        cwd=os.path.expanduser('~/SensorFusion/visuslam_ws/src/MonoVisualSlam/src/mv_cpp/launch')
    )

    # 4) XSens Driver Launch (custom command with correct port and baudrate)
    xsens_driver = delayed_action(
        2.0,
        [
            'ros2', 'run', 'xsens_driver', 'xsens_driver_node', '--ros-args', '-p', 'device:=/dev/ttyUSB0', '-p', 'baudrate:=921600'
        ],
        cwd=os.path.expanduser('~/SensorFusion/fusion_ws')
    )

    # 5) Step Counter Node
    step_counter = delayed_action(
        1.0,
        [
            'ros2', 'run', 'step_counter', 'step_counter_node'
        ],
        cwd=os.path.expanduser('~/SensorFusion/fusion_ws/src/step_counter')
    )

    # Return the full sequence launch description
    return LaunchDescription([
        mmwave_0,
        mmwave_1,
        mmwave_2,
        mmwave_3,

        pressure_node_1,
        pressure_node_2,
        pressure_node_real,

        camera_imu_pub,
        xsens_driver,
        step_counter,
    ])
