#!/usr/bin/env python3
"""
PX4 SITL + Gazebo (baylands) + MicroXRCE + QGroundControl
SAFE & VERIFIED
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
import os


def generate_launch_description():

    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    qgc_path = os.path.expanduser('~/Downloads/QGroundControl-x86_64.AppImage')

    return LaunchDescription([

        # 1. PX4 SITL + Gazebo X500 (Baylands world)
        ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--title=PX4 SITL',
                '--',
                'bash', '-c',
                f'cd {px4_dir} && PX4_GZ_WORLD=baylands make px4_sitl gz_x500_lidar_2d; exec bash'
            ],
            output='screen'
        ),

        # 2. Micro XRCE Agent (after PX4 starts)
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal',
                        '--title=MicroXRCE Agent',
                        '--',
                        'bash', '-c',
                        'MicroXRCEAgent udp4 -p 8888; exec bash'
                    ],
                    output='screen'
                )
            ]
        ),

        # 3. QGroundControl
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[qgc_path],
                    output='screen'
                )
            ]
        ),
    ])

