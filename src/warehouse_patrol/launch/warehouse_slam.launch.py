#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'
    patrol_share = get_package_share_directory('warehouse_patrol')
    cartographer_share = get_package_share_directory('turtlebot3_cartographer')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(patrol_share, 'launch', 'warehouse_simulation.launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'use_sim_time': 'true',
            'x_pose': '-8.0',
            'y_pose': '-9.5',
        }.items(),
    )

    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_share, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_rviz': LaunchConfiguration('rviz'),
            'resolution': '0.05',
            'publish_period_sec': '1.0',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),
        simulation,
        TimerAction(period=4.0, actions=[cartographer]),
    ])
