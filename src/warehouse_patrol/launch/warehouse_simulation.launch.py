#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'

    warehouse_share = get_package_share_directory('warehouse_patrol')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_share = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    urdf_path = os.path.join(
        turtlebot3_gazebo_share,
        'urdf',
        'turtlebot3_waffle_pi.urdf',
    )

    model_path = os.path.join(
        turtlebot3_gazebo_share,
        'models',
        'turtlebot3_waffle_pi',
        'model.sdf',
    )

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_share,
                'launch',
                'gzserver.launch.py',
            )
        ),
        launch_arguments={
            'world': world,
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_share,
                'launch',
                'gzclient.launch.py',
            )
        ),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_warehouse_turtlebot3',
        output='screen',
        arguments=[
            '-entity', 'waffle_pi',
            '-file', model_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                warehouse_share,
                'worlds',
                'warehouse_zones.world',
            ),
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='-8.0',
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='-9.5',
        ),
        SetEnvironmentVariable(
            'TURTLEBOT3_MODEL',
            'waffle_pi',
        ),
        gzserver,
        gzclient,
        robot_state_publisher,
        TimerAction(
            period=2.0,
            actions=[spawn_robot],
        ),
    ])
