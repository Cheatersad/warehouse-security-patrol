#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'

    patrol_share = get_package_share_directory('warehouse_patrol')
    nav2_share = get_package_share_directory('nav2_bringup')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    waypoints_file = LaunchConfiguration('waypoints_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(patrol_share, 'launch', 'warehouse_simulation.launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'use_sim_time': use_sim_time,
            'x_pose': '-8.0',
            'y_pose': '-9.5',
        }.items(),
    )


    odom_tf_guard = Node(
        package='warehouse_patrol',
        executable='odom_tf_guard',
        name='odom_tf_guard',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': '/odom',
            'parent_frame': 'odom',
            'child_frame': 'base_footprint',
            'detection_seconds': 3.0,
        }],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='warehouse_rviz',
        output='screen',
        arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    patrol = Node(
        package='warehouse_patrol',
        executable='patrol_manager',
        name='warehouse_patrol_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoints_file': waypoints_file,
            'autostart': True,
            'startup_delay_seconds': 10.0,
            'publish_initial_pose': False,
        }],
        condition=IfCondition(LaunchConfiguration('start_patrol')),
    )

    perception = Node(
        package='security_perception',
        executable='yolo_node',
        name='yolo_detection_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': LaunchConfiguration('model_path'),
            'image_topic': '/camera/image_raw',
            'scan_topic': '/scan',
            'confidence_threshold': 0.40,
            'save_snapshots': True,
            'snapshot_directory': '/ws_slam/security_events',
        }],
        condition=IfCondition(LaunchConfiguration('start_perception')),
    )


    sim_detector = Node(
        package='security_perception',
        executable='sim_anomaly_detector',
        name='sim_anomaly_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': '/camera/image_raw',
            'scan_topic': '/scan',
            'minimum_contour_area': 180.0,
            'process_every_n_frames': 2,
            'alert_cooldown_seconds': 12.0,
            'save_snapshots': True,
            'snapshot_directory': '/ws_slam/security_events',
        }],
        condition=IfCondition(LaunchConfiguration('start_sim_detector')),
    )


    gazebo_alert_visualizer = Node(
        package='security_perception',
        executable='gazebo_alert_visualizer',
        name='gazebo_alert_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'visibility_topic': '/security/visibility',
            'visibility_timeout_seconds': 0.8,
            'reference_frame': 'world',
        }],
        condition=IfCondition(
            LaunchConfiguration('start_gazebo_alert_visualizer')
        ),
    )

    logger = Node(
        package='security_perception',
        executable='event_logger_node',
        name='security_event_logger',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'text_log_path': '/ws_slam/security_log.txt',
            'jsonl_log_path': '/ws_slam/security_events.jsonl',
            'report_directory': '/ws_slam/patrol_reports',
        }],
        condition=IfCondition(LaunchConfiguration('start_logger')),
    )

    demo_alerts = Node(
        package='warehouse_patrol',
        executable='demo_anomaly_publisher',
        name='demo_anomaly_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('demo_alerts')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_patrol', default_value='true'),
        DeclareLaunchArgument('start_perception', default_value='true'),
        DeclareLaunchArgument('start_sim_detector', default_value='true'),
        DeclareLaunchArgument('start_gazebo_alert_visualizer', default_value='true'),
        DeclareLaunchArgument('start_logger', default_value='true'),
        DeclareLaunchArgument('demo_alerts', default_value='false'),
        DeclareLaunchArgument('model_path', default_value='/ws_slam/yolov8n.pt'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(patrol_share, 'maps', 'warehouse_map.yaml'),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(patrol_share, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=os.path.join(patrol_share, 'config', 'patrol_waypoints.yaml'),
        ),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),
        simulation,
        TimerAction(period=3.0, actions=[odom_tf_guard]),
        TimerAction(period=8.0, actions=[nav2]),
        TimerAction(period=12.0, actions=[rviz]),
        logger,
        perception,
        sim_detector,
        gazebo_alert_visualizer,
        patrol,
        demo_alerts,
    ])
