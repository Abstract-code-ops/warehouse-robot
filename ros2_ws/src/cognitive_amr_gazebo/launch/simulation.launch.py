#!/usr/bin/env python3
"""
cognitive_amr simulation launch file
Starts: Gazebo, robot_state_publisher, ros2_control, Nav2, Foxglove Bridge
"""

import os
from pathlib import Path
from unittest import result
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, RegisterEventHandler,
                             TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                   PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import re
import xacro
import subprocess


def generate_launch_description():

    # ── Package directories ────────────────────────────────────
    pkg_description = get_package_share_directory('cognitive_amr_description')
    pkg_gazebo      = get_package_share_directory('cognitive_amr_gazebo')
    pkg_nav2        = get_package_share_directory('nav2_bringup')

    # ── Launch arguments ───────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulated clock')

    world_arg = DeclareLaunchArgument(
        'world', default_value=os.path.join(pkg_gazebo, 'worlds', 'warehouse.world'),
        description='Gazebo world file')

    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove', default_value='true',
        description='Launch Foxglove bridge')

    use_sim_time  = LaunchConfiguration('use_sim_time')
    world_file    = LaunchConfiguration('world')
    use_foxglove  = LaunchConfiguration('use_foxglove')

    # ── Robot description (URDF via xacro) ────────────────────
    robot_xacro_path = os.path.join(
        pkg_description,
        'urdf',
        'robot.urdf.xacro'
    )

    result = subprocess.run(
        ['xacro', robot_xacro_path],
        capture_output=True, text=True, check=True
    )
    robot_xml = result.stdout

    # Avoid gazebo_ros2_control / rcl parser choking on XML declaration or comments
    robot_xml = re.sub(r'<\?xml.*?\?>', '', robot_xml).strip()
    robot_xml = re.sub(r'<!--.*?-->', '', robot_xml, flags=re.DOTALL)

    robot_description = {'robot_description': robot_xml}

    # ── Nodes ──────────────────────────────────────────────────

    # 1. Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 2. Gazebo (headless on EC2 — gzserver only, no GUI)
    # Delayed 2s so robot_state_publisher is up before gazebo_ros2_control
    # reads the robot_description parameter (avoids <?xml CLI parser error)
    gazebo = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so',
                 world_file],
            output='screen',
            additional_env={'GAZEBO_MODEL_PATH': os.path.join(pkg_gazebo, 'models')},
        )]
    )

    # 3. Spawn robot into Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', 'cognitive_amr',
            '-topic', '/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
            '-Y', '0.0'
        ]
    )

    # 4. Controller manager spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Start controllers AFTER Gazebo is ready
    delayed_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner])
            ]
        )
    )

    delayed_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(period=1.0, actions=[diff_drive_spawner])
            ]
        )
    )

    # 5. LiDAR fusion node (custom merger — no external dependencies)
    laser_merger = Node(
        package='cognitive_amr_gazebo',
        executable='laser_merger',
        name='laser_merger',
        output='screen',
        parameters=[os.path.join(pkg_gazebo, 'config', 'laser_merger.yaml'),
                    {'use_sim_time': use_sim_time}]
    )

    # 6. Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  os.path.join(pkg_gazebo, 'config', 'nav2_params.yaml'),
            'map':          os.path.join(pkg_gazebo, 'maps', 'warehouse_map.yaml'),
        }.items()
    )

    delayed_nav2 = RegisterEventHandler(
    event_handler=OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[
                TimerAction(period=2.0, actions=[nav2])
            ]
        )
    )

    # 7. Foxglove Bridge
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',  # Accept connections from any IP
            'tls':  False,
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(use_foxglove)
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        use_foxglove_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        delayed_joint_state,
        delayed_diff_drive,
        laser_merger,
        delayed_nav2,
        foxglove_bridge,
    ])