"""
cognitive_amr.launch.py
────────────────────────────────────────────────────────────────────────────
Launches all Cognitive AMR nodes in a single ROS 2 launch file.

Usage (after source install/setup.bash):

  # With no LLM key (mock mode):
  ros2 launch cognitive_amr cognitive_amr.launch.py

  # With Anthropic key:
  ANTHROPIC_API_KEY=<key> ros2 launch cognitive_amr cognitive_amr.launch.py

  # Custom DB path:
  ros2 launch cognitive_amr cognitive_amr.launch.py \\
      db_path:=/home/user/my_warehouse.db
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value='/ros2_ws/src/cognitive_amr/config/inventory.db',
        description='Absolute path to the SQLite inventory database'
    )

    db_path = LaunchConfiguration('db_path')

    # ── Core infrastructure ──────────────────────────────────────────────

    inventory_manager = Node(
        package='cognitive_amr',
        executable='inventory_manager',
        name='inventory_manager_node',
        output='screen',
        parameters=[{'db_path': db_path, 'use_sim_time': True}],
    )

    # ── Perception ───────────────────────────────────────────────────────

    aruco_detector = Node(
        package='cognitive_amr',
        executable='aruco_detector',
        name='aruco_detector_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'image_topic': '/camera/color/image_raw'},
            {'publish_every_n_frames': 3},
            {'jpeg_quality': 60},
        ],
    )

    tag_localization = Node(
        package='cognitive_amr',
        executable='tag_localization',
        name='tag_localization_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── Navigation / scan ────────────────────────────────────────────────

    # ── Brain ────────────────────────────────────────────────────────────

    task_planner = Node(
        package='cognitive_amr',
        executable='task_planner',
        name='task_planner_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── LLM gateway (litellm — set CEREBRAS_API_KEY, ANTHROPIC_API_KEY, or OPENAI_API_KEY) ──

    llm_gateway = Node(
        package='cognitive_amr',
        executable='llm_gateway',
        name='llm_gateway_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        additional_env={k: v for k, v in {
            'CEREBRAS_API_KEY':  os.environ.get('CEREBRAS_API_KEY',  ''),
            'ANTHROPIC_API_KEY': os.environ.get('ANTHROPIC_API_KEY', ''),
            'OPENAI_API_KEY':    os.environ.get('OPENAI_API_KEY',    ''),
            'LLM_MODEL':         os.environ.get('LLM_MODEL',         ''),
        }.items() if v},
    )

    # ── HMI / visualisation ──────────────────────────────────────────────

    operator_interface = Node(
        package='cognitive_amr',
        executable='operator_interface',
        name='operator_interface_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── Pick simulation ──────────────────────────────────────────────────

    pick_simulator = Node(
        package='cognitive_amr',
        executable='pick_simulator',
        name='pick_simulator_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        db_path_arg,
        inventory_manager,
        aruco_detector,
        tag_localization,
        task_planner,
        llm_gateway,
        operator_interface,
        pick_simulator,
    ])
