"""Joystick teleoperation chain.

    joy_node  ->  joy_teleop  ->  twist_mux  ->  cmd_relay
    (gamepad)     (buttons ->    (picks the      (/cmd_vel_unstamped ->
                   Twist)         active source)  wheel velocity commands)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

PACKAGE = 'program_bringup'


def config_path(file_name):
    """Absolute path of a file in program_bringup/config."""
    return os.path.join(get_package_share_directory(PACKAGE), 'config', file_name)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Per-wheel speed cap in m/s (0 = no limit); set from startup.yaml
    # (hardware.max_wheel_speed, always 0 in simulation).
    max_wheel_speed = ParameterValue(
        LaunchConfiguration('max_wheel_speed'), value_type=float)

    # ----------------------------- Arguments -----------------------------
    arguments = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulated time'),
        DeclareLaunchArgument(
            'joy_config', default_value=config_path('joystick_config.yaml'),
            description='joy_node parameter file'),
        DeclareLaunchArgument(
            'teleop_config', default_value=config_path('joystick_teleop.yaml'),
            description='joy_teleop parameter file'),
        DeclareLaunchArgument(
            'twist_mux_topics', default_value=config_path('twist_mux_topics.yaml'),
            description='twist_mux input topics'),
        DeclareLaunchArgument(
            'twist_mux_locks', default_value=config_path('twist_mux_locks.yaml'),
            description='twist_mux lock topics'),
        DeclareLaunchArgument(
            'max_wheel_speed', default_value='0.0',
            description='cmd_relay per-wheel speed limit in m/s (0 = no limit)'),
    ]

    # ------------------------------- Nodes -------------------------------
    joystick_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        output='screen',
        parameters=[LaunchConfiguration('teleop_config')],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        output='screen',
        parameters=[LaunchConfiguration('joy_config')],
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            LaunchConfiguration('twist_mux_topics'),
            LaunchConfiguration('twist_mux_locks'),
        ],
        remappings=[('/cmd_vel_out', '/cmd_vel_unstamped')],
    )

    relay_node = Node(
        package='helpers',
        executable='cmd_relay',
        name='cmd_relay',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             'max_wheel_speed_m_s': max_wheel_speed},
            # track width and wheel radius (single source of truth)
            config_path('robot_dimensions.yaml'),
        ],
    )

    return LaunchDescription([
        *arguments,
        joystick_node,
        joy_node,
        twist_mux_node,
        relay_node,
    ])
