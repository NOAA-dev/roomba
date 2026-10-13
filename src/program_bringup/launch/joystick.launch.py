from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription , DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    program_bringup_pkg = get_package_share_directory("program_bringup")

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",description="Use simulated time")

    # Per-wheel speed cap (m/s) for the REAL robot only; the simulation is never
    # limited. 0.20 m/s is ~70% of the ESP32's PWM_MAX ceiling (~0.30 m/s) and is
    # still UNVALIDATED: use e.g. 0.10 for the first floor tests.
    max_wheel_speed_arg = DeclareLaunchArgument(
        name="max_wheel_speed", default_value="0.20",
        description="Per-wheel speed limit in m/s on real hardware (0 = no limit)")
    max_wheel_speed = ParameterValue(
        PythonExpression(["0.0 if '", LaunchConfiguration("use_sim_time"),
                          "'.lower() == 'true' else ", LaunchConfiguration("max_wheel_speed")]),
        value_type=float)

    joystick_teleop_config = os.path.join(
        program_bringup_pkg,
        'config',
        'joystick_teleop.yaml'
    )

    joystick_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        output='screen',
        parameters=[joystick_teleop_config]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        output='screen',
        parameters = [os.path.join(
            get_package_share_directory('program_bringup'),
            'config',
            'joystick_config.yaml')]
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time")
            },
            os.path.join(program_bringup_pkg, "config", "twist_mux_topics.yaml"),
            os.path.join(program_bringup_pkg, "config", "twist_mux_locks.yaml"),
        ],
        remappings=[
            ("/cmd_vel_out", "/cmd_vel_unstamped")
        ]
    )

    relay_node = Node(
        package="helpers",
        executable="cmd_relay",
        name="cmd_relay",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time"),
             "max_wheel_speed_m_s": max_wheel_speed},
            os.path.join(program_bringup_pkg, "config", "robot_dimensions.yaml"),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        max_wheel_speed_arg,
        joystick_node,
        joy_node,
        twist_mux_node,
        relay_node
    ]
    )