"""Single entry point for the roomba.

    ros2 launch program_bringup startup.launch.py

Everything that normally changes between runs (sim or hardware, serial port,
which nodes to start, ...) lives in config/startup.yaml - edit that file
instead of passing launch arguments. This file only reads the YAML, checks
it, and starts the building blocks in launch/modules/:

    joystick     joy -> joy_teleop -> twist_mux -> cmd_relay
    slam         map_saver_server + slam_toolbox
    robot        robot_state_publisher (+ ros2_control_node on the hardware)
    simulation   Gazebo, robot spawn, ROS<->Gazebo bridge          (sim only)
    hardware_sensors YDLidar + BNO055 IMU                         (hardware only)
    controllers  ros2_control controller spawners
    localization robot_localization EKF
    navigation   A* global planner + pure pursuit local planner
    exploration  map validator, frontier/reactive explorers, manager
    tools        RViz + data logger

The only launch argument is `startup_config`, to use another YAML file.
"""

import os
from collections import namedtuple

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    Shutdown,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PACKAGE = 'program_bringup'

MODES = ('sim', 'hardware')

# Robot description (xacro in the urdfs package) used for each mode.
URDF_FILE = {
    'sim': 'roomba.urdf.xacro',
    'hardware': 'roomba_hardware.urdf.xacro',
}

# Every key startup.yaml may contain, with the value used when it is missing.
# Anything not listed here is rejected, so a typo cannot silently do nothing.
DEFAULTS = {
    'mode': 'sim',
    'sim': {
        'world': 'small_house.world',
        'gz_flags': '-r -s -v 1',
        'rviz': True,
    },
    'hardware': {
        'serial_port': '/dev/roomba_esp32',
        'baud_rate': 115200,
        'rviz': False,
        'lidar': True,
        'imu': True,
    },
    'slam_config': None,
    'ekf_config': None,
    'rviz_config': None,
    'nodes': {
        'joystick': True,
        'slam': True,
        'ekf': True,
        'global_planner': True,
        'local_planner': True,
        'map_validator': True,
        'frontier_explorer': True,
        'reactive_explorer': True,
        'manager': True,
        'data_logger': True,
    },
}

# One building block to include: label (for the log), file inside
# launch/modules/, and the launch arguments to pass to it.
Module = namedtuple('Module', ['label', 'file', 'args'])


class StartupConfigError(Exception):
    """startup.yaml is missing, malformed or points at a file that is not there."""


# ----------------------------------------------------------------------------
#  Reading and checking startup.yaml
# ----------------------------------------------------------------------------
def load_startup_config(path):
    """Read the YAML file and return it merged over DEFAULTS."""
    if not os.path.isfile(path):
        raise StartupConfigError(f'startup config not found: {path}')
    with open(path, 'r', encoding='utf-8') as stream:
        try:
            user = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            raise StartupConfigError(f'{path} is not valid YAML: {error}') from error
    if user is None:
        user = {}
    if not isinstance(user, dict):
        raise StartupConfigError(f'{path} must contain key: value pairs at the top level')

    cfg = _merge(DEFAULTS, user, '')
    if cfg['mode'] not in MODES:
        raise StartupConfigError(f"mode must be one of {list(MODES)}, got {cfg['mode']!r}")
    return cfg


def _merge(defaults, user, where):
    """Overlay `user` on `defaults`, rejecting unknown keys and wrong types."""
    merged = {}
    for key in user:
        if key not in defaults:
            raise StartupConfigError(
                f"unknown key '{where}{key}' in startup config "
                f'(valid here: {", ".join(defaults)})')
    for key, default in defaults.items():
        name = f'{where}{key}'
        value = user.get(key, default)
        if isinstance(default, dict):
            if value is None:
                value = {}
            if not isinstance(value, dict):
                raise StartupConfigError(f"'{name}' must be a group of key: value pairs")
            merged[key] = _merge(default, value, f'{name}.')
        else:
            merged[key] = _check_type(name, value, default)
    return merged


def _check_type(name, value, default):
    """Value must have the same type as its default (None = optional file name)."""
    if default is None:
        if value is not None and not isinstance(value, str):
            raise StartupConfigError(f"'{name}' must be a file name or null, got {value!r}")
    elif isinstance(default, bool):
        if not isinstance(value, bool):
            raise StartupConfigError(f"'{name}' must be true or false, got {value!r}")
    elif isinstance(default, int):
        if isinstance(value, bool) or not isinstance(value, int):
            raise StartupConfigError(f"'{name}' must be a whole number, got {value!r}")
    elif not isinstance(value, str):
        raise StartupConfigError(f"'{name}' must be text, got {value!r}")
    return value


def _resolve_file(name, folder, what):
    """A plain file name is looked up in `folder`; an absolute path is used as is."""
    path = name if os.path.isabs(name) else os.path.join(folder, name)
    if not os.path.isfile(path):
        raise StartupConfigError(f'{what} not found: {path}')
    return path


# ----------------------------------------------------------------------------
#  Deciding what to start
# ----------------------------------------------------------------------------
def build_plan(cfg, manual_test=False, manual_autonomy=False):
    """Turn the checked config into the ordered list of modules to include."""
    share = get_package_share_directory(PACKAGE)
    sim = cfg['mode'] == 'sim'
    mode_cfg = cfg[cfg['mode']]
    nodes = cfg['nodes']
    use_sim_time = sim

    # Optional overrides: only passed on when set, so each module keeps its own default.
    slam_extra, ekf_extra, rviz_extra = {}, {}, {}
    if cfg['slam_config']:
        slam_extra['slam_config'] = _resolve_file(
            cfg['slam_config'], os.path.join(share, 'config'), 'slam_config')
    if cfg['ekf_config']:
        ekf_extra['ekf_config'] = _resolve_file(
            cfg['ekf_config'], os.path.join(share, 'config'), 'ekf_config')
    if cfg['rviz_config']:
        urdfs_rviz = os.path.join(get_package_share_directory('urdfs'), 'rviz')
        rviz_extra['rviz_config'] = _resolve_file(cfg['rviz_config'], urdfs_rviz, 'rviz_config')

    plan = []

    if nodes['joystick']:
        plan.append(Module('joystick', 'joystick.launch.py', {
            'use_sim_time': use_sim_time,
        }))

    if nodes['slam']:
        plan.append(Module('slam', 'slam.launch.xml', {
            'use_sim_time': use_sim_time, **slam_extra,
        }))

    robot_args = {'sim': sim, 'use_sim_time': use_sim_time, 'urdf_file': URDF_FILE[cfg['mode']]}
    if not sim:
        robot_args['serial_port'] = mode_cfg['serial_port']
        robot_args['baud_rate'] = mode_cfg['baud_rate']
    plan.append(Module('robot', 'robot.launch.xml', robot_args))

    if sim:
        world = _resolve_file(mode_cfg['world'], os.path.join(share, 'world'), 'sim.world')
        plan.append(Module('simulation', 'simulation.launch.xml', {
            'world': world,
            'gz_flags': mode_cfg['gz_flags'],
        }))
    elif mode_cfg['lidar'] or mode_cfg['imu']:
        plan.append(Module('hardware_sensors', 'hardware_sensors.launch.xml', {
            'use_sim_time': use_sim_time,
            'enable_lidar': mode_cfg['lidar'],
            'enable_imu': mode_cfg['imu'],
        }))

    plan.append(Module('controllers', 'controllers.launch.xml', {}))

    if manual_test:
        plan.append(Module('manual_test', 'manual_test.launch.xml', {
            'use_sim_time': False,
            'autonomy': manual_autonomy,
        }))

    if nodes['ekf']:
        plan.append(Module('localization', 'localization.launch.xml', {
            'use_sim_time': use_sim_time, **ekf_extra,
        }))

    if nodes['global_planner'] or nodes['local_planner']:
        plan.append(Module('navigation', 'navigation.launch.xml', {
            'use_sim_time': use_sim_time,
            'enable_global_planner': nodes['global_planner'],
            'enable_local_planner': nodes['local_planner'],
        }))

    if any(nodes[n] for n in ('map_validator', 'frontier_explorer', 'reactive_explorer', 'manager')):
        plan.append(Module('exploration', 'exploration.launch.xml', {
            'use_sim_time': use_sim_time,
            'enable_map_validator': nodes['map_validator'],
            'enable_frontier_explorer': nodes['frontier_explorer'],
            'enable_reactive_explorer': nodes['reactive_explorer'],
            'enable_manager': nodes['manager'],
        }))

    if mode_cfg['rviz'] or nodes['data_logger']:
        plan.append(Module('tools', 'tools.launch.xml', {
            'sim': sim,
            'use_sim_time': use_sim_time,
            'rviz': mode_cfg['rviz'],
            'enable_data_logger': nodes['data_logger'],
            **rviz_extra,
        }))

    return plan


def _as_launch_argument(value):
    """Launch arguments are strings; booleans use the lower-case form XML conditions expect."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


# ----------------------------------------------------------------------------
#  Launch description
# ----------------------------------------------------------------------------
def launch_setup(context):
    config_path = LaunchConfiguration('startup_config').perform(context)
    manual_test = LaunchConfiguration('manual_test').perform(context).lower() in ('true', '1', 'yes')
    manual_autonomy = LaunchConfiguration('manual_autonomy').perform(context).lower() in ('true', '1', 'yes')
    manual_rviz = LaunchConfiguration('manual_rviz').perform(context).lower() in ('true', '1', 'yes')
    try:
        cfg = load_startup_config(config_path)
        if manual_test:
            # This explicit opt-in always selects the real robot and leaves only
            # hardware, sensors, controllers, odometry, terminal control, and RViz.
            cfg['mode'] = 'hardware'
            cfg['hardware']['rviz'] = manual_rviz
            cfg['hardware']['lidar'] = False
            cfg['rviz_config'] = 'manual_test.rviz'
            if manual_autonomy:
                cfg['rviz_config'] = None  # default.rviz: map, scan, path, 2D Goal Pose tool
                cfg['hardware']['lidar'] = True
            for name in ('joystick', 'slam', 'ekf', 'global_planner', 'local_planner',
                         'map_validator', 'frontier_explorer', 'reactive_explorer',
                         'manager', 'data_logger'):
                cfg['nodes'][name] = False
        if manual_test and manual_autonomy:
            # SLAM + EKF + A* + pure pursuit + map_validator; no manager/explorers/joystick/logger.
            for name in ('slam', 'ekf', 'global_planner', 'local_planner', 'map_validator'):
                cfg['nodes'][name] = True
        plan = build_plan(cfg, manual_test=manual_test, manual_autonomy=manual_autonomy)
    except StartupConfigError as error:
        message = f'[startup] {error}'
        return [LogInfo(msg=message), Shutdown(reason=message)]

    modules_dir = os.path.join(get_package_share_directory(PACKAGE), 'launch', 'modules')
    actions = [LogInfo(msg=(
        f"[startup] config: {config_path}\n"
        f"[startup] mode: {cfg['mode'].upper()}\n"
        f"[startup] modules: {', '.join(m.label for m in plan)}"))]

    for module in plan:
        actions.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(modules_dir, module.file)),
            launch_arguments={k: _as_launch_argument(v) for k, v in module.args.items()}.items(),
        ))
    return actions


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory(PACKAGE), 'config', 'startup.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'startup_config', default_value=default_config,
            description='Startup YAML file (sim or hardware, nodes to start, ...)'),
        DeclareLaunchArgument(
            'manual_test', default_value='false',
            description='Run hardware-only manual motion and odometry testing mode'),
        DeclareLaunchArgument(
            'manual_rviz', default_value='false',
            description='Start local RViz2 in manual mode (requires a graphical display)'),
        DeclareLaunchArgument(
            'manual_autonomy', default_value='true',
            description='With manual_test: also run lidar, SLAM, EKF, A*, pure pursuit and '
                        'map_validator (goals from RViz). false = odometry-only test mode'),
        OpaqueFunction(function=launch_setup),
    ])
