"""Static checks for the single startup launch (no ROS needed)."""
import importlib.util
import pathlib
import re
import sys
import types
import xml.etree.ElementTree as ET

import pytest
import yaml

SRC = pathlib.Path(__file__).resolve().parents[2]
BRINGUP = SRC / 'program_bringup'
MODULES = BRINGUP / 'launch' / 'modules'
STARTUP_YAML = BRINGUP / 'config' / 'startup.yaml'


class _Dummy:
    def __init__(self, *a, **k):
        pass


def _load_startup():
    stubs = {
        'ament_index_python': types.ModuleType('ament_index_python'),
        'ament_index_python.packages': types.ModuleType('ament_index_python.packages'),
        'launch': types.ModuleType('launch'),
        'launch.actions': types.ModuleType('launch.actions'),
        'launch.launch_description_sources': types.ModuleType('launch.launch_description_sources'),
        'launch.substitutions': types.ModuleType('launch.substitutions'),
    }
    stubs['ament_index_python.packages'].get_package_share_directory = lambda pkg: str(SRC / pkg)
    stubs['launch'].LaunchDescription = _Dummy
    for n in ('DeclareLaunchArgument', 'IncludeLaunchDescription', 'LogInfo', 'OpaqueFunction', 'Shutdown'):
        setattr(stubs['launch.actions'], n, _Dummy)
    stubs['launch.launch_description_sources'].AnyLaunchDescriptionSource = _Dummy
    stubs['launch.substitutions'].LaunchConfiguration = _Dummy
    saved = {k: sys.modules.get(k) for k in stubs}
    sys.modules.update(stubs)
    try:
        spec = importlib.util.spec_from_file_location(
            'startup_launch_under_test', BRINGUP / 'launch' / 'startup.launch.py')
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


startup = _load_startup()


def _cfg(tmp_path, text):
    path = tmp_path / 'startup.yaml'
    path.write_text(text)
    return startup.load_startup_config(str(path))


def _labels(plan):
    return [m.label for m in plan]


def test_shipped_yaml_loads_and_defaults_to_sim():
    cfg = startup.load_startup_config(str(STARTUP_YAML))
    assert cfg['mode'] == 'sim'
    assert set(yaml.safe_load(STARTUP_YAML.read_text())) <= set(startup.DEFAULTS)


def test_sim_plan():
    plan = startup.build_plan(startup.load_startup_config(str(STARTUP_YAML)))
    labels = _labels(plan)
    assert 'simulation' in labels and 'hardware_sensors' not in labels
    joystick = next(m for m in plan if m.label == 'joystick')
    assert 'max_wheel_speed' not in joystick.args and joystick.args['use_sim_time'] is True
    assert next(m for m in plan if m.label == 'robot').args['urdf_file'] == 'roomba.urdf.xacro'
    assert next(m for m in plan if m.label == 'tools').args['rviz'] is True


def test_hardware_plan(tmp_path):
    plan = startup.build_plan(_cfg(tmp_path, 'mode: hardware\n'))
    labels = _labels(plan)
    assert 'hardware_sensors' in labels and 'simulation' not in labels
    robot = next(m for m in plan if m.label == 'robot')
    assert robot.args['sim'] is False and robot.args['use_sim_time'] is False
    assert robot.args['urdf_file'] == 'roomba_hardware.urdf.xacro'
    assert robot.args['serial_port'] == '/dev/roomba_esp32'
    assert next(m for m in plan if m.label == 'tools').args['rviz'] is False  # RViz stays off the Pi


def test_hardware_options_and_node_switches(tmp_path):
    plan = startup.build_plan(_cfg(tmp_path, (
        'mode: hardware\nhardware: {lidar: false, imu: false}\n'
        'nodes: {joystick: true, slam: false, data_logger: false}\n')))
    labels = _labels(plan)
    assert 'hardware_sensors' not in labels and 'slam' not in labels and 'tools' not in labels


@pytest.mark.parametrize('text', [
    'mode: real\n',                              # old name is rejected
    'mode: sim\nbogus: 1\n',                     # unknown key
    'mode: hardware\nhardware: {baud_rate: x}\n',
    'mode: hardware\nhardware: {max_wheel_speed: 0.1}\n',      # now lives in robot_dimensions.yaml
    'mode: sim\nnodes: {slam: maybe}\n',
])
def test_invalid_config_rejected(tmp_path, text):
    with pytest.raises(startup.StartupConfigError):
        _cfg(tmp_path, text)


def _declared(path):
    text = path.read_text()
    if path.suffix == '.py':
        return set(re.findall(r"DeclareLaunchArgument\(\s*'(\w+)'", text))
    root = ET.fromstring(text)
    return {e.get('name') for e in root.iter() if e.tag in ('arg', 'let')}


@pytest.mark.parametrize('mode', ['sim', 'hardware'])
def test_every_planned_module_exists_and_accepts_its_args(tmp_path, mode):
    for m in startup.build_plan(_cfg(tmp_path, f'mode: {mode}\n')):
        path = MODULES / m.file
        assert path.is_file(), path
        assert set(m.args) <= _declared(path), (m.label, set(m.args) - _declared(path))


@pytest.mark.parametrize('path', sorted(MODULES.glob('*.xml')), ids=lambda p: p.name)
def test_xml_modules_only_use_declared_variables(path):
    used = set(re.findall(r'\$\(var (\w+)\)', path.read_text()))
    assert used <= _declared(path), used - _declared(path)


def test_workspace_executables_exist():
    scripts = {}
    for pkg in ('helpers', 'global_planners', 'local_planners'):
        setup = (SRC / pkg / 'setup.py').read_text()
        scripts[pkg] = set(re.findall(r"['\"](\w+)\s*=\s*[\w.]+:\w+['\"]", setup))
    text = ''.join(p.read_text() for p in [*MODULES.glob('*.xml'), *MODULES.glob('*.py')])
    pairs = re.findall(r'pkg="(\w+)"\s+exec="(\w+)"', text) + re.findall(
        r"package='(\w+)',\s*executable='(\w+)'", text)
    checked = [(p, e) for p, e in pairs if p in scripts]
    assert checked
    for pkg, exe in checked:
        assert exe in scripts[pkg], (pkg, exe)


def test_dimensions_wired_into_every_consumer():
    dims = 'robot_dimensions'
    assert dims in (MODULES / 'robot.launch.xml').read_text()        # xacro
    assert dims in (MODULES / 'navigation.launch.xml').read_text()   # a_star
    assert dims in (MODULES / 'joystick.launch.py').read_text()      # cmd_relay


def test_no_references_to_removed_launch_files():
    stale = re.compile(r'replay_rosbag|roomba_hardware_only|roomba\.launch|slam_plus_lifecycle|real_sensors|mode: real')
    hits = []
    for p in SRC.rglob('*'):
        if p.suffix in ('.py', '.xml', '.yaml', '.md', '.txt', '.cfg') and p.is_file() \
                and 'ydlidar_ros2_driver' not in p.parts and p.name != 'test_startup_launch.py':
            for n, line in enumerate(p.read_text(errors='ignore').splitlines(), 1):
                if stale.search(line):
                    hits.append(f'{p.relative_to(SRC)}:{n}: {line.strip()}')
    assert not hits, '\n'.join(hits)
