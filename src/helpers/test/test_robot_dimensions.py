"""Consistency checks for the shared robot geometry (robot_dimensions.yaml)."""
import math
import pathlib
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

SRC = pathlib.Path(__file__).resolve().parents[2]
DIMS_YAML = SRC / 'program_bringup' / 'config' / 'robot_dimensions.yaml'
URDF_DIR = SRC / 'urdfs' / 'urdf'


def _dims():
    return yaml.safe_load(DIMS_YAML.read_text())['/**']['ros__parameters']['robot']


def test_confirmed_measurements():
    d = _dims()
    assert (d['footprint']['length_m'], d['footprint']['width_m']) == (0.20, 0.20)
    assert d['drive_wheel']['track_width_m'] == 0.24
    assert d['drive_wheel']['radius_m'] == 0.035
    assert d['drive_wheel']['width_m'] == 0.02
    assert d['imu']['x_m'] == -0.08
    assert d['body']['top_above_wheel_center_m'] == 0.20
    assert d['caster']['diameter_m'] == 0.035


def test_radius_diameter_agree():
    d = _dims()
    assert math.isclose(d['drive_wheel']['diameter_m'], 2 * d['drive_wheel']['radius_m'])
    assert math.isclose(d['caster']['diameter_m'], 2 * d['caster']['radius_m'])


def test_controllers_yaml_matches():
    # diff_drive_controller block cannot read the shared file; keep it in sync.
    c = yaml.safe_load((SRC / 'program_bringup' / 'config' / 'controllers.yaml').read_text())
    dd = c['diff_drive_controller']['ros__parameters']
    d = _dims()
    assert dd['wheel_separation'] == d['drive_wheel']['track_width_m']
    assert dd['wheel_radius'] == d['drive_wheel']['radius_m']


def test_no_stale_geometry_literals():
    stale = re.compile(r'0\.333\b|0\.1925|0\.1665|0\.01755|0\.385')
    hits = []
    for p in list(URDF_DIR.glob('*.xacro')) + list((SRC / 'program_bringup').rglob('*.y*ml')) \
            + list((SRC / 'program_bringup').rglob('*.xml')) \
            + [SRC / 'helpers' / 'helpers' / 'relay.py',
               SRC / 'global_planners' / 'global_planners' / 'a_star_node.py']:
        for n, line in enumerate(p.read_text().splitlines(), 1):
            if stale.search(line):
                hits.append(f'{p.relative_to(SRC)}:{n}: {line.strip()}')
    assert not hits, '\n'.join(hits)


def _xyz(joint):
    return [float(v) for v in joint.find('origin').get('xyz').split()]


@pytest.mark.parametrize('name,sim', [('roomba_hardware', 'false'), ('roomba', 'true')])
def test_generated_urdf(name, sim):
    xacro = pytest.importorskip('xacro')
    import sys
    argv = sys.argv
    out = None
    try:
        out = xacro.process_file(
            str(URDF_DIR / f'{name}.urdf.xacro'),
            mappings={'robot_dimensions_path': str(DIMS_YAML),
                      'use_sim': sim, 'controllers_config_path': 'x'}).toxml()
    finally:
        sys.argv = argv
    root = ET.fromstring(out)
    d = _dims()
    joints = {j.get('name'): j for j in root.findall('joint')}
    r, lw = _xyz(joints['base_right_wheel_joint']), _xyz(joints['base_left_wheel_joint'])
    assert math.isclose(lw[1] - r[1], d['drive_wheel']['track_width_m'])      # 24 cm track
    assert math.isclose(lw[1], -r[1])
    assert math.isclose(_xyz(joints['spine'])[2], d['drive_wheel']['radius_m'])
    link = {k.get('name'): k for k in root.findall('link')}
    box = link['base_link'].find('visual/geometry/box').get('size').split()
    assert [float(box[0]), float(box[1])] == [0.20, 0.20]                  # footprint
    top = float(link['base_link'].find('visual/origin').get('xyz').split()[2]) + float(box[2]) / 2
    assert math.isclose(top, d['body']['top_above_wheel_center_m'])        # 20 cm above wheel centre
    wheel = link['right_wheel'].find('visual/geometry/cylinder')
    assert math.isclose(float(wheel.get('radius')), 0.035)
    assert math.isclose(float(wheel.get('length')), 0.02)
    caster = float(link['caster_wheel'].find('visual/geometry/sphere').get('radius'))
    assert math.isclose(caster, 0.0175)
    # caster touches the ground: centre z (base_link) - radius == -wheel radius
    cz = _xyz(joints['base_caster_wheel_joint'])[2]
    assert math.isclose(cz - caster, -0.035)
    imu = _xyz(joints['base_imu_joint'])
    assert math.isclose(imu[0], -0.08)
    assert joints['base_imu_joint'].find('origin').get('rpy').split() == ['0', '0', '0']  # no 2nd mount rotation
    if name == 'roomba':
        assert f'<wheel_separation>{d["drive_wheel"]["track_width_m"]}</wheel_separation>' in out


def _limit():
    p = yaml.safe_load(DIMS_YAML.read_text())['/**']['ros__parameters']
    return p['limits']['max_wheel_speed_m_s'], p['robot']['drive_wheel']['track_width_m']


def test_speed_limit_is_025_and_static_yamls_match():
    v, track = _limit()
    assert v == 0.25
    w = round(2 * v / track, 2)  # pure-spin limit = 2.083 rad/s
    cfg = SRC / 'program_bringup' / 'config'
    joy = (cfg / 'joystick_teleop.yaml').read_text()
    assert re.search(r'axis: 1\s*\n\s*scale: %s\b' % v, joy)
    assert re.search(r'axis: 2\s*\n\s*scale: %s\b' % w, joy)
    c = yaml.safe_load((cfg / 'controllers.yaml').read_text())['diff_drive_controller']['ros__parameters']
    assert c['linear.x.max_velocity'] == v and c['linear.x.min_velocity'] == -v
    assert c['angular.z.max_velocity'] == w and c['angular.z.min_velocity'] == -w
    mux = yaml.safe_load((cfg / 'twist_mux_joy.yaml').read_text())
    flat = str(mux)
    assert str(v) in flat and str(w) in flat


def test_astar_primitives_scale_keeps_geometry():
    import ast
    import numpy as np
    src = (SRC / 'global_planners' / 'global_planners' / 'a_star_node.py').read_text()
    tree = ast.parse(src)
    keep = [n for n in tree.body if (isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') == 'PLAN_REF_SPEED')
            or (isinstance(n, ast.FunctionDef) and n.name == 'motion_primitives')]
    ns = {'np': np}
    exec(compile(ast.Module(keep, []), 'a_star', 'exec'), ns)
    ref = ns['motion_primitives'](0.8)
    dt, vs, ws = ns['motion_primitives'](0.25)
    assert max(vs) == pytest.approx(0.25)
    assert [v * dt for v in vs] == pytest.approx([v * ref[0] for v in ref[1]])
    assert [w * dt for w in ws] == pytest.approx([w * ref[0] for w in ref[2]])
