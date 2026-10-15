"""Wheel-speed limit and stale-command stop in relay.py (no ROS needed)."""
import sys
import types

import pytest


class _Any:
    def __init__(self, *a, **k):
        pass

    def __getattr__(self, n):
        return _Any()


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, m):
        self.sent.append(m)


class _Time:
    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, o):
        return _Time(self.nanoseconds - o.nanoseconds)

    def to_msg(self):
        return self


class _Clock:
    ns = 0

    def now(self):
        return _Time(_Clock.ns)


class _Node:
    overrides = {}

    def __init__(self, *a):
        self._p = {}
        self.pubs = {}

    def declare_parameter(self, name, default=None):
        self._p[name] = self.overrides.get(name, None if not isinstance(default, (int, float)) else default)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._p[name])

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None)

    def get_clock(self):
        return _Clock()

    def create_publisher(self, _t, topic, _q):
        return self.pubs.setdefault(topic, _Pub())

    def __getattr__(self, n):
        if n.startswith('_'):
            raise AttributeError(n)
        return lambda *a, **k: _Any()


for m in ['rclpy', 'rclpy.time', 'rclpy.node', 'rclpy.parameter', 'geometry_msgs', 'geometry_msgs.msg',
          'std_msgs', 'std_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg', 'tf_transformations',
          'nav_msgs', 'nav_msgs.msg']:
    sys.modules[m] = types.ModuleType(m)
sys.modules['rclpy.node'].Node = _Node
sys.modules['rclpy.time'].Time = _Time
sys.modules['rclpy.parameter'].Parameter = types.SimpleNamespace(Type=types.SimpleNamespace(DOUBLE='d'))
for mod, names in (('geometry_msgs.msg', ['TwistStamped', 'Twist']), ('std_msgs.msg', ['Float64MultiArray']),
                   ('sensor_msgs.msg', ['JointState']), ('nav_msgs.msg', ['Odometry'])):
    for n in names:
        setattr(sys.modules[mod], n, _Any)
sys.modules['tf_transformations'].quaternion_from_euler = lambda *a: [0, 0, 0, 1]

from helpers.relay import CMDRelayNode, limit_wheel_speeds  # noqa: E402

R, L = 0.035, 0.24


def test_limit_scales_both_wheels_together():
    left, right, limited = limit_wheel_speeds(9.14, 3.43, R, 0.20)   # 0.32 and 0.12 m/s
    assert limited
    assert max(abs(left), abs(right)) * R == pytest.approx(0.20)
    assert left / right == pytest.approx(9.14 / 3.43)                  # turn radius preserved


def test_limit_off_or_not_needed():
    assert limit_wheel_speeds(2.0, 2.0, R, 0.0) == (2.0, 2.0, False)
    assert limit_wheel_speeds(2.0, -2.0, R, 0.20) == (2.0, -2.0, False)


def test_non_finite_stops():
    assert limit_wheel_speeds(float('nan'), 1.0, R, 0.20) == (0.0, 0.0, True)
    assert limit_wheel_speeds(1.0, float('inf'), R, 0.0) == (0.0, 0.0, True)


def _node(max_speed=0.25, timeout=0.5):
    _Node.overrides = {'robot.drive_wheel.track_width_m': L, 'robot.drive_wheel.radius_m': R,
                       'limits.max_wheel_speed_m_s': max_speed, 'cmd_timeout_s': timeout}
    _Clock.ns = 0
    return CMDRelayNode()


def _cmd(v, w):
    return types.SimpleNamespace(linear=types.SimpleNamespace(x=v), angular=types.SimpleNamespace(z=w))


def test_command_is_limited_end_to_end():
    n = _node()
    n.genrate_cmd_vel(_cmd(0.8, 0.0))
    left, right = n.cmd_vel_publisher_.sent[-1].data
    assert left * R == pytest.approx(0.25) and right * R == pytest.approx(0.25)


def test_stale_command_stops_wheels_once():
    n = _node()
    n.genrate_cmd_vel(_cmd(0.1, 0.0))
    sent = n.cmd_vel_publisher_.sent
    _Clock.ns = int(0.3e9)
    n.cmd_watchdog()
    assert len(sent) == 1                       # still fresh
    _Clock.ns = int(0.6e9)
    n.cmd_watchdog()
    assert sent[-1].data == [0.0, 0.0] and len(sent) == 2
    n.cmd_watchdog()
    assert len(sent) == 2                       # stop is published only once


def test_zero_command_and_disabled_timeout_do_not_trigger():
    n = _node()
    n.genrate_cmd_vel(_cmd(0.0, 0.0))
    _Clock.ns = int(5e9)
    n.cmd_watchdog()
    assert len(n.cmd_vel_publisher_.sent) == 1
    n = _node(timeout=0.0)
    n.genrate_cmd_vel(_cmd(0.1, 0.0))
    _Clock.ns = int(5e9)
    n.cmd_watchdog()
    assert len(n.cmd_vel_publisher_.sent) == 1
