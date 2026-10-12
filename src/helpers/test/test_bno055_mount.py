"""Math checks for the BNO055 mounting transform (no ROS / hardware needed)."""
import sys
import types

import numpy as np
from scipy.spatial.transform import Rotation as R

# bno055.py imports ROS / I2C modules at top level; stub them for this pure-math test.
for name in ('rclpy', 'rclpy.node', 'sensor_msgs', 'sensor_msgs.msg',
             'geometry_msgs', 'geometry_msgs.msg', 'std_msgs', 'std_msgs.msg', 'smbus2'):
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules['rclpy.node'].Node = object
for mod, names in (('sensor_msgs.msg', ['Imu']),
                   ('geometry_msgs.msg', ['Quaternion', 'Vector3']),
                   ('std_msgs.msg', ['Float64'])):
    for n in names:
        setattr(sys.modules[mod], n, object)

from helpers.bno055 import MOUNT_MATRIX, mount_orientation, mount_vector  # noqa: E402


def test_axis_mapping():
    # IMU +X = robot right (-Y_flu), IMU -Y = forward (+X_flu), IMU -Z = up (+Z_flu)
    assert np.allclose(mount_vector([1, 0, 0]), [0, -1, 0])
    assert np.allclose(mount_vector([0, -1, 0]), [1, 0, 0])
    assert np.allclose(mount_vector([0, 0, -1]), [0, 0, 1])
    assert np.isclose(np.linalg.det(MOUNT_MATRIX), 1.0)


def test_yaw_ccw_is_positive_and_consistent_with_gyro():
    rate, dt = 0.7, 1e-3                      # robot turning CCW (seen from above)

    def q_ws(th):
        # What the chip would report: sensor pose in world = robot pose * M^-1
        return R.from_euler('z', th) * R.from_matrix(MOUNT_MATRIX).inv()

    def yaw(t):
        return mount_orientation(q_ws(t)).as_euler('zyx')[0]

    th = 0.4
    assert np.isclose(yaw(th), th)            # no yaw sign flip
    assert np.isclose(mount_orientation(q_ws(th)).as_euler('zyx')[1:], 0).all()  # level
    gyro_imu = MOUNT_MATRIX.T @ np.array([0, 0, rate])   # what the sensor measures
    gyro_link = mount_vector(gyro_imu)
    assert np.allclose(gyro_link, [0, 0, rate])
    assert np.isclose((yaw(th + rate * dt) - yaw(th)) / dt, gyro_link[2], rtol=1e-3)


def test_pre_multiply_would_be_wrong():
    q_ws = R.from_euler('z', 0.4) * R.from_matrix(MOUNT_MATRIX).inv()
    wrong = (R.from_matrix(MOUNT_MATRIX) * q_ws).as_euler('zyx')[0]
    assert not np.isclose(wrong, 0.4)
