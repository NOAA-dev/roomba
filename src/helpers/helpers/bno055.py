#!/usr/bin/env python3

import struct
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation as R
import smbus2

# BNO055 Register Definitions
BNO055_ADDRESS = 0x28        # Default I2C address (0x29 if ADR pin is high)
BNO055_CHIP_ID_REG = 0x00
BNO055_OPR_MODE_REG = 0x3D
BNO055_PWR_MODE_REG = 0x3E
BNO055_SYS_TRIGGER_REG = 0x3F
BNO055_UNIT_SEL_REG = 0x3B

# Operation Modes
OPERATION_MODE_CONFIG = 0x00
OPERATION_MODE_IMUPLUS = 0x08  # Accel + Gyro fusion, magnetometer OFF (relative yaw)

# Units: bit1 = 1 -> gyro in rad/s. Everything else default
# (accel m/s^2, Euler degrees, Celsius, Windows orientation convention).
UNIT_SEL_GYRO_RADS = 0x02

# Burst read: 26 contiguous bytes starting at the gyro data registers.
BNO055_BURST_START = 0x14
BNO055_BURST_LEN = 26
#   offset  0..5   gyro X/Y/Z        (int16 LE)
#   offset  6..11  Euler H/R/P       (unused here)
#   offset 12..19  quaternion W/X/Y/Z (int16 LE)
#   offset 20..25  linear accel X/Y/Z (int16 LE, gravity removed)

# Scale factors for the unit configuration above
SCALE_QUAT = 1.0 / 16384.0   # 16384 LSB = 1 (unit quaternion)
SCALE_GYRO = 1.0 / 900.0     # 900 LSB = 1 rad/s
SCALE_ACCEL = 1.0 / 100.0    # 100 LSB = 1 m/s^2


class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_imu_node')

        # Declare parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', BNO055_ADDRESS)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)  # 50 Hz
        self.declare_parameter('accel_filter_alpha', 0.2)  # EMA smoothing factor, 0-1
        self.declare_parameter('publish_debug', False)  # bno/yaw, bno/gyro, bno/accel

        self.bus_num = self.get_parameter('i2c_bus').value
        self.addr = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        pub_rate = self.get_parameter('publish_rate').value
        self.accel_alpha = self.get_parameter('accel_filter_alpha').value
        self.publish_debug = self.get_parameter('publish_debug').value

        # Exponential-moving-average state for linear acceleration, applied
        # after mounting-rotation so it's smoothing in the published (imu_link)
        # frame. None until the first sample arrives, so we don't filter
        # toward a bogus zero on startup.
        self._filtered_accel = None

        # Publishers
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        if self.publish_debug:
            self.yaw_pub = self.create_publisher(Float64, 'bno/yaw', 10)
            self.gyro_pub = self.create_publisher(Vector3, 'bno/gyro', 10)
            self.accel_pub = self.create_publisher(Vector3, 'bno/accel', 10)

        # Compute fixed orientation mounting transformation matrix
        # Mounting condition: Inverted (180 deg around X-axis) + Clockwise 180 deg around Z-axis
        r_x_inv = R.from_euler('x', -180, degrees=True)
        r_z_cw = R.from_euler('z', -180, degrees=True)
        # Combined mounting rotation matrix (Sensor to Link transform)
        self.R_mount = r_z_cw * r_x_inv

        # Initialize Hardware
        if not self.init_bno055():
            self.get_logger().error(
                'BNO055 initialization failed -- not publishing. '
                'Check wiring / I2C address / bus number and restart the node.'
            )
            return

        # Timer setup (50 Hz -> 0.02s period)
        timer_period = 1.0 / pub_rate
        self.timer = self.create_timer(timer_period, self.publish_imu)
        self.get_logger().info(
            f'BNO055 Node initialized (IMUPLUS mode, no magnetometer). '
            f'Publishing on /imu at {pub_rate} Hz '
            f'(accel EMA alpha={self.accel_alpha}).'
        )
        self.get_logger().info(
            'Yaw is relative to the heading at startup and will drift slowly. '
            'Keep the robot still for 1-2 s after launch so the gyro can settle.'
        )

    def init_bno055(self):
        """Resets the BNO055 and configures IMUPLUS fusion mode (accel + gyro only).

        Returns True only if the chip answered and the mode readback matches.
        """
        try:
            self.bus = smbus2.SMBus(self.bus_num)

            # Go to CONFIG mode, then trigger a full system reset so we start
            # from a known state on every node restart.
            self.bus.write_byte_data(self.addr, BNO055_OPR_MODE_REG, OPERATION_MODE_CONFIG)
            time.sleep(0.025)  # any mode -> CONFIG takes up to 19 ms
            try:
                self.bus.write_byte_data(self.addr, BNO055_SYS_TRIGGER_REG, 0x20)
            except OSError:
                pass  # chip may NACK while it starts resetting; the poll below confirms

            # Poll the chip ID until the chip is back (reset takes ~650 ms).
            time.sleep(0.4)
            chip_ok = False
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                try:
                    if self.bus.read_byte_data(self.addr, BNO055_CHIP_ID_REG) == 0xA0:
                        chip_ok = True
                        break
                except OSError:
                    pass
                time.sleep(0.05)
            if not chip_ok:
                self.get_logger().error('BNO055 did not respond (expected chip ID 0xA0) after reset.')
                return False
            time.sleep(0.05)

            # Normal power mode (must be written while in CONFIG mode)
            self.bus.write_byte_data(self.addr, BNO055_PWR_MODE_REG, 0x00)
            time.sleep(0.01)

            # Units: gyro rad/s, accel m/s^2, Euler degrees, Windows orientation
            self.bus.write_byte_data(self.addr, BNO055_UNIT_SEL_REG, UNIT_SEL_GYRO_RADS)
            time.sleep(0.01)

            # Switch to IMUPLUS (accel + gyro fusion, magnetometer disabled)
            self.bus.write_byte_data(self.addr, BNO055_OPR_MODE_REG, OPERATION_MODE_IMUPLUS)
            time.sleep(0.020)  # CONFIG -> any mode takes up to 7 ms

            # Read the mode back and confirm it took
            for _ in range(10):
                mode = self.bus.read_byte_data(self.addr, BNO055_OPR_MODE_REG) & 0x0F
                if mode == OPERATION_MODE_IMUPLUS:
                    self.get_logger().info('BNO055 confirmed in IMUPLUS mode.')
                    return True
                time.sleep(0.05)

            self.get_logger().error(
                f'BNO055 mode readback mismatch: expected {hex(OPERATION_MODE_IMUPLUS)}, got {hex(mode)}.'
            )
            return False
        except Exception as e:
            self.get_logger().error(f'I2C Communication Error during initialization: {e}')
            return False

    def _apply_accel_filter(self, accel_link):
        """Exponential moving average low-pass filter on linear acceleration.

        filtered = alpha * new + (1 - alpha) * filtered_previous

        Lower accel_filter_alpha = heavier smoothing, more lag.
        Higher accel_filter_alpha = less smoothing, more responsive.
        Tune based on your publish_rate: for a cutoff frequency f_c (Hz),
        alpha ~= 1 / (1 + 1/(2*pi*f_c*dt)) where dt = 1/publish_rate.
        """
        if self._filtered_accel is None:
            self._filtered_accel = list(accel_link)
        else:
            a = self.accel_alpha
            self._filtered_accel = [
                a * new + (1.0 - a) * prev
                for new, prev in zip(accel_link, self._filtered_accel)
            ]
        return self._filtered_accel

    def publish_imu(self):
        try:
            # One burst read: gyro, Euler, quaternion and linear accel all
            # come from the same fusion cycle in a single I2C transaction.
            raw = bytes(self.bus.read_i2c_block_data(
                self.addr, BNO055_BURST_START, BNO055_BURST_LEN))
            if len(raw) < BNO055_BURST_LEN:
                return  # drop incomplete frame

            # Gyro (rad/s, sensor frame)
            gx_raw, gy_raw, gz_raw = struct.unpack('<hhh', raw[0:6])
            gyro_sensor = [gx_raw * SCALE_GYRO, gy_raw * SCALE_GYRO, gz_raw * SCALE_GYRO]

            # Fused quaternion (W, X, Y, Z order on the chip)
            w_raw, x_raw, y_raw, z_raw = struct.unpack('<hhhh', raw[12:20])
            q_sensor = R.from_quat([x_raw * SCALE_QUAT, y_raw * SCALE_QUAT,
                                    z_raw * SCALE_QUAT, w_raw * SCALE_QUAT])

            # Fused linear acceleration, gravity removed (m/s^2, sensor frame)
            ax_raw, ay_raw, az_raw = struct.unpack('<hhh', raw[20:26])
            accel_sensor = [ax_raw * SCALE_ACCEL, ay_raw * SCALE_ACCEL, az_raw * SCALE_ACCEL]

            # -------------------------------------------------------------
            # Apply Frame Transformation (Sensor Frame -> Mounted imu_link)
            # -------------------------------------------------------------
            # Transformed Orientation Quaternion
            q_link = self.R_mount * q_sensor
            qx, qy, qz, qw = q_link.as_quat()

            # Transformed Vector Quantities (Angular Velocity and Linear Accel)
            gyro_link = self.R_mount.apply(gyro_sensor)
            accel_link = self.R_mount.apply(accel_sensor)

            # Low-pass filter the linear acceleration only -- gyro and
            # orientation come from BNO055's own onboard fusion filter
            # already and don't benefit from a second software filter here.
            accel_filtered = self._apply_accel_filter(accel_link)

            # Construct ROS 2 IMU Message
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            msg.angular_velocity = Vector3(x=gyro_link[0], y=gyro_link[1], z=gyro_link[2])
            msg.linear_acceleration = Vector3(x=accel_filtered[0], y=accel_filtered[1], z=accel_filtered[2])

            # Set orientation & sensor covariance (-1 if unknown / not calculated)
            # Yaw variance is higher than roll/pitch: without the magnetometer,
            # yaw has no absolute reference and drifts slowly.
            msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.05]
            msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
            msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

            self.publisher_.publish(msg)

            if self.publish_debug:
                # Yaw in degrees, taken from the published (imu_link) orientation
                # so it matches /imu rather than the raw sensor frame.
                yaw_deg = q_link.as_euler('zyx', degrees=True)[0]
                self.yaw_pub.publish(Float64(data=float(yaw_deg)))
                self.gyro_pub.publish(Vector3(x=gyro_link[0], y=gyro_link[1], z=gyro_link[2]))
                self.accel_pub.publish(Vector3(x=accel_filtered[0], y=accel_filtered[1], z=accel_filtered[2]))

        except Exception as e:
            self.get_logger().warn(
                f'Failed to read or publish IMU data: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()