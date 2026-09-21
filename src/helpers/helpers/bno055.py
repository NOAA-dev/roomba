#!/usr/bin/env python3

import struct
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
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
OPERATION_MODE_NDOF = 0x0C    # 9DoF Sensor Fusion mode

# Register Data Addresses
BNO055_GYRO_DATA_X_LSB = 0x14  # 6 bytes
BNO055_QUATERNION_DATA_W_LSB = 0x20  # 8 bytes
BNO055_LINEAR_ACCEL_DATA_X_LSB = 0x28  # 6 bytes (Gravity removed fused accel)


class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_imu_node')

        # Declare parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', BNO055_ADDRESS)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)  # 50 Hz

        self.bus_num = self.get_parameter('i2c_bus').value
        self.addr = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        pub_rate = self.get_parameter('publish_rate').value

        # Publisher setup
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)

        # Compute fixed orientation mounting transformation matrix
        # Mounting condition: Inverted (180 deg around X-axis) + Clockwise 180 deg around Z-axis
        r_x_inv = R.from_euler('x', 180, degrees=True)
        r_z_cw = R.from_euler('z', -180, degrees=True)
        # Combined mounting rotation matrix (Sensor to Link transform)
        self.R_mount = r_z_cw * r_x_inv

        # Initialize Hardware
        self.init_bno055()

        # Timer setup (50 Hz -> 0.02s period)
        timer_period = 1.0 / pub_rate
        self.timer = self.create_timer(timer_period, self.publish_imu)
        self.get_logger().info(f'BNO055 Node initialized. Publishing on /imu at {pub_rate} Hz.')

    def init_bno055(self):
        """Initializes BNO055 into NDOF fusion mode via smbus2."""
        try:
            self.bus = smbus2.SMBus(self.bus_num)

            # Check Chip ID (Should be 0xA0)
            chip_id = self.bus.read_byte_data(self.addr, BNO055_CHIP_ID_REG)
            if chip_id != 0xA0:
                self.get_logger().error(f'Failed to find BNO055! Expected 0xA0, got {hex(chip_id)}')
                return

            # Switch to CONFIG mode to configure settings
            self.bus.write_byte_data(self.addr, BNO055_OPR_MODE_REG, OPERATION_MODE_CONFIG)
            rclpy.spin_once(self, timeout_sec=0.05)

            # Reset System & set normal power mode
            self.bus.write_byte_data(self.addr, BNO055_PWR_MODE_REG, 0x00)

            # Configure Units: Android orientation, Celsius, Degrees, dps, m/s^2
            self.bus.write_byte_data(self.addr, BNO055_UNIT_SEL_REG, 0x00)

            # Switch to NDOF (Nine Degrees of Freedom) Fused Mode
            self.bus.write_byte_data(self.addr, BNO055_OPR_MODE_REG, OPERATION_MODE_NDOF)
            rclpy.spin_once(self, timeout_sec=0.05)

            self.get_logger().info('BNO055 successfully configured in NDOF mode.')
        except Exception as e:
            self.get_logger().error(f'I2C Communication Error during initialization: {e}')

    def publish_imu(self):
        try:
            # 1. Read Fused Quaternion (8 bytes: W, X, Y, Z)
            quat_bytes = self.bus.read_i2c_block_data(self.addr, BNO055_QUATERNION_DATA_W_LSB, 8)
            w_raw, x_raw, y_raw, z_raw = struct.unpack('<hhhh', bytes(quat_bytes))
            
            # 1 LSB = 1/16384
            scale_q = 1.0 / 16384.0
            q_sensor = R.from_quat([x_raw * scale_q, y_raw * scale_q, z_raw * scale_q, w_raw * scale_q])

            # 2. Read Fused Angular Velocity / Gyro (6 bytes: X, Y, Z)
            gyro_bytes = self.bus.read_i2c_block_data(self.addr, BNO055_GYRO_DATA_X_LSB, 6)
            gx_raw, gy_raw, gz_raw = struct.unpack('<hhh', bytes(gyro_bytes))
            # 1 LSB = 16 dps -> convert to rad/s
            scale_g = (1.0 / 16.0) * (math.pi / 180.0)
            gyro_sensor = [gx_raw * scale_g, gy_raw * scale_g, gz_raw * scale_g]

            # 3. Read Fused Linear Acceleration (Gravity removed) (6 bytes: X, Y, Z)
            accel_bytes = self.bus.read_i2c_block_data(self.addr, BNO055_LINEAR_ACCEL_DATA_X_LSB, 6)
            ax_raw, ay_raw, az_raw = struct.unpack('<hhh', bytes(accel_bytes))
            # 1 LSB = 100 LSB / (m/s^2)
            scale_a = 1.0 / 100.0
            accel_sensor = [ax_raw * scale_a, ay_raw * scale_a, az_raw * scale_a]

            # -------------------------------------------------------------
            # Apply Frame Transformation (Sensor Frame -> Mounted imu_link)
            # -------------------------------------------------------------
            # Transformed Orientation Quaternion
            q_link = self.R_mount * q_sensor
            qx, qy, qz, qw = q_link.as_quat()

            # Transformed Vector Quantities (Angular Velocity and Linear Accel)
            gyro_link = self.R_mount.apply(gyro_sensor)
            accel_link = self.R_mount.apply(accel_sensor)

            # Construct ROS 2 IMU Message
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            msg.angular_velocity = Vector3(x=gyro_link[0], y=gyro_link[1], z=gyro_link[2])
            msg.linear_acceleration = Vector3(x=accel_link[0], y=accel_link[1], z=accel_link[2])

            # Set orientation & sensor covariance (-1 if unknown / not calculated)
            msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
            msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Failed to read or publish IMU data: {e}')


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