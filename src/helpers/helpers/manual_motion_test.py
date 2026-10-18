#!/usr/bin/env python3
"""Odometry-feedback terminal controls for manual hardware testing."""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class ManualMotionTest(Node):
    """Read terminal commands and drive through the existing cmd_relay."""

    def __init__(self):
        super().__init__('manual_motion_test')
        self.declare_parameter('linear_speed_m_s', 0.10)
        self.declare_parameter('angular_speed_rad_s', 0.50)
        self.declare_parameter('max_linear_speed_m_s', 0.25)
        self.declare_parameter('max_angular_speed_rad_s', 2.08)
        self.declare_parameter('odom_timeout_sec', 1.0)
        self.declare_parameter('startup_odom_wait_sec', 5.0)
        self.declare_parameter('max_motion_timeout_sec', 45.0)
        self.declare_parameter('max_velocity_duration_sec', 60.0)
        self.linear_speed = float(self.get_parameter('linear_speed_m_s').value)
        self.angular_speed = float(self.get_parameter('angular_speed_rad_s').value)
        self.max_linear = float(self.get_parameter('max_linear_speed_m_s').value)
        self.max_angular = float(self.get_parameter('max_angular_speed_rad_s').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        self.odom_wait = float(self.get_parameter('startup_odom_wait_sec').value)
        self.max_motion_timeout = float(self.get_parameter('max_motion_timeout_sec').value)
        self.max_velocity_duration = float(self.get_parameter('max_velocity_duration_sec').value)

        self._lock = threading.Lock()
        self._odom = None
        self._odom_received_at = None
        self._yaw_unwrapped = None
        self._last_yaw = None
        self._latest_transform = None
        self._quit = False
        self._cancel_motion = threading.Event()
        self._motion_thread = None
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self._odom_sub = self.create_subscription(Odometry, '/odometry', self._on_odom, 10)
        # Read-only cross-checks for the report (no effect on control).
        self._imu_yaw = None
        self._imu_last = None
        self._ekf = None
        self._ekf_last = None
        self.create_subscription(Imu, '/imu', self._on_imu, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_ekf, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_timer = self.create_timer(0.05, self._publish_odom_tf)

    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _yaw_of(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_imu(self, msg):
        yaw = self._yaw_of(msg.orientation)
        with self._lock:
            self._imu_yaw = yaw if self._imu_last is None else \
                self._imu_yaw + self._wrap(yaw - self._imu_last)
            self._imu_last = yaw

    def _on_ekf(self, msg):
        yaw = self._yaw_of(msg.pose.pose.orientation)
        with self._lock:
            unwrapped = yaw if self._ekf is None else self._ekf[2] + self._wrap(yaw - self._ekf_last)
            self._ekf_last = yaw
            self._ekf = (msg.pose.pose.position.x, msg.pose.pose.position.y, unwrapped)

    def _aux_snapshot(self):
        with self._lock:
            return self._imu_yaw, self._ekf

    def _settle(self):
        # Wait (max 3 s) for wheel odometry to report standstill for 0.3 s.
        deadline = time.monotonic() + 3.0
        still_since = None
        while time.monotonic() < deadline:
            odom, _ = self._snapshot()
            now = time.monotonic()
            if odom is not None and abs(odom[4]) < 0.002 and abs(odom[5]) < 0.01:
                still_since = still_since or now
                if now - still_since >= 0.3:
                    return
            else:
                still_since = None
            time.sleep(0.05)

    def _on_odom(self, msg):
        yaw = self._yaw_of(msg.pose.pose.orientation)
        with self._lock:
            if self._last_yaw is None:
                self._yaw_unwrapped = yaw
            else:
                self._yaw_unwrapped += self._wrap(yaw - self._last_yaw)
            self._last_yaw = yaw
            self._odom = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                          yaw, self._yaw_unwrapped, msg.twist.twist.linear.x,
                          msg.twist.twist.angular.z)
            self._odom_received_at = time.monotonic()
            transform = TransformStamped()
            transform.header = msg.header
            transform.header.frame_id = msg.header.frame_id or 'odom'
            transform.child_frame_id = msg.child_frame_id or 'roomba'
            transform.transform.translation.x = msg.pose.pose.position.x
            transform.transform.translation.y = msg.pose.pose.position.y
            transform.transform.translation.z = msg.pose.pose.position.z
            transform.transform.rotation = msg.pose.pose.orientation
            self._latest_transform = transform

    def _publish_odom_tf(self):
        with self._lock:
            transform = self._latest_transform
        if transform is not None:
            transform.header.stamp = self.get_clock().now().to_msg()
            self._tf_broadcaster.sendTransform(transform)

    def _snapshot(self):
        with self._lock:
            return self._odom, self._odom_received_at

    def _wait_for_odom(self):
        deadline = time.monotonic() + self.odom_wait
        while time.monotonic() < deadline:
            if self._cancel_motion.is_set():
                raise RuntimeError('stopped while waiting for fresh /odometry')
            odom, received = self._snapshot()
            if odom is not None and received is not None and time.monotonic() - received <= self.odom_timeout:
                return odom
            time.sleep(0.05)
        raise RuntimeError('no fresh /odometry received; check cmd_relay and wheel joint_states')

    def _publish_velocity(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._cmd_pub.publish(msg)

    def _stop(self):
        # Repeat to ensure the relay receives a stop even during transient scheduling delays.
        for _ in range(3):
            self._publish_velocity()
            time.sleep(0.02)

    @staticmethod
    def _print_measurement(kind, requested, measured, unit):
        error = measured - requested
        print(f'{kind}: requested {requested:.3f} {unit}, measured {measured:.3f} {unit}, '
              f'error {error:+.3f} {unit}', flush=True)

    @staticmethod
    def _print_settled(kind, unit, start, stop_odom, final_odom, imu0, imu1, ekf0, ekf1):
        k = 180.0 / math.pi if kind == 'turn' else 100.0

        def along(pose, ref):
            return (pose[0] - ref[0]) * math.cos(ref[2]) + (pose[1] - ref[1]) * math.sin(ref[2])

        def main(pose, ref):  # distance (straight) or unwrapped yaw change (turn)
            return (pose[3] - ref[3] if kind == 'turn' else along(pose, ref)) * k

        if stop_odom is not None and final_odom is not None:
            print(f'  odom at stop cmd: {main(stop_odom, start):.3f} {unit} | '
                  f'odom settled: {main(final_odom, start):.3f} {unit}', flush=True)
        heading = []
        if final_odom is not None:
            heading.append(f'odom {math.degrees(final_odom[3] - start[3]):+.2f}')
        if imu0 is not None and imu1 is not None:
            heading.append(f'IMU {math.degrees(imu1 - imu0):+.2f}')
        if ekf0 is not None and ekf1 is not None:
            heading.append(f'EKF {math.degrees(ekf1[2] - ekf0[2]):+.2f}')
        print('  heading change (deg): ' + (' | '.join(heading) or 'n/a'), flush=True)
        if kind == 'straight' and ekf0 is not None and ekf1 is not None:
            print(f'  EKF settled distance: {along(ekf1, ekf0) * k:.3f} {unit}', flush=True)

    def _closed_loop(self, kind, target, speed):
        start = self._wait_for_odom()
        start_time = time.monotonic()
        timeout = min(self.max_motion_timeout, max(5.0, abs(target) / max(abs(speed), 0.02) * 3.0 + 5.0))
        initial_yaw_unwrapped = start[3]
        imu0, ekf0 = self._aux_snapshot()
        measured = 0.0
        failure = None
        try:
            while True:
                if self._cancel_motion.is_set():
                    failure = f'{kind} stopped by operator'
                    break
                now = time.monotonic()
                if now - start_time > timeout:
                    failure = f'{kind} timed out after {timeout:.1f} s'
                    break
                odom, received = self._snapshot()
                if odom is None or received is None or now - received > self.odom_timeout:
                    failure = f'{kind} aborted: /odometry is stale (>{self.odom_timeout:.1f} s)'
                    break
                if kind == 'straight':
                    dx, dy = odom[0] - start[0], odom[1] - start[1]
                    measured = dx * math.cos(start[2]) + dy * math.sin(start[2])
                    done = measured >= target if target >= 0 else measured <= target
                    linear, angular = math.copysign(abs(speed), target), 0.0
                else:
                    measured = odom[3] - initial_yaw_unwrapped
                    done = measured >= target if target >= 0 else measured <= target
                    linear, angular = 0.0, math.copysign(abs(speed), target)
                if done:
                    break
                self._publish_velocity(linear, angular)
                time.sleep(0.05)
        finally:
            self._stop()
        stop_odom, _ = self._snapshot()
        if failure is None:
            self._settle()
        final_odom, _ = self._snapshot()
        imu1, ekf1 = self._aux_snapshot()
        if kind == 'turn':
            target, measured = math.degrees(target), math.degrees(measured)
            unit = 'deg'
        else:
            target, measured = target * 100.0, measured * 100.0
            unit = 'cm'
        self._print_measurement(kind, target, measured, unit)
        self._print_settled(kind, unit, start, stop_odom, final_odom, imu0, imu1, ekf0, ekf1)
        if failure:
            print(failure, flush=True)

    def _velocity_for(self, linear, angular, duration):
        if not all(math.isfinite(v) for v in (linear, angular, duration)):
            raise ValueError('velocity and duration must be finite numbers')
        if abs(linear) > self.max_linear or abs(angular) > self.max_angular:
            raise ValueError(f'speed limit exceeded (|linear| <= {self.max_linear:.3f} m/s, '
                             f'|angular| <= {self.max_angular:.3f} rad/s)')
        if duration <= 0.0 or duration > self.max_velocity_duration:
            raise ValueError(f'duration must be > 0 and <= {self.max_velocity_duration:.1f} s')
        self._wait_for_odom()
        end = time.monotonic() + duration
        try:
            while time.monotonic() < end and not self._cancel_motion.is_set():
                _, received = self._snapshot()
                if received is None or time.monotonic() - received > self.odom_timeout:
                    print(f'velocity aborted: /odometry is stale (>{self.odom_timeout:.1f} s)', flush=True)
                    break
                self._publish_velocity(linear, angular)
                time.sleep(0.05)
        finally:
            self._stop()

    def _show_odom(self):
        odom, received = self._snapshot()
        if odom is None:
            print('No /odometry received yet.', flush=True)
            return
        age = time.monotonic() - received
        print(f'/odometry age {age:.2f} s | x={odom[0]:.3f} m y={odom[1]:.3f} m '
              f'yaw={math.degrees(odom[2]):.1f} deg | v={odom[4]:.3f} m/s '
              f'w={odom[5]:.3f} rad/s', flush=True)

    @staticmethod
    def _help():
        print('Commands:\n'
              '  straight <cm>                         drive signed distance\n'
              '  turn <degrees>                        rotate signed angle\n'
              '  velocity <linear_m_s> <angular_rad_s> <duration_s>\n'
              '  stop                                  stop now\n'
              '  odom                                  print latest odometry\n'
              '  help                                  show commands\n'
              '  quit                                  stop and exit', flush=True)

    def _start_motion(self, callback, *args):
        if self._motion_thread is not None and self._motion_thread.is_alive():
            print('A motion is already active; type stop first.', flush=True)
            return
        self._cancel_motion.clear()
        def run():
            try:
                callback(*args)
            except (ValueError, RuntimeError) as exc:
                print(f'Command rejected: {exc}', flush=True)
        self._motion_thread = threading.Thread(target=run, daemon=True)
        self._motion_thread.start()

    def run_terminal(self):
        print('Manual hardware motion test. Keep the robot in a clear area; type help for commands.', flush=True)
        self._help()
        try:
            while not self._quit:
                try:
                    line = input('roomba> ').strip()
                except EOFError:
                    break
                if not line:
                    continue
                fields = line.split()
                command = fields[0].lower()
                try:
                    if command == 'help' and len(fields) == 1:
                        self._help()
                    elif command == 'stop' and len(fields) == 1:
                        self._cancel_motion.set()
                        if self._motion_thread is not None:
                            self._motion_thread.join(timeout=1.0)
                        self._stop()
                        print('Stopped.', flush=True)
                    elif command == 'odom' and len(fields) == 1:
                        self._show_odom()
                    elif command == 'quit' and len(fields) == 1:
                        self._cancel_motion.set()
                        if self._motion_thread is not None:
                            self._motion_thread.join(timeout=1.0)
                        self._stop()
                        self._quit = True
                    elif command == 'straight' and len(fields) == 2:
                        cm = float(fields[1])
                        if not math.isfinite(cm) or cm == 0.0:
                            raise ValueError('distance must be a finite non-zero number of centimeters')
                        self._start_motion(self._closed_loop, 'straight', cm / 100.0, self.linear_speed)
                    elif command == 'turn' and len(fields) == 2:
                        degrees = float(fields[1])
                        if not math.isfinite(degrees) or degrees == 0.0:
                            raise ValueError('angle must be a finite non-zero number of degrees')
                        self._start_motion(self._closed_loop, 'turn', math.radians(degrees), self.angular_speed)
                    elif command == 'velocity' and len(fields) == 4:
                        values = tuple(float(v) for v in fields[1:])
                        self._start_motion(self._velocity_for, *values)
                    else:
                        print('Invalid command or argument count. Type help.', flush=True)
                except (ValueError, RuntimeError) as exc:
                    print(f'Command rejected: {exc}', flush=True)
        finally:
            self._cancel_motion.set()
            if self._motion_thread is not None:
                self._motion_thread.join(timeout=1.0)
            self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = ManualMotionTest()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run_terminal()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
