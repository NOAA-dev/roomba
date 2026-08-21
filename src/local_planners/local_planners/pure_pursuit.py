#!/usr/bin/env python3
import rclpy
import numpy as np
import math
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration
from nav_msgs.msg import Path
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from custom_interfaces.msg import NodeEnableStates


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # sole authority for this is manager_node — default OFF
        self.enabled = False

        # ---- robot pose (from TF) ----
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.last_tf_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.02, self.check_tf)

        # ---- path storage ----
        self.path_x = []
        self.path_y = []
        self.path_theta = []
        self.prev_header_stamp = None
        self.last_nearest_index = 0
        self.path_sub = self.create_subscription(Path, "/path", self.path_callback, path_qos)

        # ---- tunable parameters ----
        self.declare_parameter("lookahead_distance", 0.5)
        self.declare_parameter("min_lookahead", 0.3)
        self.declare_parameter("max_lookahead", 1.0)
        self.declare_parameter("k_lookahead", 0.3)
        self.declare_parameter("base_velocity", 0.5)
        self.declare_parameter("min_velocity", 0.1)
        self.declare_parameter("max_angular_vel", 1.3)
        self.declare_parameter("tf_timeout_sec", 0.5)
        self.declare_parameter("avoid_start_distance", 0.8)
        self.declare_parameter("avoid_stop_distance", 0.3)
        self.declare_parameter("clear_default", 10.0)

        self.lookahead_distance = self.get_parameter("lookahead_distance").value
        self.min_lookahead = self.get_parameter("min_lookahead").value
        self.max_lookahead = self.get_parameter("max_lookahead").value
        self.k_lookahead = self.get_parameter("k_lookahead").value
        self.base_velocity = self.get_parameter("base_velocity").value
        self.min_velocity = self.get_parameter("min_velocity").value
        self.max_angular_vel = self.get_parameter("max_angular_vel").value
        self.tf_timeout_sec = self.get_parameter("tf_timeout_sec").value
        self.avoid_start_distance = self.get_parameter("avoid_start_distance").value
        self.avoid_stop_distance = self.get_parameter("avoid_stop_distance").value
        self.clear_default = self.get_parameter("clear_default").value

        self.publisher_ = self.create_publisher(Twist, "/autonomous_cmd_vel", 10)
        self.timer_ = self.create_timer(0.02, self.pure_pursuit)  # 50 Hz

        self.front_clearance = self.clear_default
        self.right_clearance = self.clear_default
        self.left_clearance = self.clear_default
        self.subscriber_ = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)

        enable_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enable_sub = self.create_subscription(
            NodeEnableStates, "/manager/enable_states", self.enable_callback, enable_qos)

    def enable_callback(self, msg: NodeEnableStates):
        self.enabled = msg.pure_pursuit

    # ------------------------------------------------------------------
    def check_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "roomba", rclpy.time.Time(), timeout=Duration(seconds=0.2))
            self.x_pos = transform.transform.translation.x
            self.y_pos = transform.transform.translation.y
            q = transform.transform.rotation
            _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.last_tf_time = self.get_clock().now()
        except TransformException as ex:
            self.get_logger().warn(f"TF unavailable: {ex}", throttle_duration_sec=2.0)

    def tf_is_stale(self):
        if self.last_tf_time is None:
            return True
        age = (self.get_clock().now() - self.last_tf_time).nanoseconds / 1e9
        return age > self.tf_timeout_sec

    # ------------------------------------------------------------------
    def laser_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        ranges = msg.ranges

        front_range = []
        right_range = []
        left_range = []

        angle = angle_min
        front_angle = np.deg2rad(35)
        left_angle = np.deg2rad(120)
        right_angle = np.deg2rad(-120)
        for r in ranges:
            if math.isfinite(r):
                if right_angle <= angle <= -front_angle:
                    right_range.append(r)
                if -front_angle <= angle <= front_angle:
                    front_range.append(r)
                if front_angle <= angle <= left_angle:
                    left_range.append(r)
            angle += angle_inc

        self.front_clearance = min(front_range) if front_range else self.clear_default
        self.right_clearance = min(right_range) if right_range else self.clear_default
        self.left_clearance = min(left_range) if left_range else self.clear_default

    # ------------------------------------------------------------------
    def path_callback(self, msg: Path):
        if self.prev_header_stamp is None:
            self.prev_header_stamp = msg.header.stamp
        else:
            if self.prev_header_stamp == msg.header.stamp:
                return
            self.prev_header_stamp = msg.header.stamp

        self.path_x = []
        self.path_y = []
        self.path_theta = []
        self.last_nearest_index = 0

        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            q = pose_stamped.pose.orientation
            _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.path_x.append(x)
            self.path_y.append(y)
            self.path_theta.append(theta)

    # ------------------------------------------------------------------
    def find_nearest_index(self, x, y):
        search_start = self.last_nearest_index
        xs = np.array(self.path_x[search_start:])
        ys = np.array(self.path_y[search_start:])
        distances = np.hypot(xs - x, ys - y)
        nearest_index = search_start + int(np.argmin(distances))
        self.last_nearest_index = nearest_index
        return nearest_index

    def find_lookahead_point(self, x, y, nearest_index, L_d):
        for i in range(nearest_index, len(self.path_x)):
            dist = math.hypot(self.path_x[i] - x, self.path_y[i] - y)
            if dist >= L_d:
                return self.path_x[i], self.path_y[i], i
        return self.path_x[-1], self.path_y[-1], len(self.path_x) - 1

    # ------------------------------------------------------------------
    def compute_velocity(self, index):
        lookahead_step = 1
        n = len(self.path_x)

        remaining = n - 1 - index
        ramp_zone = 7
        if remaining <= 0:
            return 0.0

        if index + 3 * lookahead_step >= n:
            v = self.base_velocity * min(remaining / ramp_zone, 1.0)
            return max(v, self.min_velocity if remaining > 0 else 0.0)

        i1, i2, i3, i4 = index, index + lookahead_step, index + 2 * lookahead_step, index + 3 * lookahead_step
        d_theta1 = math.atan2(self.path_y[i2] - self.path_y[i1], self.path_x[i2] - self.path_x[i1])
        d_theta2 = math.atan2(self.path_y[i3] - self.path_y[i2], self.path_x[i3] - self.path_x[i2])
        d_theta3 = math.atan2(self.path_y[i4] - self.path_y[i3], self.path_x[i4] - self.path_x[i3])
        d1 = math.atan2(math.sin(d_theta2 - d_theta1), math.cos(d_theta2 - d_theta1))
        d2 = math.atan2(math.sin(d_theta3 - d_theta2), math.cos(d_theta3 - d_theta2))

        total_turning = abs(d1) + abs(d2)
        normalized_turn = min(total_turning / math.pi, 1.0)
        reduction = normalized_turn ** 2

        v = self.base_velocity - reduction * (self.base_velocity - self.min_velocity)
        v = max(v, self.min_velocity)

        if remaining < ramp_zone:
            v *= remaining / ramp_zone

        return v

    # ------------------------------------------------------------------
    def pure_pursuit(self):
        if not self.enabled:
            self.publisher_.publish(Twist())
            return

        if not self.path_x or not self.path_y:
            return

        if self.tf_is_stale():
            self.get_logger().warn("TF stale - holding robot", throttle_duration_sec=1.0)
            self.publisher_.publish(Twist())
            return

        nearest_index = self.find_nearest_index(self.x_pos, self.y_pos)
        self.v += (self.compute_velocity(nearest_index) - self.v) * 0.7

        L_d = np.clip(self.lookahead_distance + self.k_lookahead * self.v, self.min_lookahead, self.max_lookahead)

        goal_x, goal_y, goal_index = self.find_lookahead_point(self.x_pos, self.y_pos, nearest_index, L_d)

        dx = goal_x - self.x_pos
        dy = goal_y - self.y_pos
        x_r = math.cos(-self.yaw) * dx - math.sin(-self.yaw) * dy
        y_r = math.sin(-self.yaw) * dx + math.cos(-self.yaw) * dy

        L_d_actual = math.hypot(x_r, y_r)
        if L_d_actual < 1e-3:
            omega = 0.0
        else:
            curvature = 2.0 * y_r / (L_d_actual ** 2)
            omega = self.v * curvature

        omega = float(np.clip(omega, -self.max_angular_vel, self.max_angular_vel))

        if goal_index >= len(self.path_x) - 1:
            dist_to_end = math.hypot(self.path_x[-1] - self.x_pos, self.path_y[-1] - self.y_pos)
            if dist_to_end < 0.05:
                self.publisher_.publish(Twist())
                return

        # ---- continuous obstacle-avoidance blend ----
        v_scale = np.clip((self.front_clearance - self.avoid_stop_distance)/max(self.avoid_start_distance - self.avoid_stop_distance, 1e-6), 0.0, 1.0)
        self.v *= v_scale

        nearest_clearance = min(self.front_clearance, self.left_clearance, self.right_clearance)
        if nearest_clearance < self.avoid_start_distance:
            urgency = np.clip(
                (self.avoid_start_distance - nearest_clearance) /
                max(self.avoid_start_distance - self.avoid_stop_distance, 1e-6),
                0.0, 1.0
            )
            clearance_error = self.left_clearance - self.right_clearance
            avoid_omega = urgency * np.clip(clearance_error, -1.0, 1.0) * self.max_angular_vel
            omega = (1 - urgency) * omega + urgency * avoid_omega

        omega = float(np.clip(omega, -self.max_angular_vel, self.max_angular_vel))

        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = omega
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()