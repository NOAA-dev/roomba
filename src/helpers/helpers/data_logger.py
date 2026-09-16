#!/usr/bin/env python3
"""
data_logger_node.py

Subscribes to:
  - /odometry/filtered              (nav_msgs/msg/Odometry)
  - /validated_map                  (custom_interfaces/msg/Validatedmap)
  - /discarded_map                  (custom_interfaces/msg/Validatedmap)
  - /joint_states                   (sensor_msgs/msg/JointState)
  - /simple_velocity_controller/commands (std_msgs/msg/Float64MultiArray)

and samples the map -> roomba transform off /tf (+ /tf_static) on its own timer.

Writes six separate CSVs under ~/roomba/collected_data (all sharing one run
timestamp so they're easy to correlate):

  1. odom_log_<stamp>.csv
     Every single /odometry/filtered message, logged as it arrives. This pose
     is in the odom frame: wheel-odometry + IMU fusion only, so it carries
     accumulated drift with no map-frame correction applied.

  2. validated_map_log_<stamp>.csv
     One summary row per Validatedmap event (map_score, fragmentation,
     drift, etc.) plus the odometry sample cached at that instant.

  3. map_data_log_<stamp>.csv
     One row per Validatedmap event containing the FULL occupancy grid
     (flattened, space-separated, in the map's own row-major order) along
     with its resolution/origin/dimensions and the odometry sample at that
     instant.

  4. joint_states_log_<stamp>.csv
     Every /joint_states message, logged as it arrives: per-wheel position
     (rad, cumulative since the last "Z" on real hardware) and velocity
     (rad/s) for base_left_wheel_joint / base_right_wheel_joint, matched by
     name (not index -- message order isn't guaranteed). This is the ESP32's
     telemetry on real hardware, or Gazebo's equivalent in sim.

  5. wheel_cmd_log_<stamp>.csv
     Every /simple_velocity_controller/commands message (the per-wheel
     velocity commands relay.py sends downstream), so commanded vs measured
     velocity can be compared against the joint_states log above.
     Float64MultiArray has no header/stamp, so ros_time_sec here is this
     node's own clock at receipt time, not a hardware timestamp.

  6. tf_log_<stamp>.csv
     The map -> roomba transform, sampled on a timer (tf_log_rate_hz) rather
     than per-message, since /tf carries every frame pair in the tree and the
     composed map->roomba chain only exists as a lookup. This IS the
     drift-corrected pose: map->odom is whatever SLAM's loop closures /
     relocalisations currently say, composed with odom->roomba. Plotting this
     against odom_log is exactly plotting the map->odom correction, which is
     why the two paths diverge on the same map.

All six share a "counter" + "source" pair (map logs) or are simply timestamped
(odom/joint_states/wheel_cmd/tf logs) so rows can be correlated across files by
ros_time_sec if needed.
"""

import os
import csv
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from rclpy.time import Time as RclpyTime

import tf2_ros
from tf2_ros import TransformException

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from custom_interfaces.msg import Validatedmap
from tf_transformations import euler_from_quaternion


# Fixed by ros2_hardwareinterface.xacro -- not expected to change.
LEFT_JOINT = "base_left_wheel_joint"
RIGHT_JOINT = "base_right_wheel_joint"


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__("data_logger_node")

        self.declare_parameter("output_dir", os.path.expanduser("~/roomba/collected_data"))
        output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        os.makedirs(output_dir, exist_ok=True)

        # Passed straight through from the launch file's `sim` arg -- lets
        # visualize_data_logs.py tell whether the telemetry-interval
        # reference lines (calibrated to the real ESP32's SerialLink timing)
        # are meaningful for this particular run, or whether it's a Gazebo
        # run publishing at the plain control-loop rate instead.
        self.declare_parameter("sim", True)
        self.sim = self.get_parameter("sim").get_parameter_value().bool_value

        # map -> roomba sampling. Parameterised rather than hardcoded so the
        # frames can be overridden if the URDF's base link is ever renamed.
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "roomba")
        self.declare_parameter("tf_log_rate_hz", 20.0)
        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        tf_rate = self.get_parameter("tf_log_rate_hz").get_parameter_value().double_value

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.odom_csv_path = os.path.join(output_dir, f"odom_log_{stamp}.csv")
        self.summary_csv_path = os.path.join(output_dir, f"validated_map_log_{stamp}.csv")
        self.map_data_csv_path = os.path.join(output_dir, f"map_data_log_{stamp}.csv")
        self.joint_states_csv_path = os.path.join(output_dir, f"joint_states_log_{stamp}.csv")
        self.wheel_cmd_csv_path = os.path.join(output_dir, f"wheel_cmd_log_{stamp}.csv")
        self.tf_csv_path = os.path.join(output_dir, f"tf_log_{stamp}.csv")

        self._odom_header = [
            "wall_time",
            "ros_time_sec",
            "x",
            "y",
            "z",
            "yaw",
            "qx",
            "qy",
            "qz",
            "qw",
            "lin_vel_x",
            "lin_vel_y",
            "lin_vel_z",
            "ang_vel_x",
            "ang_vel_y",
            "ang_vel_z",
        ]

        self._summary_header = [
            "wall_time",
            "ros_time_sec",
            "source",          # "valid" or "invalid"
            "counter",
            "map_score",
            "valid",
            "fragmented",
            "fragments",
            "is_dominant",
            "reachable_ratio",
            "drift_detected",
            "drift_score",
            "drift_distance",
            "map_width",
            "map_height",
            "map_resolution",
            "odom_time_sec",
            "odom_x",
            "odom_y",
            "odom_yaw",
            "odom_lin_vel_x",
            "odom_lin_vel_y",
            "odom_ang_vel_z",
        ]

        self._map_data_header = [
            "wall_time",
            "ros_time_sec",
            "source",          # "valid" or "invalid"
            "counter",
            "map_width",
            "map_height",
            "map_resolution",
            "origin_x",
            "origin_y",
            "origin_z",
            "origin_qx",
            "origin_qy",
            "origin_qz",
            "origin_qw",
            "odom_time_sec",
            "odom_x",
            "odom_y",
            "odom_yaw",
            "grid_data",       # space-separated int8 values, row-major (width x height)
        ]

        self._joint_states_header = [
            "wall_time",
            "ros_time_sec",
            "left_pos_rad",
            "left_vel_rad_s",
            "right_pos_rad",
            "right_vel_rad_s",
        ]

        self._wheel_cmd_header = [
            "wall_time",
            "ros_time_sec",
            "left_cmd_vel",
            "right_cmd_vel",
        ]

        # Same column names as the odom log for x/y/yaw so the plotting code
        # can treat the two frames interchangeably.
        self._tf_header = [
            "wall_time",
            "ros_time_sec",
            "parent_frame",
            "child_frame",
            "x",
            "y",
            "z",
            "yaw",
            "qx",
            "qy",
            "qz",
            "qw",
        ]

        self._odom_file = open(self.odom_csv_path, mode="w", newline="")
        self._odom_writer = csv.writer(self._odom_file)
        self._odom_writer.writerow(self._odom_header)
        self._odom_file.flush()

        self._summary_file = open(self.summary_csv_path, mode="w", newline="")
        self._summary_writer = csv.writer(self._summary_file)
        self._summary_writer.writerow(self._summary_header)
        self._summary_file.flush()

        self._map_data_file = open(self.map_data_csv_path, mode="w", newline="")
        self._map_data_writer = csv.writer(self._map_data_file)
        self._map_data_writer.writerow(self._map_data_header)
        self._map_data_file.flush()

        self._joint_states_file = open(self.joint_states_csv_path, mode="w", newline="")
        self._joint_states_writer = csv.writer(self._joint_states_file)
        self._joint_states_writer.writerow(self._joint_states_header)
        self._joint_states_file.flush()

        self._wheel_cmd_file = open(self.wheel_cmd_csv_path, mode="w", newline="")
        self._wheel_cmd_writer = csv.writer(self._wheel_cmd_file)
        self._wheel_cmd_writer.writerow(self._wheel_cmd_header)
        self._wheel_cmd_file.flush()

        self._tf_file = open(self.tf_csv_path, mode="w", newline="")
        self._tf_writer = csv.writer(self._tf_file)
        self._tf_writer.writerow(self._tf_header)
        self._tf_file.flush()

        self.run_meta_path = os.path.join(output_dir, f"run_meta_{stamp}.json")
        with open(self.run_meta_path, "w") as f:
            json.dump(
                {
                    "stamp": stamp,
                    "sim": self.sim,
                    "map_frame": self.map_frame,
                    "base_frame": self.base_frame,
                    "tf_log_rate_hz": tf_rate,
                },
                f,
            )

        self.latest_odom = None  # cached latest Odometry message, used to tag map events

        # MAPValidatorNode's publish_map() timer republishes the last good/bad
        # Validatedmap every 1s regardless of whether validate_map() actually
        # produced a new result. msg.counter only increments on a genuine new
        # validation, so we dedupe on it to avoid logging the same map repeatedly.
        self.last_valid_counter = None
        self.last_invalid_counter = None

        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Validatedmap publishers use TRANSIENT_LOCAL depth=1 (see MAPValidatorNode);
        # match it here so we don't miss a late-joined connection's last message.
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, odom_qos
        )
        self.valid_sub = self.create_subscription(
            Validatedmap, "/validated_map", self.valid_map_callback, map_qos
        )
        self.invalid_sub = self.create_subscription(
            Validatedmap, "/discarded_map", self.invalid_map_callback, map_qos
        )
        # Default depth-10 QoS to match joint_state_broadcaster's publisher and
        # relay.py's own subscription to this same topic.
        self.joint_states_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )
        # Matches relay.py's publisher QoS for this topic.
        self.wheel_cmd_sub = self.create_subscription(
            Float64MultiArray,
            "/simple_velocity_controller/commands",
            self.wheel_cmd_callback,
            10,
        )

        # TransformListener handles /tf and /tf_static subscriptions itself.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._last_tf_stamp = None          # dedupe: don't log a stale lookup twice
        self._tf_warned = False             # only warn once per outage
        self.tf_timer = self.create_timer(1.0 / tf_rate, self.tf_timer_callback)

        self.get_logger().info(f"Logging odometry to:        {self.odom_csv_path}")
        self.get_logger().info(f"Logging map summaries to:   {self.summary_csv_path}")
        self.get_logger().info(f"Logging full map grids to:  {self.map_data_csv_path}")
        self.get_logger().info(f"Logging joint states to:    {self.joint_states_csv_path}")
        self.get_logger().info(f"Logging wheel commands to:  {self.wheel_cmd_csv_path}")
        self.get_logger().info(
            f"Logging {self.map_frame}->{self.base_frame} TF to: {self.tf_csv_path} "
            f"(sampled at {tf_rate:g} Hz)"
        )
        self.get_logger().info(f"Run mode: {'sim' if self.sim else 'real hardware'} (see {self.run_meta_path})")

    # ------------------------------------------------------------------
    # Odometry: log every message, and cache the latest for map tagging
    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

        ros_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        row = [
            datetime.now().isoformat(),
            ros_time_sec,
            x,
            y,
            z,
            yaw,
            q.x,
            q.y,
            q.z,
            q.w,
            lin.x,
            lin.y,
            lin.z,
            ang.x,
            ang.y,
            ang.z,
        ]

        self._odom_writer.writerow(row)
        self._odom_file.flush()

    # ------------------------------------------------------------------
    # map -> roomba TF: the drift-corrected pose, sampled on a timer
    # ------------------------------------------------------------------
    def tf_timer_callback(self):
        try:
            # RclpyTime() (i.e. time zero) means "latest available" rather than
            # a specific stamp -- avoids ExtrapolationException when SLAM's
            # map->odom lags the odom->roomba chain, which it usually does.
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, RclpyTime()
            )
        except TransformException as e:
            if not self._tf_warned:
                self.get_logger().warn(
                    f"{self.map_frame}->{self.base_frame} not available yet ({e}); "
                    "will keep trying silently"
                )
                self._tf_warned = True
            return

        self._tf_warned = False

        ros_time_sec = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        # The timer runs faster than the transform updates, so the same
        # transform gets looked up repeatedly -- only log genuinely new ones.
        if self._last_tf_stamp is not None and ros_time_sec <= self._last_tf_stamp:
            return
        self._last_tf_stamp = ros_time_sec

        t = tf.transform.translation
        q = tf.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        row = [
            datetime.now().isoformat(),
            ros_time_sec,
            tf.header.frame_id,
            tf.child_frame_id,
            t.x,
            t.y,
            t.z,
            yaw,
            q.x,
            q.y,
            q.z,
            q.w,
        ]

        self._tf_writer.writerow(row)
        self._tf_file.flush()

    # ------------------------------------------------------------------
    # Joint states: raw per-wheel position/velocity, matched by name
    # ------------------------------------------------------------------
    def joint_states_callback(self, msg: JointState):
        try:
            idx_l = msg.name.index(LEFT_JOINT)
            idx_r = msg.name.index(RIGHT_JOINT)
        except ValueError:
            # Names don't match what ros2_hardwareinterface.xacro declares --
            # skip rather than log misaligned/guessed data.
            return

        ros_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        row = [
            datetime.now().isoformat(),
            ros_time_sec,
            msg.position[idx_l],
            msg.velocity[idx_l],
            msg.position[idx_r],
            msg.velocity[idx_r],
        ]

        self._joint_states_writer.writerow(row)
        self._joint_states_file.flush()

    # ------------------------------------------------------------------
    # Wheel velocity commands: what relay.py actually sent downstream
    # ------------------------------------------------------------------
    def wheel_cmd_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 2:
            return  # malformed, shouldn't happen given relay.py always sends [left, right]

        row = [
            datetime.now().isoformat(),
            self.get_clock().now().nanoseconds * 1e-9,  # no header on this msg type
            msg.data[0],
            msg.data[1],
        ]

        self._wheel_cmd_writer.writerow(row)
        self._wheel_cmd_file.flush()

    # ------------------------------------------------------------------
    # Validatedmap: one summary row + one full-grid row per event
    # ------------------------------------------------------------------
    def valid_map_callback(self, msg: Validatedmap):
        if msg.counter == self.last_valid_counter:
            return  # same message republished by MAPValidatorNode's 1s timer
        self.last_valid_counter = msg.counter

        self.log_summary_row(msg, source="valid")
        self.log_map_data_row(msg, source="valid")

    def invalid_map_callback(self, msg: Validatedmap):
        if msg.counter == self.last_invalid_counter:
            return  # same message republished by MAPValidatorNode's 1s timer
        self.last_invalid_counter = msg.counter

        self.log_summary_row(msg, source="invalid")
        self.log_map_data_row(msg, source="invalid")

    def _cached_odom_fields(self):
        """Returns (odom_time_sec, x, y, yaw) from the cached odom, or blanks."""
        if self.latest_odom is None:
            self.get_logger().warn(
                "No odometry received yet; tagging map event with blank odom fields"
            )
            return "", "", "", ""

        odom = self.latest_odom
        odom_time = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        odom_x = odom.pose.pose.position.x
        odom_y = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        _, _, odom_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return odom_time, odom_x, odom_y, odom_yaw

    def log_summary_row(self, msg: Validatedmap, source: str):
        odom_time, odom_x, odom_y, odom_yaw = self._cached_odom_fields()
        odom_vx = odom_vy = odom_wz = ""
        if self.latest_odom is not None:
            odom_vx = self.latest_odom.twist.twist.linear.x
            odom_vy = self.latest_odom.twist.twist.linear.y
            odom_wz = self.latest_odom.twist.twist.angular.z

        ros_time_sec = msg.validation_time.sec + msg.validation_time.nanosec * 1e-9

        row = [
            datetime.now().isoformat(),
            ros_time_sec,
            source,
            msg.counter,
            msg.map_score,
            msg.valid,
            msg.fragmented,
            msg.fragments,
            msg.is_dominant,
            msg.reachable_ratio,
            msg.drift_detected,
            msg.drift_score,
            msg.drift_distance,
            msg.map.info.width,
            msg.map.info.height,
            msg.map.info.resolution,
            odom_time,
            odom_x,
            odom_y,
            odom_yaw,
            odom_vx,
            odom_vy,
            odom_wz,
        ]

        self._summary_writer.writerow(row)
        self._summary_file.flush()

    def log_map_data_row(self, msg: Validatedmap, source: str):
        odom_time, odom_x, odom_y, odom_yaw = self._cached_odom_fields()

        info = msg.map.info
        origin = info.origin

        # msg.map.data is already row-major (row 0 first), int8 values in
        # {-1, 0..100}. Space-separated so it stays a single CSV field.
        grid_str = " ".join(str(v) for v in msg.map.data)

        ros_time_sec = msg.validation_time.sec + msg.validation_time.nanosec * 1e-9

        row = [
            datetime.now().isoformat(),
            ros_time_sec,
            source,
            msg.counter,
            info.width,
            info.height,
            info.resolution,
            origin.position.x,
            origin.position.y,
            origin.position.z,
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
            odom_time,
            odom_x,
            odom_y,
            odom_yaw,
            grid_str,
        ]

        self._map_data_writer.writerow(row)
        self._map_data_file.flush()

    def destroy_node(self):
        for f in (
            self._odom_file,
            self._summary_file,
            self._map_data_file,
            self._joint_states_file,
            self._wheel_cmd_file,
            self._tf_file,
        ):
            try:
                f.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()