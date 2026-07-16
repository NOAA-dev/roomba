#!/usr/bin/env python3
"""
data_logger_node.py

Subscribes to:
  - /validated_map     (custom_interfaces/msg/Validatedmap)
  - /discarded_map     (custom_interfaces/msg/Validatedmap)
  - /odometry/filtered (nav_msgs/msg/Odometry)

Writes three separate CSVs under ~/roomba (all sharing one run timestamp so
they're easy to correlate):

  1. odom_log_<stamp>.csv
     Every single /odometry/filtered message, logged as it arrives.

  2. validated_map_log_<stamp>.csv
     One summary row per Validatedmap event (map_score, fragmentation,
     drift, etc.) plus the odometry sample cached at that instant. Same
     shape as before.

  3. map_data_log_<stamp>.csv
     One row per Validatedmap event containing the FULL occupancy grid
     (flattened, space-separated, in the map's own row-major order) along
     with its resolution/origin/dimensions and the odometry sample at that
     instant, so the raw grid can be reconstructed and overlaid on the
     robot pose later.

All three share a "counter" + "source" pair so rows can be joined across
files if needed.
"""

import os
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry
from custom_interfaces.msg import Validatedmap
from tf_transformations import euler_from_quaternion


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__("data_logger_node")

        self.declare_parameter("output_dir", os.path.expanduser("~/roomba/collected_data"))
        output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        os.makedirs(output_dir, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.odom_csv_path = os.path.join(output_dir, f"odom_log_{stamp}.csv")
        self.summary_csv_path = os.path.join(output_dir, f"validated_map_log_{stamp}.csv")
        self.map_data_csv_path = os.path.join(output_dir, f"map_data_log_{stamp}.csv")

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

        self.get_logger().info(f"Logging odometry to:       {self.odom_csv_path}")
        self.get_logger().info(f"Logging map summaries to:  {self.summary_csv_path}")
        self.get_logger().info(f"Logging full map grids to: {self.map_data_csv_path}")

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
        for f in (self._odom_file, self._summary_file, self._map_data_file):
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