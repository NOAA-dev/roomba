#!/usr/bin/env python3
import rclpy
import numpy as np
import math
from rclpy.node import Node
from scipy import ndimage
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion
from rclpy.duration import Duration
from custom_interfaces.msg import Validatedmap, NodeEnableStates
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from nav2_msgs.srv import SaveMap
from slam_toolbox.srv import SerializePoseGraph
import os
from datetime import datetime


class MAPValidatorNode(Node):
    def __init__(self):
        super().__init__("map_validator_")

        self.map_ = OccupancyGrid()
        self.map_ss = OccupancyGrid()
        self.send_to_validate = False

        self.x_pos = 0.0
        self.y_pos = 0.0

        self.good_map_ = Validatedmap()
        self.good_map_counter = 0
        self.bad_map_ = Validatedmap()
        self.bad_map_counter = 0

        self.enabled = True

        # a map->odom step bigger than this inside one tick is a SLAM
        # correction (loop closure / scan-match jump), not odometry drift
        self.declare_parameter("loop_jump_m", 0.10)
        self.declare_parameter("loop_jump_rad", 0.05)
        self.declare_parameter("saved_maps_dir", "~/roomba/saved_maps")
        self.loop_jump_m = self.get_parameter("loop_jump_m").value
        self.loop_jump_rad = self.get_parameter("loop_jump_rad").value
        self.saved_maps_dir = os.path.expanduser(self.get_parameter("saved_maps_dir").value)

        self.prev_map_odom = None      # (x, y, yaw) from the previous tick
        self.win_jump_m = 0.0          # max step since the last snapshot
        self.win_jump_rad = 0.0
        self.snap_jump_m = 0.0         # window values frozen at snapshot time
        self.snap_jump_rad = 0.0
        self.latest_valid = False      # result of the MOST RECENT validation

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_subscriber = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.good_map_pub_ = self.create_publisher(Validatedmap, "/validated_map", path_qos)
        self.bad_map_pub_ = self.create_publisher(Validatedmap, "/discarded_map", path_qos)

        self.timer_1 = self.create_timer(3.0, self.check_map)
        self.timer_2 = self.create_timer(0.5, self.validate_map)
        self.timer_3 = self.create_timer(1.0, self.publish_map)
        self.tf_timer = self.create_timer(0.1, self.check_tf)

        self.map_saved = False
        self.pose_graph_saved = False
        self.saving = False
        self.candidate_base = None

        enable_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enable_sub = self.create_subscription(
            NodeEnableStates, "/manager/enable_states", self.enable_callback, enable_qos)
        # saving is now gated: manager publishes this only after the user
        # confirmed (ros2 service call /manager/confirm_save ...)
        self.save_confirmed_sub = self.create_subscription(
            Bool, "/manager/save_confirmed", self.save_confirmed_callback, 10)
        self.candidate_pub = self.create_publisher(String, "/map_validator/candidate_saved", 10)

        self.save_map_client = self.create_client(SaveMap, "/map_saver_server/save_map")
        self.serialize_map_client = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")

    def enable_callback(self, msg: NodeEnableStates):
        self.enabled = msg.map_validator

    def save_confirmed_callback(self, msg: Bool):
        if not msg.data or self.saving:
            return

        if self.good_map_counter == 0 or not self.latest_valid:
            self.get_logger().error(
                "save refused: the most recent validation was not VALID (or no valid map yet)")
            return
        if self.snap_jump_m >= self.loop_jump_m or self.snap_jump_rad >= self.loop_jump_rad:
            self.get_logger().error(
                "save refused: map->odom was still jumping in the last window "
                f"({self.snap_jump_m:.3f} m, {self.snap_jump_rad:.3f} rad); wait for it to settle")
            return

        cand_dir = os.path.join(self.saved_maps_dir, "candidates")
        os.makedirs(cand_dir, exist_ok=True)
        self.candidate_base = os.path.join(
            cand_dir, "map_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
        self.saving = True
        self.map_saved = False
        self.pose_graph_saved = False
        self.get_logger().info(f"saving candidate map -> {self.candidate_base}")
        self._save_occupancy_grid(self.candidate_base)
        self._serialize_pose_graph(self.candidate_base)

    def _finish_save_if_done(self):
        if not self.saving:
            return
        if self.map_saved and self.pose_graph_saved:
            self.saving = False
            self.candidate_pub.publish(String(data=self.candidate_base))
            self.get_logger().info(f"candidate saved: {self.candidate_base}")

    def _abort_save(self):
        # a half-written candidate (yaml without posegraph, or the reverse)
        # is useless — remove it so it can never be picked up
        self.saving = False
        d, b = os.path.split(self.candidate_base)
        for f in os.listdir(d):
            if f.startswith(b + "."):
                try:
                    os.remove(os.path.join(d, f))
                except OSError:
                    pass
        self.get_logger().error("candidate save failed — partial files removed")

    def _save_occupancy_grid(self, map_path):
        if not self.save_map_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("map_saver_server/save_map service unavailable, cannot save map")
            self._abort_save()
            return

        req = SaveMap.Request()
        req.map_topic = "/map"
        req.map_url = map_path
        req.image_format = "pgm"
        req.map_mode = "trinary"
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65

        future = self.save_map_client.call_async(req)
        future.add_done_callback(self._on_save_map_done)

    def _on_save_map_done(self, future):
        try:
            result = future.result()
            if result.result:
                self.map_saved = True
                self.get_logger().info("map (yaml/pgm) saved successfully")
                self._finish_save_if_done()
            else:
                self.get_logger().error("map_saver_server reported save failure")
                self._abort_save()
        except Exception as e:
            self.get_logger().error(f"save_map call failed: {e}")
            self._abort_save()

    def _serialize_pose_graph(self, map_path):
        # saves slam_toolbox's full pose-graph (not just the flattened
        # occupancy grid) so a future run can resume mapping from exactly
        # where this session left off via /slam_toolbox/deserialize_map,
        # instead of starting SLAM from a blank slate every time
        if not self.serialize_map_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("slam_toolbox/serialize_map service unavailable, cannot save pose graph")
            self._abort_save()
            return

        req = SerializePoseGraph.Request()
        req.filename = map_path

        future = self.serialize_map_client.call_async(req)
        future.add_done_callback(self._on_serialize_done)

    def _on_serialize_done(self, future):
        try:
            result = future.result()
            if result.result == 0:
                self.pose_graph_saved = True
                self.get_logger().info("pose graph serialized successfully")
                self._finish_save_if_done()
            else:
                self.get_logger().error(f"pose graph serialization failed, code={result.result}")
                self._abort_save()
        except Exception as e:
            self.get_logger().error(f"serialize_map call failed: {e}")
            self._abort_save()

    def map_callback(self, msg: OccupancyGrid):
        if not self.enabled:
            return
        self.map_ = msg

    def world_to_map(self, x, y, map_msg: OccupancyGrid):
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        resolution = map_msg.info.resolution

        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)

        return map_x, map_y

    def check_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "roomba", rclpy.time.Time(), timeout=Duration(seconds=0.2))

            self.x_pos = transform.transform.translation.x
            self.y_pos = transform.transform.translation.y

        except TransformException as ex:
            self.get_logger().warn(f"TF unavailable: {ex}", throttle_duration_sec=2.0)

        self.track_map_odom()

    def track_map_odom(self):
        # map->odom only changes when SLAM applies a correction, so a step
        # here separates a loop closure from genuine drift
        try:
            t = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
        except TransformException:
            return
        q = t.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        cur = (t.transform.translation.x, t.transform.translation.y, yaw)

        if self.prev_map_odom is not None:
            step_m = math.hypot(cur[0] - self.prev_map_odom[0], cur[1] - self.prev_map_odom[1])
            dyaw = cur[2] - self.prev_map_odom[2]
            step_rad = abs(math.atan2(math.sin(dyaw), math.cos(dyaw)))
            self.win_jump_m = max(self.win_jump_m, step_m)
            self.win_jump_rad = max(self.win_jump_rad, step_rad)
            if step_m >= self.loop_jump_m or step_rad >= self.loop_jump_rad:
                self.get_logger().info(
                    f"map->odom step {step_m:.3f} m / {step_rad:.3f} rad (SLAM correction)")
        self.prev_map_odom = cur

    def _nearest_labeled_cell(self, labeled, row, col, search_radius):
        height, width = labeled.shape
        for r in range(1, search_radius + 1):
            r0, r1 = max(0, row - r), min(height, row + r + 1)
            c0, c1 = max(0, col - r), min(width, col + r + 1)
            window = labeled[r0:r1, c0:c1]
            nonzero = window[window > 0]
            if nonzero.size > 0:
                return int(nonzero[0])
        return None

    def check_robot_connectivity(self, labeled, num_components, free_clean, row, col):
        height, width = labeled.shape

        if not (0 <= row < height and 0 <= col < width):
            self.get_logger().warn("robot cell outside map bounds")
            return False, 0.0

        robot_label = labeled[row, col]
        if robot_label == 0:
            robot_label = self._nearest_labeled_cell(labeled, row, col, search_radius=5)
            if robot_label is None:
                self.get_logger().warn("no free cell found near robot position")
                return False, 0.0

        total_free = free_clean.sum()
        robot_component_size = np.count_nonzero(labeled == robot_label)
        reachable_ratio = robot_component_size / total_free if total_free > 0 else 0.0

        is_dominant = robot_component_size == total_free if num_components == 1 else robot_component_size >= (total_free * 0.9)

        return is_dominant, reachable_ratio

    def check_map(self):
        if not self.enabled:
            return
        self.send_to_validate = False

        if not self.map_.data:
            return

        self.send_to_validate = True
        self.map_ss = self.map_

        # freeze the map->odom activity since the previous snapshot
        self.snap_jump_m = self.win_jump_m
        self.snap_jump_rad = self.win_jump_rad
        self.win_jump_m = 0.0
        self.win_jump_rad = 0.0

    def validate_map(self):
        if not self.enabled:
            return
        if not self.send_to_validate:
            return
        self.send_to_validate = False
        width = self.map_ss.info.width
        height = self.map_ss.info.height
        grid = np.array(self.map_ss.data, dtype=np.int8).reshape(height, width)

        free_mask = (grid == 0)
        free_clean = ndimage.binary_opening(free_mask, structure=np.ones((3, 3)))

        labeled, num_components = ndimage.label(free_clean, structure=np.ones((3, 3)))

        largest_component_size = 0
        if num_components > 0:
            sizes = ndimage.sum(free_clean, labeled, range(1, num_components + 1))
            largest_component_size = sizes.max()

        fragmented = num_components > 1 and largest_component_size < free_clean.sum() * 0.95

        map_x, map_y = self.world_to_map(self.x_pos, self.y_pos, self.map_ss)
        is_dominant, reachable_ratio = self.check_robot_connectivity(
            labeled, num_components, free_clean, map_y, map_x)

        # pass/fail only — no score. Drift is no longer judged from map
        # snapshots; map->odom steps are reported in the message instead.
        fail_reasons = []
        if fragmented:
            fail_reasons.append("map is fragmented")
        if not is_dominant and reachable_ratio < 0.8:
            fail_reasons.append("robot is not in major connected region")
        valid = len(fail_reasons) == 0

        loop_closure_suspected = (self.snap_jump_m >= self.loop_jump_m
                                  or self.snap_jump_rad >= self.loop_jump_rad)

        msg = Validatedmap()
        msg.valid = bool(valid)
        msg.fail_reason = "; ".join(fail_reasons)
        msg.map_odom_jump_m = float(self.snap_jump_m)
        msg.map_odom_jump_rad = float(self.snap_jump_rad)
        msg.loop_closure_suspected = bool(loop_closure_suspected)
        msg.validation_time = self.get_clock().now().to_msg()
        msg.map = self.map_ss

        self.latest_valid = bool(valid)
        if valid:
            self.good_map_counter += 1
            msg.counter = self.good_map_counter
            self.good_map_ = msg
            self.get_logger().info(f"VALID (#{self.good_map_counter}) reachable={reachable_ratio:.2f}")
        else:
            self.bad_map_counter += 1
            msg.counter = self.bad_map_counter
            self.bad_map_ = msg
            self.get_logger().warn(f"INVALID {fail_reasons} (#{self.bad_map_counter})")

    def publish_map(self):
        if self.good_map_counter > 0:
            self.good_map_pub_.publish(self.good_map_)
        if self.bad_map_counter > 0:
            self.bad_map_pub_.publish(self.bad_map_)


def main(args=None):
    rclpy.init(args=args)
    node = MAPValidatorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
