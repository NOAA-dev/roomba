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
from std_msgs.msg import Bool
from nav2_msgs.srv import SaveMap
from slam_toolbox.srv import SerializePoseGraph
import os


class MAPValidatorNode(Node):
    def __init__(self):
        super().__init__("map_validator_")

        self.map_ = OccupancyGrid()
        self.map_ss = OccupancyGrid()
        self.prev_map_ss = OccupancyGrid()
        self.send_to_validate = False
        self.keep_exploring = True

        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw = 0.0

        self.good_map_ = Validatedmap()
        self.good_map_counter = 0
        self.bad_map_ = Validatedmap()
        self.bad_map_counter = 0

        self.enabled = False

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

        self.exploration_done = False
        self.map_saved = False
        self.pose_graph_saved = False

        enable_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enable_sub = self.create_subscription(
            NodeEnableStates, "/manager/enable_states", self.enable_callback, enable_qos)
        self.exploration_done_sub = self.create_subscription(
            Bool, "/exploration_complete", self.exploration_done_callback, 10)

        self.save_map_client = self.create_client(SaveMap, "/map_saver_server/save_map")
        self.serialize_map_client = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")

    def enable_callback(self, msg: NodeEnableStates):
        self.enabled = msg.map_validator

    def exploration_done_callback(self, msg: Bool):
        if msg.data and not self.exploration_done:
            self.exploration_done = True
            self.get_logger().info("exploration complete signal received — freezing map, no further SLAM updates will be processed")
            self.save_final_map()

    def save_final_map(self):
        save_dir = os.path.expanduser("~/roomba/saved_maps")
        os.makedirs(save_dir, exist_ok=True)
        stamp = self.get_clock().now().nanoseconds
        map_path = os.path.join(save_dir, f"final_map_{stamp}")

        if not self.map_saved:
            self._save_occupancy_grid(map_path)

        if not self.pose_graph_saved:
            self._serialize_pose_graph(map_path)

    def _save_occupancy_grid(self, map_path):
        if not self.save_map_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("map_saver_server/save_map service unavailable, cannot save final map")
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
                self.get_logger().info("final map (yaml/pgm) saved successfully")
            else:
                self.get_logger().error("map_saver_server reported save failure")
        except Exception as e:
            self.get_logger().error(f"save_map call failed: {e}")

    def _serialize_pose_graph(self, map_path):
        # saves slam_toolbox's full pose-graph (not just the flattened
        # occupancy grid) so a future run can resume mapping from exactly
        # where this session left off via /slam_toolbox/deserialize_map,
        # instead of starting SLAM from a blank slate every time
        if not self.serialize_map_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("slam_toolbox/serialize_map service unavailable, cannot save pose graph")
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
            else:
                self.get_logger().error(f"pose graph serialization failed, code={result.result}")
        except Exception as e:
            self.get_logger().error(f"serialize_map call failed: {e}")

    def map_callback(self, msg: OccupancyGrid):
        if not self.enabled or self.exploration_done:
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
            z = transform.transform.translation.z
            q = transform.transform.rotation
            _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        except TransformException as ex:
            self.get_logger().warn(f"TF unavailable: {ex}", throttle_duration_sec=2.0)

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

    def align_to_common_frame(self, map_a: OccupancyGrid, map_b: OccupancyGrid):
        res = map_a.info.resolution
        if abs(res - map_b.info.resolution) > 1e-6:
            self.get_logger().warn("resolution mismatch between snapshots, skipping shift check")
            return None, None

        ax0, ay0 = map_a.info.origin.position.x, map_a.info.origin.position.y
        bx0, by0 = map_b.info.origin.position.x, map_b.info.origin.position.y
        a_x1, a_y1 = ax0 + map_a.info.width * res, ay0 + map_a.info.height * res
        b_x1, b_y1 = bx0 + map_b.info.width * res, by0 + map_b.info.height * res

        ix0, iy0 = max(ax0, bx0), max(ay0, by0)
        ix1, iy1 = min(a_x1, b_x1), min(a_y1, b_y1)

        if ix1 <= ix0 or iy1 <= iy0:
            return None, None

        def crop(map_msg):
            grid = np.array(map_msg.data, dtype=np.int8).reshape(map_msg.info.height, map_msg.info.width)
            col0, row0 = self.world_to_map(ix0, iy0, map_msg)
            col1, row1 = self.world_to_map(ix1, iy1, map_msg)
            return grid[row0:row1, col0:col1]

        grid_a = crop(map_a)
        grid_b = crop(map_b)

        h = min(grid_a.shape[0], grid_b.shape[0])
        w = min(grid_a.shape[1], grid_b.shape[1])
        if h <= 0 or w <= 0:
            return None, None

        return grid_a[:h, :w], grid_b[:h, :w]

    def detect_snapshot_shift(self, grid_prev, grid_curr, max_search_px=3):
        occ_prev = (grid_prev == 100)
        occ_curr = (grid_curr == 100)
        known = (grid_prev != -1) & (grid_curr != -1)

        if occ_prev.sum() < 20 or occ_curr.sum() < 20:
            return 0, 0, None

        best_score, best_dx, best_dy = -1.0, 0, 0

        for dy in range(-max_search_px, max_search_px + 1):
            for dx in range(-max_search_px, max_search_px + 1):
                shifted = np.roll(np.roll(occ_curr, dy, axis=0), dx, axis=1)

                valid = known.copy()
                if dy > 0: valid[:dy, :] = False
                elif dy < 0: valid[dy:, :] = False
                if dx > 0: valid[:, :dx] = False
                elif dx < 0: valid[:, dx:] = False

                match = np.count_nonzero(occ_prev & shifted & valid)
                union = np.count_nonzero((occ_prev | shifted) & valid)
                score = match / union if union > 0 else 0.0

                if score > best_score:
                    best_score, best_dx, best_dy = score, dx, dy

        return best_dx, best_dy, best_score

    def scoring_function(self, dominant, reachable_ratio, fragments, fragmented, drift_score, cell_drift_distance):
        fail_reasons = []
        score = 100.0

        if fragmented == True:
            fail_reasons.append("map is fragmented")
            penalty = -50.0
        else:
            penalty = -min((fragments), 7)

        if dominant == True:
            penalty += -(1.0 - reachable_ratio) * 100
        elif dominant == False and reachable_ratio >= 0.8:
            penalty += -20
        elif dominant == False and reachable_ratio < 0.8:
            fail_reasons.append("robot is not in major connected region")
            penalty += -50.0

        if drift_score is None:
            penalty += -25.0
        elif drift_score < 0.7 or cell_drift_distance > 1.4:
            fail_reasons.append("map has drifted")
            penalty += -50
        else:
            penalty += -(1 - drift_score) * 100

        score = score + penalty

        return score, fail_reasons

    def check_map(self):
        if not self.enabled or self.exploration_done:
            return
        self.keep_exploring = True
        self.send_to_validate = False

        if not self.map_.data:
            return

        data = np.array(self.map_.data, dtype=np.int8)

        num_of_free = np.count_nonzero(data == 0)
        num_of_unknown = np.count_nonzero(data == -1)
        num_of_occupied = np.count_nonzero(data == 100)
        total_cells = data.size

        if total_cells == 0:
            return

        p_explored = 100.0 * ((num_of_occupied + num_of_free) / total_cells)
        p_unknown = 100.0 * (num_of_unknown / total_cells)

        if p_explored >= 60.0:
            self.send_to_validate = True
            self.keep_exploring = False
            if self.map_ss.data:
                self.prev_map_ss = self.map_ss
            self.map_ss = self.map_
        else:
            self.keep_exploring = True
            self.send_to_validate = False

    def validate_map(self):
        if not self.enabled or self.exploration_done:
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
        if fragmented:
            self.get_logger().warn("Map is fragmented.")

        map_x, map_y = self.world_to_map(self.x_pos, self.y_pos, self.map_ss)
        row, col = map_y, map_x
        is_dominant, reachable_ratio = self.check_robot_connectivity(labeled, num_components, free_clean, row, col)
        if not is_dominant and reachable_ratio < 0.8:
            self.get_logger().warn("robot not in dominant connected region")

        drift_score = None
        shift_cells = 0.0
        shift_meters = 0.0
        drifted = False

        if self.prev_map_ss.data:
            grid_prev, grid_curr = self.align_to_common_frame(self.prev_map_ss, self.map_ss)
            if grid_prev is not None:
                dx, dy, s = self.detect_snapshot_shift(grid_prev, grid_curr)
                if s is not None:
                    drift_score = s
                    shift_cells = math.hypot(dx, dy)
                    shift_meters = shift_cells * self.map_ss.info.resolution
                    drifted = shift_cells > 1.4 or drift_score < 0.7

        map_score, fail_flags = self.scoring_function(is_dominant, reachable_ratio, num_components, fragmented, drift_score, shift_cells)

        valid = len(fail_flags) == 0
        drift_detected = drifted if drift_score is not None else False

        msg = Validatedmap()
        msg.map_score = float(map_score)
        msg.valid = bool(valid)
        msg.fragmented = bool(fragmented)
        msg.fragments = int(num_components)
        msg.is_dominant = bool(is_dominant)
        msg.reachable_ratio = float(reachable_ratio)
        msg.drift_detected = bool(drift_detected)
        msg.drift_score = float(drift_score) if drift_score is not None else -1.0
        msg.drift_distance = float(shift_meters)
        msg.validation_time = self.get_clock().now().to_msg()
        msg.map = self.map_ss

        if valid:
            self.good_map_counter += 1
            msg.counter = self.good_map_counter
            self.good_map_ = msg
            self.get_logger().info(f"map_score={map_score:.1f} -> VALID (#{self.good_map_counter})")
        else:
            self.bad_map_counter += 1
            msg.counter = self.bad_map_counter
            self.bad_map_ = msg
            self.get_logger().warn(f"map_score={map_score:.1f} -> INVALID {fail_flags} (#{self.bad_map_counter})")

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
