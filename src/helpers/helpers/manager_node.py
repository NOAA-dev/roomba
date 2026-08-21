#!/usr/bin/env python3
import rclpy
import os
import yaml
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Pose2D
from nav_msgs.msg import OccupancyGrid
from scipy import ndimage
from custom_interfaces.msg import Validatedmap, NodeEnableStates
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from tf_transformations import quaternion_from_euler
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration
from slam_toolbox.srv import DeserializePoseGraph, Pause


class ManagerNode(Node):
    def __init__(self):
        super().__init__("manager_node")

        self.saved_maps_dir = os.path.expanduser("~/roomba/saved_maps")
        self.state = "STARTUP"
        self.verify_start_time = None
        self.verify_timeout_sec = 120.0
        self.similarity_threshold = 0.85  # placeholder — tune from real data

        self.saved_grid = None
        self.saved_resolution = None
        self.saved_origin = None
        self.pending_resume_base = None
        self.blob_targets = []
        self.blob_index = 0

        self.slam_paused = False  # locally tracked, since pause_new_measurements toggles rather than sets

        enable_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


        self.node_enable_pub = self.create_publisher(NodeEnableStates, "/manager/enable_states", enable_qos)
        self._enable_state = {
            "map_validator": False,
            "pure_pursuit": False,
            "frontier_explorer": False,
            "reactive_explorer": False,
            "a_star": False,
        }
        self._publish_enable_state()

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.validated_map_sub = self.create_subscription(
            Validatedmap, "/validated_map", self.validated_map_callback, 10)

        # ---- TF tracking, needed to detect when the robot has actually
        # reached a queued blob-center waypoint during verification ----
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.1, self.check_tf)

        self.current_blob_goal = None
        self.blob_goal_set_time = None
        self.blob_goal_reached_radius = 0.4
        self.blob_goal_timeout_sec = 15.0
        self.blob_nav_timer = None

        # ---- direct lifecycle control, no external lifecycle manager ----
        self.slam_get_state_client = self.create_client(GetState, "/slam_toolbox/get_state")
        self.slam_change_state_client = self.create_client(ChangeState, "/slam_toolbox/change_state")
        self.map_saver_get_state_client = self.create_client(GetState, "/map_saver_server/get_state")
        self.map_saver_change_state_client = self.create_client(ChangeState, "/map_saver_server/change_state")

        self.slam_pause_client = self.create_client(Pause, "/slam_toolbox/pause_new_measurements")
        self.deserialize_client = self.create_client(DeserializePoseGraph, "/slam_toolbox/deserialize_map")

        self.startup_timer = self.create_timer(1.0, self.run_startup_check)

    # ------------------------------------------------------------------
    def _publish_enable_state(self, **changes):
        self._enable_state.update(changes)
        msg = NodeEnableStates()
        msg.map_validator = self._enable_state["map_validator"]
        msg.pure_pursuit = self._enable_state["pure_pursuit"]
        msg.frontier_explorer = self._enable_state["frontier_explorer"]
        msg.reactive_explorer = self._enable_state["reactive_explorer"]
        msg.a_star = self._enable_state["a_star"]
        self.node_enable_pub.publish(msg)

    def check_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "roomba", rclpy.time.Time(), timeout=Duration(seconds=0.2))
            self.x_pos = transform.transform.translation.x
            self.y_pos = transform.transform.translation.y
        except TransformException as ex:
            self.get_logger().warn(f"TF unavailable: {ex}", throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    # BRING-UP: manager owns every lifecycle transition for both nodes.
    # slam_toolbox is brought up first, then map_saver_server, one at a
    # time, each waited on patiently (services can take a few seconds to
    # appear after process start) rather than a single short timeout.
    # ------------------------------------------------------------------
    def run_startup_check(self):
        self.startup_timer.cancel()
        self.get_logger().info("bringing up slam_toolbox...")
        self._bringup_target = "slam"
        self._bringup_wait_attempts = 0
        self._bringup_timer = self.create_timer(1.0, self._bringup_tick)

    def _bringup_tick(self):
        target = self._bringup_target
        get_client = self.slam_get_state_client if target == "slam" else self.map_saver_get_state_client
        change_client = self.slam_change_state_client if target == "slam" else self.map_saver_change_state_client

        if not get_client.service_is_ready():
            self._bringup_wait_attempts += 1
            if self._bringup_wait_attempts >= 20:  # ~20s patience
                self.get_logger().error(f"{target} get_state never became available, skipping")
                self._bringup_timer.cancel()
                self._advance_bringup()
            return

        self._bringup_timer.cancel()
        future = get_client.call_async(GetState.Request())
        future.add_done_callback(
            lambda f: self._on_bringup_get_state(f, target, get_client, change_client))

    def _on_bringup_get_state(self, future, target, get_client, change_client):
        try:
            state_id = future.result().current_state.id
        except Exception as e:
            self.get_logger().error(f"{target} get_state failed: {e}")
            self._advance_bringup()
            return

        if state_id == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info(f"{target} already active")
            self._advance_bringup()
        elif state_id == State.PRIMARY_STATE_INACTIVE:
            self._send_transition(change_client, Transition.TRANSITION_ACTIVATE,
                                   lambda ok: self._on_bringup_transition(ok, target))
        elif state_id == State.PRIMARY_STATE_UNCONFIGURED:
            self._send_transition(change_client, Transition.TRANSITION_CONFIGURE,
                                   lambda ok: self._on_bringup_configured(ok, target, change_client))
        else:
            self.get_logger().error(f"{target} in unexpected lifecycle state {state_id}")
            self._advance_bringup()

    def _on_bringup_configured(self, success, target, change_client):
        if not success:
            self.get_logger().error(f"{target} configure transition failed")
            self._advance_bringup()
            return
        self._send_transition(change_client, Transition.TRANSITION_ACTIVATE,
                               lambda ok: self._on_bringup_transition(ok, target))

    def _on_bringup_transition(self, success, target):
        if success:
            self.get_logger().info(f"{target} activated")
        else:
            self.get_logger().error(f"{target} activate transition failed")
        self._advance_bringup()

    def _advance_bringup(self):
        if self._bringup_target == "slam":
            self._bringup_target = "map_saver"
            self._bringup_wait_attempts = 0
            self._bringup_timer = self.create_timer(1.0, self._bringup_tick)
        else:
            self.get_logger().info("bring-up complete for slam_toolbox and map_saver_server")
            self._on_bringup_done()

    def _send_transition(self, change_client, transition_id, done_cb):
        if not change_client.service_is_ready():
            self.get_logger().error("change_state client not ready")
            done_cb(False)
            return
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = change_client.call_async(req)
        future.add_done_callback(lambda f: self._on_transition_response(f, done_cb))

    def _on_transition_response(self, future, done_cb):
        try:
            result = future.result()
            done_cb(bool(result.success))
        except Exception as e:
            self.get_logger().error(f"change_state call failed: {e}")
            done_cb(False)

    # ------------------------------------------------------------------
    def _on_bringup_done(self):
        latest_base = self.find_latest_saved_map()
        if latest_base is None:
            self.get_logger().info("no complete saved map found — entering full exploration")
            self.enter_full_exploration()
            return

        if not self.load_saved_map(latest_base + ".yaml"):
            self.get_logger().warn("failed to load saved map metadata — entering full exploration")
            self.enter_full_exploration()
            return

        self.pending_resume_base = latest_base
        self.get_logger().info(f"found saved map: {latest_base} — attempting resume")
        self.begin_resume_sequence()

    def find_latest_saved_map(self):
        if not os.path.isdir(self.saved_maps_dir):
            return None

        yaml_files = [f for f in os.listdir(self.saved_maps_dir) if f.endswith(".yaml")]
        complete_bases = []
        for y in yaml_files:
            base = y[:-5]
            posegraph_path = os.path.join(self.saved_maps_dir, base + ".posegraph")
            if os.path.exists(posegraph_path):
                complete_bases.append(base)

        if not complete_bases:
            return None

        complete_bases.sort(
            key=lambda b: os.path.getmtime(os.path.join(self.saved_maps_dir, b + ".yaml")))
        return os.path.join(self.saved_maps_dir, complete_bases[-1])

    def load_saved_map(self, yaml_path):
        try:
            with open(yaml_path, "r") as f:
                meta = yaml.safe_load(f)
            pgm_path = os.path.join(os.path.dirname(yaml_path), meta["image"])

            from PIL import Image
            img = np.array(Image.open(pgm_path))

            img = np.flipud(img)

            grid = np.full(img.shape, -1, dtype=np.int8)
            grid[img > 250] = 0
            grid[img < 10] = 100

            self.saved_grid = grid
            self.saved_resolution = meta["resolution"]
            self.saved_origin = meta["origin"]
            return True
        except Exception as e:
            self.get_logger().error(f"failed to load saved map: {e}")
            return False

    def begin_resume_sequence(self):
        self._set_slam_paused(True, self._on_paused_for_deserialize)

    def _set_slam_paused(self, desired_paused, done_cb):
        if self.slam_paused == desired_paused:
            done_cb(True)
            return
        if not self.slam_pause_client.service_is_ready():
            self.get_logger().error("pause_new_measurements service not ready")
            done_cb(False)
            return
        future = self.slam_pause_client.call_async(Pause.Request())
        future.add_done_callback(lambda f: self._on_pause_response(f, desired_paused, done_cb))

    def _on_pause_response(self, future, desired_paused, done_cb):
        try:
            result = future.result()
            self.slam_paused = getattr(result, "status", desired_paused)
        except Exception as e:
            self.get_logger().error(f"pause_new_measurements call failed: {e}")
            done_cb(False)
            return
        done_cb(True)

    def _on_paused_for_deserialize(self, ok):
        if not ok:
            self.get_logger().error("failed to pause slam_toolbox — entering full exploration")
            self.enter_full_exploration()
            return
        self._call_deserialize()

    def _call_deserialize(self):
        if not self.deserialize_client.service_is_ready():
            self.get_logger().error("deserialize_map service not ready — entering full exploration")
            self._set_slam_paused(False, lambda ok: None)
            self.enter_full_exploration()
            return

        req = DeserializePoseGraph.Request()
        req.filename = self.pending_resume_base
        req.match_type = DeserializePoseGraph.Request.START_AT_FIRST_NODE
        req.initial_pose = Pose2D(x=0.0, y=0.0, theta=0.0)

        future = self.deserialize_client.call_async(req)
        future.add_done_callback(self._on_deserialize_called)

    def _on_deserialize_called(self, future):
        try:
            future.result()  # response has no fields — only the call itself can throw
        except Exception as e:
            self.get_logger().error(f"deserialize_map call failed: {e} — entering full exploration")
            self._set_slam_paused(False, lambda ok: None)
            self.enter_full_exploration()
            return

        self.get_logger().info("deserialize_map call completed (unconfirmed) — unpausing to inspect /map")
        self._set_slam_paused(False, self._on_unpaused_after_deserialize)

    def _on_unpaused_after_deserialize(self, ok):
        if not ok:
            self.get_logger().error("failed to unpause slam_toolbox after deserialize — entering full exploration")
            self.enter_full_exploration()
            return

        self._latest_early_map = None
        self._early_check_sub = self.create_subscription(
            OccupancyGrid, "/map", self._early_check_map_cb, 10)
        self._early_check_timer = self.create_timer(3.0, self._early_check_timeout)

    def _early_check_map_cb(self, msg):
        self._latest_early_map = msg

    def _early_check_timeout(self):
        self._early_check_timer.cancel()
        self.destroy_subscription(self._early_check_sub)

        if self._latest_early_map is None:
            self.get_logger().warn("no /map received during early check — entering full exploration")
            self.enter_full_exploration()
            return

        live = self._latest_early_map
        live_grid = np.array(live.data, dtype=np.int8).reshape(live.info.height, live.info.width)
        live_occupied = int((live_grid == 100).sum())
        saved_occupied = int((self.saved_grid == 100).sum())

        self.get_logger().info(f"early check: live_occupied={live_occupied}, saved_occupied={saved_occupied}")

        if saved_occupied > 0 and live_occupied < max(20, saved_occupied * 0.1):
            self.get_logger().warn(
                "live map looks blank relative to saved map — deserialize likely failed, entering full exploration")
            self.enter_full_exploration()
            return

        self.get_logger().info("early check passed — entering verification drive")
        self.enter_verify_state()

    # ------------------------------------------------------------------
    def enter_full_exploration(self):
        # slam_toolbox and map_saver_server are already active from bring-up
        # — no lifecycle calls needed here, just enable the exploration
        # pipeline nodes
        self.state = "FULL_EXPLORATION"
        self._cancel_blob_nav_timer()
        self._publish_enable_state(
            map_validator=True, pure_pursuit=True, frontier_explorer=True,
            reactive_explorer=True, a_star=True)
        self.get_logger().info("state -> FULL_EXPLORATION")

    def enter_verify_state(self):
        self.state = "VERIFY_SAVED_MAP"
        self._publish_enable_state(
            map_validator=True, pure_pursuit=True, frontier_explorer=False,
            reactive_explorer=False, a_star=True)

        self.blob_targets = self.compute_large_free_blob_centers(
            self.saved_grid, self.saved_resolution, self.saved_origin)
        self.blob_index = 0
        self.verify_start_time = self.get_clock().now()

        if self.blob_targets:
            self.send_next_blob_goal()
            self.blob_nav_timer = self.create_timer(0.5, self._blob_nav_tick)

        self.get_logger().info(f"state -> VERIFY_SAVED_MAP, {len(self.blob_targets)} waypoints queued")

    def compute_large_free_blob_centers(self, grid, resolution, origin, top_n=4, min_area_m2=1.0, min_clearance_m=0.3):
        free_mask = (grid == 0)
        free_clean = ndimage.binary_opening(free_mask, structure=np.ones((3, 3)))
        labeled, num = ndimage.label(free_clean, structure=np.ones((3, 3)))
        if num == 0:
            return []

        occupied_mask = (grid == 100)
        occupied_dilated = ndimage.binary_dilation(occupied_mask, structure=np.ones((3, 3)))
        dist_from_occupied_m = ndimage.distance_transform_edt(~occupied_dilated) * resolution

        sizes = ndimage.sum(free_clean, labeled, range(1, num + 1))
        cell_area = resolution * resolution
        order = np.argsort(sizes)[::-1]

        targets = []
        for idx in order[:top_n]:
            label_id = idx + 1
            if sizes[idx] * cell_area < min_area_m2:
                continue

            blob_mask = (labeled == label_id)
            blob_clearance = np.where(blob_mask, dist_from_occupied_m, -1.0)
            best_row, best_col = np.unravel_index(np.argmax(blob_clearance), blob_clearance.shape)
            best_clearance = blob_clearance[best_row, best_col]

            if best_clearance < min_clearance_m:
                self.get_logger().warn(
                    f"blob {label_id} best clearance {best_clearance:.2f}m below {min_clearance_m}m, skipping")
                continue

            world_x = origin[0] + best_col * resolution
            world_y = origin[1] + best_row * resolution
            targets.append((world_x, world_y))
        return targets

    def send_next_blob_goal(self):
        if self.blob_index >= len(self.blob_targets):
            return
        wx, wy = self.blob_targets[self.blob_index]
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = wx
        goal.pose.position.y = wy
        q = quaternion_from_euler(0, 0, 0.0)
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q
        self.goal_pub.publish(goal)
        self.current_blob_goal = (wx, wy)
        self.blob_goal_set_time = self.get_clock().now()
        self.get_logger().info(
            f"VERIFY: driving to blob center {self.blob_index + 1}/{len(self.blob_targets)} ({wx:.2f},{wy:.2f})")

    def _blob_nav_tick(self):
        if self.state != "VERIFY_SAVED_MAP" or self.current_blob_goal is None:
            return

        gx, gy = self.current_blob_goal
        dist = ((gx - self.x_pos) ** 2 + (gy - self.y_pos) ** 2) ** 0.5
        elapsed = (self.get_clock().now() - self.blob_goal_set_time).nanoseconds / 1e9

        reached = dist <= self.blob_goal_reached_radius
        timed_out = elapsed >= self.blob_goal_timeout_sec

        if not (reached or timed_out):
            return

        if timed_out and not reached:
            self.get_logger().warn(
                f"VERIFY: waypoint {self.blob_index + 1} timed out after {elapsed:.0f}s, moving on")

        self.blob_index += 1
        if self.blob_index < len(self.blob_targets):
            self.send_next_blob_goal()
        else:
            self.current_blob_goal = None
            self._cancel_blob_nav_timer()
            self.get_logger().info("VERIFY: all waypoints visited, holding position while similarity check continues")

    def _cancel_blob_nav_timer(self):
        if self.blob_nav_timer is not None:
            self.blob_nav_timer.cancel()
            self.blob_nav_timer = None
        self.current_blob_goal = None

    # ------------------------------------------------------------------
    def validated_map_callback(self, msg: Validatedmap):
        if self.state != "VERIFY_SAVED_MAP":
            return

        elapsed = (self.get_clock().now() - self.verify_start_time).nanoseconds / 1e9
        similarity = self.compute_map_similarity_aligned(self.saved_grid, self.saved_resolution,
                                                           self.saved_origin, msg.map)

        if similarity is not None:
            self.get_logger().info(f"map similarity so far: {similarity:.2f} (elapsed {elapsed:.0f}s)")

        if elapsed >= self.verify_timeout_sec:
            if similarity is not None and similarity >= self.similarity_threshold:
                self.enter_freeze_and_reuse()
            else:
                self.get_logger().warn(f"map similarity {similarity} below threshold — entering full exploration")
                self.enter_full_exploration()

    def compute_map_similarity_aligned(self, saved_grid, saved_resolution, saved_origin, live_map_msg):

        live_res = live_map_msg.info.resolution
        if abs(saved_resolution - live_res) > 1e-6:
            return None

        s_h, s_w = saved_grid.shape
        s_ox, s_oy = saved_origin[0], saved_origin[1]
        s_x1 = s_ox + s_w * saved_resolution
        s_y1 = s_oy + s_h * saved_resolution

        l_ox = live_map_msg.info.origin.position.x
        l_oy = live_map_msg.info.origin.position.y
        l_h, l_w = live_map_msg.info.height, live_map_msg.info.width
        l_x1 = l_ox + l_w * live_res
        l_y1 = l_oy + l_h * live_res

        ix0, iy0 = max(s_ox, l_ox), max(s_oy, l_oy)
        ix1, iy1 = min(s_x1, l_x1), min(s_y1, l_y1)
        if ix1 <= ix0 or iy1 <= iy0:
            return None

        def crop(grid, ox, oy, res, height, width):
            col0 = int(round((ix0 - ox) / res))
            row0 = int(round((iy0 - oy) / res))
            col1 = int(round((ix1 - ox) / res))
            row1 = int(round((iy1 - oy) / res))
            col0, row0 = max(0, col0), max(0, row0)
            col1, row1 = min(width, col1), min(height, row1)
            return grid[row0:row1, col0:col1]

        saved_crop = crop(saved_grid, s_ox, s_oy, saved_resolution, s_h, s_w)
        live_grid = np.array(live_map_msg.data, dtype=np.int8).reshape(l_h, l_w)
        live_crop = crop(live_grid, l_ox, l_oy, live_res, l_h, l_w)

        h = min(saved_crop.shape[0], live_crop.shape[0])
        w = min(saved_crop.shape[1], live_crop.shape[1])
        if h <= 0 or w <= 0:
            return None
        a = saved_crop[:h, :w]
        b = live_crop[:h, :w]

        occ_a = (a == 100)
        occ_b = (b == 100)
        known = (a != -1) & (b != -1)

        if occ_a.sum() < 20:
            return None

        match = np.count_nonzero(occ_a & occ_b & known)
        union = np.count_nonzero((occ_a | occ_b) & known)
        return match / union if union > 0 else 0.0

    def enter_freeze_and_reuse(self):
        self.state = "FROZEN_REUSE"
        self._cancel_blob_nav_timer()
        self._set_slam_paused(True, lambda ok: None)
        self._publish_enable_state(map_validator=False, a_star=False)
        self.get_logger().info("map confirmed unchanged — pausing SLAM/validator, reusing saved map")


def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
