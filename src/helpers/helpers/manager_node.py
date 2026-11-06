#!/usr/bin/env python3
import math
import os
import yaml
import numpy as np
from rclpy.node import Node
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Pose2D, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from scipy import ndimage
from custom_interfaces.msg import Validatedmap, NodeEnableStates
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration
from slam_toolbox.srv import DeserializePoseGraph, Pause

# states in which the robot is driving around and a "home" pose may be captured
_MOTION_STATES = ("VERIFY_SAVED_MAP", "FULL_EXPLORATION", "SWEEP")


class ManagerNode(Node):
    """
    State flow
      STARTUP -> (no saved map)  FULL_EXPLORATION
              -> (saved map)     AWAIT_INIT_POSE -> VERIFY_SAVED_MAP -> SWEEP
      FULL_EXPLORATION / SWEEP -> RETURN_HOME -> AWAIT_CONFIRM -> SAVING
                               -> COMPLETE  (or PENDING_PROMOTION)
      FULL_EXPLORATION -> RETURN_HOME -> DWELL_HOME -> FULL_EXPLORATION   (periodic, revisit_distance_m > 0)

    CLI (all plain ROS 2, no joystick):
      ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \\
        "{header: {frame_id: map}, pose: {pose: {position: {x: 1.0, y: 2.0},
          orientation: {z: 0.0, w: 1.0}}}}"
      ros2 service call /manager/confirm_save      std_srvs/srv/Trigger {}
      ros2 service call /manager/promote_candidate std_srvs/srv/Trigger {}
      ros2 service call /manager/return_home       std_srvs/srv/Trigger {}
      ros2 topic echo /manager/state
    """

    def __init__(self):
        super().__init__("manager_node")

        # ---------------- parameters ----------------
        self.declare_parameter("saved_maps_dir", "~/roomba/saved_maps")
        self.declare_parameter("verify_timeout_sec", 120.0)
        self.declare_parameter("similarity_threshold", 0.85)          # placeholder — tune from real data
        self.declare_parameter("sweep_on_resume", True)               # False = old behaviour (freeze after verify)
        self.declare_parameter("sweep_spacing_m", 1.5)
        self.declare_parameter("sweep_max_waypoints", 15)
        self.declare_parameter("sweep_goal_timeout_sec", 30.0)
        self.declare_parameter("home_reached_radius_m", 0.4)
        self.declare_parameter("return_home_timeout_sec", 90.0)
        self.declare_parameter("return_home_retries", 2)
        self.declare_parameter("revisit_distance_m", 0.0)             # >0: periodic return-to-start; 0 = off
        self.declare_parameter("settle_validations", 3)               # clean valid snapshots needed before a save is offered
        self.declare_parameter("promote_similarity_threshold", 0.6)   # placeholder — tune from real data
        self.declare_parameter("promote_min_known_ratio", 0.9)        # placeholder — tune from real data
        self.declare_parameter("init_pose_timeout_sec", 300.0)        # 0 = wait forever
        self.declare_parameter("initial_pose", [0.0] * 0)             # [x, y, yaw] in the saved map frame
        self.declare_parameter("assume_origin_start", False)          # True = old behaviour (robot at saved start)
        self.declare_parameter("save_timeout_sec", 45.0)
        self.declare_parameter("periodic_dwell_sec", 8.0)             # hold at home after a periodic return so SLAM can close the loop
        self.declare_parameter("periodic_dwell_max_sec", 30.0)        # give up waiting for the map to settle after this
        self.declare_parameter("periodic_dwell_settle", 1)            # clean valid snapshots needed at home (0 = time only)
        self.declare_parameter("min_exploration_sec", 30.0)           # ignore /exploration_complete earlier than this

        gp = lambda n: self.get_parameter(n).value
        self.saved_maps_dir = os.path.expanduser(gp("saved_maps_dir"))
        self.verify_timeout_sec = gp("verify_timeout_sec")
        self.similarity_threshold = gp("similarity_threshold")
        self.sweep_on_resume = gp("sweep_on_resume")
        self.sweep_spacing_m = gp("sweep_spacing_m")
        self.sweep_max_waypoints = int(gp("sweep_max_waypoints"))
        self.sweep_goal_timeout_sec = gp("sweep_goal_timeout_sec")
        self.home_reached_radius = gp("home_reached_radius_m")
        self.return_home_timeout_sec = gp("return_home_timeout_sec")
        self.return_home_retries = int(gp("return_home_retries"))
        self.revisit_distance_m = gp("revisit_distance_m")
        self.settle_validations = int(gp("settle_validations"))
        self.promote_similarity_threshold = gp("promote_similarity_threshold")
        self.promote_min_known_ratio = gp("promote_min_known_ratio")
        self.init_pose_timeout_sec = gp("init_pose_timeout_sec")
        self.param_initial_pose = list(gp("initial_pose"))
        self.assume_origin_start = gp("assume_origin_start")
        self.save_timeout_sec = gp("save_timeout_sec")
        self.min_exploration_sec = gp("min_exploration_sec")
        self.periodic_dwell_sec = gp("periodic_dwell_sec")
        self.periodic_dwell_max_sec = gp("periodic_dwell_max_sec")
        self.periodic_dwell_settle = int(gp("periodic_dwell_settle"))
        self.dwell_start = None
        self._dwell_timer = None
        self.exploration_started = None

        # ---------------- state ----------------
        self.state = "STARTUP"
        self.verify_start_time = None

        self.saved_grid = None
        self.saved_resolution = None
        self.saved_origin = None
        self.pending_resume_base = None
        self.blob_targets = []
        self.blob_goal_reached_radius = 0.4

        self.slam_paused = False  # locally tracked, since pause_new_measurements toggles rather than sets

        # return-to-start bookkeeping
        self.home_pose = None            # (x, y, yaw) in the map frame
        self.home_reached = False
        self.return_reason = ""
        self.return_attempt = 0
        self.return_max_jump_m = 0.0
        self.return_max_jump_rad = 0.0
        self.odom_travelled_m = 0.0
        self._last_odom_xy = None

        # validation tracking (from /validated_map and /discarded_map)
        self.last_good = None
        self.last_good_t = -1.0
        self.last_good_counter = -1
        self.last_bad_t = -1.0
        self.last_bad_counter = -1
        self.settled = 0

        # save / promotion
        self.pending_candidate = None
        self._save_timer = None

        # generic waypoint runner
        self._run = None
        self._run_timer = None

        # resume at an arbitrary pose
        self._resume_pose = None
        self._init_pose_sub = None
        self._init_pose_timer = None
        self._init_pose_waited = 0.0

        enable_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.node_enable_pub = self.create_publisher(NodeEnableStates, "/manager/enable_states", enable_qos)
        self.state_pub = self.create_publisher(String, "/manager/state", enable_qos)
        self.saved_map_pub = self.create_publisher(OccupancyGrid, "/saved_map", enable_qos)
        self.save_confirmed_pub = self.create_publisher(Bool, "/manager/save_confirmed", 10)
        self._enable_state = {
            "map_validator": False,
            "pure_pursuit": False,
            "frontier_explorer": False,
            "reactive_explorer": False,
            "a_star": False,
        }
        self._publish_enable_state()
        self._set_state("STARTUP")

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.validated_map_sub = self.create_subscription(
            Validatedmap, "/validated_map", self.validated_map_callback, 10)
        self.discarded_map_sub = self.create_subscription(
            Validatedmap, "/discarded_map", self.discarded_map_callback, 10)
        self.exploration_done_sub = self.create_subscription(
            Bool, "/exploration_complete", self.exploration_complete_callback, 10)
        self.candidate_sub = self.create_subscription(
            String, "/map_validator/candidate_saved", self.candidate_saved_callback, 10)

        self.confirm_srv = self.create_service(Trigger, "/manager/confirm_save", self.confirm_save_srv)
        self.promote_srv = self.create_service(Trigger, "/manager/promote_candidate", self.promote_srv_cb)
        self.return_home_srv = self.create_service(Trigger, "/manager/return_home", self.return_home_srv_cb)

        # ---- TF tracking ----
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.1, self.check_tf)
        self.status_timer = self.create_timer(5.0, self._status_tick)

        # ---- direct lifecycle control, no external lifecycle manager ----
        self.slam_get_state_client = self.create_client(GetState, "/slam_toolbox/get_state")
        self.slam_change_state_client = self.create_client(ChangeState, "/slam_toolbox/change_state")
        self.map_saver_get_state_client = self.create_client(GetState, "/map_saver_server/get_state")
        self.map_saver_change_state_client = self.create_client(ChangeState, "/map_saver_server/change_state")

        self.slam_pause_client = self.create_client(Pause, "/slam_toolbox/pause_new_measurements")
        self.deserialize_client = self.create_client(DeserializePoseGraph, "/slam_toolbox/deserialize_map")

        self.startup_timer = self.create_timer(1.0, self.run_startup_check)

    # ------------------------------------------------------------------
    def _set_state(self, name):
        self.state = name
        self.state_pub.publish(String(data=name))
        self.get_logger().info(f"state -> {name}")

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
            q = transform.transform.rotation
            _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        except TransformException as ex:
            self.get_logger().warn(f"TF unavailable: {ex}", throttle_duration_sec=2.0)
            return

        # home = where this run started driving, captured once, before motion
        if self.home_pose is None and self.state in _MOTION_STATES:
            self.home_pose = (self.x_pos, self.y_pos, self.yaw)
            self.get_logger().info(
                f"home pose captured: ({self.x_pos:.2f}, {self.y_pos:.2f}, yaw {self.yaw:.2f})")

        # path length since the last return-to-start, measured in the odom
        # frame so SLAM corrections of the map frame don't inflate it
        if self.revisit_distance_m > 0 and self.state == "FULL_EXPLORATION":
            try:
                t = self.tf_buffer.lookup_transform("odom", "roomba", rclpy.time.Time())
                xy = (t.transform.translation.x, t.transform.translation.y)
                if self._last_odom_xy is not None:
                    self.odom_travelled_m += math.hypot(xy[0] - self._last_odom_xy[0],
                                                        xy[1] - self._last_odom_xy[1])
                self._last_odom_xy = xy
            except TransformException:
                pass
            if self.odom_travelled_m >= self.revisit_distance_m and self.home_pose is not None:
                self.get_logger().info(
                    f"travelled {self.odom_travelled_m:.1f} m since last closure — returning to start")
                self.begin_return_home("periodic")

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

    def _is_complete_base(self, base):
        return os.path.exists(base + ".yaml") and os.path.exists(base + ".posegraph")

    def find_latest_saved_map(self):
        # 1) the promoted map, if one exists — never "newest by mtime", so a
        #    worse re-scan cannot silently replace a good map
        pointer = os.path.join(self.saved_maps_dir, "CURRENT")
        if os.path.isfile(pointer):
            try:
                with open(pointer, "r") as f:
                    base = f.read().strip()
                if base and self._is_complete_base(base):
                    return base
                self.get_logger().warn(f"CURRENT points to an incomplete map ({base}) — falling back to legacy lookup")
            except OSError as e:
                self.get_logger().warn(f"could not read CURRENT: {e} — falling back to legacy lookup")

        # 2) legacy maps saved before promotion existed (top-level only —
        #    unpromoted candidates live in candidates/ and are never picked)
        if not os.path.isdir(self.saved_maps_dir):
            return None

        complete_bases = []
        for y in os.listdir(self.saved_maps_dir):
            if y.endswith(".yaml"):
                base = os.path.join(self.saved_maps_dir, y[:-5])
                if self._is_complete_base(base):
                    complete_bases.append(base)

        if not complete_bases:
            return None

        complete_bases.sort(key=lambda b: os.path.getmtime(b + ".yaml"))
        return complete_bases[-1]

    def _load_map_files(self, yaml_path):
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
            return grid, meta["resolution"], meta["origin"]
        except Exception as e:
            self.get_logger().error(f"failed to load map {yaml_path}: {e}")
            return None

    def load_saved_map(self, yaml_path):
        loaded = self._load_map_files(yaml_path)
        if loaded is None:
            return False
        self.saved_grid, self.saved_resolution, self.saved_origin = loaded
        return True

    def _grid_to_msg(self, grid, resolution, origin):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(resolution)
        msg.info.width = int(grid.shape[1])
        msg.info.height = int(grid.shape[0])
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.orientation.w = 1.0
        msg.data = [int(v) for v in grid.flatten()]
        return msg

    # ------------------------------------------------------------------
    # RESUME: the robot is NOT assumed to be at the original start pose.
    # The pose in the SAVED map's frame comes from (in priority order)
    #   assume_origin_start:=true  -> old behaviour, robot at the saved start
    #   initial_pose:=[x,y,yaw]    -> launch parameter
    #   /initialpose               -> ros2 topic pub, or RViz "2D Pose Estimate"
    # The saved map is republished on /saved_map so it can be shown in RViz
    # while picking the pose.
    # ------------------------------------------------------------------
    def begin_resume_sequence(self):
        self.saved_map_pub.publish(
            self._grid_to_msg(self.saved_grid, self.saved_resolution, self.saved_origin))

        if self.assume_origin_start:
            self.get_logger().warn("assume_origin_start=true — resuming at the saved start pose (0,0,0)")
            self._start_resume(0.0, 0.0, 0.0, first_node=True)
            return

        if len(self.param_initial_pose) == 3:
            x, y, yaw = [float(v) for v in self.param_initial_pose]
            self.get_logger().info(f"resuming at initial_pose parameter ({x:.2f}, {y:.2f}, {yaw:.2f})")
            self._start_resume(x, y, yaw, first_node=False)
            return

        self._set_state("AWAIT_INIT_POSE")
        self.get_logger().info(
            "saved map found, robot start pose unknown — publish its pose in the SAVED map frame, e.g.\n"
            "  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
            "\"{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {z: 0.0, w: 1.0}}}}\"\n"
            "(the saved map is on /saved_map for RViz)")
        self._init_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._initial_pose_cb, 10)
        self._init_pose_waited = 0.0
        self._init_pose_timer = self.create_timer(5.0, self._init_pose_tick)

    def _init_pose_tick(self):
        self._init_pose_waited += 5.0
        if self.init_pose_timeout_sec > 0 and self._init_pose_waited >= self.init_pose_timeout_sec:
            self._stop_init_pose_wait()
            self.get_logger().warn("no /initialpose received in time — entering full exploration")
            self.enter_full_exploration()
        elif int(self._init_pose_waited) % 30 == 0:
            self.get_logger().info("still waiting for /initialpose (robot start pose in the saved map frame)")

    def _stop_init_pose_wait(self):
        if self._init_pose_timer is not None:
            self.destroy_timer(self._init_pose_timer)
            self._init_pose_timer = None
        if self._init_pose_sub is not None:
            self.destroy_subscription(self._init_pose_sub)
            self._init_pose_sub = None

    def _initial_pose_cb(self, msg: PoseWithCovarianceStamped):
        if self.state != "AWAIT_INIT_POSE":
            return
        if msg.header.frame_id not in ("map", ""):
            self.get_logger().warn(f"/initialpose is in frame '{msg.header.frame_id}', need 'map' — ignored")
            return
        p = msg.pose.pose
        _, _, yaw = euler_from_quaternion(
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        self._stop_init_pose_wait()
        self.get_logger().info(f"initial pose received: ({p.position.x:.2f}, {p.position.y:.2f}, {yaw:.2f})")
        self._start_resume(p.position.x, p.position.y, yaw, first_node=False)

    def _start_resume(self, x, y, yaw, first_node):
        self._resume_pose = (float(x), float(y), float(yaw), bool(first_node))
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

        x, y, yaw, first_node = self._resume_pose
        req = DeserializePoseGraph.Request()
        req.filename = self.pending_resume_base
        if first_node:
            req.match_type = DeserializePoseGraph.Request.START_AT_FIRST_NODE
        else:
            req.match_type = DeserializePoseGraph.Request.START_AT_GIVEN_POSE
        req.initial_pose = Pose2D(x=x, y=y, theta=yaw)

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
    # WAYPOINT RUNNER (generalised from the old blob-goal logic)
    # ------------------------------------------------------------------
    def send_goal(self, x, y, yaw=0.0):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        q = quaternion_from_euler(0, 0, float(yaw))
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q
        self.goal_pub.publish(goal)

    def start_run(self, targets, radius, timeout, on_done, label):
        """Drive through targets [(x, y[, yaw]), ...]; on_done(ok) fires after the last one,
        ok = the LAST waypoint was actually reached (not timed out)."""
        self._cancel_run()
        if not targets:
            on_done(True)
            return
        self._run = {"targets": list(targets), "idx": 0, "radius": radius, "timeout": timeout,
                     "on_done": on_done, "label": label, "t0": None, "last_ok": False}
        self._send_run_goal()
        self._run_timer = self.create_timer(0.5, self._run_tick)

    def _send_run_goal(self):
        r = self._run
        t = r["targets"][r["idx"]]
        yaw = t[2] if len(t) > 2 else 0.0
        self.send_goal(t[0], t[1], yaw)
        r["t0"] = self.get_clock().now()
        self.get_logger().info(
            f"{r['label']}: driving to waypoint {r['idx'] + 1}/{len(r['targets'])} ({t[0]:.2f},{t[1]:.2f})")

    def _run_tick(self):
        r = self._run
        if r is None:
            return
        t = r["targets"][r["idx"]]
        dist = math.hypot(t[0] - self.x_pos, t[1] - self.y_pos)
        elapsed = (self.get_clock().now() - r["t0"]).nanoseconds / 1e9
        reached = dist <= r["radius"]
        timed_out = elapsed >= r["timeout"]
        if not (reached or timed_out):
            return

        if timed_out and not reached:
            self.get_logger().warn(
                f"{r['label']}: waypoint {r['idx'] + 1} timed out after {elapsed:.0f}s ({dist:.2f} m short)")
        r["last_ok"] = reached
        r["idx"] += 1
        if r["idx"] < len(r["targets"]):
            self._send_run_goal()
            return

        on_done, ok = r["on_done"], r["last_ok"]
        self._cancel_run()
        on_done(ok)

    def _cancel_run(self):
        if self._run_timer is not None:
            self.destroy_timer(self._run_timer)
            self._run_timer = None
        self._run = None

    def _destroy_timer(self, attr):
        t = getattr(self, attr)
        if t is not None:
            self.destroy_timer(t)
            setattr(self, attr, None)

    # ------------------------------------------------------------------
    def _explore_enable(self):
        self._publish_enable_state(
            map_validator=True, pure_pursuit=True, frontier_explorer=True,
            reactive_explorer=True, a_star=True)

    def _drive_only_enable(self):
        # explorers off, planner/controller/validator on
        self._publish_enable_state(
            map_validator=True, pure_pursuit=True, frontier_explorer=False,
            reactive_explorer=False, a_star=True)

    def enter_full_exploration(self):
        # slam_toolbox and map_saver_server are already active from bring-up
        self._cancel_run()
        self._stop_init_pose_wait()
        self.home_pose = None            # re-captured on the first TF in the new state
        self.home_reached = False
        self.odom_travelled_m = 0.0
        self._last_odom_xy = None
        self._set_state("FULL_EXPLORATION")
        self.exploration_started = self.get_clock().now()
        self._explore_enable()

    def _resume_exploration(self):
        # back to exploring after a periodic return-to-start (home is kept)
        self._set_state("FULL_EXPLORATION")
        self._explore_enable()

    def enter_verify_state(self):
        self._cancel_run()
        self.home_pose = None            # robot is now localised in the saved frame
        self.home_reached = False
        self._set_state("VERIFY_SAVED_MAP")
        self._drive_only_enable()

        self.blob_targets = self.compute_large_free_blob_centers(
            self.saved_grid, self.saved_resolution, self.saved_origin)
        self.verify_start_time = self.get_clock().now()
        self.get_logger().info(f"{len(self.blob_targets)} verification waypoints queued")
        self.start_run(self.blob_targets, self.blob_goal_reached_radius, 15.0,
                       lambda ok: self.get_logger().info(
                           "VERIFY: all waypoints visited, holding position while similarity check continues"),
                       "VERIFY")

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

    # ------------------------------------------------------------------
    # SWEEP: with a known map there are no frontiers, so frontier_explorer
    # would declare completion instantly. Visit well-spread free-space
    # points of the saved map instead, so the whole area is re-observed.
    # ------------------------------------------------------------------
    def compute_sweep_waypoints(self, grid, resolution, origin, start_xy,
                                spacing_m=1.5, max_points=15, min_clearance_m=0.35):
        free_clean = ndimage.binary_opening(grid == 0, structure=np.ones((3, 3)))
        labeled, num = ndimage.label(free_clean, structure=np.ones((3, 3)))
        if num == 0:
            return []
        sizes = ndimage.sum(free_clean, labeled, range(1, num + 1))
        main = labeled == (int(np.argmax(sizes)) + 1)

        occ_dil = ndimage.binary_dilation(grid == 100, structure=np.ones((3, 3)))
        clearance = ndimage.distance_transform_edt(~occ_dil) * resolution
        ok = main & (clearance >= min_clearance_m)

        height, width = grid.shape
        pts = []
        for _ in range(12):  # widen the lattice until the waypoint count fits
            step = max(1, int(round(spacing_m / resolution)))
            pts = []
            for r0 in range(0, height, step):
                for c0 in range(0, width, step):
                    block = np.where(ok[r0:r0 + step, c0:c0 + step],
                                     clearance[r0:r0 + step, c0:c0 + step], -1.0)
                    if block.max() < 0:
                        continue
                    br, bc = np.unravel_index(np.argmax(block), block.shape)
                    pts.append((origin[0] + (c0 + bc + 0.5) * resolution,
                                origin[1] + (r0 + br + 0.5) * resolution))
            if len(pts) <= max_points:
                break
            spacing_m *= 1.25

        # greedy nearest-neighbour order from the robot's position
        ordered, cur = [], start_xy
        remaining = pts[:]
        while remaining:
            nxt = min(remaining, key=lambda p: math.hypot(p[0] - cur[0], p[1] - cur[1]))
            ordered.append(nxt)
            remaining.remove(nxt)
            cur = nxt
        return ordered

    def begin_sweep(self):
        self._set_state("SWEEP")
        self._drive_only_enable()
        targets = self.compute_sweep_waypoints(
            self.saved_grid, self.saved_resolution, self.saved_origin,
            (self.x_pos, self.y_pos), self.sweep_spacing_m, self.sweep_max_waypoints)
        self.get_logger().info(f"sweep: {len(targets)} waypoints")
        self.start_run(targets, self.blob_goal_reached_radius, self.sweep_goal_timeout_sec,
                       lambda ok: self.begin_return_home("sweep finished"), "SWEEP")

    # ------------------------------------------------------------------
    # RETURN HOME -> AWAIT_CONFIRM -> SAVING
    # ------------------------------------------------------------------
    def exploration_complete_callback(self, msg: Bool):
        # published every 2 s while no frontiers remain -> only the first one acts
        if msg.data and self.state == "FULL_EXPLORATION":
            if self.exploration_started is not None:
                t = (self.get_clock().now() - self.exploration_started).nanoseconds / 1e9
                if t < self.min_exploration_sec:
                    self.get_logger().warn(
                        f"/exploration_complete after only {t:.0f}s — ignored (min_exploration_sec={self.min_exploration_sec:.0f})",
                        throttle_duration_sec=5.0)
                    return
            self.get_logger().info("exploration complete — returning to start before saving")
            self.begin_return_home("exploration complete")

    def begin_return_home(self, reason):
        if self.state == "RETURN_HOME":
            return
        if self.home_pose is None:
            self.get_logger().error("cannot return home: no home pose was captured")
            return
        self._cancel_run()
        self.return_reason = reason
        self.return_attempt = 0
        self.home_reached = False
        self.return_max_jump_m = 0.0
        self.return_max_jump_rad = 0.0
        self._set_state("RETURN_HOME")
        self._drive_only_enable()        # explorers off while returning
        self._send_home()

    def _send_home(self):
        self.get_logger().info(
            f"returning to start ({self.home_pose[0]:.2f}, {self.home_pose[1]:.2f}) — {self.return_reason}")
        self.start_run([self.home_pose], self.home_reached_radius, self.return_home_timeout_sec,
                       self._on_return_done, "RETURN_HOME")

    def _on_return_done(self, ok):
        if not ok:
            if self.return_attempt < self.return_home_retries:
                self.return_attempt += 1
                self.get_logger().warn(
                    f"start pose not reached — retry {self.return_attempt}/{self.return_home_retries}")
                self._send_home()
                return
            self.get_logger().error(
                "could not get back to the start pose; save is blocked. "
                "Fix the obstruction, then: ros2 service call /manager/return_home std_srvs/srv/Trigger {}")
            if self.return_reason == "periodic":
                self._resume_exploration()
            else:
                self._set_state("AWAIT_CONFIRM")   # home_reached stays False -> save refused
            return

        if self.return_reason == "periodic":
            self._begin_dwell()
            return

        self.home_reached = True
        self.settled = 0                 # only snapshots taken AT home count towards readiness
        self._set_state("AWAIT_CONFIRM")
        self.get_logger().info("back at start — waiting for the map to settle before offering the save")

    # ------------------------------------------------------------------
    # DWELL at home after a periodic return: hold still so slam_toolbox gets
    # scans at the start pose and can close the loop. Resume exploring once
    # the minimum dwell time has passed AND the map has stopped jumping
    # (periodic_dwell_settle clean valid snapshots), or at the max time.
    # ------------------------------------------------------------------
    def _begin_dwell(self):
        self._cancel_run()
        self._set_state("DWELL_HOME")
        self.dwell_start = self.get_clock().now()
        self.settled = 0
        self.return_max_jump_m = 0.0
        self.return_max_jump_rad = 0.0
        self._destroy_timer("_dwell_timer")
        self._dwell_timer = self.create_timer(0.5, self._dwell_tick)
        self.get_logger().info(
            f"back at start — holding {self.periodic_dwell_sec:.0f}s (max {self.periodic_dwell_max_sec:.0f}s) "
            "so SLAM can close the loop")

    def _dwell_tick(self):
        if self.state != "DWELL_HOME":
            self._destroy_timer("_dwell_timer")
            return
        elapsed = (self.get_clock().now() - self.dwell_start).nanoseconds / 1e9
        if elapsed < self.periodic_dwell_sec:
            return
        settled_ok = self.settled >= self.periodic_dwell_settle
        if not settled_ok and elapsed < self.periodic_dwell_max_sec:
            return

        self._destroy_timer("_dwell_timer")
        if settled_ok:
            self.get_logger().info(
                f"dwell done after {elapsed:.0f}s (largest map->odom correction at home: "
                f"{self.return_max_jump_m:.3f} m, {self.return_max_jump_rad:.3f} rad) — resuming exploration")
        else:
            self.get_logger().warn(
                f"dwell hit the {self.periodic_dwell_max_sec:.0f}s cap without a settled valid map — resuming exploration anyway")
        self.odom_travelled_m = 0.0
        self._last_odom_xy = None
        self._resume_exploration()

    def _save_readiness(self):
        if not self.home_reached:
            return False, "robot has not returned to its start pose"
        if self.last_good is None:
            return False, "no valid map yet"
        if self.last_bad_t > self.last_good_t:
            return False, "the latest validation was INVALID"
        if self.settled < self.settle_validations:
            return False, f"map still settling ({self.settled}/{self.settle_validations} clean valid snapshots)"
        return True, "ok"

    def _status_tick(self):
        if self.state == "AWAIT_CONFIRM":
            ok, reason = self._save_readiness()
            if ok:
                self.get_logger().info(
                    f"READY to save (largest map->odom correction on return: {self.return_max_jump_m:.3f} m, "
                    f"{self.return_max_jump_rad:.3f} rad). Confirm with:\n"
                    "  ros2 service call /manager/confirm_save std_srvs/srv/Trigger {}")
            else:
                self.get_logger().info(f"not ready to save: {reason}")
        elif self.state == "PENDING_PROMOTION":
            self.get_logger().info(
                "candidate held back — to promote it anyway: "
                "ros2 service call /manager/promote_candidate std_srvs/srv/Trigger {}")

    def confirm_save_srv(self, request, response):
        if self.state != "AWAIT_CONFIRM":
            response.success = False
            response.message = f"not ready: state is {self.state}"
            return response
        ok, reason = self._save_readiness()
        if not ok:
            response.success = False
            response.message = f"not ready: {reason}"
            return response

        self._set_state("SAVING")
        self.save_confirmed_pub.publish(Bool(data=True))
        self._destroy_timer("_save_timer")
        self._save_timer = self.create_timer(self.save_timeout_sec, self._save_timeout)
        response.success = True
        response.message = "save requested (validator re-checks validity before writing)"
        return response

    def _save_timeout(self):
        self._destroy_timer("_save_timer")
        if self.state == "SAVING":
            self.get_logger().error(
                "no candidate map reported — the validator refused or a save failed (see its log); back to AWAIT_CONFIRM")
            self._set_state("AWAIT_CONFIRM")

    def return_home_srv_cb(self, request, response):
        if self.state not in ("FULL_EXPLORATION", "SWEEP", "AWAIT_CONFIRM"):
            response.success = False
            response.message = f"cannot return home from state {self.state}"
            return response
        if self.home_pose is None:
            response.success = False
            response.message = "no home pose captured"
            return response
        self.begin_return_home("user request")
        response.success = True
        response.message = "returning to start"
        return response

    # ------------------------------------------------------------------
    # CANDIDATE PROMOTION
    # ------------------------------------------------------------------
    def candidate_saved_callback(self, msg: String):
        if self.state != "SAVING":
            self.get_logger().warn(f"candidate {msg.data} reported in state {self.state} — ignored")
            return
        self._destroy_timer("_save_timer")

        ok, detail = self._check_candidate(msg.data)
        if ok:
            self.get_logger().info(f"candidate accepted ({detail})")
            self._promote(msg.data)
        else:
            self.pending_candidate = msg.data
            self._set_state("PENDING_PROMOTION")
            self.get_logger().warn(
                f"candidate NOT promoted automatically: {detail}. The previous map stays current. {msg.data}")

    def _check_candidate(self, base):
        loaded = self._load_map_files(base + ".yaml")
        if loaded is None:
            return False, "candidate map could not be read back"
        if self.saved_grid is None:
            return True, "no previous map to compare against"

        grid, res, origin = loaded
        cand_msg = self._grid_to_msg(grid, res, origin)
        similarity = self.compute_map_similarity_aligned(
            self.saved_grid, self.saved_resolution, self.saved_origin, cand_msg)
        if similarity is None:
            return False, "could not compare with the previous map (resolution/extent mismatch)"

        known_ratio = (int((grid != -1).sum()) / max(1, int((self.saved_grid != -1).sum())))
        self.get_logger().info(
            f"candidate vs previous map: similarity {similarity:.2f}, known-area ratio {known_ratio:.2f}")

        if similarity < self.promote_similarity_threshold:
            return False, f"similarity {similarity:.2f} < {self.promote_similarity_threshold:.2f}"
        if known_ratio < self.promote_min_known_ratio:
            return False, f"candidate covers only {known_ratio:.2f} of the previous map's known area"
        return True, f"similarity {similarity:.2f}, known-area ratio {known_ratio:.2f}"

    def promote_srv_cb(self, request, response):
        if self.state != "PENDING_PROMOTION" or not self.pending_candidate:
            response.success = False
            response.message = "no held-back candidate to promote"
            return response
        self._promote(self.pending_candidate)
        response.success = True
        response.message = f"promoted {self.pending_candidate}"
        return response

    def _promote(self, base):
        os.makedirs(self.saved_maps_dir, exist_ok=True)
        pointer = os.path.join(self.saved_maps_dir, "CURRENT")
        tmp = pointer + ".tmp"
        with open(tmp, "w") as f:
            f.write(base + "\n")
        os.replace(tmp, pointer)   # atomic: a crash can never leave a half-written pointer

        self.pending_candidate = None
        self._set_slam_paused(True, lambda ok: None)
        self._publish_enable_state(map_validator=False, frontier_explorer=False,
                                   reactive_explorer=False, a_star=True)
        self._set_state("COMPLETE")
        self.get_logger().info(f"promoted {base} -> now the map used on the next start")

    # ------------------------------------------------------------------
    # VALIDATION TRACKING + VERIFY
    # ------------------------------------------------------------------
    @staticmethod
    def _stamp_sec(msg: Validatedmap):
        return msg.validation_time.sec + msg.validation_time.nanosec * 1e-9

    def validated_map_callback(self, msg: Validatedmap):
        self._verify_step(msg)

        if msg.counter == self.last_good_counter:
            return  # validator republishes the same map every second
        self.last_good_counter = msg.counter
        self.last_good = msg
        self.last_good_t = self._stamp_sec(msg)

        if msg.loop_closure_suspected:
            self.settled = 0
            self.get_logger().info(
                f"map correction seen (map->odom {msg.map_odom_jump_m:.3f} m, {msg.map_odom_jump_rad:.3f} rad)")
        else:
            self.settled += 1

        if self.state in ("RETURN_HOME", "AWAIT_CONFIRM", "DWELL_HOME"):
            self.return_max_jump_m = max(self.return_max_jump_m, msg.map_odom_jump_m)
            self.return_max_jump_rad = max(self.return_max_jump_rad, msg.map_odom_jump_rad)

    def discarded_map_callback(self, msg: Validatedmap):
        if msg.counter == self.last_bad_counter:
            return
        self.last_bad_counter = msg.counter
        self.last_bad_t = self._stamp_sec(msg)
        self.settled = 0

    def _verify_step(self, msg: Validatedmap):
        if self.state != "VERIFY_SAVED_MAP":
            return

        elapsed = (self.get_clock().now() - self.verify_start_time).nanoseconds / 1e9
        similarity = self.compute_map_similarity_aligned(self.saved_grid, self.saved_resolution,
                                                           self.saved_origin, msg.map)

        if similarity is not None:
            self.get_logger().info(f"map similarity so far: {similarity:.2f} (elapsed {elapsed:.0f}s)")

        if elapsed >= self.verify_timeout_sec:
            if similarity is not None and similarity >= self.similarity_threshold:
                if self.sweep_on_resume:
                    self.get_logger().info("saved map verified — sweeping the area to re-scan it")
                    self.begin_sweep()
                else:
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
        self._cancel_run()
        self._set_state("FROZEN_REUSE")
        self._set_slam_paused(True, lambda ok: None)
        self._publish_enable_state(map_validator=False, a_star=True)
        self.get_logger().info("map confirmed unchanged — pausing SLAM/validator, reusing saved map")


def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
