#!/usr/bin/env python3
"""
pure_pursuit.py -- path tracker for the octile A* (global_planners/astar2.py).

Pipeline
  /path (8-connected grid, 0.05 m cells)
     -> drop duplicates -> Douglas-Peucker (bounded deviation, `simplify_tol`)
     -> resample at fixed arc length (`resample_step`)
     -> curvature (3-point Menger over `curvature_window`)
     -> speed profile: wheel-speed cap vs curvature + decel-limited approach to the end
  /goal_pose -> final heading (matched to the path end)
  50 Hz loop: pose (TF map->roomba, stamp-checked) -> project onto path ->
     lookahead point (tangent extension past the end) -> alpha gate / speed / omega ->
     obstacle layer -> slew limiter -> /autonomous_cmd_vel

State machine: IDLE -> FOLLOW (with ROTATE sub-mode) -> ALIGN (optional) -> DONE | LOST

Review items implemented (numbers from the review):
  1  heading-error gate: rotate in place above `rotate_enter_deg` (hysteresis), v scaled by cos(alpha)
  2  pose staleness from the TF header stamp, not from "lookup succeeded"
  3  /scan watchdog (`scan_timeout_sec`)
  4  corner cutting limited: shorter lookahead, bounded path simplification, cross-track check
     (the clearance_radius increase itself belongs in astar2.py)
  5  path resampled at fixed spacing; speed from curvature over ~0.4 m, not from 3 grid steps
  6  every speed limit is applied BEFORE omega = v * kappa, to a local target (no state mutation)
  7  avoidance triggered only by a front corridor of robot width (doorway-safe), single-beam
     rejection, fast-attack / rate-limited release, urgency applied once, sticky direction
  8  accel / decel slew limiter sized to the firmware ramps (600 / 1200 deg/s2 at r = 0.035 m)
  9  lookahead 0.25 .. 0.50 m (0.25 + 0.6 v)
 10  tangent extension past the end, stop on along-track distance, final heading from /goal_pose
 11  omega limited by 2 (v_lim - |v|) / track, and v capped so that omega = v*kappa fits
"""
import math

import numpy as np
import rclpy
from custom_interfaces.msg import NodeEnableStates
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion


# ----------------------------------------------------------------------
# Pure helpers (no ROS) -- easy to unit test
# ----------------------------------------------------------------------
def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def simplify_dp(pts, tol):
    """Iterative Douglas-Peucker. Max deviation from the original polyline <= tol."""
    n = len(pts)
    if n < 3:
        return pts
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = pts[i], pts[j]
        ab = b - a
        length = math.hypot(ab[0], ab[1])
        seg = pts[i + 1:j]
        if length < 1e-9:
            d = np.hypot(seg[:, 0] - a[0], seg[:, 1] - a[1])
        else:
            d = np.abs(ab[0] * (seg[:, 1] - a[1]) - ab[1] * (seg[:, 0] - a[0])) / length
        k = int(np.argmax(d))
        if d[k] > tol:
            m = i + 1 + k
            keep[m] = True
            stack.append((i, m))
            stack.append((m, j))
    return pts[keep]


def resample(pts, step):
    """Uniform arc-length resampling. Returns (points, cumulative arc length)."""
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    s = np.concatenate(([0.0], np.cumsum(seg)))
    total = s[-1]
    n = max(int(math.ceil(total / step)), 1)
    s_new = np.linspace(0.0, total, n + 1)
    out = np.column_stack((np.interp(s_new, s, pts[:, 0]), np.interp(s_new, s, pts[:, 1])))
    return out, s_new


def menger_curvature(pts, k):
    """Signed curvature (+ = left turn) from points i-k, i, i+k. Zero at the ends."""
    n = len(pts)
    idx = np.arange(n)
    a = pts[np.clip(idx - k, 0, n - 1)]
    c = pts[np.clip(idx + k, 0, n - 1)]
    b = pts
    ab, bc, ac = b - a, c - b, c - a
    cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
    den = np.hypot(ab[:, 0], ab[:, 1]) * np.hypot(bc[:, 0], bc[:, 1]) * np.hypot(ac[:, 0], ac[:, 1])
    return np.where(den > 1e-9, 2.0 * cross / np.maximum(den, 1e-9), 0.0)


def build_speed_profile(s, kappa, v_base, v_lim, track, a_dec, v_end, goal_tol):
    """Per-point speed limit.
    - curvature: v (1 + |k| track/2) <= v_lim keeps the faster wheel under the cap
      without relay.py having to rescale (rescaling would lower v behind our back)
    - end: v <= sqrt(v_end^2 + 2 a_dec (distance_to_end - goal_tol)), so the robot arrives
      at the goal tolerance at creep speed
    - backward pass: decel-limited, so slow-downs start before the curve
    """
    cap = v_lim / (1.0 + np.abs(kappa) * track / 2.0)
    v = np.minimum(v_base, cap)
    rem = np.maximum(s[-1] - s - goal_tol, 0.0)
    v = np.minimum(v, np.sqrt(v_end ** 2 + 2.0 * a_dec * rem))
    for i in range(len(v) - 2, -1, -1):
        ds = s[i + 1] - s[i]
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * a_dec * ds))
    return v


def _kth_min(vals, k, default):
    """k-th smallest value; fewer than k beams => treated as noise (default)."""
    if vals.size < k:
        return default
    return float(np.partition(vals, k - 1)[k - 1])


def _required_float(node, name):
    """Geometry/limits must come from robot_dimensions.yaml (no silent defaults)."""
    node.declare_parameter(name, Parameter.Type.DOUBLE)
    try:
        value = node.get_parameter(name).value
    except Exception:
        value = None
    if value is None:
        raise RuntimeError(
            f"Parameter '{name}' is not set. Load "
            "program_bringup/config/robot_dimensions.yaml into this node.")
    return float(value)


# ----------------------------------------------------------------------
class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")

        # ---- shared robot limits / geometry (robot_dimensions.yaml) ----
        self.v_lim = _required_float(self, "limits.max_wheel_speed_m_s")
        self.track = _required_float(self, "robot.drive_wheel.track_width_m")
        footprint_w = _required_float(self, "robot.footprint.width_m")
        if self.v_lim <= 0.0 or self.track <= 0.0:
            raise RuntimeError("robot_dimensions.yaml (limits / track width) invalid")
        pure_spin = 2.0 * self.v_lim / self.track      # rad/s with the faster wheel at the cap

        # ---- tunables ----
        p = self._param
        self.lookahead = p("lookahead_distance", 0.25)
        self.min_lookahead = p("min_lookahead", 0.25)
        self.max_lookahead = p("max_lookahead", 0.50)
        self.k_lookahead = p("k_lookahead", 0.6)

        base = p("base_velocity", 0.0)                 # 0 -> wheel-speed limit
        self.base_velocity = min(base, self.v_lim) if base > 0.0 else self.v_lim
        wmax = p("max_angular_vel", 0.0)               # 0 -> pure-spin limit
        self.w_cap = min(wmax, pure_spin) if wmax > 0.0 else pure_spin
        self.creep_velocity = p("creep_velocity", 0.03)
        self.max_accel = p("max_accel", 0.30)          # firmware: 600 deg/s2 = 0.37 m/s2
        self.max_decel = p("max_decel", 0.50)          # firmware: 1200 deg/s2 = 0.73 m/s2
        self.profile_decel = p("profile_decel", 0.30)

        self.tf_timeout = p("tf_timeout_sec", 0.5)
        self.scan_timeout = p("scan_timeout_sec", 0.5)  # <= 0 disables

        self.rot_enter = math.radians(p("rotate_enter_deg", 70.0))
        self.rot_exit = math.radians(p("rotate_exit_deg", 30.0))
        self.rot_gain = p("rotate_gain", 1.5)
        self.rot_w_max = min(p("rotate_angular_vel", 1.0), self.w_cap)
        self.rot_w_min = min(p("rotate_min_angular_vel", 0.25), self.rot_w_max)  # motor deadband

        self.simplify_tol = p("simplify_tol", 0.03)
        self.resample_step = max(p("resample_step", 0.10), 0.02)
        self.curv_window = p("curvature_window", 0.40)
        self.search_window = p("nearest_search_window", 1.0)
        self.max_cross_track = p("max_cross_track", 0.75)

        self.goal_tol = p("goal_tolerance", 0.07)
        self.align_final = bool(p("align_final_heading", True))
        self.yaw_tol = math.radians(p("yaw_tolerance_deg", 6.0))
        self.align_timeout = p("align_timeout_sec", 10.0)
        self.goal_match_tol = p("goal_match_tolerance", 0.25)

        self.avoid_start = p("avoid_start_distance", 0.8)
        self.avoid_stop = p("avoid_stop_distance", 0.3)
        self.corridor_half = footprint_w / 2.0 + p("corridor_margin", 0.05)
        self.outlier_beams = max(int(p("scan_outlier_beams", 2)), 1)
        self.release_rate = p("avoid_release_rate", 1.0)           # m/s
        self.avoid_w = min(p("avoid_angular_vel", 0.8), self.w_cap)
        self.clear_default = p("clear_default", 10.0)
        # Hardware URDF: lidar 0.10 m ahead of base_link. Sim lidar.xacro uses x = 0 ->
        # run the sim with lidar_offset_x:=0.0.
        self.lidar_offset_x = p("lidar_offset_x", 0.0)
        rate = p("control_rate_hz", 50.0)

        # ---- state ----
        self.enabled = False                 # sole authority: manager_node
        self.state = "IDLE"
        self._was_active = False
        self._zero_burst = 0
        self._last_tick = None

        self.x_pos = self.y_pos = self.yaw = 0.0
        self.v = 0.0
        self.rotating = False
        self.avoid_dir = 0

        self.X = self.Y = self.S = self.V = None
        self.end_tx, self.end_ty = 1.0, 0.0
        self.prev_stamp = None
        self.last_idx = 0
        self.first_projection = True

        self.latest_goal = None              # (x, y, yaw) from /goal_pose
        self.path_goal_yaw = None
        self.align_start = None

        self.front_clearance = self.clear_default
        self.left_clearance = self.clear_default
        self.right_clearance = self.clear_default
        self.last_scan_time = None

        # ---- ROS interfaces ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        enable_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, "/path", self.path_callback, path_qos)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.laser_callback, qos_profile_sensor_data)
        self.create_subscription(
            NodeEnableStates, "/manager/enable_states", self.enable_callback, enable_qos)
        self.publisher_ = self.create_publisher(Twist, "/autonomous_cmd_vel", 10)
        self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            f"pure_pursuit ready: v_max={self.base_velocity:.2f} m/s, w_cap={self.w_cap:.2f} rad/s, "
            f"corridor half-width={self.corridor_half:.2f} m")

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def enable_callback(self, msg: NodeEnableStates):
        self.enabled = msg.pure_pursuit

    def goal_callback(self, msg: PoseStamped):
        """Remember the goal pose. The path only carries positions; the heading comes from here."""
        if msg.header.frame_id not in ("map", ""):
            self.get_logger().warn(
                f"/goal_pose in frame '{msg.header.frame_id}', expected 'map' -- ignored for heading",
                throttle_duration_sec=5.0)
            return
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_goal = (msg.pose.position.x, msg.pose.position.y, yaw)
        self._match_goal()            # covers "goal arrives after its path"

    def _match_goal(self):
        if self.latest_goal is None or self.X is None:
            return
        gx, gy, gyaw = self.latest_goal
        if math.hypot(gx - self.X[-1], gy - self.Y[-1]) <= self.goal_match_tol:
            self.path_goal_yaw = gyaw

    def path_callback(self, msg: Path):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.prev_stamp:
            return
        self.prev_stamp = stamp
        if msg.header.frame_id not in ("map", ""):
            self.get_logger().warn(f"/path in frame '{msg.header.frame_id}', expected 'map' -- ignored")
            return

        pts = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses],
                       dtype=float).reshape(-1, 2)
        if len(pts) >= 2:
            keep = np.concatenate(([True], np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1])) > 1e-6))
            pts = pts[keep]
        if len(pts) < 2:
            self.X = self.Y = self.S = self.V = None
            self.state = "IDLE"
            self.get_logger().warn("received a path with fewer than 2 distinct points -- ignored")
            return

        pts = simplify_dp(pts, self.simplify_tol)
        pts, s = resample(pts, self.resample_step)
        ds = max(float(s[-1]) / max(len(s) - 1, 1), 1e-6)
        k = max(1, int(round(self.curv_window / 2.0 / ds)))
        kappa = menger_curvature(pts, k)

        self.X, self.Y, self.S = pts[:, 0].copy(), pts[:, 1].copy(), s
        self.V = build_speed_profile(
            s, kappa, self.base_velocity, self.v_lim, self.track,
            self.profile_decel, self.creep_velocity, self.goal_tol)

        # Tangent at the end, used to extend the path so the lookahead stays defined.
        j = max(len(pts) - 1 - k, 0)
        tx, ty = pts[-1, 0] - pts[j, 0], pts[-1, 1] - pts[j, 1]
        norm = math.hypot(tx, ty)
        self.end_tx, self.end_ty = (tx / norm, ty / norm) if norm > 1e-9 else (1.0, 0.0)

        self.path_goal_yaw = None
        self._match_goal()

        self.last_idx = 0
        self.first_projection = True
        self.rotating = False
        self.avoid_dir = 0
        self.state = "FOLLOW"             # self.v is kept: a replan mid-motion stays smooth
        self.get_logger().info(
            f"new path: {len(msg.poses)} grid pts -> {len(pts)} pts, {s[-1]:.2f} m, "
            f"goal heading {'%.0f deg' % math.degrees(self.path_goal_yaw) if self.path_goal_yaw is not None else 'n/a'}")

    def laser_callback(self, msg: LaserScan):
        now = self.get_clock().now()
        r = np.asarray(msg.ranges, dtype=np.float64)
        if r.size == 0:
            return
        ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
        rmin = max(float(getattr(msg, "range_min", 0.0)), 1e-3)
        rmax = msg.range_max if msg.range_max > 0.0 else np.inf
        ok = np.isfinite(r) & (r > rmin) & (r <= rmax)      # 0.0 / < range_min = no return
        x = r[ok] * np.cos(ang[ok]) + self.lidar_offset_x   # base_link frame
        y = r[ok] * np.sin(ang[ok])
        dist = np.hypot(x, y)
        bearing = np.arctan2(y, x)

        in_corridor = (x > 0.0) & (np.abs(y) <= self.corridor_half)
        left = (bearing >= math.radians(35.0)) & (bearing <= math.radians(120.0))
        right = (bearing <= -math.radians(35.0)) & (bearing >= -math.radians(120.0))
        new = (
            _kth_min(x[in_corridor], self.outlier_beams, self.clear_default),
            _kth_min(dist[left], self.outlier_beams, self.clear_default),
            _kth_min(dist[right], self.outlier_beams, self.clear_default),
        )

        dt = 0.1
        if self.last_scan_time is not None:
            dt = min(max((now - self.last_scan_time).nanoseconds * 1e-9, 0.01), 0.5)
        self.last_scan_time = now

        def attack_release(old, val):    # closer -> immediately, farther -> rate limited
            return val if val <= old else min(val, old + self.release_rate * dt)

        self.front_clearance = attack_release(self.front_clearance, new[0])
        self.left_clearance = attack_release(self.left_clearance, new[1])
        self.right_clearance = attack_release(self.right_clearance, new[2])

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.publisher_.publish(msg)

    def _flush_zero_burst(self):
        # A few zeros after stopping (survives a lost message), then silence so the
        # keyboard (priority 80) is not locked out by this node (priority 90).
        if self._zero_burst > 0:
            self._publish(0.0, 0.0)
            self._zero_burst -= 1

    def _reset_motion(self):
        self.v = 0.0
        self.rotating = False
        self.avoid_dir = 0

    def _safe_stop(self, reason):
        """Fault: hold the robot (zeros every tick) until the fault clears."""
        self._reset_motion()
        self._publish(0.0, 0.0)
        self.get_logger().warn(f"holding: {reason}", throttle_duration_sec=1.0)

    def _finish(self, state, msg, warn=False):
        self.state = state
        self._reset_motion()
        self._zero_burst = 5
        self._publish(0.0, 0.0)
        self._zero_burst -= 1
        (self.get_logger().warn if warn else self.get_logger().info)(msg)

    def _update_pose(self):
        """map->roomba, rejected if its own timestamp is older than tf_timeout.
        Time() = 'latest available'; the stamp of that result is what tells us the EKF/SLAM
        are still alive. Zero timeout, so this never blocks the control loop."""
        try:
            t = self.tf_buffer.lookup_transform("map", "roomba", Time())
        except TransformException as ex:
            self.get_logger().warn(f"TF unavailable: {ex}", throttle_duration_sec=2.0)
            return False
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        if self.tf_timeout > 0.0 and age > self.tf_timeout:
            self.get_logger().warn(f"map->roomba is {age:.2f} s old", throttle_duration_sec=1.0)
            return False
        self.x_pos = t.transform.translation.x
        self.y_pos = t.transform.translation.y
        q = t.transform.rotation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return True

    def _scan_is_stale(self, now):
        if self.scan_timeout <= 0.0:
            return False
        if self.last_scan_time is None:
            return True
        return (now - self.last_scan_time).nanoseconds * 1e-9 > self.scan_timeout

    def _project(self):
        """Nearest point on the path -> (arc length s, cross-track distance).
        Searches a window ahead of the last index (global only on a new path), then projects
        onto the two adjacent segments for sub-spacing accuracy."""
        n = len(self.X)
        if self.first_projection:
            lo, hi = 0, n
            self.first_projection = False
        else:
            lo = self.last_idx
            hi = min(n, int(np.searchsorted(self.S, self.S[lo] + self.search_window)) + 1)
        d2 = (self.X[lo:hi] - self.x_pos) ** 2 + (self.Y[lo:hi] - self.y_pos) ** 2
        i = lo + int(np.argmin(d2))
        self.last_idx = i

        best, s_proj = float("inf"), float(self.S[i])
        for j in (i - 1, i):
            if j < 0 or j + 1 >= n:
                continue
            ax, ay = self.X[j], self.Y[j]
            dx, dy = self.X[j + 1] - ax, self.Y[j + 1] - ay
            l2 = max(dx * dx + dy * dy, 1e-12)
            t = min(max(((self.x_pos - ax) * dx + (self.y_pos - ay) * dy) / l2, 0.0), 1.0)
            d = math.hypot(self.x_pos - (ax + t * dx), self.y_pos - (ay + t * dy))
            if d < best:
                best, s_proj = d, float(self.S[j] + t * math.sqrt(l2))
        return s_proj, best

    def _point_at(self, s):
        """Path point at arc length s; beyond the end, extend along the end tangent."""
        if s <= self.S[-1]:
            return float(np.interp(s, self.S, self.X)), float(np.interp(s, self.S, self.Y))
        e = s - self.S[-1]
        return self.X[-1] + e * self.end_tx, self.Y[-1] + e * self.end_ty

    def _obstacle_scale(self):
        span = max(self.avoid_start - self.avoid_stop, 1e-6)
        return min(max((self.front_clearance - self.avoid_stop) / span, 0.0), 1.0)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def control_loop(self):
        now = self.get_clock().now()
        dt = 0.02 if self._last_tick is None else (now - self._last_tick).nanoseconds * 1e-9
        dt = min(max(dt, 1e-3), 0.1)
        self._last_tick = now

        if not self.enabled:
            if self._was_active:
                self._was_active = False
                self._reset_motion()
                self._zero_burst = 5
            self._flush_zero_burst()
            return
        self._was_active = True

        if self.X is None or self.state in ("IDLE", "DONE", "LOST"):
            self._flush_zero_burst()
            return

        if not self._update_pose():
            self._safe_stop("pose stale or unavailable")
            return
        if self._scan_is_stale(now):
            self._safe_stop("no recent /scan")
            return

        if self.state == "ALIGN":
            self._align_step(now)
            return

        # ---------------- FOLLOW ----------------
        s_proj, cte = self._project()
        remaining = float(self.S[-1]) - s_proj
        dist_end = math.hypot(self.X[-1] - self.x_pos, self.Y[-1] - self.y_pos)

        if remaining <= self.goal_tol or dist_end <= self.goal_tol:
            self._position_reached(now)
            return
        if cte > self.max_cross_track:
            self._finish("LOST", f"{cte:.2f} m off the path -- stopped, waiting for a new path", warn=True)
            return

        # lookahead point (item 9 + 10)
        L_d = min(max(self.lookahead + self.k_lookahead * self.v, self.min_lookahead), self.max_lookahead)
        gx, gy = self._point_at(s_proj + L_d)
        dx, dy = gx - self.x_pos, gy - self.y_pos
        c, sn = math.cos(self.yaw), math.sin(self.yaw)
        x_r = c * dx + sn * dy
        y_r = -sn * dx + c * dy
        L_act = math.hypot(x_r, y_r)
        alpha = math.atan2(y_r, x_r) if L_act > 1e-3 else 0.0

        # alpha gate with hysteresis (item 1)
        if self.rotating:
            if abs(alpha) < self.rot_exit:
                self.rotating = False
        elif abs(alpha) > self.rot_enter:
            self.rotating = True
            self.get_logger().info(f"target {math.degrees(alpha):+.0f} deg off heading -- rotating in place")

        v_scale = self._obstacle_scale()
        urgency = 1.0 - v_scale                       # applied exactly once (item 7)
        if urgency > 0.0:
            if self.avoid_dir == 0:                   # sticky for the whole episode
                self.avoid_dir = 1 if self.left_clearance >= self.right_clearance else -1
        else:
            self.avoid_dir = 0

        if self.rotating:
            v_target, kappa, v_cap = 0.0, 0.0, self.base_velocity
        else:
            kappa = 2.0 * y_r / (L_act ** 2) if L_act > 1e-3 else 0.0
            k_abs = abs(kappa)
            # caps that make omega = v*kappa legal (item 11): wheel-speed cap and omega cap
            v_cap = self.v_lim / (1.0 + k_abs * self.track / 2.0)
            if k_abs > 1e-6:
                v_cap = min(v_cap, self.w_cap / k_abs)
            v_prof = float(np.interp(s_proj, self.S, self.V))
            # every limit is applied to a LOCAL target before omega is derived (item 6)
            v_target = min(v_prof * max(math.cos(alpha), 0.0), v_cap) * v_scale

        # slew limiter (item 8)
        if v_target > self.v:
            self.v = min(v_target, self.v + self.max_accel * dt)
        else:
            self.v = max(v_target, self.v - self.max_decel * dt)
        self.v = min(self.v, v_cap)                    # hard guard, never exceeds the caps

        if self.rotating:
            w = math.copysign(
                min(max(self.rot_gain * abs(alpha), self.rot_w_min), self.rot_w_max), alpha)
        else:
            w_pp = self.v * kappa                      # curvature preserved, v already final
            w = (1.0 - urgency) * w_pp + urgency * self.avoid_dir * self.avoid_w

        w_lim = min(self.w_cap, 2.0 * max(self.v_lim - abs(self.v), 0.0) / self.track)
        w = min(max(w, -w_lim), w_lim)
        self._publish(self.v, w)

    def _position_reached(self, now):
        if self.align_final and self.path_goal_yaw is not None:
            self.state = "ALIGN"
            self.align_start = now
            self._reset_motion()
            self._publish(0.0, 0.0)
            self.get_logger().info(
                f"position reached -- aligning to goal heading {math.degrees(self.path_goal_yaw):+.0f} deg")
        else:
            self._finish("DONE", "goal reached")

    def _align_step(self, now):
        err = wrap_pi(self.path_goal_yaw - self.yaw)
        if abs(err) <= self.yaw_tol:
            self._finish("DONE", f"goal reached, heading error {math.degrees(err):+.1f} deg")
        elif (now - self.align_start).nanoseconds * 1e-9 > self.align_timeout:
            self._finish("DONE", f"heading alignment timed out, error {math.degrees(err):+.1f} deg",
                         warn=True)
        else:
            w = math.copysign(
                min(max(self.rot_gain * abs(err), self.rot_w_min), self.rot_w_max), err)
            self._publish(0.0, w)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()