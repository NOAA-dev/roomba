#!/usr/bin/env python3
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
from local_planners.trajectory_generator import TrajectoryConfig, dedupe_points, generate_trajectory

def _required_float(node, name):
    """Read a geometry parameter that must come from robot_dimensions.yaml.

    There is deliberately no default: robot geometry has a single source of
    truth (program_bringup/config/robot_dimensions.yaml), so a launch that
    forgets to load it fails loudly instead of silently using stale numbers.
    """
    node.declare_parameter(name, Parameter.Type.DOUBLE)
    try:
        value = node.get_parameter(name).value
    except Exception:
        value = None
    if value is None:
        raise RuntimeError(
            f"Parameter '{name}' is not set. Load "
            "program_bringup/config/robot_dimensions.yaml into this node "
            "(<param from=...> / parameters=[...]).")
    return float(value)

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit")

        self.enabled = False
        self.pose_valid = False
        self.pose_stamp = Time()
        self.pose_timeout = 0.5   # [s]
        self.scan_stamp = None
        self.scan_timeout = 0.5   # [s]

        self.L_ = _required_float(self, "robot.drive_wheel.track_width_m")
        self.R_ = _required_float(self, "robot.drive_wheel.radius_m")
        self.max_wheel_speed = _required_float(self, "limits.max_wheel_speed_m_s")

        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0
        self.lookahead = 3  

        self.goal = None            # latest goal (x, y, yaw), may be missing or stale
        self.park_target = None     # (x, y, yaw or None), derived from traj end + goal
        self.path_key = None        # fingerprint of the current path
        self.goal_match_tol = 0.15  # [m] goal counts as "this path's goal" if within this of the path end
        self.path_start_tol = 0.5   # [m] a fresh path starts near the robot
        self.park_release = 0.3     # [m] leave the parking latch if pushed this far from the end
        self.traj = None
        self.cfg = TrajectoryConfig(v_lim=self.max_wheel_speed, track=self.L_)
        self.last_i = 0
        self.search_window = 40      # points ahead of the last index (40 x 0.10 m = 4 m)

        self.lo_deg, self.hi_deg, self.step_deg = -90.0, 90.0, 5.0
        self.scan_dist = np.array([], dtype=np.float64)
        self.scan_theta = np.array([], dtype=np.float64)
        self.stop = False

        self.pos_tol, self.pos_exit = 0.015, 0.03                                  # enter / leave parking [m]
        self.yaw_tol, self.yaw_exit = math.radians(2.0), math.radians(4.0)         # done / un-done
        self.rot_enter, self.rot_exit = math.radians(20.0), math.radians(5.0)      # turn-in-place hysteresis
        self.reverse_zone, self.steer_fade = 0.05, 0.05                            # [m]
        self.k_v, self.v_max, self.v_min = 3.0, 0.1, 0.02                          # [1/s], [m/s]
        self.k_w, self.k_steer = 1.5, 1.5                                          # [1/s]
        self.w_max, self.w_min, self.w_steer_max = 1.0, 0.25, 0.5                  # [rad/s]
        self.park_phase = "APPROACH"      # reset to "APPROACH" and park_rotating = False on every new goal
        self.park_rotating = False
        self.once = False
        self.align_rotating = False
        self.align_dir = 1.0
        self.align_enter, self.align_exit = math.radians(30.0), math.radians(10.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Path, "/path", self.path_callback, qos)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.laser_callback, qos_profile_sensor_data)
        self.create_subscription(NodeEnableStates, "/manager/enable_states", self.enable_callback, qos)

        self.publisher_ = self.create_publisher(Twist, "/autonomous_cmd_vel", 10)

        self.tf_timer = self.create_timer(0.01, self.check_tf)
        self.control_timer = self.create_timer(0.05, self.control_loop)   # 20 Hz

    def enable_callback(self, msg: NodeEnableStates):
        self.enabled = msg.pure_pursuit

    def goal_callback(self, msg: PoseStamped):
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.goal = (msg.pose.position.x, msg.pose.position.y, yaw)

        if self.traj is not None:
            d = math.hypot(self.goal[0] - self.traj.x[-1], self.goal[1] - self.traj.y[-1])
            if d > self.goal_match_tol:
                # new destination and no path for it yet (A* may fail): stop, don't keep driving the old one
                self.traj = None
                self.path_key = None
                self.reset_parking()
        self.update_park_target()

    def reset_parking(self):
        self.once = False
        self.park_phase = "APPROACH"
        self.park_rotating = False
        self.align_rotating = False

    def update_park_target(self):
        if self.traj is None:
            self.park_target = None
            return
        ex, ey = float(self.traj.x[-1]), float(self.traj.y[-1])
        g = self.goal
        if g is not None and math.hypot(g[0] - ex, g[1] - ey) < self.goal_match_tol:
            self.park_target = g                    # exact goal position + yaw
        else:
            self.park_target = (ex, ey, None)       # goal missed: park on the path end, no final turn

    def check_tf(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "roomba", Time())
        except TransformException:
            return
        self.x_pos = t.transform.translation.x
        self.y_pos = t.transform.translation.y
        q = t.transform.rotation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_stamp = Time.from_msg(t.header.stamp)
        self.pose_valid = True

    def pose_ok(self):
        if not self.pose_valid:
            return False
        age = (self.get_clock().now() - self.pose_stamp).nanoseconds * 1e-9
        return age < self.pose_timeout
    
    def path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            return
        pts = dedupe_points(np.array([[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses]))
        if len(pts) < 2:
            return

        key = pts.round(3).tobytes()
        if key == self.path_key:
            return                                  # same path republished, nothing changed

        # a stale latched path (transient local replay) won't start near the robot
        if not self.pose_ok() or math.hypot(pts[0, 0] - self.x_pos, pts[0, 1] - self.y_pos) > self.path_start_tol:
            return

        ex, ey = float(pts[-1, 0]), float(pts[-1, 1])
        new_dest = (self.traj is None or
                    math.hypot(ex - self.traj.x[-1], ey - self.traj.y[-1]) > self.goal_match_tol)

        matched = (self.goal is not None and math.hypot(self.goal[0] - ex, self.goal[1] - ey) < self.goal_match_tol)
        traj_goal = self.goal if matched else None

        self.path_key = key
        self.traj = generate_trajectory(pts, traj_goal, self.cfg)
        self.last_i = 0
        if new_dest:
            self.reset_parking()
        self.update_park_target()

    def scan_ok(self):
        if self.scan_stamp is None:
            return False
        return (self.get_clock().now() - self.scan_stamp).nanoseconds * 1e-9 < self.scan_timeout
    
    def laser_callback(self, msg: LaserScan):

        r = np.asarray(msg.ranges, dtype=np.float64)
        if r.size == 0:
            return
        self.scan_stamp = self.get_clock().now()

        ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
        rmin = max(float(getattr(msg, "range_min", 0.0)), 1e-3)
        rmax = msg.range_max if msg.range_max > 0.0 else np.inf
        ok = np.isfinite(r) & (r > rmin) & (r <= rmax)
        x = r[ok] * np.cos(ang[ok])
        y = r[ok] * np.sin(ang[ok])
        dist = np.hypot(x, y)
        theta = np.arctan2(y, x)
        front = np.abs(theta) < math.radians(90)
        self.stop = bool(np.any(dist[front] < 0.18))

        lo, hi, step = np.radians(self.lo_deg), np.radians(self.hi_deg), np.radians(self.step_deg)
        n_bins = int(round((hi - lo) / step))

        theta_out = []
        dist_out = []
        for b in range(n_bins):
            start = lo + b * step
            end = start + step
            if b == n_bins - 1:
                in_bin = (theta >= start) & (theta <= end)     # last bin includes the hi edge
            else:
                in_bin = (theta >= start) & (theta < end)

            if np.any(in_bin):                                  # skip bins with no valid beams
                theta_out.append(np.mean(theta[in_bin]))
                dist_out.append(np.mean(dist[in_bin]))

        self.scan_dist = np.array(dist_out, dtype=np.float64)
        self.scan_theta = np.array(theta_out, dtype=np.float64)

    def nearest_index(self):
        n = len(self.traj.x)
        self.last_i = min(self.last_i, n - 1)
        lo = max(self.last_i - 5, 0)
        hi = min(self.last_i + self.search_window, n)
        d2 = (self.traj.x[lo:hi] - self.x_pos) ** 2 + (self.traj.y[lo:hi] - self.y_pos) ** 2
        i = lo + int(np.argmin(d2))
        d = math.hypot(self.traj.x[i] - self.x_pos, self.traj.y[i] - self.y_pos)
        if d > 0.5:                                  # lost the window, search the whole path
            d2 = (self.traj.x - self.x_pos) ** 2 + (self.traj.y - self.y_pos) ** 2
            i = int(np.argmin(d2))
            d = math.sqrt(float(d2[i]))
        self.last_i = i
        return i, d
        

    def check_goal_end_behavior(self):
        d = math.hypot(self.traj.x[-1] - self.x_pos, self.traj.y[-1] - self.y_pos)
        if self.once and d > self.park_release:
            self.once = False
        return self.once or d < 0.07


    def parking_behavior(self):
        gx, gy, gyaw = self.park_target                  
        dx, dy = gx - self.x_pos, gy - self.y_pos
        dist = math.hypot(dx, dy)

        # phase changes with hysteresis (no chatter at the tolerance edge)
        if self.park_phase == "APPROACH":
            if dist < self.pos_tol:
                self.park_phase = "ALIGN"
        elif dist > self.pos_exit:
            self.park_phase = "APPROACH"

        if self.park_phase != "APPROACH":                          # ALIGN / DONE
            err = 0.0 if gyaw is None else math.atan2(math.sin(gyaw - self.yaw), math.cos(gyaw - self.yaw))
            tol = self.yaw_exit if self.park_phase == "DONE" else self.yaw_tol
            if abs(err) <= tol:
                self.park_phase = "DONE"
                self.v = 0.0
                self.w = 0.0
                return
            self.park_phase = "ALIGN"
            w = float(np.clip(self.k_w * err, -self.w_max, self.w_max))
            if abs(w) < self.w_min:
                w = math.copysign(self.w_min, err)
            self.v = 0.0
            self.w = w  
            return

        # APPROACH: bearing to the goal in the robot frame
        bearing = math.atan2(math.sin(math.atan2(dy, dx) - self.yaw), math.cos(math.atan2(dy, dx) - self.yaw))
        direction = 1.0
        if abs(bearing) > math.pi / 2 and dist < self.reverse_zone:
            bearing = math.atan2(math.sin(bearing + math.pi), math.cos(bearing + math.pi))                   # goal just behind: back up
            direction = -1.0

        if self.park_rotating:
            if abs(bearing) < self.rot_exit:
                self.park_rotating = False
        elif abs(bearing) > self.rot_enter:
            self.park_rotating = True

        if self.park_rotating:                                     # turn in place, proportional
            w = float(np.clip(self.k_w * bearing, -self.w_max, self.w_max))
            if abs(w) < self.w_min:
                w = math.copysign(self.w_min, bearing)
            self.v = 0.0
            self.w = w
        else:                                                      # drive: v ∝ distance, w ∝ bearing
            v = float(np.clip(self.k_v * (dist - self.pos_tol), self.v_min, self.v_max))
            fade = min(dist / self.steer_fade, 1.0)                # bearing is noise at a few cm
            w = float(np.clip(self.k_steer * bearing * fade, -self.w_steer_max, self.w_steer_max))
            self.v = direction * v
            self.w = w

    
    def heading_alligned(self, i):
        err = self.traj.theta[i] - self.yaw
        heading_err = math.atan2(math.sin(err), math.cos(err))

        if self.align_rotating:
            if abs(heading_err) < self.align_exit:
                self.align_rotating = False
        elif abs(heading_err) > self.align_enter:
            self.align_rotating = True
            self.align_dir = 1.0 if heading_err >= 0.0 else -1.0   # latch direction at start

        if self.align_rotating:
            w = float(np.clip(self.k_w * abs(heading_err), self.w_min, self.w_max))
            self.v = 0.0
            self.w = self.align_dir * w
            return False
        return True
            
    def compute_velocities(self, i):
        
        j = min(i + self.lookahead, len(self.traj.x) - 1)
        tx, ty = self.traj.x[j], self.traj.y[j]
        L_d = math.hypot(tx - self.x_pos, ty - self.y_pos)
        alpha = math.atan2(ty - self.y_pos, tx - self.x_pos) - self.yaw

        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        k = 2.0 * math.sin(alpha) / (L_d + 0.01)
        self.v = self.traj.v_ref[i] * max(0.0, math.cos(alpha))
        self.w = k * self.v

    def avoid_obstacles(self):
        theta, dist = self.scan_theta, self.scan_dist
        front_zone = np.abs(theta) < math.radians(30)
        right_zone = (theta > math.radians(-90)) & (theta < math.radians(-30))
        left_zone  = (theta > math.radians(30)) & (theta < math.radians(90))
        right_clearance = dist[right_zone].min(initial=10.0)
        left_clearance  = dist[left_zone].min(initial=10.0)
        clearance = right_clearance - left_clearance

        near  = dist <= 0.8
        side_near = dist <= 0.25
        front = near & front_zone
        side  = side_near & ~front_zone

        front_theta = theta[front]
        if clearance >= 0:
            front_theta = np.clip(-math.radians(90) + front_theta, -math.radians(90), -math.radians(59))
        else:
            front_theta = np.clip(math.radians(90) + front_theta, math.radians(59), math.radians(90))
        side_theta = -theta[side]

        front_gain = np.clip((0.8 - dist[front]) / (0.8 - 0.18), 0.0, 1.0)
        side_gain  = np.clip((0.25 - dist[side])  / (0.25 - 0.18), 0.0, 1.0)

        # resultant push: sum each bin's vector (angle = push direction, length = severity)
        phi = np.concatenate((front_theta, side_theta))
        g   = np.concatenate((front_gain, side_gain)) 
        vx = float(np.sum(g * np.cos(phi)))
        vy = float(np.sum(g * np.sin(phi)))
        mag = math.hypot(vx, vy)
        push_angle = math.atan2(vy, vx) if mag > 1e-6 else 0.0
        push_gain = min(mag, 1.0)                           
        front_severity = float(front_gain.max(initial=0.0))

        self.v = self.v * (1.0 - front_severity)
        self.w = self.w + float(np.clip(push_gain * 0.5 * push_angle,-0.8, 0.8)) 

    def safety_stop(self):
        if self.stop:
            self.v = min(self.v, 0.0) 

    def scale_velocities(self, v, w, max_wheel_speed, L):
        v_left = v - (L / 2.0) * w
        v_right = v + (L / 2.0) * w
        max_speed = max(abs(v_left), abs(v_right))
        if max_speed > max_wheel_speed:
            scale = max_wheel_speed / max_speed
            v_left *= scale
            v_right *= scale
        v_scaled = (v_left + v_right) / 2.0
        w_scaled = (v_right - v_left) / L
        return v_scaled, w_scaled

    def publish_cmds(self):
        v , w = self.scale_velocities(self.v, self.w, self.max_wheel_speed, self.L_)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.publisher_.publish(cmd)

    def halt(self):
        self.v = 0.0
        self.w = 0.0
        self.publish_cmds()

    def control_loop(self):
        if not self.enabled:
            return
        if (not self.pose_ok() or not self.scan_ok() or self.traj is None or self.park_target is None):
            self.halt()
            return

        i, _ = self.nearest_index()

        if self.check_goal_end_behavior():
            if not self.once:
                self.park_phase = "APPROACH"
                self.park_rotating = False
                self.once = True
            self.parking_behavior()
        else:
            if self.heading_alligned(i):
                self.compute_velocities(i)
                self.avoid_obstacles()
        self.safety_stop()
        self.publish_cmds()

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()