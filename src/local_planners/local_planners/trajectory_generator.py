"""
trajectory_generator.py -- standalone trajectory generation for the pure pursuit node.

No ROS imports: pure numpy, so it can be imported anywhere and unit tested.
The logic is the one from pure_pursuit.py, unchanged.

Usage in your node
------------------
    from local_planners.trajectory_generator import (
        TrajectoryConfig, dedupe_points, generate_trajectory)

    # once, in __init__ (v_lim and track come from robot_dimensions.yaml)
    self.cfg = TrajectoryConfig(v_lim=v_lim, track=track)

    # in path_callback
    pts = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses])
    pts = dedupe_points(pts)
    if len(pts) < 2:
        ...                                   # no valid path
    # latest_goal = (x, y, yaw) from /goal_pose, or None
    traj = generate_trajectory(pts, latest_goal, self.cfg)

    traj.x, traj.y, traj.s, traj.theta, traj.kappa, traj.v_ref     # numpy arrays, same length
    traj.end_tx, traj.end_ty        # unit tangent at the end (to extend the path past it)
    traj.goal_yaw                   # final heading from the goal, None if no goal matched
    traj.snapped                    # True if the last point was moved onto the exact goal

A goal that arrives AFTER its path: call generate_trajectory again with the stored raw points
(the deduped array you passed the first time) and the new goal.

Pipeline
--------
  raw A* path (ends at the CENTRE of the goal cell, up to 0.035 m from the goal)
    -> snap the last point to the exact goal (only if within `snap_max_dist`)
    -> Douglas-Peucker (<= `simplify_tol` deviation)
    -> uniform resampling (`resample_step`)
    -> signed curvature (3-point Menger over `curvature_window`)
    -> speed profile: wheel-speed cap vs curvature, decel-limited approach to the end,
       creep speed from `goal_tol` before the end
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


# ----------------------------------------------------------------------
# Building blocks (unchanged from pure_pursuit.py)
# ----------------------------------------------------------------------
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


def dedupe_points(pts):
    """Drop consecutive points closer than 1 micrometre (what path_callback did first)."""
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    if len(pts) >= 2:
        keep = np.concatenate(([True], np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1])) > 1e-6))
        pts = pts[keep]
    return pts


# ----------------------------------------------------------------------
# Configuration and result
# ----------------------------------------------------------------------
@dataclass
class TrajectoryConfig:
    """Same parameters and defaults as pure_pursuit.py.
    v_lim [m/s] = limits.max_wheel_speed_m_s and track [m] = robot.drive_wheel.track_width_m
    from robot_dimensions.yaml; they have no default on purpose."""
    v_lim: float
    track: float
    base_velocity: float = 0.0       # 0 -> v_lim (and never above v_lim)
    creep_velocity: float = 0.03     # speed at the end of the profile [m/s]
    profile_decel: float = 0.30      # decel used by the speed profile [m/s^2]
    simplify_tol: float = 0.03       # Douglas-Peucker tolerance [m]
    resample_step: float = 0.10      # resampling spacing [m] (min 0.02)
    curvature_window: float = 0.40   # Menger curvature window [m]
    goal_tolerance: float = 0.02     # profile reaches creep speed this far from the end [m]
    snap_path_to_goal: bool = True
    snap_max_dist: float = 0.06      # > half a 0.05 m cell diagonal [m]
    goal_match_tolerance: float = 0.25   # goal counts as this path's goal within [m]

    def __post_init__(self):
        if self.v_lim <= 0.0 or self.track <= 0.0:
            raise ValueError("v_lim and track must be positive")

    @property
    def v_base(self):
        return min(self.base_velocity, self.v_lim) if self.base_velocity > 0.0 else self.v_lim


@dataclass
class Trajectory:
    """All arrays have the same length n, in the frame of the input path (map)."""
    x: np.ndarray                 # [m]
    y: np.ndarray                 # [m]
    s: np.ndarray                 # cumulative arc length, s[0] = 0 [m]
    theta: np.ndarray             # tangent heading of the path [rad]
    kappa: np.ndarray             # signed curvature, + = left turn [1/m]
    v_ref: np.ndarray             # speed limit along the path [m/s]
    end_tx: float                 # unit tangent at the end
    end_ty: float
    goal_yaw: Optional[float]     # final heading from the goal, None if no goal matched [rad]
    snapped: bool                 # last point was moved onto the exact goal

    @property
    def length(self):
        return float(self.s[-1])


# ----------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------
def generate_trajectory(raw_pts, goal: Optional[Tuple[float, float, float]],
                        cfg: TrajectoryConfig) -> Trajectory:
    """raw_pts: (N, 2) path in the map frame, N >= 2, consecutive duplicates removed
    (use dedupe_points). goal: (x, y, yaw) from /goal_pose, or None."""
    pts = np.asarray(raw_pts, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2:
        raise ValueError("raw_pts must be an (N, 2) array with N >= 2")

    # The A* path ends at the centre of the goal cell (up to half a cell diagonal =
    # 0.035 m from the requested point); put the exact goal there so we can stop AT it.
    snapped = False
    if cfg.snap_path_to_goal and goal is not None:
        d = math.hypot(goal[0] - pts[-1, 0], goal[1] - pts[-1, 1])
        if 1e-4 < d <= cfg.snap_max_dist:
            pts = pts.copy()
            pts[-1] = (goal[0], goal[1])
            snapped = True

    pts = simplify_dp(pts, cfg.simplify_tol)
    pts, s = resample(pts, max(cfg.resample_step, 0.02))
    ds = max(float(s[-1]) / max(len(s) - 1, 1), 1e-6)
    k = max(1, int(round(cfg.curvature_window / 2.0 / ds)))
    kappa = menger_curvature(pts, k)
    v_ref = build_speed_profile(s, kappa, cfg.v_base, cfg.v_lim, cfg.track,
                                cfg.profile_decel, cfg.creep_velocity, cfg.goal_tolerance)

    x, y = pts[:, 0].copy(), pts[:, 1].copy()
    theta = np.arctan2(np.gradient(y), np.gradient(x))

    # Tangent at the end, used to extend the path so a lookahead past the end stays defined.
    j = max(len(pts) - 1 - k, 0)
    tx, ty = x[-1] - x[j], y[-1] - y[j]
    norm = math.hypot(tx, ty)
    end_tx, end_ty = (tx / norm, ty / norm) if norm > 1e-9 else (1.0, 0.0)

    goal_yaw = None
    if goal is not None and math.hypot(goal[0] - x[-1], goal[1] - y[-1]) <= cfg.goal_match_tolerance:
        goal_yaw = goal[2]

    return Trajectory(x=x, y=y, s=s, theta=theta, kappa=kappa, v_ref=v_ref,
                      end_tx=end_tx, end_ty=end_ty, goal_yaw=goal_yaw, snapped=snapped)
