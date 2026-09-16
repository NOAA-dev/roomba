#!/usr/bin/env python3
"""
visualize_data_logs.py

Visualizes the CSVs produced by data_logger_node.py:
  - odom_log_<stamp>.csv            (always present)
  - validated_map_log_<stamp>.csv   (always present)
  - map_data_log_<stamp>.csv        (always present)
  - joint_states_log_<stamp>.csv    (present on runs after the joint_states
                                      logging update -- older runs skip it)
  - wheel_cmd_log_<stamp>.csv       (same as above)
  - tf_log_<stamp>.csv              (present on runs after the map->roomba TF
                                      logging update -- older runs fall back to
                                      plotting the odom path alone)
  - run_meta_<stamp>.json           (same as above -- records sim vs real
                                      hardware, used to decide whether the
                                      telemetry-interval reference lines mean
                                      anything for this run)

Two paths, deliberately kept separate:
  * odom_log  -- /odometry/filtered, i.e. the odom frame. Wheel + IMU fusion
                 only, so it drifts and is NOT corrected to the map.
  * tf_log    -- the map -> roomba transform, i.e. odom->roomba composed with
                 whatever map->odom correction SLAM currently holds. This is
                 the pose that actually belongs on top of the occupancy grid.
The gap between them at any instant IS the map->odom correction, which is what
the drift figure plots.

Default usage (looks in ~/roomba/collected_data, grabs the most recent run
automatically):
    python3 visualize_data_logs.py

Pick a specific run by its timestamp suffix:
    python3 visualize_data_logs.py --stamp 20260716_142233

Static figures, one per window (trajectory / scores over time / latest map
snapshot / odom-vs-map drift / wheel velocity / telemetry interval histogram):
    python3 visualize_data_logs.py --save out.png

Step through every logged map snapshot as an animation (map + both trajectories
superimposed, map_score readout in the top-right corner):
    python3 visualize_data_logs.py --animate --save out.mp4
    python3 visualize_data_logs.py --animate --save out.gif
"""

import argparse
import glob
import json
import os
import re

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation


DEFAULT_DIR = os.path.expanduser("~/roomba/collected_data")

ODOM_STYLE = dict(color="tab:orange", linestyle="--", linewidth=1.2)
TF_STYLE = dict(color="tab:blue", linestyle="-", linewidth=1.4)
ODOM_LABEL = "odom frame (uncorrected)"
TF_LABEL = "map -> roomba TF (drift-corrected)"


# ----------------------------------------------------------------------
# File discovery
# ----------------------------------------------------------------------
def find_run(data_dir, stamp=None):
    """Locate the CSVs for a run. If stamp is None, use the most recently
    modified odom_log_*.csv to infer the stamp. joint_states/wheel_cmd/tf logs
    are optional -- older runs (before that logging was added) won't have
    them, so their paths are returned as None rather than raising."""
    if stamp is None:
        candidates = sorted(
            glob.glob(os.path.join(data_dir, "odom_log_*.csv")),
            key=os.path.getmtime,
        )
        if not candidates:
            raise FileNotFoundError(f"No odom_log_*.csv found in {data_dir}")
        latest = candidates[-1]
        match = re.search(r"odom_log_(.+)\.csv$", os.path.basename(latest))
        stamp = match.group(1)

    odom_path = os.path.join(data_dir, f"odom_log_{stamp}.csv")
    summary_path = os.path.join(data_dir, f"validated_map_log_{stamp}.csv")
    map_data_path = os.path.join(data_dir, f"map_data_log_{stamp}.csv")

    for p in (odom_path, summary_path, map_data_path):
        if not os.path.exists(p):
            raise FileNotFoundError(f"Expected file not found: {p}")

    joint_states_path = os.path.join(data_dir, f"joint_states_log_{stamp}.csv")
    wheel_cmd_path = os.path.join(data_dir, f"wheel_cmd_log_{stamp}.csv")
    tf_path = os.path.join(data_dir, f"tf_log_{stamp}.csv")
    if not os.path.exists(joint_states_path):
        joint_states_path = None
    if not os.path.exists(wheel_cmd_path):
        wheel_cmd_path = None
    if not os.path.exists(tf_path):
        tf_path = None

    # sim=True on a run predating this metadata file: unknown, not assumed --
    # keeps old real-hardware runs from silently being mislabeled as sim.
    sim = None
    run_meta_path = os.path.join(data_dir, f"run_meta_{stamp}.json")
    if os.path.exists(run_meta_path):
        with open(run_meta_path) as f:
            sim = json.load(f).get("sim")

    return (
        odom_path,
        summary_path,
        map_data_path,
        joint_states_path,
        wheel_cmd_path,
        tf_path,
        sim,
        stamp,
    )


def has_tf(tf_df):
    """The log file can exist but be header-only if map->roomba never became
    available during the run (SLAM not up, or the node died early)."""
    return tf_df is not None and not tf_df.empty


# ----------------------------------------------------------------------
# Grid reconstruction
# ----------------------------------------------------------------------
def reconstruct_grid(row):
    width = int(row["map_width"])
    height = int(row["map_height"])
    values = np.array(row["grid_data"].split(), dtype=np.int16)
    grid = values.reshape(height, width)
    return grid


def grid_extent(row):
    """Return (xmin, xmax, ymin, ymax) in world coords for imshow's extent."""
    res = row["map_resolution"]
    width = int(row["map_width"])
    height = int(row["map_height"])
    ox = row["origin_x"]
    oy = row["origin_y"]
    return ox, ox + width * res, oy, oy + height * res


def draw_grid(ax, row):
    grid = reconstruct_grid(row)
    extent = grid_extent(row)
    # Occupancy convention: -1 unknown, 0 free, 100 occupied.
    display = np.ma.masked_equal(grid, -1)
    ax.imshow(
        display,
        origin="lower",
        extent=extent,
        cmap="Greys",
        vmin=0,
        vmax=100,
        interpolation="nearest",
    )
    # Draw unknown cells as a light background so they're distinguishable
    # from confirmed-free space.
    ax.set_facecolor("#dce6f0")


# ----------------------------------------------------------------------
# Static figures (separate windows)
# ----------------------------------------------------------------------
def make_trajectory_figure(odom_df, tf_df, stamp):
    fig, ax = plt.subplots(figsize=(7, 7), num=f"Trajectory - {stamp}")

    ax.plot(odom_df["x"], odom_df["y"], label=ODOM_LABEL, **ODOM_STYLE)
    ax.scatter(odom_df["x"].iloc[0], odom_df["y"].iloc[0], color="green", zorder=5, label="start")
    ax.scatter(odom_df["x"].iloc[-1], odom_df["y"].iloc[-1], color="red", zorder=5, label="end")

    if has_tf(tf_df):
        ax.plot(tf_df["x"], tf_df["y"], label=TF_LABEL, **TF_STYLE)
        ax.scatter(tf_df["x"].iloc[-1], tf_df["y"].iloc[-1], color="tab:blue",
                   marker="s", zorder=5, s=35, label="end (TF)")
        title = "Robot trajectory: odom frame vs map -> roomba TF"
    else:
        title = "Robot trajectory (odom frame only -- no tf_log for this run)"

    ax.set_title(title)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def make_scores_figure(summary_df, stamp):
    fig, ax = plt.subplots(figsize=(9, 5), num=f"Scores - {stamp}")
    valid = summary_df[summary_df["source"] == "valid"]
    invalid = summary_df[summary_df["source"] == "invalid"]
    if not valid.empty:
        ax.plot(valid["ros_time_sec"], valid["map_score"], "o-", color="tab:green", label="valid map_score")
    if not invalid.empty:
        ax.plot(invalid["ros_time_sec"], invalid["map_score"], "x-", color="tab:red", label="invalid map_score")
    ax.axhline(0, color="gray", linewidth=0.5)
    ax.set_title("Map validation score over time")
    ax.set_xlabel("ros time (s)")
    ax.set_ylabel("map_score")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def make_map_figure(odom_df, map_df, tf_df, stamp):
    fig, ax = plt.subplots(figsize=(7, 7), num=f"Latest map - {stamp}")
    last_row = map_df.iloc[-1]
    draw_grid(ax, last_row)

    # The odom path is in the odom frame, not the map frame -- it's drawn here
    # only so the accumulated correction is visible against the grid, and it
    # should NOT be read as where the robot actually was on this map.
    ax.plot(odom_df["x"], odom_df["y"], alpha=0.8, label=ODOM_LABEL, **ODOM_STYLE)
    if has_tf(tf_df):
        ax.plot(tf_df["x"], tf_df["y"], alpha=0.9, label=TF_LABEL, **TF_STYLE)

    ax.set_title(
        f"Latest map snapshot (counter={int(last_row['counter'])}, source={last_row['source']})"
    )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    return fig


def _wrap_to_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def make_drift_figure(odom_df, tf_df, stamp):
    """How far the uncorrected odom pose sits from the map->roomba pose over
    time. That gap is the map->odom correction: gradual growth is accumulating
    wheel/IMU drift, and a step is a loop closure or relocalisation snapping
    the correction to a new value."""
    t = tf_df["ros_time_sec"].to_numpy(dtype=float)
    ot = odom_df["ros_time_sec"].to_numpy(dtype=float)

    # Only compare where the two logs actually overlap -- np.interp would
    # otherwise clamp to the endpoints and invent a flat offset.
    mask = (t >= ot.min()) & (t <= ot.max())
    t = t[mask]
    if t.size == 0:
        return None

    odom_x = np.interp(t, ot, odom_df["x"].to_numpy(dtype=float))
    odom_y = np.interp(t, ot, odom_df["y"].to_numpy(dtype=float))
    # Unwrap before interpolating so a +pi/-pi crossing doesn't interpolate
    # straight through zero.
    odom_yaw = np.interp(t, ot, np.unwrap(odom_df["yaw"].to_numpy(dtype=float)))

    tf_x = tf_df["x"].to_numpy(dtype=float)[mask]
    tf_y = tf_df["y"].to_numpy(dtype=float)[mask]
    tf_yaw = tf_df["yaw"].to_numpy(dtype=float)[mask]

    d_trans = np.hypot(tf_x - odom_x, tf_y - odom_y)
    d_yaw_deg = np.degrees(_wrap_to_pi(tf_yaw - odom_yaw))

    fig, (ax_t, ax_y) = plt.subplots(
        2, 1, figsize=(10, 6), sharex=True, num=f"Odom vs map drift - {stamp}"
    )

    ax_t.plot(t, d_trans, "-", color="tab:purple", linewidth=1.2)
    ax_t.set_ylabel("offset (m)")
    ax_t.set_title("Translation offset")
    ax_t.grid(True, alpha=0.3)

    ax_y.plot(t, d_yaw_deg, "-", color="tab:brown", linewidth=1.2)
    ax_y.axhline(0, color="gray", linewidth=0.5)
    ax_y.set_ylabel("offset (deg)")
    ax_y.set_xlabel("ros time (s)")
    ax_y.set_title("Heading offset")
    ax_y.grid(True, alpha=0.3)

    fig.suptitle("map -> odom correction (TF pose minus odom pose)")
    fig.tight_layout()
    return fig


def make_wheel_velocity_figure(joint_df, cmd_df, sim, stamp):
    """Commanded vs measured per-wheel velocity, one subplot per wheel.
    Plateaus in 'measured' while 'commanded' is nonzero indicate the ESP32
    watchdog stopped a motor (no V1/V2 refresh within its timeout) or a
    reconnect happened; the two subplots side by side also expose left/right
    tracking asymmetry (see the firmware's uncharacterized Motor A feedforward).
    In sim, expect near-perfect tracking -- Gazebo's velocity actuator isn't
    modeling the real PID/feedforward/encoder chain, so a clean match here
    doesn't validate anything about the real hardware."""
    fig, (ax_l, ax_r) = plt.subplots(2, 1, figsize=(10, 7), sharex=True, num=f"Wheel velocity - {stamp}")

    ax_l.plot(cmd_df["ros_time_sec"], cmd_df["left_cmd_vel"], "-", color="tab:orange", label="commanded", linewidth=1)
    ax_l.plot(joint_df["ros_time_sec"], joint_df["left_vel_rad_s"], "-", color="tab:blue", label="measured", linewidth=1, alpha=0.85)
    ax_l.set_title("Left wheel")
    ax_l.set_ylabel("rad/s")
    ax_l.legend(loc="best")
    ax_l.grid(True, alpha=0.3)

    ax_r.plot(cmd_df["ros_time_sec"], cmd_df["right_cmd_vel"], "-", color="tab:orange", label="commanded", linewidth=1)
    ax_r.plot(joint_df["ros_time_sec"], joint_df["right_vel_rad_s"], "-", color="tab:blue", label="measured", linewidth=1, alpha=0.85)
    ax_r.set_title("Right wheel")
    ax_r.set_xlabel("ros time (s)")
    ax_r.set_ylabel("rad/s")
    ax_r.legend(loc="best")
    ax_r.grid(True, alpha=0.3)

    mode_label = {True: " (simulation)", False: " (real hardware)", None: " (mode unknown -- pre-dates run_meta)"}[sim]
    fig.suptitle("Wheel velocity: commanded vs measured" + mode_label)
    fig.tight_layout()
    return fig


def make_telemetry_interval_figure(joint_df, sim, stamp):
    """Histogram of time between consecutive /joint_states messages.

    On real hardware this is a sanity check for SerialLink's design: ~150ms
    while moving (active streaming), ~200ms at rest (idle poll via 'P'). A
    long tail well past 200ms usually means reconnects or dropped serial
    bytes.

    In sim, joint_state_broadcaster just publishes every control-manager
    cycle (10ms at the 100Hz update_rate in controllers.yaml) -- nothing
    like the real serial link's timing -- so the 150/200ms reference lines
    would be meaningless there and are skipped."""
    intervals_ms = joint_df["ros_time_sec"].diff().dropna() * 1000.0
    intervals_ms = intervals_ms[intervals_ms > 0]  # drop any non-monotonic timestamp glitches

    fig, ax = plt.subplots(figsize=(8, 5), num=f"Telemetry interval - {stamp}")
    ax.hist(intervals_ms, bins=40, color="tab:purple", alpha=0.8)

    if sim is False:
        ax.axvline(150, color="tab:blue", linestyle="--", linewidth=1, label="150ms (active stream)")
        ax.axvline(200, color="tab:orange", linestyle="--", linewidth=1, label="200ms (idle poll)")
        ax.legend(loc="best")
        title_suffix = " (real hardware)"
    elif sim is True:
        title_suffix = " (simulation -- expect ~10ms, control-loop rate, not SerialLink timing)"
    else:
        title_suffix = " (mode unknown -- pre-dates run_meta, reference lines omitted)"

    ax.set_title("/joint_states inter-arrival time" + title_suffix)
    ax.set_xlabel("interval (ms)")
    ax.set_ylabel("count")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


# ----------------------------------------------------------------------
# Animation over map snapshots
# ----------------------------------------------------------------------
def make_animation(odom_df, summary_df, map_df, tf_df):
    fig, ax = plt.subplots(figsize=(7, 7), num="Map + trajectory animation")

    # map_data_log doesn't carry map_score itself - pull it in from the
    # summary log by matching on (counter, source), which both logs share.
    scores = summary_df[["counter", "source", "map_score"]]
    map_df = map_df.merge(scores, on=["counter", "source"], how="left")
    show_tf = has_tf(tf_df)

    def update(i):
        ax.clear()
        row = map_df.iloc[i]
        draw_grid(ax, row)

        traj = odom_df[odom_df["ros_time_sec"] <= row["ros_time_sec"]]
        if not traj.empty:
            ax.plot(traj["x"], traj["y"], label=ODOM_LABEL, **ODOM_STYLE)
            ax.scatter(traj["x"].iloc[-1], traj["y"].iloc[-1],
                       color="tab:orange", edgecolor="black", zorder=5, s=30)

        if show_tf:
            tf_traj = tf_df[tf_df["ros_time_sec"] <= row["ros_time_sec"]]
            if not tf_traj.empty:
                ax.plot(tf_traj["x"], tf_traj["y"], label=TF_LABEL, **TF_STYLE)
                ax.scatter(tf_traj["x"].iloc[-1], tf_traj["y"].iloc[-1],
                           color="red", zorder=6, s=30)

        score_color = "tab:green" if row["source"] == "valid" else "tab:red"
        score_text = f"{row['map_score']:.1f}" if pd.notna(row["map_score"]) else "n/a"
        ax.text(
            0.98, 0.98,
            f"score: {score_text}\n{row['source']} (#{int(row['counter'])})",
            transform=ax.transAxes,
            ha="right", va="top",
            fontsize=12, fontweight="bold", color=score_color,
            bbox=dict(boxstyle="round,pad=0.4", facecolor="white", edgecolor=score_color, alpha=0.85),
        )

        ax.set_title(f"t = {row['ros_time_sec']:.1f}s")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.axis("equal")
        ax.legend(loc="lower left", fontsize=8)
        return []

    anim = animation.FuncAnimation(
        fig, update, frames=len(map_df), interval=500, blit=False
    )
    return fig, anim


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Visualize data_logger_node CSV output")
    parser.add_argument("--dir", default=DEFAULT_DIR, help="Directory containing the CSV logs")
    parser.add_argument("--stamp", default=None, help="Run timestamp suffix, e.g. 20260716_142233")
    parser.add_argument("--animate", action="store_true", help="Step through every map snapshot")
    parser.add_argument("--save", default=None, help="Output path (.png for static, .mp4/.gif for --animate)")
    args = parser.parse_args()

    (
        odom_path,
        summary_path,
        map_data_path,
        joint_states_path,
        wheel_cmd_path,
        tf_path,
        sim,
        stamp,
    ) = find_run(args.dir, args.stamp)

    odom_df = pd.read_csv(odom_path)
    summary_df = pd.read_csv(summary_path)
    map_df = pd.read_csv(map_data_path)

    joint_df = pd.read_csv(joint_states_path) if joint_states_path else None
    cmd_df = pd.read_csv(wheel_cmd_path) if wheel_cmd_path else None
    tf_df = pd.read_csv(tf_path) if tf_path else None

    if joint_df is None or cmd_df is None:
        print("joint_states_log/wheel_cmd_log not found for this run (older run, or logged "
              "before the joint_states/wheel_cmd logging was added) -- skipping wheel-velocity "
              "and telemetry-interval figures.")
    if tf_path is None:
        print("tf_log not found for this run (older run, or logged before map->roomba TF "
              "logging was added) -- plotting the odom path alone, uncorrected.")
    elif not has_tf(tf_df):
        print("tf_log is empty -- map->roomba was never available during this run "
              "(SLAM not running?) -- plotting the odom path alone, uncorrected.")

    if map_df.empty:
        raise ValueError("map_data_log CSV is empty - no map snapshots to visualize yet")

    if args.animate:
        fig, anim = make_animation(odom_df, summary_df, map_df, tf_df)
        if args.save:
            if args.save.endswith(".gif"):
                anim.save(args.save, writer="pillow", fps=2)
            else:
                anim.save(args.save, writer="ffmpeg", fps=2)
            print(f"Saved animation to {args.save}")
        else:
            plt.show()
    else:
        traj_fig = make_trajectory_figure(odom_df, tf_df, stamp)
        scores_fig = make_scores_figure(summary_df, stamp)
        map_fig = make_map_figure(odom_df, map_df, tf_df, stamp)
        drift_fig = make_drift_figure(odom_df, tf_df, stamp) if has_tf(tf_df) else None
        wheel_vel_fig = make_wheel_velocity_figure(joint_df, cmd_df, sim, stamp) if (joint_df is not None and cmd_df is not None) else None
        telemetry_fig = make_telemetry_interval_figure(joint_df, sim, stamp) if joint_df is not None else None

        if args.save:
            base, ext = os.path.splitext(args.save)
            ext = ext or ".png"
            traj_path = f"{base}_trajectory{ext}"
            scores_path = f"{base}_scores{ext}"
            map_path = f"{base}_map{ext}"

            traj_fig.savefig(traj_path, dpi=150)
            scores_fig.savefig(scores_path, dpi=150)
            map_fig.savefig(map_path, dpi=150)
            saved = [traj_path, scores_path, map_path]

            if drift_fig is not None:
                drift_path = f"{base}_drift{ext}"
                drift_fig.savefig(drift_path, dpi=150)
                saved.append(drift_path)
            if wheel_vel_fig is not None:
                wheel_vel_path = f"{base}_wheel_velocity{ext}"
                wheel_vel_fig.savefig(wheel_vel_path, dpi=150)
                saved.append(wheel_vel_path)
            if telemetry_fig is not None:
                telemetry_path = f"{base}_telemetry_interval{ext}"
                telemetry_fig.savefig(telemetry_path, dpi=150)
                saved.append(telemetry_path)

            print(f"Saved figures to {', '.join(saved)}")
        else:
            plt.show()  # opens all figures in separate windows


if __name__ == "__main__":
    main()