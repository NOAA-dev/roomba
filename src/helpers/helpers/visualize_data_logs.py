#!/usr/bin/env python3
"""
visualize_data_logs.py

Visualizes the three CSVs produced by data_logger_node.py:
  - tf_log_<stamp>.csv
  - validated_map_log_<stamp>.csv
  - map_data_log_<stamp>.csv

Default usage (looks in ~/roomba, grabs the most recent run automatically):
    python3 visualize_data_logs.py

Pick a specific run by its timestamp suffix:
    python3 visualize_data_logs.py --stamp 20260716_142233

Static figures, one per window (trajectory / scores over time / latest map
snapshot):
    python3 visualize_data_logs.py --save out.png

Step through every logged map snapshot as an animation (map + trajectory
superimposed, map_score readout in the top-right corner):
    python3 visualize_data_logs.py --animate --save out.mp4
    python3 visualize_data_logs.py --animate --save out.gif
"""

import argparse
import glob
import os
import re

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation


DEFAULT_DIR = os.path.expanduser("~/roomba/collected_data")


# ----------------------------------------------------------------------
# File discovery
# ----------------------------------------------------------------------
def find_run(data_dir, stamp=None):
    """Locate the three CSVs for a run. If stamp is None, use the most
    recently modified tf_log_*.csv to infer the stamp."""
    if stamp is None:
        candidates = sorted(
            glob.glob(os.path.join(data_dir, "tf_log_*.csv")),
            key=os.path.getmtime,
        )
        if not candidates:
            raise FileNotFoundError(f"No tf_log_*.csv found in {data_dir}")
        latest = candidates[-1]
        match = re.search(r"tf_log_(.+)\.csv$", os.path.basename(latest))
        stamp = match.group(1)

    tf_path = os.path.join(data_dir, f"tf_log_{stamp}.csv")
    summary_path = os.path.join(data_dir, f"validated_map_log_{stamp}.csv")
    map_data_path = os.path.join(data_dir, f"map_data_log_{stamp}.csv")

    for p in (tf_path, summary_path, map_data_path):
        if not os.path.exists(p):
            raise FileNotFoundError(f"Expected file not found: {p}")

    return tf_path, summary_path, map_data_path, stamp


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
# Static figures (three separate windows)
# ----------------------------------------------------------------------
def make_trajectory_figure(tf_df, stamp):
    fig, ax = plt.subplots(figsize=(7, 7), num=f"Trajectory - {stamp}")
    ax.plot(tf_df["x"], tf_df["y"], "-", linewidth=1, color="tab:blue", label="trajectory")
    ax.scatter(tf_df["x"].iloc[0], tf_df["y"].iloc[0], color="green", zorder=5, label="start")
    ax.scatter(tf_df["x"].iloc[-1], tf_df["y"].iloc[-1], color="red", zorder=5, label="end")
    ax.set_title("Robot trajectory (map -> roomba TF)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.legend(loc="best")
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


def make_map_figure(tf_df, map_df, stamp):
    fig, ax = plt.subplots(figsize=(7, 7), num=f"Latest map - {stamp}")
    last_row = map_df.iloc[-1]
    draw_grid(ax, last_row)
    ax.plot(tf_df["x"], tf_df["y"], "-", linewidth=1, color="tab:blue", alpha=0.8)
    ax.set_title(
        f"Latest map snapshot (counter={int(last_row['counter'])}, source={last_row['source']})"
    )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    fig.tight_layout()
    return fig


# ----------------------------------------------------------------------
# Animation over map snapshots
# ----------------------------------------------------------------------
def make_animation(tf_df, summary_df, map_df):
    fig, ax = plt.subplots(figsize=(7, 7), num="Map + trajectory animation")

    # map_data_log doesn't carry map_score itself - pull it in from the
    # summary log by matching on (counter, source), which both logs share.
    scores = summary_df[["counter", "source", "map_score"]]
    map_df = map_df.merge(scores, on=["counter", "source"], how="left")

    def update(i):
        ax.clear()
        row = map_df.iloc[i]
        draw_grid(ax, row)

        traj = tf_df[tf_df["ros_time_sec"] <= row["ros_time_sec"]]
        if not traj.empty:
            ax.plot(traj["x"], traj["y"], "-", linewidth=1.2, color="tab:blue")
            ax.scatter(traj["x"].iloc[-1], traj["y"].iloc[-1], color="red", zorder=5, s=30)

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

    tf_path, summary_path, map_data_path, stamp = find_run(args.dir, args.stamp)

    tf_df = pd.read_csv(tf_path)
    summary_df = pd.read_csv(summary_path)
    map_df = pd.read_csv(map_data_path)

    if map_df.empty:
        raise ValueError("map_data_log CSV is empty - no map snapshots to visualize yet")

    if args.animate:
        fig, anim = make_animation(tf_df, summary_df, map_df)
        if args.save:
            if args.save.endswith(".gif"):
                anim.save(args.save, writer="pillow", fps=2)
            else:
                anim.save(args.save, writer="ffmpeg", fps=2)
            print(f"Saved animation to {args.save}")
        else:
            plt.show()
    else:
        traj_fig = make_trajectory_figure(tf_df, stamp)
        scores_fig = make_scores_figure(summary_df, stamp)
        map_fig = make_map_figure(tf_df, map_df, stamp)

        if args.save:
            base, ext = os.path.splitext(args.save)
            ext = ext or ".png"
            traj_path = f"{base}_trajectory{ext}"
            scores_path = f"{base}_scores{ext}"
            map_path = f"{base}_map{ext}"

            traj_fig.savefig(traj_path, dpi=150)
            scores_fig.savefig(scores_path, dpi=150)
            map_fig.savefig(map_path, dpi=150)
            print(f"Saved figures to {traj_path}, {scores_path}, {map_path}")
        else:
            plt.show()  # opens all three in separate windows


if __name__ == "__main__":
    main()
