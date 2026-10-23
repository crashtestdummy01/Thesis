#!/usr/bin/env python3
"""
Plot data recorded by force_logger.py with Plotly.

Examples
--------
  # list available columns
  python3 plot_logs.py 3 --list

  # force_z vs time (default)
  python3 plot_logs.py 3

  # force_z vs x position, with mean line and +/- std band
  python3 plot_logs.py 3 --x pos_x --y force_z --stats mean std

  # several y columns, stacked on a shared x axis
  python3 plot_logs.py 3 --x time_s --y force_x force_y force_z --stats mean std min max

  # save to HTML instead of opening a browser tab
  python3 plot_logs.py 3 --y force_z --save out.html
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots


STAT_CHOICES = ["mean", "average", "std", "median", "min", "max"]


# ---------------------------------------------------------------- loading

def load_data(exp, directory, force_path=None, pose_path=None):
    """Load force (+ optional pose) CSVs and merge on the force time axis."""
    d = Path(directory)
    force_path = Path(force_path) if force_path else d / f"./data/force_data-{exp}.csv"
    pose_path = Path(pose_path) if pose_path else d / f"./data/pose_data-{exp}.csv"

    if not force_path.exists():
        sys.exit(f"Force file not found: {force_path}")
    df = pd.read_csv(force_path)

    if pose_path.exists():
        pose = pd.read_csv(pose_path).sort_values("time_s")
        t_force = df["time_s"].to_numpy()
        t_pose = pose["time_s"].to_numpy()
        # Linearly interpolate every pose column onto the force timestamps.
        # Outside the pose time range -> NaN (no extrapolation).
        # Note: linear interpolation of quaternions is only an approximation.
        for col in pose.columns:
            if col == "time_s":
                continue
            df[col] = np.interp(t_force, t_pose, pose[col].to_numpy(),
                                left=np.nan, right=np.nan)
    else:
        print(f"[warn] Pose file not found ({pose_path}); only force columns available.",
              file=sys.stderr)

    return df


# ---------------------------------------------------------------- stats

def compute_stats(df, columns, stats):
    """Return {column: {stat: value}} (NaNs ignored)."""
    out = {}
    for c in columns:
        s = df[c].dropna()
        row = {}
        for st in stats:
            if st in ("mean", "average"):
                row["mean"] = s.mean()
            elif st == "std":
                row["std"] = s.std()
            elif st == "median":
                row["median"] = s.median()
            elif st == "min":
                row["min"] = s.min()
            elif st == "max":
                row["max"] = s.max()
        out[c] = row
    return out


# ---------------------------------------------------------------- plotting

def build_figure(df, x, ys, stats, markers=False):
    fig = make_subplots(rows=len(ys), cols=1, shared_xaxes=True,
                        vertical_spacing=0.05 if len(ys) > 1 else 0.0)

    stat_vals = compute_stats(df, ys, stats)
    mode = "lines+markers" if markers else "lines"

    for i, y in enumerate(ys, start=1):
        fig.add_trace(
            go.Scatter(x=df[x], y=df[y], mode=mode, name=y,
                       marker=dict(size=4)),
            row=i, col=1,
        )
        fig.update_yaxes(title_text=y, row=i, col=1)

        s = stat_vals[y]
        mean = s.get("mean")
        std = s.get("std")

        # Mean line
        if mean is not None:
            fig.add_hline(y=mean, line=dict(dash="dash", color="crimson", width=1.5),
                          annotation_text=f"mean = {mean:.4g}",
                          annotation_position="top left", row=i, col=1)

        # Std band (centered on the mean, even if mean line is not shown)
        if std is not None:
            center = mean if mean is not None else df[y].dropna().mean()
            fig.add_hrect(y0=center - std, y1=center + std,
                          fillcolor="crimson", opacity=0.12, line_width=0,
                          annotation_text=f"±std = {std:.4g}",
                          annotation_position="bottom left", row=i, col=1)

        if "median" in s:
            fig.add_hline(y=s["median"], line=dict(dash="dot", color="green", width=1.5),
                          annotation_text=f"median = {s['median']:.4g}",
                          annotation_position="top right", row=i, col=1)
        if "min" in s:
            fig.add_hline(y=s["min"], line=dict(dash="dot", color="gray", width=1),
                          annotation_text=f"min = {s['min']:.4g}",
                          annotation_position="bottom right", row=i, col=1)
        if "max" in s:
            fig.add_hline(y=s["max"], line=dict(dash="dot", color="gray", width=1),
                          annotation_text=f"max = {s['max']:.4g}",
                          annotation_position="top right", row=i, col=1)

    fig.update_xaxes(title_text=x, row=len(ys), col=1)
    fig.update_layout(
        title=f"{', '.join(ys)} vs {x}",
        height=max(450, 320 * len(ys)),
        template="plotly_white",
        legend=dict(orientation="h", y=1.02, x=0),
    )
    return fig


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="Plot force/pose logs with Plotly.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("exp", nargs="?", default="0",
                    help="experiment number used by force_logger.py (default: 0)")
    ap.add_argument("--dir", default=".", help="folder containing the CSVs (default: .)")
    ap.add_argument("--force-file", help="explicit path to the force CSV")
    ap.add_argument("--pose-file", help="explicit path to the pose CSV")
    ap.add_argument("--x", default="time_s", help="column for the x axis (default: time_s)")
    ap.add_argument("--y", nargs="+", default=["force_z"],
                    help="one or more columns for the y axis (default: force_z)")
    ap.add_argument("--stats", nargs="*", default=[], choices=STAT_CHOICES,
                    help="metrics to compute per y column and draw on the plot")
    ap.add_argument("--markers", action="store_true", help="draw markers as well as lines")
    ap.add_argument("--list", action="store_true", help="list available columns and exit")
    ap.add_argument("--save", metavar="FILE.html", help="save to an HTML file instead of showing")
    args = ap.parse_args()

    df = load_data(args.exp, args.dir, args.force_file, args.pose_file)

    if args.list:
        print("Available columns:")
        for c in df.columns:
            print(f"  {c}")
        return

    missing = [c for c in [args.x, *args.y] if c not in df.columns]
    if missing:
        sys.exit(f"Unknown column(s): {', '.join(missing)}\n"
                 f"Available: {', '.join(df.columns)}")

    # Print the stats table for every selected column (x included)
    if args.stats:
        cols = list(dict.fromkeys([args.x, *args.y]))
        table = pd.DataFrame(compute_stats(df, cols, args.stats)).T
        print("\nPer-column statistics:")
        print(table.to_string(float_format=lambda v: f"{v:.5g}"))
        print()

    fig = build_figure(df, args.x, args.y, args.stats, markers=args.markers)

    if args.save:
        fig.write_html(args.save)
        print(f"Saved to {args.save}")
    else:
        fig.show()


if __name__ == "__main__":
    main()
