#!/usr/bin/env python3
"""
Plot experiment data (force, measured pose, target pose, ...) with Plotly.

All settings live in the CONFIG block below; edit it and run:

    python3 plot_logs.py
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.colors import qualitative
from plotly.subplots import make_subplots


# =====================================================================
# CONFIG
# =====================================================================

EXPERIMENT = 40                 # experiment number used by force_logger.py
DATA_DIR = "."                  # folder that the source paths below are relative to

# Data sources. The FIRST entry is the base: its time axis is the one every
# other source gets resampled onto. For each source:
#   name     label used in messages
#   path     CSV path relative to DATA_DIR; {exp} is replaced by EXPERIMENT
#   prefix   prepended to every column name (avoids clashes, e.g. pos_x in both
#            the measured and the target file -> "target_pos_x")
#   time_col name of the time column in that file
#   method   how to resample onto the base time axis:
#              "linear" - linear interpolation (good for measured signals)
#              "hold"   - last known value held (good for sent commands/targets)
#   required exit if the file is missing (otherwise it is skipped with a warning)
SOURCES = [
    dict(name="force",  path="data/force_data-{exp}.csv",  prefix="",        time_col="time_s", method="linear", required=True),
    dict(name="pose",   path="data/pose_data-{exp}.csv",   prefix="",        time_col="time_s", method="linear", required=True),
    dict(name="target_pose",   path="data/target_pose_data-{exp}.csv",   prefix="tgt_",        time_col="time_s", method="linear", required=True),
    dict(name="estimated_force",   path="data/estimated_force_data-{exp}.csv",   prefix="est_",        time_col="time_s", method="linear", required=True),
]

# What to plot.
X_COLUMN = "pos_x"

# One entry per subplot (stacked vertically, shared x axis).
# A string gives one trace; a list overlays several traces in the same subplot,
# e.g. to compare measured vs target:
#   Y_COLUMNS = ["force_z", ["pos_x", "target_pos_x"], ["pos_z", "target_pos_z"]]
Y_COLUMNS = ["pos_z"]

# Statistics drawn on the plot and printed as a table.
# Any of: "mean", "std", "median", "min", "max"   (empty list = none)
STATS = ["min", "max"]

SHOW_MARKERS = False            # draw markers in addition to lines

# Optional: force 1:1 scaling between the x and y axes (useful when x and y
# have the same units, e.g. plotting a path pos_y vs pos_x). Leave False for
# things like force vs time, where the units differ.
LOCK_UNIFORM_AXES = False

PLOT_HEIGHT = None              # pixels; None = fill the browser window
SAVE_PATH = None                # e.g. "out.html" to save instead of opening a browser tab
LIST_COLUMNS_ONLY = False       # True = just print the available columns and exit


# =====================================================================
# LOADING
# =====================================================================

def resample(t_base, t_src, values, method):
    """Resample one column from the source time axis onto the base time axis."""
    if method == "linear":
        # NaN outside the source time range (no extrapolation).
        return np.interp(t_base, t_src, values, left=np.nan, right=np.nan)
    if method == "hold":
        # Zero-order hold: the most recent source sample at or before each base time.
        idx = np.searchsorted(t_src, t_base, side="right") - 1
        out = values[np.clip(idx, 0, None)].astype(float)
        out[idx < 0] = np.nan  # before the first sample there is no value yet
        return out
    raise ValueError(f"Unknown resample method: {method!r}")


def load_source(src, exp, directory):
    """Read one source CSV. Returns a DataFrame sorted by time, or None if missing."""
    path = Path(directory) / src["path"].format(exp=exp)
    if not path.exists():
        msg = f"{src['name']} file not found: {path}"
        if src.get("required"):
            sys.exit(msg)
        print(f"[warn] {msg} (skipped)", file=sys.stderr)
        return None
    df = pd.read_csv(path)
    time_col = src.get("time_col", "time_s")
    if time_col not in df.columns:
        sys.exit(f"{path}: no time column '{time_col}'. Columns: {', '.join(df.columns)}")
    return df.sort_values(time_col).reset_index(drop=True)


def load_data(sources, exp, directory):
    """Load every source and merge them onto the first source's time axis."""
    base_src, *other_srcs = sources
    base = load_source(base_src, exp, directory)
    if base is None:
        sys.exit("The base source could not be loaded.")

    base_time = base_src.get("time_col", "time_s")
    prefix = base_src.get("prefix", "")
    df = base.rename(columns={c: prefix + c for c in base.columns if c != base_time})
    df = df.rename(columns={base_time: "time_s"})
    t_base = df["time_s"].to_numpy()

    for src in other_srcs:
        other = load_source(src, exp, directory)
        if other is None:
            continue
        time_col = src.get("time_col", "time_s")
        prefix = src.get("prefix", "")
        method = src.get("method", "linear")
        t_src = other[time_col].to_numpy()

        # Note: interpolating quaternions column-wise is only an approximation.
        for col in other.select_dtypes("number").columns:
            if col == time_col:
                continue
            name = prefix + col
            if name in df.columns:
                print(f"[warn] column '{name}' from {src['name']} overwrites an existing "
                      f"column; set a different prefix for this source.", file=sys.stderr)
            df[name] = resample(t_base, t_src, other[col].to_numpy(), method)

    return df


# =====================================================================
# STATS
# =====================================================================

def compute_stats(df, columns, stats):
    """Return {column: {stat: value}} (NaNs ignored)."""
    funcs = {"mean": "mean", "std": "std", "median": "median", "min": "min", "max": "max"}
    out = {}
    for c in columns:
        s = df[c].dropna()
        out[c] = {st: getattr(s, funcs[st])() for st in stats}
    return out


# =====================================================================
# PLOTTING
# =====================================================================

def normalize_groups(y_columns):
    """Turn Y_COLUMNS into a list of lists (one inner list per subplot)."""
    return [[g] if isinstance(g, str) else list(g) for g in y_columns]


def build_figure(df, x, groups, stats):
    n = len(groups)
    all_ys = [y for g in groups for y in g]
    stat_vals = compute_stats(df, all_ys, stats)
    mode = "lines+markers" if SHOW_MARKERS else "lines"
    palette = qualitative.Plotly

    fig = make_subplots(rows=n, cols=1, shared_xaxes=True,
                        vertical_spacing=0.05 if n > 1 else 0.0)

    color_i = 0
    for row, group in enumerate(groups, start=1):
        multi = len(group) > 1
        fig.update_yaxes(title_text=", ".join(group), row=row, col=1)

        for y in group:
            color = palette[color_i % len(palette)]
            color_i += 1
            fig.add_trace(
                go.Scatter(x=df[x], y=df[y], mode=mode, name=y,
                           line=dict(color=color), marker=dict(size=4, color=color)),
                row=row, col=1,
            )

            s = stat_vals[y]
            tag = f"{y} " if multi else ""  # disambiguate labels when overlaying

            if "mean" in s:
                fig.add_hline(y=s["mean"], line=dict(dash="dash", color=color, width=1.5),
                              annotation_text=f"{tag}mean = {s['mean']:.4g}",
                              annotation_position="top left", row=row, col=1)
            if "std" in s:
                center = s["mean"] if "mean" in s else df[y].dropna().mean()
                fig.add_hrect(y0=center - s["std"], y1=center + s["std"],
                              fillcolor=color, opacity=0.12, line_width=0,
                              annotation_text=f"{tag}±std = {s['std']:.4g}",
                              annotation_position="bottom left", row=row, col=1)
            if "median" in s:
                fig.add_hline(y=s["median"], line=dict(dash="dot", color=color, width=1.5),
                              annotation_text=f"{tag}median = {s['median']:.4g}",
                              annotation_position="top right", row=row, col=1)
            if "min" in s:
                fig.add_hline(y=s["min"], line=dict(dash="dot", color=color, width=1),
                              annotation_text=f"{tag}min = {s['min']:.4g}",
                              annotation_position="bottom right", row=row, col=1)
            if "max" in s:
                fig.add_hline(y=s["max"], line=dict(dash="dot", color=color, width=1),
                              annotation_text=f"{tag}max = {s['max']:.4g}",
                              annotation_position="top right", row=row, col=1)

    fig.update_xaxes(title_text=x, row=n, col=1)

    if LOCK_UNIFORM_AXES:
        # Anchor every y axis to the x axis at 1:1. constrain="domain" keeps the
        # data ranges as-is and shrinks the plot area instead of changing ranges.
        fig.update_xaxes(constrain="domain")
        fig.update_yaxes(scaleanchor="x", scaleratio=1, constrain="domain")

    fig.update_layout(
        title=f"{' | '.join(', '.join(g) for g in groups)} vs {x}",
        template="plotly_white",
        legend=dict(orientation="h", y=1.02, x=0),
        autosize=True,
        height=PLOT_HEIGHT,  # None -> fill the window
    )
    return fig


# =====================================================================
# MAIN
# =====================================================================

def main():
    df = load_data(SOURCES, EXPERIMENT, DATA_DIR)

    if LIST_COLUMNS_ONLY:
        print("Available columns:")
        for c in df.columns:
            print(f"  {c}")
        return

    groups = normalize_groups(Y_COLUMNS)
    all_ys = [y for g in groups for y in g]

    missing = [c for c in [X_COLUMN, *all_ys] if c not in df.columns]
    if missing:
        sys.exit(f"Unknown column(s): {', '.join(missing)}\n"
                 f"Available: {', '.join(df.columns)}")

    if STATS:
        cols = list(dict.fromkeys([X_COLUMN, *all_ys]))
        table = pd.DataFrame(compute_stats(df, cols, STATS)).T
        print("\nPer-column statistics:")
        print(table.to_string(float_format=lambda v: f"{v:.5g}"))
        print()

    fig = build_figure(df, X_COLUMN, groups, STATS)

    if SAVE_PATH:
        # default_height/width make the saved page fill the browser window.
        fig.write_html(SAVE_PATH, default_height="100%", default_width="100%")
        print(f"Saved to {SAVE_PATH}")
    else:
        fig.show()


if __name__ == "__main__":
    main()
