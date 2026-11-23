"""
Core of the force / pose experiment viewer (no GUI code in here).

 - file name parsing   (<kind>-N, see KINDS)
 - CSV loading + cleaning, memory-friendly (float32 + size-limited LRU cache)
 - velocity / acceleration columns computed from pose_data (NOT target_pose_data)
 - plotly figure building from a list of subplot specifications

Requirements:  pip install pandas numpy plotly
"""
from __future__ import annotations

import functools
import os
import re
import threading
import time
import webbrowser
from collections import OrderedDict
from dataclasses import dataclass, field
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.colors import qualitative
from plotly.subplots import make_subplots

# --------------------------------------------------------------------------- #
# Data layout
# --------------------------------------------------------------------------- #
POSE_COLS = ["time_s", "pos_x", "pos_y", "pos_z", "quat_x", "quat_y", "quat_z", "quat_w"]
FORCE_COLS = ["time_s", "force_x", "force_y", "force_z", "torque_x", "torque_y", "torque_z"]
Q_TWIST_COLUMNS = ["time_s"] + [f"pos_q{i}" for i in range(7)] + [f"vel_q{i}" for i in range(7)]
Q_EFFORT_COLUMNS = ["time_s"] + [f"effort_q{i}" for i in range(7)]

# File name prefix -> expected columns. File names are "<prefix>-<number>[.csv]".
# NOTE: the prefixes of the two q_* kinds are a guess -- rename them here if your
# files are called something else (nothing else needs to change).
BASE_COLUMNS = {
    "force_data": FORCE_COLS,
    "estimated_force_data": FORCE_COLS,
    "pose_data": POSE_COLS,
    "target_pose_data": POSE_COLS,
    "joint_twist_data": Q_TWIST_COLUMNS,
    "joint_effort_data": Q_EFFORT_COLUMNS,
}
KINDS = list(BASE_COLUMNS)

# columns that add_derivatives() appends to pose_data
DERIVED_COLS = [
    f"{prefix}_{axis}"
    for prefix in ("vel", "acc", "ang_vel", "ang_acc")
    for axis in ("x", "y", "z", "norm")
]

# everything that can be selected as an axis, per source
SOURCE_COLUMNS = dict(BASE_COLUMNS)
SOURCE_COLUMNS["pose_data"] = POSE_COLS + DERIVED_COLS

_FILE_RE = re.compile(
    r"^(" + "|".join(sorted(KINDS, key=len, reverse=True)) + r")-(\d+)(?:\.csv)?$",
    re.IGNORECASE,
)


def parse_filename(path: str):
    """'/x/pose_data-12.csv' -> ('pose_data', 12); None if the name doesn't match."""
    m = _FILE_RE.match(os.path.basename(path))
    if not m:
        return None
    return m.group(1).lower(), int(m.group(2))


def axis_options() -> list[str]:
    """Labels for the column picker: 'source: column'."""
    return [f"{kind}: {col}" for kind in KINDS for col in SOURCE_COLUMNS[kind]]


def split_axis(label: str) -> tuple[str, str]:
    source, column = label.split(": ", 1)
    return source, column


# --------------------------------------------------------------------------- #
# Loading
# --------------------------------------------------------------------------- #
def read_experiment_csv(path: str, kind: str) -> pd.DataFrame:
    """Read one data file (only the expected columns), coerce to numbers,
    sort by time and drop duplicated time stamps. Returns float64."""
    expected = BASE_COLUMNS[kind]
    wanted = set(expected)
    df = pd.read_csv(path, usecols=lambda c: str(c).strip() in wanted)
    df.columns = [str(c).strip() for c in df.columns]
    missing = [c for c in expected if c not in df.columns]
    if missing:
        raise ValueError(f"{os.path.basename(path)}: missing column(s) {', '.join(missing)}")
    df = df[expected].apply(pd.to_numeric, errors="coerce")
    df = df.dropna(subset=["time_s"]).sort_values("time_s", kind="stable")
    df = df.drop_duplicates(subset="time_s", keep="first").reset_index(drop=True)
    return df


def _compact(df: pd.DataFrame) -> pd.DataFrame:
    """float32 for everything except time_s (time stamps need float64 precision)."""
    return df.astype({c: np.float32 for c in df.columns if c != "time_s"})


def normalize_exp_key(value) -> int | str:
    """'007' -> 7 so description rows match file numbers; non-numeric stays a string."""
    s = str(value).strip()
    return int(s) if s.isdigit() else s


def exp_sort_key(k):
    return (0, k, "") if isinstance(k, int) else (1, 0, str(k))


def read_description(path: str):
    """Read the optional experiment description CSV (everything as strings).

    Returns (columns_without_exp_num, {exp_key: {column: str}}).
    """
    df = pd.read_csv(path, dtype=str, keep_default_na=False)
    df.columns = [str(c).strip() for c in df.columns]
    if len(df.columns) == 0:
        return [], {}
    key_col = next((c for c in df.columns if c.lower() == "exp_num"), df.columns[0])
    cols = [c for c in df.columns if c != key_col]
    rows: dict = {}
    for _, row in df.iterrows():
        key = normalize_exp_key(row[key_col])
        rows.setdefault(key, {c: str(row[c]).strip() for c in cols})
    return cols, rows


# --------------------------------------------------------------------------- #
# Derivatives (pose_data only)
# --------------------------------------------------------------------------- #
def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of quaternion arrays stored as (x, y, z, w)."""
    ax, ay, az, aw = a.T
    bx, by, bz, bw = b.T
    return np.stack(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        axis=1,
    )


def angular_velocity(q: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Angular velocity (rad/s, expressed in the task/world frame) from a
    quaternion time series q (n,4) = (x, y, z, w)."""
    n = len(q)
    if n < 2:
        return np.full((n, 3), np.nan)
    norms = np.linalg.norm(q, axis=1, keepdims=True)
    norms[norms == 0] = np.nan
    q = q / norms
    # remove q / -q sign flips so consecutive samples are on the same hemisphere
    sign = np.where(np.sum(q[1:] * q[:-1], axis=1) < 0, -1.0, 1.0)
    q = q * np.concatenate([[1.0], np.cumprod(sign)])[:, None]
    # relative rotation between consecutive samples: q[i+1] * conj(q[i])
    conj = q[:-1] * np.array([-1.0, -1.0, -1.0, 1.0])
    rel = _qmul(q[1:], conj)
    rel[rel[:, 3] < 0] *= -1.0
    v, w = rel[:, :3], rel[:, 3]
    vn = np.linalg.norm(v, axis=1)
    safe = np.maximum(vn, 1e-12)
    scale = np.where(vn > 1e-12, 2.0 * np.arctan2(vn, w) / safe, 2.0)
    seg = v * scale[:, None] / np.diff(t)[:, None]  # velocity between samples i and i+1
    omega = np.empty((n, 3))
    omega[0], omega[-1] = seg[0], seg[-1]
    omega[1:-1] = 0.5 * (seg[:-1] + seg[1:])
    return omega


def add_derivatives(df: pd.DataFrame, smooth: int = 1) -> pd.DataFrame:
    """Append velocity / acceleration columns to a pose_data frame.

    vel_{x,y,z,norm}      d(pos)/dt              (position units per second)
    acc_{x,y,z,norm}      d(vel)/dt
    ang_vel_{x,y,z,norm}  angular velocity       (rad/s, world frame, from quaternions)
    ang_acc_{x,y,z,norm}  d(ang_vel)/dt          (rad/s^2)

    Uses central differences on the actual time stamps (np.gradient), so
    non-uniform sampling is handled. `smooth` > 1 applies a centred moving
    average (in samples) to the velocities before the second derivative.
    """
    out = df.copy()
    n = len(out)
    t = out["time_s"].to_numpy(float)
    smooth = max(1, int(smooth))

    def smoothed(a):
        if smooth <= 1:
            return a
        return pd.DataFrame(a).rolling(smooth, center=True, min_periods=1).mean().to_numpy()

    def grad(a):
        if n < 2:
            return np.full_like(a, np.nan)
        return np.gradient(a, t, axis=0)

    pos = out[["pos_x", "pos_y", "pos_z"]].to_numpy(float)
    vel = smoothed(grad(pos))
    acc = grad(vel)
    omega = smoothed(angular_velocity(out[["quat_x", "quat_y", "quat_z", "quat_w"]].to_numpy(float), t))
    alpha = grad(omega)

    for prefix, arr in (("vel", vel), ("acc", acc), ("ang_vel", omega), ("ang_acc", alpha)):
        for i, axis in enumerate("xyz"):
            out[f"{prefix}_{axis}"] = arr[:, i]
        out[f"{prefix}_norm"] = np.linalg.norm(arr, axis=1)
    return out


# --------------------------------------------------------------------------- #
# Data store
# --------------------------------------------------------------------------- #
class DataStore:
    """Knows which files belong to which experiment; loads them lazily.

    Memory handling:
      * files are only read when a plot needs them, and only the expected columns
      * everything except time_s is stored as float32 (derivatives are still
        computed in float64 first)
      * loaded frames live in an LRU cache limited to `cache_mb`; the least recently
        used files are dropped first and simply re-read if they are needed again
    """

    def __init__(self, cache_mb: int = 1024):
        self.files: dict[tuple[str, int], str] = {}
        self.cache_bytes = int(cache_mb) * 1024 * 1024
        self._cache: OrderedDict[str, tuple] = OrderedDict()  # path -> (mtime, smooth, df, nbytes)
        self._bytes = 0

    # ---- bookkeeping ------------------------------------------------------- #
    def add_paths(self, paths) -> tuple[int, list[str]]:
        """Add files (or the files inside folders). Returns (#added, skipped names)."""
        added, skipped = 0, []
        for p in paths:
            if os.path.isdir(p):
                entries = [os.path.join(p, f) for f in sorted(os.listdir(p))]
                a, _ = self.add_paths([e for e in entries if os.path.isfile(e)])
                added += a
                continue
            key = parse_filename(p)
            if key is None:
                skipped.append(os.path.basename(p))
                continue
            self.files[key] = os.path.abspath(p)
            added += 1
        return added, skipped

    def experiments(self) -> list[int]:
        return sorted({n for _, n in self.files})

    def has(self, kind: str, num) -> bool:
        return (kind, num) in self.files

    def _drop_cached(self, path: str) -> None:
        entry = self._cache.pop(path, None)
        if entry:
            self._bytes -= entry[3]

    def remove(self, nums) -> None:
        nums = set(nums)
        for key in [k for k in self.files if k[1] in nums]:
            self._drop_cached(self.files.pop(key))

    def clear(self) -> None:
        self.files.clear()
        self._cache.clear()
        self._bytes = 0

    # ---- loading ----------------------------------------------------------- #
    def get(self, kind: str, num: int, smooth: int = 1):
        """DataFrame for (kind, experiment) or None if there is no such file.
        pose_data comes with the derived columns."""
        path = self.files.get((kind, num))
        if path is None:
            return None
        s = smooth if kind == "pose_data" else 1
        mtime = os.path.getmtime(path)
        hit = self._cache.get(path)
        if hit and hit[0] == mtime and hit[1] == s:
            self._cache.move_to_end(path)
            return hit[2]

        df = read_experiment_csv(path, kind)
        if kind == "pose_data":
            df = add_derivatives(df, s)
        df = _compact(df)

        self._drop_cached(path)
        nbytes = int(df.memory_usage(index=True).sum())
        self._cache[path] = (mtime, s, df, nbytes)
        self._bytes += nbytes
        while self._bytes > self.cache_bytes and len(self._cache) > 1:
            _, evicted = self._cache.popitem(last=False)
            self._bytes -= evicted[3]
        return df


# --------------------------------------------------------------------------- #
# Plotting
# --------------------------------------------------------------------------- #
@dataclass
class TraceSpec:
    x: str  # "source: column"
    y: str
    axis: int = 1  # y-axis group; only used when the subplot has split_y set


@dataclass
class SubplotSpec:
    traces: list[TraceSpec] = field(default_factory=list)
    equal_aspect: bool = False  # lock x:y scaling to 1:1 (e.g. for x/y pose plots)
    split_y: bool = False  # traces with different `axis` numbers get their own y axis


class MissingData(Exception):
    def __init__(self, kind: str):
        super().__init__(kind)
        self.kind = kind


def resolve_xy(store: DataStore, num: int, trace: TraceSpec, smooth: int):
    """Return the x and y arrays of one trace for one experiment.

    - x = time_s: the time stamps of the y source are used directly.
    - x and y from the same source: plain columns.
    - x and y from different sources (e.g. force_z vs pos_z): y is linearly
      interpolated onto the x source's time stamps, restricted to the time
      range both recordings cover.
    """
    xs, xc = split_axis(trace.x)
    ys, yc = split_axis(trace.y)
    dfy = store.get(ys, num, smooth)
    if dfy is None:
        raise MissingData(ys)
    if xc == "time_s":
        return dfy["time_s"].to_numpy(), dfy[yc].to_numpy()
    dfx = store.get(xs, num, smooth)
    if dfx is None:
        raise MissingData(xs)
    if xs == ys:
        return dfx[xc].to_numpy(), dfy[yc].to_numpy()
    if len(dfx) == 0 or len(dfy) == 0:
        return np.array([]), np.array([])
    tx, ty = dfx["time_s"].to_numpy(), dfy["time_s"].to_numpy()
    mask = (tx >= ty[0]) & (tx <= ty[-1])
    y = np.interp(tx[mask], ty, dfy[yc].to_numpy().astype(np.float64)).astype(np.float32)
    return dfx[xc].to_numpy()[mask], y


def decimate(x: np.ndarray, y: np.ndarray, max_points: int):
    """Min/max decimation: keep the lowest and highest y of every bucket of samples,
    so spikes survive. max_points <= 0 keeps everything."""
    n = len(y)
    if max_points <= 0 or n <= max_points:
        return x, y
    buckets = max(max_points // 2, 1)
    edges = np.linspace(0, n, buckets + 1).astype(int)
    keep = []
    for a, b in zip(edges[:-1], edges[1:]):
        if b <= a:
            continue
        seg = y[a:b]
        if np.all(np.isnan(seg)):
            keep.append(a)
        else:
            keep.extend((a + int(np.nanargmin(seg)), a + int(np.nanargmax(seg))))
    idx = np.unique(keep)
    return x[idx], y[idx]


def _yref(k: int) -> str:
    return "y" if k == 1 else f"y{k}"


def _xref(k: int) -> str:
    return "x" if k == 1 else f"x{k}"


def _ylayout(k: int) -> str:
    return "yaxis" if k == 1 else f"yaxis{k}"


def build_figure(store: DataStore, experiments: list[int], subplots: list[SubplotSpec],
                 smooth: int = 1, height_per_plot: int = 380, max_points: int = 0):
    """One plotly figure, one row per subplot spec.

    Every plotted series (experiment x y-column) gets its own colour, the same one
    everywhere in the figure. With `split_y`, traces are grouped by their `axis`
    number and every group gets its own y axis (alternating left / right).
    Returns (figure, warnings).
    """
    n = len(subplots)
    fig = make_subplots(rows=n, cols=1, vertical_spacing=min(0.08, 0.9 / max(n - 1, 1)))
    palette = list(qualitative.Plotly) + list(qualitative.Dark24)
    colors: dict[str, str] = {}

    def color_of(name: str) -> str:
        if name not in colors:
            colors[name] = palette[len(colors) % len(palette)]
        return colors[name]

    # --- plan the y axes of every subplot -------------------------------------
    plans = []  # per subplot: (group index of every trace, plotly axis number of every group)
    next_axis = n + 1
    for row, spec in enumerate(subplots, start=1):
        if spec.split_y:
            order = list(dict.fromkeys(t.axis for t in spec.traces))
            group_of = [order.index(t.axis) for t in spec.traces]
        else:
            order, group_of = [None], [0] * len(spec.traces)
        axis_ids = [row]
        for _ in order[1:]:
            axis_ids.append(next_axis)
            next_axis += 1
        plans.append((group_of, axis_ids))

    # --- traces (experiment outermost so each experiment's files are read once) ---
    warnings: set[str] = set()
    shown: set[str] = set()
    for num in experiments:
        for row, (spec, (group_of, axis_ids)) in enumerate(zip(subplots, plans), start=1):
            for ti, tr in enumerate(spec.traces):
                try:
                    x, y = resolve_xy(store, num, tr, smooth)
                except MissingData as e:
                    warnings.add(f"exp {num}: no {e.kind} file loaded")
                    continue
                except Exception as e:  # unreadable / malformed file
                    warnings.add(f"exp {num}: {e}")
                    continue
                x, y = decimate(x, y, max_points)
                name = f"exp {num} | {tr.y}"
                fig.add_trace(
                    go.Scattergl(
                        x=x, y=y, mode="lines", name=name, legendgroup=name,
                        showlegend=name not in shown,
                        line=dict(color=color_of(name), width=1.5),
                    ),
                    row=row, col=1,
                )
                shown.add(name)
                if group_of[ti] > 0:
                    fig.data[-1].yaxis = _yref(axis_ids[group_of[ti]])

    # --- axes layout ---------------------------------------------------------------
    step = 0.075  # paper-width reserved per extra y axis
    for row, (spec, (group_of, axis_ids)) in enumerate(zip(subplots, plans), start=1):
        fig.update_xaxes(title_text=" / ".join(dict.fromkeys(t.x for t in spec.traces)),
                         row=row, col=1)
        groups = len(axis_ids)
        if groups == 1:
            fig.update_yaxes(title_text=" / ".join(dict.fromkeys(t.y for t in spec.traces)),
                             row=row, col=1)
            if spec.equal_aspect:
                # anchor y to x with ratio 1 -> same units per pixel on both axes
                fig.update_yaxes(scaleanchor=_xref(row), scaleratio=1, row=row, col=1)
            continue

        # split y axes (the 1:1 lock is ignored here: it only makes sense for one y axis)
        n_left, n_right = (groups + 1) // 2, groups // 2
        x0, x1 = step * (n_left - 1), 1 - step * n_right
        fig.update_xaxes(domain=[x0, x1], row=row, col=1)
        domain = list(getattr(fig.layout, _ylayout(row)).domain)
        for g, k in enumerate(axis_ids):
            labels = list(dict.fromkeys(
                t.y for ti, t in enumerate(spec.traces) if group_of[ti] == g))
            props: dict = {"title": {"text": " / ".join(labels)}}
            if len(experiments) == 1 and len(labels) == 1:
                name = f"exp {experiments[0]} | {labels[0]}"
                if name in colors:  # tint the axis like its trace
                    props["color"] = colors[name]
            if g == 0:
                fig.update_layout({_ylayout(k): props})
                continue
            side, j = ("left" if g % 2 == 0 else "right"), g // 2
            props.update(
                overlaying=_yref(row), side=side, domain=domain,
                showgrid=False, zeroline=False,
                anchor=_xref(row) if (side == "right" and j == 0) else "free",
                position=(x0 - step * j) if side == "left" else (x1 + step * j),
            )
            fig.update_layout({_ylayout(k): props})

    fig.update_layout(height=max(500, height_per_plot * n), template="plotly_white",
                      margin=dict(l=70, r=70, t=40, b=60))
    return fig, sorted(warnings)


# --------------------------------------------------------------------------- #
# Output
# --------------------------------------------------------------------------- #
PLOT_DIR = os.path.join(os.path.expanduser("~"), "force_pose_plots")
_server = None


class _QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *args):  # keep the console clean
        pass


def _ensure_server() -> ThreadingHTTPServer:
    """Tiny local web server for PLOT_DIR. Opening plots via http://127.0.0.1 works
    even when the browser can't read local files (snap/flatpak Firefox can't see /tmp)."""
    global _server
    if _server is None:
        os.makedirs(PLOT_DIR, exist_ok=True)
        handler = functools.partial(_QuietHandler, directory=PLOT_DIR)
        _server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        threading.Thread(target=_server.serve_forever, daemon=True).start()
    return _server


def export_figure(fig, path: str | None = None) -> str:
    """Write the figure to HTML. With a path: just save it there. Without a path:
    save it into ~/force_pose_plots and open it in the browser via localhost."""
    config = {"scrollZoom": True, "displaylogo": False}
    if path is not None:
        fig.write_html(path, auto_open=False, config=config)
        return path
    os.makedirs(PLOT_DIR, exist_ok=True)
    name = f"force_pose_plot_{int(time.time())}.html"
    path = os.path.join(PLOT_DIR, name)
    fig.write_html(path, auto_open=False, config=config)
    webbrowser.open(f"http://127.0.0.1:{_ensure_server().server_address[1]}/{name}")
    return path
