"""
Core of the force / pose experiment viewer (no GUI code in here).

 - file name parsing   (force_data-N, estimated_force_data-N, pose_data-N, target_pose_data-N)
 - CSV loading + cleaning
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
KINDS = ["force_data", "estimated_force_data", "pose_data", "target_pose_data"]

POSE_COLS = ["time_s", "pos_x", "pos_y", "pos_z", "quat_x", "quat_y", "quat_z", "quat_w"]
FORCE_COLS = ["time_s", "force_x", "force_y", "force_z", "torque_x", "torque_y", "torque_z"]

BASE_COLUMNS = {
    "force_data": FORCE_COLS,
    "estimated_force_data": FORCE_COLS,
    "pose_data": POSE_COLS,
    "target_pose_data": POSE_COLS,
}

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
    r"^(estimated_force_data|force_data|pose_data|target_pose_data)-(\d+)(?:\.csv)?$",
    re.IGNORECASE,
)


def parse_filename(path: str):
    """'/x/pose_data-12.csv' -> ('pose_data', 12); None if the name doesn't match."""
    m = _FILE_RE.match(os.path.basename(path))
    if not m:
        return None
    return m.group(1).lower(), int(m.group(2))


def axis_options() -> list[str]:
    """Labels for the axis dropdowns: 'source: column'."""
    return [f"{kind}: {col}" for kind in KINDS for col in SOURCE_COLUMNS[kind]]


def split_axis(label: str) -> tuple[str, str]:
    source, column = label.split(": ", 1)
    return source, column


# --------------------------------------------------------------------------- #
# Loading
# --------------------------------------------------------------------------- #
def read_experiment_csv(path: str, kind: str) -> pd.DataFrame:
    """Read one data file, keep the expected columns, coerce to numbers,
    sort by time and drop duplicated time stamps."""
    df = pd.read_csv(path)
    df.columns = [str(c).strip() for c in df.columns]
    expected = BASE_COLUMNS[kind]
    missing = [c for c in expected if c not in df.columns]
    if missing:
        raise ValueError(f"{os.path.basename(path)}: missing column(s) {', '.join(missing)}")
    df = df[expected].apply(pd.to_numeric, errors="coerce")
    df = df.dropna(subset=["time_s"]).sort_values("time_s", kind="stable")
    df = df.drop_duplicates(subset="time_s", keep="first").reset_index(drop=True)
    return df


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
    """Knows which files belong to which experiment; loads them lazily and caches."""

    def __init__(self):
        self.files: dict[tuple[str, int], str] = {}
        self._cache: dict = {}

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

    def remove(self, nums) -> None:
        nums = set(nums)
        for key in [k for k in self.files if k[1] in nums]:
            del self.files[key]

    def clear(self) -> None:
        self.files.clear()
        self._cache.clear()

    def get(self, kind: str, num: int, smooth: int = 1):
        """DataFrame for (kind, experiment) or None if there is no such file.
        pose_data comes with the derived columns."""
        path = self.files.get((kind, num))
        if path is None:
            return None
        s = smooth if kind == "pose_data" else 1
        key = (path, os.path.getmtime(path), s)
        if key not in self._cache:
            df = read_experiment_csv(path, kind)
            if kind == "pose_data":
                df = add_derivatives(df, s)
            self._cache[key] = df
        return self._cache[key]


# --------------------------------------------------------------------------- #
# Plotting
# --------------------------------------------------------------------------- #
@dataclass
class TraceSpec:
    x: str  # "source: column"
    y: str


@dataclass
class SubplotSpec:
    traces: list[TraceSpec] = field(default_factory=list)
    equal_aspect: bool = False  # lock x:y scaling to 1:1 (e.g. for x/y pose plots)


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
    return dfx[xc].to_numpy()[mask], np.interp(tx[mask], ty, dfy[yc].to_numpy())


def build_figure(store: DataStore, experiments: list[int], subplots: list[SubplotSpec],
                 smooth: int = 1, height_per_plot: int = 380):
    """One plotly figure, one row per subplot spec. Colour = experiment,
    line dash = trace index within the subplot. Returns (figure, warnings)."""
    n = len(subplots)
    fig = make_subplots(rows=n, cols=1, vertical_spacing=min(0.08, 0.9 / max(n - 1, 1)))
    palette = qualitative.Plotly
    colors = {num: palette[i % len(palette)] for i, num in enumerate(experiments)}
    dashes = ["solid", "dash", "dot", "dashdot", "longdash", "longdashdot"]
    warnings: set[str] = set()
    shown: set[str] = set()

    for row, spec in enumerate(subplots, start=1):
        for ti, tr in enumerate(spec.traces):
            for num in experiments:
                try:
                    x, y = resolve_xy(store, num, tr, smooth)
                except MissingData as e:
                    warnings.add(f"exp {num}: no {e.kind} file loaded")
                    continue
                except Exception as e:  # unreadable / malformed file
                    warnings.add(f"exp {num}: {e}")
                    continue
                name = f"exp {num} | {tr.y}"
                fig.add_trace(
                    go.Scattergl(
                        x=x, y=y, mode="lines", name=name, legendgroup=name,
                        showlegend=name not in shown,
                        line=dict(color=colors[num], dash=dashes[ti % len(dashes)], width=1.5),
                    ),
                    row=row, col=1,
                )
                shown.add(name)
        xt = " / ".join(dict.fromkeys(t.x for t in spec.traces))
        yt = " / ".join(dict.fromkeys(t.y for t in spec.traces))
        fig.update_xaxes(title_text=xt, row=row, col=1)
        fig.update_yaxes(title_text=yt, row=row, col=1)
        if spec.equal_aspect:
            # anchor this subplot's y axis to its x axis with ratio 1 -> same units per pixel
            fig.update_yaxes(scaleanchor="x" if row == 1 else f"x{row}", scaleratio=1,
                             row=row, col=1)

    fig.update_layout(height=max(500, height_per_plot * n), template="plotly_white",
                      margin=dict(l=70, r=30, t=40, b=60))
    return fig, sorted(warnings)


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
