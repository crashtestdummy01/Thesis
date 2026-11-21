import pandas as pd
import plotly.graph_objects as go

Q_TWIST_COLUMNS = [
    "time_s",
    "pos_q0", "pos_q1", "pos_q2", "pos_q3", "pos_q4", "pos_q5", "pos_q6",
    "vel_q0", "vel_q1", "vel_q2", "vel_q3", "vel_q4", "vel_q5", "vel_q6",
]

Q_EFFORT_COLUMNS = [
    "time_s",
    "effort_q0", "effort_q1", "effort_q2", "effort_q3", "effort_q4", "effort_q5", "effort_q6",
]


def plot_joint_velocities(twist_csv_path: str) -> go.Figure:
    """Load a twist CSV and plot velocities of joints 1, 3, and 5 against time."""
    df = pd.read_csv(twist_csv_path)
    missing = set(Q_TWIST_COLUMNS) - set(df.columns)
    if missing:
        raise ValueError(f"Missing expected twist columns: {sorted(missing)}")

    fig = go.Figure()
    for joint in (1, 3, 5):
        col = f"vel_q{joint}"
        fig.add_trace(
            go.Scatter(
                x=df["time_s"],
                y=df[col],
                mode="lines",
                name=col,
            )
        )

    fig.update_layout(
        title="Joint Velocities (q1, q3, q5) vs. Time",
        xaxis_title="Time (s)",
        yaxis_title="Velocity",
        hovermode="x unified",
    )
    return fig


def plot_joint_efforts(effort_csv_path: str) -> go.Figure:
    """Load an effort CSV and plot efforts of joints 1, 3, and 5 against time."""
    df = pd.read_csv(effort_csv_path)
    missing = set(Q_EFFORT_COLUMNS) - set(df.columns)
    if missing:
        raise ValueError(f"Missing expected effort columns: {sorted(missing)}")

    fig = go.Figure()
    for joint in (1, 3, 5):
        col = f"effort_q{joint}"
        fig.add_trace(
            go.Scatter(
                x=df["time_s"],
                y=df[col],
                mode="lines",
                name=col,
            )
        )

    fig.update_layout(
        title="Joint Efforts (q1, q3, q5) vs. Time",
        xaxis_title="Time (s)",
        yaxis_title="Effort",
        hovermode="x unified",
    )
    return fig


if __name__ == "__main__":
    twist_file = "joint_position_data-63.csv"
    effort_file = "joint_effort_data-63.csv"

    vel_fig = plot_joint_velocities(twist_file)
    effort_fig = plot_joint_efforts(effort_file)

    vel_fig.show()
    effort_fig.show()
