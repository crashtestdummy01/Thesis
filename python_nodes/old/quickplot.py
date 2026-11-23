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

FORCE_COLUMNS = [
    "time_s",
    "force_x", "force_y", "force_z",
    "torque_x", "torque_y", "torque_z",
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


def plot_force_z_and_vel_q3(force_csv_path: str, twist_csv_path: str) -> go.Figure:
    """Load force and twist CSVs and plot force_z and vel_q3 against time_s."""
    force_df = pd.read_csv(force_csv_path)
    missing_force = set(FORCE_COLUMNS) - set(force_df.columns)
    if missing_force:
        raise ValueError(f"Missing expected force columns: {sorted(missing_force)}")

    twist_df = pd.read_csv(twist_csv_path)
    missing_twist = set(Q_TWIST_COLUMNS) - set(twist_df.columns)
    if missing_twist:
        raise ValueError(f"Missing expected twist columns: {sorted(missing_twist)}")

    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=force_df["time_s"],
            y=force_df["force_z"],
            mode="lines",
            name="force_z",
            yaxis="y1",
        )
    )
    fig.add_trace(
        go.Scatter(
            x=twist_df["time_s"],
            y=twist_df["vel_q3"],
            mode="lines",
            name="vel_q3",
            yaxis="y2",
        )
    )

    fig.update_layout(
        title="force_z and vel_q3 vs. Time",
        xaxis_title="Time (s)",
        yaxis=dict(title="Force Z"),
        yaxis2=dict(
            title="Velocity q3",
            overlaying="y",
            side="right",
        ),
        hovermode="x unified",
    )
    return fig


if __name__ == "__main__":
    twist_file = "joint_position_data-63.csv"
    effort_file = "joint_effort_data-63.csv"
    force_file = "force_data-63.csv"

    vel_fig = plot_joint_velocities(twist_file)
    effort_fig = plot_joint_efforts(effort_file)
    force_vel_fig = plot_force_z_and_vel_q3(force_file, twist_file)

    #vel_fig.show()
    #effort_fig.show()
    force_vel_fig.show()
