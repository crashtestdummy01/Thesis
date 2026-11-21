#!/usr/bin/env python3
"""Triggered multi-topic CSV logger for ROS 2.

To log another topic, add one ChannelSpec to CHANNELS below (and, if the
message type is new, one small row-extractor function). Nothing else changes.
"""

import csv
import os
import sys
from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Optional, Sequence

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState


# ----------------------------------------------------------------------
# Row extractors: message -> list of values (must match the columns)
# ----------------------------------------------------------------------

def wrench_row(msg):
    f, t = msg.wrench.force, msg.wrench.torque
    return [f.x, f.y, f.z, t.x, t.y, t.z]


def pose_row(msg):
    p, q = msg.pose.position, msg.pose.orientation
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]

def q_twist_row(msg: JointState):
    q, q_dot = msg.position, msg.velocity
    return [q[0], q[1], q[2], q[3], q[4], q[5], q[6], q_dot[0], q_dot[1], q_dot[2], q_dot[3], q_dot[4], q_dot[5], q_dot[6]]

def q_effort_row(msg: JointState):
    q_tau = msg.effort
    return [q_tau[0], q_tau[1], q_tau[2], q_tau[3], q_tau[4], q_tau[5], q_tau[6]]


WRENCH_COLUMNS = ['force_x', 'force_y', 'force_z',
                  'torque_x', 'torque_y', 'torque_z']
POSE_COLUMNS = ['pos_x', 'pos_y', 'pos_z',
                'quat_x', 'quat_y', 'quat_z', 'quat_w']
Q_TWIST_COLUMNS = ["pos_q0", "pos_q1", "pos_q2", "pos_q3", "pos_q4", "pos_q5", "pos_q6",
                   "vel_q0", "vel_q1", "vel_q2", "vel_q3", "vel_q4", "vel_q5", "vel_q6"]
Q_EFFORT_COLUMNS = ["effort_q0", "effort_q1", "effort_q2", "effort_q3", "effort_q4", "effort_q5", "effort_q6"]


# ----------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------

def header_stamp(msg):
    """Default timestamp source: msg.header.stamp as seconds."""
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


@dataclass
class ChannelSpec:
    name: str                          # used in the output filename
    topic: str
    msg_type: type
    columns: Sequence[str]             # CSV columns after time_s
    extract: Callable                  # msg -> list of values
    stamp_fn: Callable = header_stamp  # override for messages without a header
    buffer_size: int = 500             # pre-trigger samples kept
    qos_depth: int = 10


@dataclass
class TriggerSpec:
    channel: str                       # ChannelSpec.name that starts recording
    predicate: Callable                # msg -> bool


DATA_DIR = "./data"
CONTACT_THRESH_N = 0.0

CHANNELS = [
    ChannelSpec(
        name="force",
        topic="/bota_ft/wrench_compensated",
        msg_type=WrenchStamped,
        columns=WRENCH_COLUMNS,
        extract=wrench_row,
    ),
    ChannelSpec(
        name="pose",
        topic="/current_pose",
        msg_type=PoseStamped,
        columns=POSE_COLUMNS,
        extract=pose_row,
    ),
    ChannelSpec(
        name="target_pose",
        topic="/target_pose",
        msg_type=PoseStamped,
        columns=POSE_COLUMNS,
        extract=pose_row,
    ),
    ChannelSpec(
        name="estimated_force",
        topic="/cartesian_impedance_controller/measured_force",
        msg_type=WrenchStamped,
        columns=WRENCH_COLUMNS,
        extract=wrench_row,
    ),
    ChannelSpec(
        name="joint_position",
        topic="/joint_states",
        msg_type=JointState,
        columns=Q_TWIST_COLUMNS,
        extract=q_twist_row
    ),
    ChannelSpec(
        name="joint_effort",
        topic="/joint_states",
        msg_type=JointState,
        columns=Q_EFFORT_COLUMNS,
        extract=q_effort_row
    )
]

TRIGGER = TriggerSpec(
    channel="force",
    predicate=lambda msg: abs(msg.wrench.force.z) >= CONTACT_THRESH_N,
)


# ----------------------------------------------------------------------
# Implementation
# ----------------------------------------------------------------------

class LoggerState(Enum):
    READY = auto()
    REC = auto()


class Channel:
    """One topic -> one CSV file, with a pre-trigger ring buffer."""

    def __init__(self, spec: ChannelSpec, exp_num: str):
        self.spec = spec
        self.buffer = deque(maxlen=spec.buffer_size)
        self.path = os.path.join(DATA_DIR, f"{spec.name}_data-{exp_num}.csv")
        self.file = open(self.path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time_s', *spec.columns])

    def buffer_msg(self, msg):
        self.buffer.append(msg)

    def oldest_stamp(self) -> Optional[float]:
        return self.spec.stamp_fn(self.buffer[0]) if self.buffer else None

    def write(self, msg, t0):
        self.writer.writerow(
            [self.spec.stamp_fn(msg) - t0, *self.spec.extract(msg)])

    def flush_buffer(self, t0):
        """Write buffered samples inside the logged time window."""
        for msg in self.buffer:
            if self.spec.stamp_fn(msg) >= t0:
                self.write(msg, t0)
        self.buffer.clear()

    def close(self) -> bool:
        if self.file.closed:
            return False
        self.file.close()
        return True


class ExperimentLogger(Node):

    def __init__(self, exp_num, channels=CHANNELS, trigger=TRIGGER):
        super().__init__("force_logger")

        names = [c.name for c in channels]
        if len(set(names)) != len(names):
            raise ValueError(f"Duplicate channel names: {names}")
        if trigger.channel not in names:
            raise ValueError(f"Trigger channel '{trigger.channel}' not in {names}")

        os.makedirs(DATA_DIR, exist_ok=True)

        self.trigger = trigger
        self.state = LoggerState.READY
        self.start_time = None  # shared time origin for all CSVs

        self.channels = {}
        self._subs = []
        for spec in channels:
            ch = Channel(spec, exp_num)
            self.channels[spec.name] = ch
            self._subs.append(self.create_subscription(
                spec.msg_type, spec.topic,
                lambda msg, ch=ch: self._on_msg(ch, msg),
                spec.qos_depth))

        files = ", ".join(f"'{c.path}'" for c in self.channels.values())
        self.get_logger().info(
            f"Logger READY -> {files} | trigger on '{trigger.channel}'")

    # ---------- callbacks ----------

    def _on_msg(self, ch: Channel, msg):
        if self.state is LoggerState.REC:
            ch.write(msg, self.start_time)
            return

        ch.buffer_msg(msg)
        if ch.spec.name == self.trigger.channel and self.trigger.predicate(msg):
            self._start_recording()

    def _start_recording(self):
        self.state = LoggerState.REC
        self.get_logger().info("Trigger hit! Flushing buffers and recording.")

        # Time origin = oldest buffered sample of the trigger channel
        self.start_time = self.channels[self.trigger.channel].oldest_stamp()
        for ch in self.channels.values():
            ch.flush_buffer(self.start_time)

    # ---------- cleanup ----------

    def destroy_node(self):
        closed_any = False
        for ch in getattr(self, 'channels', {}).values():
            closed_any |= ch.close()
        if closed_any:
            print("Recording done.")
        super().destroy_node()


def main(args=None):
    exp_num = sys.argv[1] if len(sys.argv) > 1 else "0"

    rclpy.init(args=args)
    node = ExperimentLogger(exp_num)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
