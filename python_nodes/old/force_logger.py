#!/usr/bin/env python3

import csv
import rclpy
import sys

from enum import Enum, auto
from collections import deque
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped


CONTACT_THRESH_N = 0.0
FORCE_TOPIC = "/bota_ft/wrench_compensated"


class LoggerState(Enum):
    READY = auto()
    REC = auto()


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ForceLogger(Node):

    def __init__(self, exp_num):
        super().__init__("force_logger")

        self.force_sub = self.create_subscription(
            WrenchStamped,
            FORCE_TOPIC,
            self.wrench_callback,
            10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.pose_callback,
            10
        )

        # Pre-contact buffers (one per topic, since the rates differ)
        self.wrench_buffer = deque(maxlen=500)
        self.pose_buffer = deque(maxlen=500)

        self.state = LoggerState.READY
        self.start_time = None  # shared time origin for both CSVs

        force_name = f"./data/force_data-{exp_num}.csv"
        pose_name = f"./data/pose_data-{exp_num}.csv"

        self.force_file = open(force_name, 'w', newline='')
        self.force_writer = csv.writer(self.force_file)
        self.force_writer.writerow(
            ['time_s', 'force_x', 'force_y', 'force_z',
             'torque_x', 'torque_y', 'torque_z'])

        self.pose_file = open(pose_name, 'w', newline='')
        self.pose_writer = csv.writer(self.pose_file)
        self.pose_writer.writerow(
            ['time_s', 'pos_x', 'pos_y', 'pos_z',
             'quat_x', 'quat_y', 'quat_z', 'quat_w'])

        self.get_logger().info(
            f"Logger READY -> '{force_name}', '{pose_name}' | "
            f"Trigger threshold: {CONTACT_THRESH_N} N")

    # ---------- callbacks ----------

    def wrench_callback(self, msg):
        force_z = abs(msg.wrench.force.z)

        if self.state is LoggerState.READY:
            self.wrench_buffer.append(msg)

            if force_z >= CONTACT_THRESH_N:
                self.state = LoggerState.REC
                self.get_logger().info(
                    f"Contact hit ({force_z:.2f} N)! Flushing buffers and recording.")

                # Time origin = oldest buffered wrench sample
                self.start_time = stamp_to_sec(self.wrench_buffer[0].header.stamp)

                for buf_msg in self.wrench_buffer:
                    self.write_wrench(buf_msg)

                # Only keep poses that fall inside the logged time window
                for buf_pose in self.pose_buffer:
                    if stamp_to_sec(buf_pose.header.stamp) >= self.start_time:
                        self.write_pose(buf_pose)

                self.wrench_buffer.clear()
                self.pose_buffer.clear()
        else:
            self.write_wrench(msg)

    def pose_callback(self, msg):
        if self.state is LoggerState.READY:
            self.pose_buffer.append(msg)
        else:
            self.write_pose(msg)

    # ---------- writers ----------

    def write_wrench(self, msg):
        rel_time = stamp_to_sec(msg.header.stamp) - self.start_time
        force = msg.wrench.force
        torque = msg.wrench.torque
        self.force_writer.writerow([
            rel_time,
            force.x, force.y, force.z,
            torque.x, torque.y, torque.z
        ])

    def write_pose(self, msg):
        rel_time = stamp_to_sec(msg.header.stamp) - self.start_time
        p = msg.pose.position
        q = msg.pose.orientation
        self.pose_writer.writerow([
            rel_time,
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w
        ])

    # ---------- cleanup ----------

    def destroy_node(self):
        closed_any = False
        for name in ('force_file', 'pose_file'):
            f = getattr(self, name, None)
            if f is not None and not f.closed:
                f.close()
                closed_any = True
        if closed_any:
            print("Recording done.")

        super().destroy_node()


def main(args=None):
    exp_num = sys.argv[1] if len(sys.argv) > 1 else "0"

    rclpy.init(args=args)
    node = ForceLogger(exp_num)
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
