#!/usr/bin/env python3

import csv
import rclpy
import numpy as np


from enum import Enum, auto
from collections import deque
from string import Template
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


CONTACT_THRESH_N = 6.5
CSV_FILE = Template("exp-${seq_number}")


class LoggerState(Enum):
    READY = auto()
    REC = auto()

class ForceLogger(Node):

    def __init__(self, filename):
        super().__init__("force_logger")
        
        self.force_sub = self.create_subscription(
            WrenchStamped,
            '/cartesian_impedance_controller/measured_force',
            self.wrench_callback,
            10
        )

        self.ring_buffer = deque(maxlen=100)

        self.state = LoggerState.READY
        self.start_time = None
        self.time_since_start = 0

        self.csv_file = open(f"{filename}.csv", 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow(['time_s', 'force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z'])

        self.get_logger().info(f"Logger READY -> '{filename}' | Trigger threshold: {CONTACT_THRESH_N} N")

    def wrench_callback(self, msg):
        force_z = abs(msg.wrench.force.z)

        if self.state is LoggerState.READY:
            self.ring_buffer.append(msg)

            if force_z >= CONTACT_THRESH_N:
                self.state = LoggerState.REC
                self.get_logger().info(f"Contact hit ({force_z:.2f} N)! Flushing buffer and recording.")

                if len(self.ring_buffer) > 0:
                    for buf_msg in self.ring_buffer:
                        self.write_row(buf_msg)
        else:
            self.write_row(msg)

    def write_row(self, msg):
        self.latest_msg = msg

        stamp = msg.header.stamp
        t = stamp.sec + (stamp.nanosec * 1e-9)

        if self.start_time is None:
            self.start_time = t

        rel_time = t - self.start_time

        force = msg.wrench.force
        torque = msg.wrench.torque
        self.csv_writer.writerow([
            rel_time, 
            force.x, force.y, force.z, 
            torque.x, torque.y, torque.z
        ])

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info("Recording done.")

        super().destroy_node()


def main(args=None):
    exp_num = sys.argv[1] if len(sys.argv) > 1 else "0"

    rclpy.init(args=args)
    node = ForceLogger(f"force_data-{exp_num}")
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
