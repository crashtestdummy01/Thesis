#!/usr/bin/env python3
import csv
import os

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

HEADER = ["x", "y", "z", "qx", "qy", "qz", "qw"]


class PoseRecorder(Node):
    def __init__(self):
        super().__init__("pose_recorder")

        self.declare_parameter("base_frame", "fr3_link0")
        self.declare_parameter("ee_frame", "fr3_link8")
        self.declare_parameter("csv_path", "calibration_poses.csv")
        self.declare_parameter("overwrite", True)  # False -> keep appending to existing file

        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.csv_path = os.path.expanduser(self.get_parameter("csv_path").value)

        
        with open(self.csv_path, "w", newline="") as f:
            csv.writer(f).writerow(HEADER)
        self.count = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_service(Trigger, "~/record_pose", self.on_record)
        self.get_logger().info(
            f"Ready. Recording {self.base_frame} -> {self.ee_frame} to {self.csv_path} "
            f"({self.count} poses already in file)"
        )

    def on_record(self, request, response):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, Time(), timeout=Duration(seconds=1.0)
            )
        except Exception as e:  # tf2 exceptions
            response.success = False
            response.message = f"TF lookup failed: {e}"
            self.get_logger().error(response.message)
            return response

        t, q = tf.transform.translation, tf.transform.rotation
        row = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
        with open(self.csv_path, "a", newline="") as f:
            csv.writer(f).writerow([f"{v:.8f}" for v in row])

        self.count += 1
        response.success = True
        response.message = f"Recorded pose #{self.count}"
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = PoseRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
