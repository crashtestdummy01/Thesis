#!/usr/bin/env python3
"""Node 2: moves the arm through the saved calibration poses, one per service call.

    ros2 service call /pose_player/next_pose std_srvs/srv/Trigger   # blocks until arrived + settled
    ros2 service call /pose_player/restart   std_srvs/srv/Trigger   # go back to pose #1 next

Motion logic mirrors the Reset behavior: linear interpolation for position,
SLERP for orientation, with a fixed max speed per tick. /target_pose is published
every control tick (20 Hz). The target starts from the *current* TF pose so the
arm never jumps when the node starts.
"""
import csv
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R, Slerp
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


class PosePlayer(Node):
    def __init__(self):
        super().__init__("pose_player")

        self.declare_parameter("base_frame", "fr3_link0")
        self.declare_parameter("ee_frame", "fr3_link8")
        self.declare_parameter("csv_path", "calibration_poses.csv")
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("move_speed", 0.04)      # m/s
        self.declare_parameter("angular_speed", 0.15)    # rad/s
        self.declare_parameter("settle_time", 1.0)      # s to wait after arrival before the service returns
        self.declare_parameter("move_timeout", 60.0)    # s

        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.move_speed = self.get_parameter("move_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.settle_time = self.get_parameter("settle_time").value
        self.move_timeout = self.get_parameter("move_timeout").value
        self.dt = 1.0 / self.get_parameter("rate_hz").value

        self.poses = self._load_poses(os.path.expanduser(self.get_parameter("csv_path").value))
        self.get_logger().info(f"Loaded {len(self.poses)} poses")

        self.lock = threading.Lock()
        self.next_idx = 0
        self.goal = None            # (pos np[3], quat np[4]) or None when idle
        self.arrived = threading.Event()
        self.target_pose = None     # PoseStamped, initialised from TF

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseStamped, self.get_parameter("target_topic").value, 10)

        # Reentrant group + MultiThreadedExecutor so the blocking service doesn't stall the control loop.
        cb = ReentrantCallbackGroup()
        self.create_timer(self.dt, self.tick, callback_group=cb)
        self.create_service(Trigger, "~/next_pose", self.on_next, callback_group=cb)
        self.create_service(Trigger, "~/restart", self.on_restart, callback_group=cb)

    # ------------------------------------------------------------------ setup
    @staticmethod
    def _load_poses(path):
        poses = []
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                p = np.array([float(row[k]) for k in ("x", "y", "z")])
                q = np.array([float(row[k]) for k in ("qx", "qy", "qz", "qw")])
                poses.append((p, q / np.linalg.norm(q)))
        return poses

    def _init_target_from_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, Time())
        except Exception:
            self.get_logger().warn("Waiting for TF...", throttle_duration_sec=2.0)
            return False
        msg = PoseStamped()
        msg.header.frame_id = self.base_frame
        t, q = tf.transform.translation, tf.transform.rotation
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = t.x, t.y, t.z
        msg.pose.orientation.x, msg.pose.orientation.y = q.x, q.y
        msg.pose.orientation.z, msg.pose.orientation.w = q.z, q.w
        self.target_pose = msg
        self.get_logger().info("Target initialised from current TF pose")
        return True

    # ------------------------------------------------------------------ services
    def on_next(self, request, response):
        with self.lock:
            if self.target_pose is None:
                response.success, response.message = False, "No TF yet, target not initialised"
                return response
            if self.goal is not None:
                response.success, response.message = False, "Still moving to previous pose"
                return response
            if self.next_idx >= len(self.poses):
                response.success, response.message = False, "No more poses (call restart to loop)"
                return response
            idx = self.next_idx
            self.goal = self.poses[idx]
            self.next_idx += 1
            self.arrived.clear()

        self.get_logger().info(f"Moving to pose {idx + 1}/{len(self.poses)}")
        if not self.arrived.wait(timeout=self.move_timeout):
            response.success, response.message = False, f"Timeout moving to pose {idx + 1}"
            return response

        time.sleep(self.settle_time)  # let the arm stop oscillating before you record the sample
        response.success = True
        response.message = f"Arrived at pose {idx + 1}/{len(self.poses)}"
        return response

    def on_restart(self, request, response):
        with self.lock:
            self.next_idx = 0
        response.success, response.message = True, "Next call to next_pose goes to pose 1"
        return response

    # ------------------------------------------------------------------ control loop
    def tick(self):
        if self.target_pose is None:
            if not self._init_target_from_tf():
                return

        with self.lock:
            if self.goal is not None:
                goal_p, goal_q = self.goal
                pos_done = self._step_position(goal_p)
                rot_done = self._step_orientation(goal_q)
                if pos_done and rot_done:
                    self.goal = None
                    self.arrived.set()

            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.target_pose)

    def _step_position(self, goal_p):
        curr = self.target_pose.pose.position
        d = goal_p - np.array([curr.x, curr.y, curr.z])
        dist = np.linalg.norm(d)
        step = self.move_speed * self.dt
        if dist <= step:
            curr.x, curr.y, curr.z = map(float, goal_p)
            return True
        d *= step / dist
        curr.x += float(d[0]); curr.y += float(d[1]); curr.z += float(d[2])
        return False

    def _step_orientation(self, goal_q, epsilon=0.001):
        o = self.target_pose.pose.orientation
        q_curr = np.array([o.x, o.y, o.z, o.w])
        q_goal = goal_q.copy()
        q_curr /= np.linalg.norm(q_curr)

        if np.dot(q_curr, q_goal) < 0:  # shortest path
            q_goal = -q_goal

        angle_error = 2.0 * math.acos(float(np.clip(np.dot(q_curr, q_goal), -1.0, 1.0)))
        step = self.angular_speed * self.dt

        if angle_error <= epsilon:
            q_next, done = q_goal, True
        else:
            ratio = min(step / angle_error, 1.0)
            slerp = Slerp([0.0, 1.0], R.from_quat([q_curr, q_goal]))
            q_next, done = slerp(ratio).as_quat(), False

        o.x, o.y, o.z, o.w = (float(v) for v in q_next)
        return done


def main():
    rclpy.init()
    node = PosePlayer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
