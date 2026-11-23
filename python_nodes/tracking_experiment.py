#!/usr/bin/env python3

from copy import deepcopy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from statemachine import State
from tf2_ros import Buffer, TransformListener, TransformException

from behaviors import StateBehavior, BehaviorGraph
from tracker import TrackMarkerSB


class FreeContactFSM(BehaviorGraph):
    init = State(initial=True)
    track = State()
    done = State()
    exit = State(final=True)

    begin_track = init.to(track)
    track_done = track.to(done)          # renamed to match TrackMarkerSB
    stop = done.to(exit)
    board_lost = done.to(track)
    abort = (
        init.to(exit)
        | track.to(exit)
        | done.to(exit)
    )

    def __init__(self, node):
        super().__init__(node)
        self.node = node

        self.attach_behavior('init', InitSB())
        self.attach_behavior('track', TrackMarkerSB())
        self.attach_behavior('done', DoneSB())


class FreeContactExperimentNode(Node):
    def __init__(self):
        super().__init__('free_contact_node')

        # Parameters
        self.declare_parameter('base_frame', 'fr3_link0')   # must match target_pose frame
        self.declare_parameter('marker_frame', 'cam1_marker')
        self.declare_parameter('marker_max_age', 1.0)         # s, older TF is treated as lost
        self.base_frame = self.get_parameter('base_frame').value
        self.marker_frame = self.get_parameter('marker_frame').value
        self.marker_max_age = self.get_parameter('marker_max_age').value

        # Motion configuration
        self.dt = 0.05                       # 20 Hz
        self.angular_speed = 0.15             # rad/s
        self.linear_speed = 0.02            # m/s (was translation_speed)
        self.standoff = 0.05                 # m (was surface_offset)
        self.init_pose_received = False

        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.initial_pose = PoseStamped()

        # TF (the listener uses this node's executor, so never block in lookups)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS setup
        self.pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/current_pose', self.current_pose_callback, 10
        )

        # FSM and control loop
        self.fsm = FreeContactFSM(node=self)
        self.fsm.validate()
        self.fsm.start_fsm()
        self.timer = self.create_timer(self.dt, self.control_loop)

    # ---------- callbacks ----------
    def current_pose_callback(self, msg: PoseStamped):
        self.current_pose = msg
        if not self.init_pose_received:
            self.target_pose = deepcopy(msg)
            self.initial_pose = deepcopy(msg)
            self.init_pose_received = True           # was never set before
            if msg.header.frame_id and msg.header.frame_id != self.base_frame:
                self.get_logger().warn(
                    f"current_pose frame '{msg.header.frame_id}' != base_frame "
                    f"'{self.base_frame}'; marker poses will be in the wrong frame."
                )
            self.get_logger().info('Pose init done.')

    # ---------- helpers used by behaviors ----------
    def get_marker_pose(self):
        """Marker pose in base_frame as (Rotation, np.ndarray(3)), or None if missing/stale."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.marker_frame, Time(),
                timeout=Duration(seconds=0.0),       # non-blocking
            )
        except TransformException as e:
            self.get_logger().warn(f"Marker TF unavailable: {e}", throttle_duration_sec=2.0)
            return None

        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        if age > self.marker_max_age:
            self.get_logger().warn(f"Marker TF stale ({age:.2f} s)", throttle_duration_sec=2.0)
            return None

        q, p = t.transform.rotation, t.transform.translation
        return R.from_quat([q.x, q.y, q.z, q.w]), np.array([p.x, p.y, p.z])

    def get_ee_pose(self):
        """Measured EE pose in base_frame (only needed if you re-add the lead clamp)."""
        o, p = self.current_pose.pose.orientation, self.current_pose.pose.position
        return R.from_quat([o.x, o.y, o.z, o.w]), np.array([p.x, p.y, p.z])

    # ---------- loop ----------
    def control_loop(self):
        self.fsm.tick()

        if not self.init_pose_received:
            return                                    # never publish the empty default pose
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.target_pose)

    def cleanup_and_shutdown(self):
        self.get_logger().info("Experiment complete. Shutting down...")
        if self.timer:
            self.timer.cancel()
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = FreeContactExperimentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class InitSB(StateBehavior):
    """Initial state. Wait for the first robot pose."""

    def tick(self, node):
        if node.init_pose_received:
            node.fsm.begin_track()
        else:
            node.get_logger().info(
                "Waiting for initial pose from state broadcaster...",
                throttle_duration_sec=2.0,
            )


class DoneSB(StateBehavior):
    """Cleanup after experiment."""

    def on_enter(self, node):
        node.get_logger().info("Finished operation successfully.")
        node.cleanup_and_shutdown()

    def tick(self, node):
        pass


if __name__ == '__main__':
    main()