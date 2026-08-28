#!/usr/bin/env python3

from copy import deepcopy

import numpy as np
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray
from statemachine import State
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

from behaviors import StateBehavior, BehaviorGraph


class FreeContactFSM(BehaviorGraph):
    init = State(initial=True)
    align_ee = State()
    approach = State()
    contact = State()
    retracting = State()
    resetting = State()
    done = State(final=True)
    stopped = State(final=True)

    begin_task = init.to(align_ee)
    rotation_done = align_ee.to(approach)
    contact_detected = approach.to(contact)
    trigger_retracting = contact.to(retracting)
    cleanup = retracting.to(resetting)
    finish = resetting.to(done)
    abort = (
            init.to(stopped)
            | align_ee.to(stopped)
            | approach.to(stopped)
            | contact.to(stopped)
            | retracting.to(stopped)
            | resetting.to(stopped)
    )

    def __init__(self, node):
        super().__init__(node)
        self.node = node

        self.attach_behavior('init', InitSB())
        self.attach_behavior('align_ee', AlignEndEffectorSB())
        self.attach_behavior('approach', ApproachSB())
        self.attach_behavior('contact', ContactSB())
        self.attach_behavior('retracting', RetractSB())
        self.attach_behavior('resetting', ResetSB())
        self.attach_behavior('done', DoneSB())


class FreeContactExperimentNode(Node):
    def __init__(self):
        super().__init__('free_contact_node')

        # Configuration & Variables
        self.angular_speed = 0.1
        self.approach_speed = 0.015
        self.reset_speed = 0.04
        self.contact_threshold_N = 6.5
        self.target_push_force_z = 6.0
        self.surface_offset = 0.01

        self.current_force = WrenchStamped().wrench.force
        self.current_pose_received = False
        self.target_pose = PoseStamped()
        self.initial_pose = PoseStamped()
        self.surface_point = np.array([0, 0, 0], dtype=float)
        #self.surface_normal = np.array([0.3536, 0.8536, -0.3536, 0.1464], dtype=float)
        self.surface_normal = np.array([0.9238795, -0.3826834, 0, 0], dtype=float)
  
        # ROS Setup
        self.pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, 'target_wrench', 10)
        self.stiffness_pub = self.create_publisher(Float64MultiArray, '/target_stiffness', 10)

        self.force_sub = self.create_subscription(
            WrenchStamped, '/cartesian_impedance_controller/measured_force', self.force_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/franka_robot_state_broadcaster/current_pose', self.current_pose_callback, 10
        )

        # Attach State Coordinator
        self.fsm = FreeContactFSM(node=self)
        self.fsm.validate()
        self.fsm.start_fsm()

        self.dt = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt, self.control_loop)

    def force_callback(self, msg: WrenchStamped):
        self.current_force = msg.wrench.force

    def current_pose_callback(self, msg: PoseStamped):
        # Capture initial pose once at startup
        if not self.current_pose_received:
            self.target_pose = deepcopy(msg)

            # Compensate for physical ee offsets
            self.target_pose.pose.position.z += 0.109175
            w_o, z_o = 0.9238795, 0.3826834

            x, y = msg.pose.orientation.x, msg.pose.orientation.y
            z, w = msg.pose.orientation.z, msg.pose.orientation.w

            self.target_pose.pose.orientation.x = x * w_o + y * z_o
            self.target_pose.pose.orientation.y = y * w_o - x * z_o
            self.target_pose.pose.orientation.z = z * w_o + w * z_o
            self.target_pose.pose.orientation.w = w * w_o - z * z_o
            print(self.target_pose.pose.orientation)
            self.current_pose_received = True

            self.initial_pose = deepcopy(self.target_pose)
            self.get_logger().info(
                f'Initial pose xyz -> {self.initial_pose.pose.position}'
            )

    def publish_wrench(self, force_z):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'hand'
        msg.wrench.force.z = force_z
        self.wrench_pub.publish(msg)

    def publish_stiffness(self, k_trans):
        msg = Float64MultiArray()

        msg.data = [
            float(k_trans[0]),
            float(k_trans[1]),
            float(k_trans[2]),
            float(30.0),
            float(30.0),
            float(30.0),
        ]

        self.stiffness_pub.publish(msg)
        self.get_logger().info(
            f'Published Stiffness -> {msg.data}'
        )

    def control_loop(self):
        self.fsm.tick()

        # 2. Publish pose target
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.target_pose)


def main(args=None):
    rclpy.init(args=args)
    node = FreeContactExperimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class InitSB(StateBehavior):
    """Initial state. Capture start state."""

    def tick(self, node):
        if node.current_pose_received:
            node.fsm.begin_task()
        else:
            node.get_logger().info_throttle_monotonic(
                node, 2000, "Waiting for initial pose from state broadcaster..."
            )


class AlignEndEffectorSB(StateBehavior):
    """Align end effector local z-axis with contact normal using step-wise error tracking."""

    def on_enter(self, node):
        node.get_logger().info("Aligning end effector orientation...")

    def tick(self, node):
        epsilon = 0.005  # ~0.3 degrees threshold
        # 1. Current target pose orientation in SciPy format [x, y, z, w]
        o_curr = node.target_pose.pose.orientation
        q_curr = np.array([o_curr.x, o_curr.y, o_curr.z, o_curr.w])

        # 2. Extract target quaternion from node.surface_normal array [w, x, y, z] -> [x, y, z, w]
        q_target = node.surface_normal

        # Maintain shortest arc rotation across antipodal quats
        if np.dot(q_curr, q_target) < 0:
            q_target = -q_target

        # 3. Compute remaining angular distance (radians)
        dot_prod = np.clip(abs(np.dot(q_curr, q_target)), -1.0, 1.0)
        angle_error = 2.0 * np.arccos(dot_prod)

        # 4. Check convergence condition
        if angle_error <= epsilon:
            # Snap exactly to target orientation
            node.target_pose.pose.orientation.x = float(q_target[0])
            node.target_pose.pose.orientation.y = float(q_target[1])
            node.target_pose.pose.orientation.z = float(q_target[2])
            node.target_pose.pose.orientation.w = float(q_target[3])
            node.fsm.rotation_done()
            return

        # 5. Advance orientation by step distance (angular_speed * dt)
        step_angle = node.angular_speed * node.dt
        ratio = min(step_angle / angle_error, 1.0)

        times = [0.0, 1.0]
        rotations = R.from_quat([q_curr, q_target])
        slerp = Slerp(times, rotations)
        q_next = slerp(ratio).as_quat()

        # Update pose target
        node.target_pose.pose.orientation.x = float(q_next[0])
        node.target_pose.pose.orientation.y = float(q_next[1])
        node.target_pose.pose.orientation.z = float(q_next[2])
        node.target_pose.pose.orientation.w = float(q_next[3])

    def on_leave(self, node):
        node.get_logger().info("Orientation alignment complete.")


class ApproachSB(StateBehavior):
    """Move the arm along the local Z-axis of the end effector until contact is detected."""

    def on_enter(self, node):
        node.get_logger().info("Moving towards surface...")

    def tick(self, node):
        # 1. Maintain zero wrench during approach
        node.publish_wrench(0.0)

        # 2. Extract current orientation quaternion [x, y, z, w]
        o = node.target_pose.pose.orientation
        q_curr = [o.x, o.y, o.z, o.w]

        # 3. Transform local +Z direction vector [0, 0, 1] into world coordinates
        rot = R.from_quat(q_curr)
        local_z_in_world = rot.apply([0.0, 0.0, 1.0])

        # 4. Advance target position along transformed local Z vector
        step = node.approach_speed * node.dt
        node.target_pose.pose.position.x += float(local_z_in_world[0] * step)
        node.target_pose.pose.position.y += float(local_z_in_world[1] * step)
        node.target_pose.pose.position.z += float(local_z_in_world[2] * step)

        # 5. Check contact condition
        if node.current_force.z >= node.contact_threshold_N:
            # Store contact location
            node.surface_point = np.array([
                node.target_pose.pose.position.x,
                node.target_pose.pose.position.y,
                node.target_pose.pose.position.z,
            ], dtype=float)

            node.fsm.contact_detected()

    def on_leave(self, node):
        node.get_logger().info(
            f"Contact detected! Force={node.current_force.z:.2f}N at Z={node.target_pose.pose.position.z:.4f}m."
        )


class ContactSB(StateBehavior):
    """Maintain contact with given force until user input"""

    def on_enter(self, node):
        node.surface = node.target_pose.pose.position
        node.publish_stiffness([800, 800, 400])

    def tick(self, node):
        ...

class RetractSB(StateBehavior):
    """Retract arm after motion is completed."""
    def on_enter(self, node: Node):
        node.get_logger().info("Task complete. Retracting arm...")
        node.publish_stiffness([800, 800, 800])

    def tick(self, node):
        node.publish_wrench(0.0)
        node.target_pose.pose.position.z += 0.01
        node.fsm.trigger_reset()

class ResetSB(StateBehavior):
    """Slowly linearly interpolate target_pose back to initial_pose."""

    def on_enter(self, node: Node):
        node.get_logger().info("Resetting arm back to initial pose...")

    def tick(self, node):
        node.publish_wrench(0.0)

        curr_p = node.target_pose.pose.position
        init_p = node.initial_pose.pose.position

        dx = init_p.x - curr_p.x
        dy = init_p.y - curr_p.y
        dz = init_p.z - curr_p.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        step_dist = node.reset_speed * node.dt

        if distance <= step_dist:
            node.target_pose.pose.position.x = init_p.x
            node.target_pose.pose.position.y = init_p.y
            node.target_pose.pose.position.z = init_p.z
            node.fsm.finish()
        else:
            ratio = step_dist / distance
            node.target_pose.pose.position.x += dx * ratio
            node.target_pose.pose.position.y += dy * ratio
            node.target_pose.pose.position.z += dz * ratio


class DoneSB(StateBehavior):
    """Cleanup after experiment."""

    def on_enter(self, node: Node):
        node.get_logger().info("Finished operation successfully.")

    def tick(self, node):
        node.publish_wrench(0.0)


if __name__ == '__main__':
    main()
