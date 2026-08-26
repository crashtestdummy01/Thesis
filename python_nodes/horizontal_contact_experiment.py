#!/usr/bin/env python3

from copy import deepcopy
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray
from statemachine import StateChart, State


class SurfaceContactFSM(StateChart):
    init = State(initial=True)
    approaching = State()
    sliding = State()
    retracting = State()
    resetting = State()
    done = State(final=True)

    start_approach = init.to(approaching)
    trigger_slide = approaching.to(sliding)
    trigger_retract = sliding.to(retracting)
    trigger_reset = retracting.to(resetting)
    finish = resetting.to(done)

    def __init__(self, node):
        self.node = node

        self.behaviors = {
            'init': InitState(),
            'approaching': ApproachState(),
            'sliding': SlideState(),
            'retracting': RetractState(),
            'resetting': ResetState(),
            'done': DoneState(),
        }

        # Track active behavior directly (starts at init)
        self.active_behavior = self.behaviors['init']
        super().__init__()

    def on_transition(self, event, state, target):
        """Lifecycle hook: calls on_enter on the new handler whenever state changes."""
        self.active_behavior = self.behaviors[target.id]
        self.active_behavior.on_enter(self.node)

    def tick(self):
        """Zero API queries, zero string lookups, zero deprecated properties."""
        self.active_behavior.tick(self.node)


class ContactExperiment(Node):
    def __init__(self):
        super().__init__('surface_contact_node')

        # Configuration & Variables
        self.approach_speed_z = 0.015
        self.slide_speed_x = 0.01
        self.reset_speed = 0.04
        self.slide_distance_x = 0.15
        self.contact_threshold_N = 6.5
        self.target_push_force_z = -6.0
        self.surface_offset_z = 0.01

        self.current_force_z = 0.0
        self.current_pose_received = False
        self.target_pose = PoseStamped()
        self.initial_pose = PoseStamped()
        self.surface_z = 0.0
        self.slide_start_x = 0.0

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
        self.fsm = SurfaceContactFSM(node=self)

        self.dt = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt, self.control_loop)

    def force_callback(self, msg: WrenchStamped):
        self.current_force_z = abs(msg.wrench.force.z)

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
            self.current_pose_received = True
            
            self.initial_pose = deepcopy(self.target_pose)

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

        # Uncomment this to work
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
    node = ContactExperiment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class StateBehavior:
    """Abstract base class for all state behaviors."""
    def on_enter(self, node): pass
    def tick(self, node): pass


class InitState(StateBehavior):
    """Initial state. Capture start state."""
    def tick(self, node):
        if node.current_pose_received:
            node.fsm.start_approach()
        else:
            node.get_logger().info_throttle_monotonic(
                node, 2000, "Waiting for initial pose from state broadcaster..."
            )


class ApproachState(StateBehavior):
    """Command the arm to slowly approach the surface. Contact is triggered by force spike along the contact normal."""
    def on_enter(self, node):
        node.get_logger().info("Starting surface approach...")

    def tick(self, node):
        node.target_pose.pose.position.z -= node.approach_speed_z * node.dt
        node.publish_wrench(0.0)

        if node.current_force_z >= node.contact_threshold_N:
            node.fsm.trigger_slide()


class SlideState(StateBehavior):
    """Move the arm while maintaining contact."""
    def on_enter(self, node):
        node.surface_z = node.target_pose.pose.position.z
        node.slide_start_x = node.target_pose.pose.position.x
        node.publish_stiffness([800, 800, 300])
        node.get_logger().info(
            f"Contact detected! Force={node.current_force_z:.2f}N at Z={node.surface_z:.4f}m. Transitioning to SLIDE."
        )

    def tick(self, node):
        node.target_pose.pose.position.z = node.surface_z + node.surface_offset_z
        node.target_pose.pose.position.x += node.slide_speed_x * node.dt
        node.publish_wrench(node.target_push_force_z)

        traveled = node.target_pose.pose.position.x - node.slide_start_x
        if traveled >= node.slide_distance_x:
            node.fsm.trigger_retract()


class RetractState(StateBehavior):
    """Retract arm after motion is completed."""
    def on_enter(self, node: Node):
        node.get_logger().info("Slide complete. Retracting arm...")
        node.publish_stiffness([800, 800, 800])

    def tick(self, node):
        node.publish_wrench(0.0)
        node.target_pose.pose.position.z += 0.01
        node.fsm.trigger_reset()


class ResetState(StateBehavior):
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


class DoneState(StateBehavior):
    """Cleanup after experiment."""
    def on_enter(self, node: Node):
        node.get_logger().info("Finished operation successfully.")

    def tick(self, node):
        node.publish_wrench(0.0)


if __name__ == '__main__':
    main()
