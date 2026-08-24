#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from copy import deepcopy

class SurfaceContactSlidingNode(Node):
    def __init__(self):
        super().__init__('surface_contact_node')

        # Topic Configurations
        self.target_pose_topic = 'target_pose'
        self.target_wrench_topic = 'target_wrench'
        self.force_topic = '/cartesian_impedance_controller/measured_force'
        self.current_pose_topic = '/franka_robot_state_broadcaster/current_pose'

        # Motion & Force Parameter
        self.approach_speed_z = 0.015   # 1 cm/s descent speed
        self.slide_speed_x = 0.01      # 1 cm/s forward sliding speed
        self.slide_distance_x = 0.1    # 10 cm slide distance
        self.contact_threshold_N = 6.5  # Force spike threshold to register contact (N)
        self.target_push_force_z = -5.0  # Constant bias force into surface (N)
        self.surface_offset_z = 0.01   # 1 mm above surface to avoid position spring fighting

        # Publishers & Subscribers
        self.pose_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, self.target_wrench_topic, 10)
        
        self.force_sub = self.create_subscription(
            WrenchStamped, self.force_topic, self.force_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, self.current_pose_topic, self.current_pose_callback, 10
        )

        # 20 Hz Control Loop
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

        # State Variables
        self.state = 'INIT'
        self.current_force_z = 0.0
        self.current_pose_received = False
        self.target_pose = PoseStamped()
        self.surface_z = 0.0
        self.slide_start_x = 0.0

        self.get_logger().info("Node initialized. Subscribing to current pose...")

    def force_callback(self, msg: WrenchStamped):
        self.current_force_z = abs(msg.wrench.force.z)

    def current_pose_callback(self, msg: PoseStamped):
        # Capture initial pose once at startup
        if not self.current_pose_received:
            self.target_pose = deepcopy(msg)
            self.target_pose.pose.position.z += 0.109175
            
            w_o, z_o = 0.9238795, 0.3826834

            x, y = msg.pose.orientation.x, msg.pose.orientation.y
            z, w = msg.pose.orientation.z, msg.pose.orientation.w

            self.target_pose.pose.orientation.x = x * w_o + y * z_o
            self.target_pose.pose.orientation.y = y * w_o - x * z_o
            self.target_pose.pose.orientation.z = z * w_o + w * z_o
            self.target_pose.pose.orientation.w = w * w_o - z * z_o
            self.current_pose_received = True

    def publish_wrench(self, force_z: float):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'hand'
        msg.wrench.force.z = force_z
        self.wrench_pub.publish(msg)

    def control_loop(self):
        # STATE 0: Wait for initial pose from broadcaster topic
        if self.state == 'INIT':
            if not self.current_pose_received:
                self.get_logger().info_throttle_monotonic(
                    self, 2000, "Waiting for initial pose from state broadcaster..."
                )
                return

            self.get_logger().info(
                f"Initialized target pose at Z={self.target_pose.pose.position.z}. Starting approach..."
            )
            self.state = 'APPROACH'

        # STATE 1: Slowly move downward (-Z) until force spike is detected
        elif self.state == 'APPROACH':
            self.target_pose.pose.position.z -= self.approach_speed_z * self.dt
            self.publish_wrench(0.0)

            if self.current_force_z >= self.contact_threshold_N:
                self.surface_z = self.target_pose.pose.position.z
                self.slide_start_x = self.target_pose.pose.position.x
                self.get_logger().info(
                    f"Contact detected! Force={self.current_force_z:.2f}N at Z={self.surface_z:.4f}m. Transitioning to SLIDE."
                )
                self.state = 'SLIDE'

        # STATE 2: Move along +X while target Z is set slightly above surface & target wrench pushes down
        elif self.state == 'SLIDE':
            self.target_pose.pose.position.z = self.surface_z + self.surface_offset_z
            self.target_pose.pose.position.x += self.slide_speed_x * self.dt

            self.publish_wrench(self.target_push_force_z)

            traveled = self.target_pose.pose.position.x - self.slide_start_x
            if traveled >= self.slide_distance_x:
                self.get_logger().info(f"Completed slide distance of {traveled:.2f}m. Retracting...")
                self.state = 'RETRACT'

        # STATE 3: Zero the wrench and lift upward (+Z)
        elif self.state == 'RETRACT':
            self.publish_wrench(0.0)
            self.target_pose.pose.position.z += 0.02
            self.pose_pub.publish(self.target_pose)
            self.get_logger().info("Finished operation successfully.")
            self.state = 'DONE'

        elif self.state == 'DONE':
            self.publish_wrench(0.0)
            return

        # Publish updated target pose
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.target_pose)


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceContactSlidingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
