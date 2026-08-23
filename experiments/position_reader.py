import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class ForceListener(Node):
    def __init__(self):
        super().__init__('pose_listener')

        self.subscription = self.create_subscription(
            PoseStamped,
            '/franka_robot_state_broadcaster/current_pose',
            self.pose_callback,
            10
        )

        self.latest_msg = None

        self.display_timer = self.create_timer(0.2, self.display_callback)

    def pose_callback(self, msg):
        self.latest_msg = msg

    def display_callback(self):
        if self.latest_msg is None:
            return

        coords = self.latest_msg.pose.position
        orientation = self.latest_msg.pose.orientation

        self.get_logger().info(
            f"\n--- Measured Pose ---\n"
            f"Coords  [m] : X={coords.x:7.2f} | Y={coords.y:7.2f} | Z={coords.z:7.2f}\n"
            f"Orientation [quat]: X={orientation.x:7.2f} | Y={orientation.y:7.2f} | Z={orientation.z:7.2f} | W={orientation.w:7.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ForceListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
