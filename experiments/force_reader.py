import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import math

class ForceListener(Node):
    def __init__(self):
        super().__init__('force_listener')

        # Subscribe to the 1 kHz topic
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/cartesian_impedance_controller/measured_force',
            self.wrench_callback,
            10
        )

        self.latest_msg = None

        # Display timer running at 5 Hz (0.2 seconds) to avoid terminal spam
        # Change 0.2 to 0.5 for 2 Hz, 1.0 for 1 Hz, etc.
        self.display_timer = self.create_timer(0.2, self.display_callback)

    def wrench_callback(self, msg: WrenchStamped):
        # Lightweight callback: just store the latest state
        self.latest_msg = msg

    def display_callback(self):
        if self.latest_msg is None:
            return

        force = self.latest_msg.wrench.force
        torque = self.latest_msg.wrench.torque

        # Calculate total vector magnitudes
        f_mag = math.sqrt(force.x**2 + force.y**2 + force.z**2)
        t_mag = math.sqrt(torque.x**2 + torque.y**2 + torque.z**2)

        # Fixed-width formatting (:7.2f) keeps numbers vertically aligned as signs change
        self.get_logger().info(
            f"\n--- Measured Wrench (5 Hz Display) ---\n"
            f"Force  [N] : X={force.x:7.2f} | Y={force.y:7.2f} | Z={force.z:7.2f} | Mag={f_mag:7.2f}\n"
            f"Torque [Nm]: X={torque.x:7.2f} | Y={torque.y:7.2f} | Z={torque.z:7.2f} | Mag={t_mag:7.2f}"
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
