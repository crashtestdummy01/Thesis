import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import math
import csv

class ForceListener(Node):
    def __init__(self):
        super().__init__('force_listener')

        self.subscription = self.create_subscription(
            WrenchStamped,
            '/cartesian_impedance_controller/measured_force',
            self.wrench_callback,
            10
        )

        self.latest_msg = None

        self.display_timer = self.create_timer(0.2, self.display_callback)

        self.csv_file = open('force_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time_s', 'force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z'])
        self.start_time = None

    def wrench_callback(self, msg: WrenchStamped):
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

    def display_callback(self):
        if self.latest_msg is None:
            return

        force = self.latest_msg.wrench.force
        torque = self.latest_msg.wrench.torque

        f_mag = math.sqrt(force.x**2 + force.y**2 + force.z**2)
        t_mag = math.sqrt(torque.x**2 + torque.y**2 + torque.z**2)

        self.get_logger().info(
            f"\n--- Measured Wrench (5 Hz Display) ---\n"
            f"Force  [N] : X={force.x:7.2f} | Y={force.y:7.2f} | Z={force.z:7.2f} | Mag={f_mag:7.2f}\n"
            f"Torque [Nm]: X={torque.x:7.2f} | Y={torque.y:7.2f} | Z={torque.z:7.2f} | Mag={t_mag:7.2f}"
        )

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ForceListener()
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
