#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

def main(args=None):
    rclpy.init(args=args)
    node = Node('wrench_cli')
    pub = node.create_publisher(WrenchStamped, '/target_wrench', 10)

    print("=== Interactive Wrench Publisher ===")
    print("Enter a Z force in Newtons and press Enter.")
    print("Type 'q' to quit.\n")

    try:
        while rclpy.ok():
            val = input("Target Z Force (N): ").strip()
            if val.lower() == 'q':
                break

            try:
                force_z = float(val)
            except ValueError:
                print("Invalid number. Try again.")
                continue

            msg = WrenchStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'hand'

            msg.wrench.force.z = force_z

            pub.publish(msg)
            print(f"-> Published Z = {force_z} N\n")

    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
