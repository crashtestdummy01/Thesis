#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class StiffnessCtrlNode(Node):

    def __init__(self):
        super().__init__('crisp_stiffness_ctrl')

        # Adjust topic name to match your CRISP controller instance namespace
        # Commonly: '/cartesian_impedance_controller/stiffness'
        self.topic_name = None

        self.publisher = self.create_publisher(
            Float64MultiArray, self.topic_name, 10
        )
        self.get_logger().info(
            f'Stiffness Tuner online. Publishing to {self.topic_name}'
        )

    def publish_stiffness(self, k_trans: list[float]):
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
        # self.publisher.publish(msg)
        self.get_logger().info(
            f'Published Stiffness -> Trans: {k_trans} N/m, Rot: {k_rot} Nm/rad'
        )


def main():
    rclpy.init()
    node = StiffnessCtrlNode()

    try:
        while rclpy.ok():
            trans_str = input(
                'Translational Stiffness: '
            )
            if trans_str.lower() == 'q':
                break

            try:
                k_trans = trans_str.split(";")
                if len(k_trans) != 3: raise ValueError()
                node.publish_stiffness(k_trans)
            except ValueError:
                print('Invalid input. Please enter numbers.')

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
